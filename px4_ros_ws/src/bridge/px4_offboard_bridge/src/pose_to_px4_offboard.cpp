#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include <quadrotor_msgs/msg/position_command.hpp>

#include <cmath>
#include <limits>
#include <mutex>
#include <string>

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using quadrotor_msgs::msg::PositionCommand;

static inline float wrap_pi(float a) {
  while (a > static_cast<float>(M_PI)) a -= static_cast<float>(2.0 * M_PI);
  while (a < -static_cast<float>(M_PI)) a += static_cast<float>(2.0 * M_PI);
  return a;
}

class PosCmdOffboardBridge : public rclcpp::Node {
public:
  PosCmdOffboardBridge() : Node("pose_to_px4_offboard") {
    // ===== Params =====
    publish_rate_hz_   = declare_parameter<double>("publish_rate_hz", 50.0);
    warmup_setpoints_  = declare_parameter<int>("warmup_setpoints", 15);
    stale_timeout_s_   = declare_parameter<double>("stale_timeout_s", 0.5);
    enu_to_ned_        = declare_parameter<bool>("enu_to_ned", true);
    auto_offboard_arm_ = declare_parameter<bool>("auto_offboard_arm", true);

    // failsafe: "hold" | "stop" | "disarm"
    failsafe_action_   = declare_parameter<std::string>("failsafe_action", "hold");

    pos_cmd_topic_     = declare_parameter<std::string>("pos_cmd_topic", "/drone_0_planning/pos_cmd");

    offboard_mode_topic_ = declare_parameter<std::string>("offboard_mode_topic", "/fmu/in/offboard_control_mode");
    traj_sp_topic_       = declare_parameter<std::string>("traj_sp_topic", "/fmu/in/trajectory_setpoint");
    vehicle_cmd_topic_   = declare_parameter<std::string>("vehicle_cmd_topic", "/fmu/in/vehicle_command");

    target_system_     = declare_parameter<int>("target_system", 1);
    target_component_  = declare_parameter<int>("target_component", 1);
    source_system_     = declare_parameter<int>("source_system", 1);
    source_component_  = declare_parameter<int>("source_component", 1);

    // ===== Pubs =====
    offboard_pub_ = create_publisher<OffboardControlMode>(offboard_mode_topic_, 10);
    traj_pub_     = create_publisher<TrajectorySetpoint>(traj_sp_topic_, 10);
    cmd_pub_      = create_publisher<VehicleCommand>(vehicle_cmd_topic_, 10);

    // ===== Sub =====
    pos_cmd_sub_ = create_subscription<PositionCommand>(
      pos_cmd_topic_, rclcpp::QoS(50).reliable(),
      std::bind(&PosCmdOffboardBridge::onPosCmd, this, std::placeholders::_1));

    // ===== Timer =====
    const double hz = std::max(1.0, publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / hz)),
      std::bind(&PosCmdOffboardBridge::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "OffboardBridge: sub=%s rate=%.1fHz warmup=%d stale=%.2fs enu_to_ned=%s failsafe=%s",
      pos_cmd_topic_.c_str(), publish_rate_hz_, warmup_setpoints_, stale_timeout_s_,
      enu_to_ned_ ? "true" : "false", failsafe_action_.c_str());
  }

private:
  enum class State { WAIT_CMD, WARMUP, ACTIVE };

  void onPosCmd(const PositionCommand::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    last_cmd_ = *msg;
    last_cmd_time_ = now();
    have_cmd_ = true;

    if (state_ == State::WAIT_CMD) {
      state_ = State::WARMUP;
      warmup_counter_ = 0;
      RCLCPP_INFO(get_logger(), "Got first pos_cmd -> WARMUP");
    }
  }

  uint64_t now_us() const {
    return static_cast<uint64_t>(this->now().nanoseconds() / 1000ULL);
  }

  void publishOffboardMode() {
    OffboardControlMode m{};
    m.timestamp = now_us();
    m.position = true;
    m.velocity = true;
    m.acceleration = true;
    m.attitude = false;
    m.body_rate = false;
    offboard_pub_->publish(m);
  }

  void publishVehicleCommand(uint16_t command, float p1=0.f, float p2=0.f, float p3=0.f,
                             float p4=0.f, float p5=0.f, float p6=0.f, float p7=0.f) {
    VehicleCommand vc{};
    vc.timestamp = now_us();
    vc.command = command;
    vc.param1 = p1; vc.param2 = p2; vc.param3 = p3; vc.param4 = p4;
    vc.param5 = p5; vc.param6 = p6; vc.param7 = p7;
    vc.target_system = static_cast<uint8_t>(target_system_);
    vc.target_component = static_cast<uint8_t>(target_component_);
    vc.source_system = static_cast<uint8_t>(source_system_);
    vc.source_component = static_cast<uint8_t>(source_component_);
    vc.from_external = true;
    vc.confirmation = 0;
    cmd_pub_->publish(vc);
  }

  TrajectorySetpoint toSetpoint(const PositionCommand &pc) const {
    TrajectorySetpoint sp{};
    sp.timestamp = now_us();

    float x = static_cast<float>(pc.position.x);
    float y = static_cast<float>(pc.position.y);
    float z = static_cast<float>(pc.position.z);

    float vx = static_cast<float>(pc.velocity.x);
    float vy = static_cast<float>(pc.velocity.y);
    float vz = static_cast<float>(pc.velocity.z);

    float ax = static_cast<float>(pc.acceleration.x);
    float ay = static_cast<float>(pc.acceleration.y);
    float az = static_cast<float>(pc.acceleration.z);

    float yaw = static_cast<float>(pc.yaw);
    float yawspeed = static_cast<float>(pc.yaw_dot);

    const float nan = std::numeric_limits<float>::quiet_NaN();
    sp.jerk = {nan, nan, nan};

    if (!enu_to_ned_) {
      sp.position = {x, y, z};
      sp.velocity = {vx, vy, vz};
      sp.acceleration = {ax, ay, az};
      sp.yaw = wrap_pi(yaw);
      sp.yawspeed = yawspeed;
      return sp;
    }

    // ENU -> NED
    const float N  = y;
    const float E  = x;
    const float D  = -z;

    const float vN = vy;
    const float vE = vx;
    const float vD = -vz;

    const float aN = ay;
    const float aE = ax;
    const float aD = -az;

    float yaw_ned = wrap_pi(static_cast<float>(M_PI_2) - yaw);
    float yawspeed_ned = -yawspeed;

    sp.position = {N, E, D};
    sp.velocity = {vN, vE, vD};
    sp.acceleration = {aN, aE, aD};
    sp.yaw = yaw_ned;
    sp.yawspeed = yawspeed_ned;

    return sp;
  }

  TrajectorySetpoint failsafeSetpoint(const TrajectorySetpoint &last) const {
    if (failsafe_action_ == "hold") {
      return last; // 继续发最后一个 setpoint
    }
    if (failsafe_action_ == "stop") {
      auto sp = last;
      sp.velocity = {0.f, 0.f, 0.f};
      sp.acceleration = {0.f, 0.f, 0.f};
      sp.yawspeed = 0.f;
      return sp;
    }
    // "disarm"：仍然发 last setpoint，另外发送 disarm（见 onTimer）
    return last;
  }

  void onTimer() {
    PositionCommand pc{};
    rclcpp::Time t_last;
    bool have = false;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      have = have_cmd_;
      pc = last_cmd_;
      t_last = last_cmd_time_;
    }

    if (!have) {
      // 没有任何轨迹输入：绝不发 /fmu/in/*
      return;
    }

    // always: offboard heartbeat
    publishOffboardMode();

    // build setpoint
    TrajectorySetpoint sp = toSetpoint(pc);

    // watchdog
    const double age = (now() - t_last).seconds();
    if (age > stale_timeout_s_) {
      sp = failsafeSetpoint(sp);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "pos_cmd stale %.3fs -> failsafe(%s)",
                           age, failsafe_action_.c_str());

      if (failsafe_action_ == "disarm") {
        // VEHICLE_CMD_COMPONENT_ARM_DISARM param1=0
        publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
      }
    }

    traj_pub_->publish(sp);

    // state machine
    if (state_ == State::WARMUP) {
      warmup_counter_++;
      if (auto_offboard_arm_ && warmup_counter_ >= static_cast<uint64_t>(warmup_setpoints_)) {
        // Set OFFBOARD mode: VEHICLE_CMD_DO_SET_MODE param1=1(custom) param2=6(offboard)
        publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
        // Arm
        publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        state_ = State::ACTIVE;
        RCLCPP_INFO(get_logger(), "WARMUP done -> sent OFFBOARD + ARM, enter ACTIVE");
      }
    }
  }

private:
  // params
  double publish_rate_hz_{50.0};
  int warmup_setpoints_{15};
  double stale_timeout_s_{0.5};
  bool enu_to_ned_{true};
  bool auto_offboard_arm_{true};
  std::string failsafe_action_{"hold"};

  int target_system_{1}, target_component_{1}, source_system_{1}, source_component_{1};

  std::string pos_cmd_topic_;
  std::string offboard_mode_topic_, traj_sp_topic_, vehicle_cmd_topic_;

  // ros
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<PositionCommand>::SharedPtr pos_cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // state
  mutable std::mutex mtx_;
  bool have_cmd_{false};
  PositionCommand last_cmd_{};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  State state_{State::WAIT_CMD};
  uint64_t warmup_counter_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PosCmdOffboardBridge>());
  rclcpp::shutdown();
  return 0;
}
