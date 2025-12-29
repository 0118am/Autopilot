#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

using namespace std::chrono_literals;

class PoseToPX4Offboard : public rclcpp::Node {
public:
  PoseToPX4Offboard() : Node("pose_to_px4_offboard") {
    this->declare_parameter<std::string>("pose_topic", "/goal_pose");

    const auto pose_topic = this->get_parameter("pose_topic").as_string();

    offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    traj_pub_     = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    cmd_pub_      = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, 10, std::bind(&PoseToPX4Offboard::on_pose, this, std::placeholders::_1));

    timer_ = create_wall_timer(50ms, std::bind(&PoseToPX4Offboard::tick, this)); // 20Hz
    start_time_ = now();

    RCLCPP_INFO(get_logger(), "Sub pose: %s", pose_topic.c_str());
  }

private:
  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // ENU -> NED（常用近似：N=y, E=x, D=-z）
    const float x_enu = static_cast<float>(msg->pose.position.x);
    const float y_enu = static_cast<float>(msg->pose.position.y);
    const float z_enu = static_cast<float>(msg->pose.position.z);

    sp_x_ = y_enu;     // N
    sp_y_ = x_enu;     // E
    sp_z_ = -z_enu;    // D
    sp_yaw_ = 0.0f;    // 先固定 0，后续你再从 EGO 的 yaw 接进来

    have_sp_ = true;
  }

  void tick() {
    const auto now_t = now();
    const uint64_t t_us = static_cast<uint64_t>(now_t.nanoseconds() / 1000);

    // 1) OffboardControlMode：用 position 控制
    px4_msgs::msg::OffboardControlMode off{};
    off.timestamp = t_us;
    off.position = true;
    off.velocity = false;
    off.acceleration = false;
    off.attitude = false;
    off.body_rate = false;
    offboard_pub_->publish(off);

    // 2) TrajectorySetpoint：按你这版 px4_msgs，用 position[3]
    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.timestamp = t_us;
    sp.position = {sp_x_, sp_y_, sp_z_};
    sp.yaw = sp_yaw_;
    traj_pub_->publish(sp);

    // 3) 起飞流程：先连续发 setpoint，再切 offboard + arm
    const double dt = (now_t - start_time_).seconds();
    if (!sent_mode_ && dt > 1.0) {
      send_vehicle_command(176, 1.0f, 6.0f); // MAV_CMD_DO_SET_MODE
      sent_mode_ = true;
    }
    if (!sent_arm_ && dt > 1.5) {
      send_vehicle_command(400, 1.0f);       // MAV_CMD_COMPONENT_ARM_DISARM
      sent_arm_ = true;
    }
  }

  void send_vehicle_command(uint16_t command, float p1 = 0.0f, float p2 = 0.0f) {
    px4_msgs::msg::VehicleCommand vc{};
    vc.timestamp = static_cast<uint64_t>(now().nanoseconds() / 1000);
    vc.param1 = p1;
    vc.param2 = p2;
    vc.command = command;
    vc.target_system = 1;
    vc.target_component = 1;
    vc.source_system = 1;
    vc.source_component = 1;
    vc.from_external = true;
    cmd_pub_->publish(vc);
  }

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time_;
  bool sent_mode_{false};
  bool sent_arm_{false};
  bool have_sp_{false};

  float sp_x_{0.0f}, sp_y_{0.0f}, sp_z_{-1.5f}, sp_yaw_{0.0f};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseToPX4Offboard>());
  rclcpp::shutdown();
  return 0;
}
