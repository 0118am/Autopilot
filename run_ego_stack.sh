#!/usr/bin/env bash
set -eo pipefail

### ===== 可配置区 =====
AUTOPILOT_DIR="${AUTOPILOT_DIR:-$HOME/Autopilot}"
EGO_WS="${EGO_WS:-$AUTOPILOT_DIR/ego_swarm_ws}"
PX4_WS="${PX4_WS:-$AUTOPILOT_DIR/px4_ros_ws}"

DRONE_ID="${DRONE_ID:-0}"

# PX4 -> ROS Odom（系统内统一用这个）
ROS_ODOM_TOPIC="${ROS_ODOM_TOPIC:-/drone_${DRONE_ID}_odom}"

# EGO 内部如果还需要一个 odometry_topic，就默认等于 ROS_ODOM_TOPIC（避免再造 /odom /odometry 的中继）
EGO_ODOMETRY_TOPIC="${EGO_ODOMETRY_TOPIC:-${ROS_ODOM_TOPIC}}"

# Gazebo 点云
GZ_PC2_TOPIC="${GZ_PC2_TOPIC:-/depth_camera/points}"

# flight control
FLIGHT_TYPE="${FLIGHT_TYPE:-1}"   # 1=手动goal模式
POS_CMD_TOPIC="${POS_CMD_TOPIC:-/drone_${DRONE_ID}_planning/pos_cmd}"

# ===== Gazebo topics（已验证）=====
GZ_RGB_IMAGE_TOPIC="${GZ_RGB_IMAGE_TOPIC:-/world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/image}"
GZ_RGB_INFO_TOPIC="${GZ_RGB_INFO_TOPIC:-/world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info}"
GZ_DEPTH_IMAGE_TOPIC="${GZ_DEPTH_IMAGE_TOPIC:-/depth_camera}"
GZ_DEPTH_INFO_TOPIC="${GZ_DEPTH_INFO_TOPIC:-/camera_info}"

# ===== ROS topics（统一带 drone_id 命名空间，避免多机冲突）=====
ROS_CAMERA_NS="${ROS_CAMERA_NS:-/drone_${DRONE_ID}_camera}"
ROS_RGB_IMAGE_TOPIC="${ROS_RGB_IMAGE_TOPIC:-${ROS_CAMERA_NS}/image_raw}"
ROS_RGB_INFO_TOPIC="${ROS_RGB_INFO_TOPIC:-${ROS_CAMERA_NS}/camera_info}"
ROS_DEPTH_IMAGE_TOPIC="${ROS_DEPTH_IMAGE_TOPIC:-${ROS_CAMERA_NS}/depth/image_raw}"
ROS_DEPTH_INFO_TOPIC="${ROS_DEPTH_INFO_TOPIC:-${ROS_CAMERA_NS}/depth/camera_info}"

# 给 external_gz_px4.launch.py 用
EGO_ODOM_ARG="${EGO_ODOM_ARG:-/drone_${DRONE_ID}_odom_sync}"

# 点云：bridge 出来就带 drone_id
# 这个 topic 会作为 pcl_render_node 的输入（如果 external_gz_px4.launch.py 内部起了 pcl_render_node）
ROS_CLOUD_TOPIC="${ROS_CLOUD_TOPIC:-/drone_${DRONE_ID}_cloud}"

EGO_LAUNCH_PKG="${EGO_LAUNCH_PKG:-ego_planner}"
EGO_LAUNCH_FILE="${EGO_LAUNCH_FILE:-external_gz_px4.launch.py}"

CLEAN_OLD="${CLEAN_OLD:-1}"
WAIT_CLOCK="${WAIT_CLOCK:-1}"
CHECK_TF="${CHECK_TF:-1}"

# 静态 TF 兜底：如果你已经有 robot_state_publisher/URDF 在发 base_link->camera_link，把它设为 0
ENABLE_BASE_CAMERA_TF="${ENABLE_BASE_CAMERA_TF:-1}"
ENABLE_WORLD_ODOM_TF="${ENABLE_WORLD_ODOM_TF:-1}"

# ===== EGO 输入适配器（把 /drone_0_camera/* + /drone_0_cloud + /drone_0_odom 喂给 /drone_0_pcl_render_node/*）=====
ENABLE_EGO_INPUT_ADAPTER="${ENABLE_EGO_INPUT_ADAPTER:-1}"

PCL_NS="/drone_${DRONE_ID}_pcl_render_node"
PCL_DEPTH_TOPIC="${PCL_NS}/depth"
PCL_DEPTH_INFO_TOPIC="${PCL_NS}/camera_info"
PCL_CAMERA_POSE_TOPIC="${PCL_NS}/camera_pose"
PCL_CLOUD_TOPIC="${PCL_NS}/cloud"

PCL_NS_DRONE="/drone_${DRONE_ID}_pcl_render_node"
PCL_NS_GLOBAL="/pcl_render_node"

PCL_DEPTH_TOPIC_A="${PCL_NS_DRONE}/depth"
PCL_DEPTH_INFO_TOPIC_A="${PCL_NS_DRONE}/camera_info"
PCL_CAMERA_POSE_TOPIC_A="${PCL_NS_DRONE}/camera_pose"

PCL_DEPTH_TOPIC_B="${PCL_NS_GLOBAL}/depth"
PCL_DEPTH_INFO_TOPIC_B="${PCL_NS_GLOBAL}/camera_info"
PCL_CAMERA_POSE_TOPIC_B="${PCL_NS_GLOBAL}/camera_pose"

LOG_DIR="${LOG_DIR:-$AUTOPILOT_DIR/_logs_ego_stack}"
mkdir -p "$LOG_DIR"

### ===== 公共函数 =====
log(){ echo -e "[run_ego_stack] $*"; }

need_file(){
  local f="$1"
  [[ -f "$f" ]] || { echo "[ERROR] missing file: $f" >&2; exit 1; }
}

start_bg(){
  local name="$1"; shift
  local logfile="$LOG_DIR/${name}.log"
  log "START  $name  -> $logfile"
  (stdbuf -oL -eL "$@" 2>&1 | tee -a "$logfile") &
  echo $! > "$LOG_DIR/${name}.pid"
}

kill_if_running(){
  local name="$1"
  local pidfile="$LOG_DIR/${name}.pid"
  if [[ -f "$pidfile" ]]; then
    local pid
    pid="$(cat "$pidfile" || true)"
    if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
      log "STOP   $name (pid=$pid)"
      kill "$pid" 2>/dev/null || true
    fi
    rm -f "$pidfile"
  fi
}

wait_topic_once(){
  local topic="$1"
  local t="${2:-10}"
  log "Waiting for ${topic} (timeout ${t}s, reliable) ..."
  timeout "${t}s" bash -lc "ros2 topic echo ${topic} --once >/dev/null" || {
    log "WARN: no message on ${topic} within ${t}s"
    return 1
  }
  return 0
}

wait_sensor_once(){
  local topic="$1"
  local t="${2:-10}"
  log "Waiting for ${topic} (timeout ${t}s, best_effort) ..."
  timeout "${t}s" bash -lc "ros2 topic echo ${topic} --once --qos-reliability best_effort >/dev/null" || {
    log "WARN: no message on ${topic} within ${t}s"
    return 1
  }
  return 0
}

cleanup(){
  log "CLEANUP..."
  kill_if_running "ego_launch"
  kill_if_running "px4_odom_to_ros"
  kill_if_running "gz_bridge"
  kill_if_running "tf_base_camera"
  kill_if_running "px4_offboard"
  kill_if_running "tf_world_odom"
  kill_if_running "ego_input_adapter"
  kill_if_running "takeoff_hold"
  kill_if_running "takeoff_to_height"
}
trap cleanup EXIT

### ===== 0) source 环境 + CycloneDDS =====
need_file "/opt/ros/humble/setup.bash"
source /opt/ros/humble/setup.bash

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

log "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
log "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
log "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"

if [[ -f "$EGO_WS/install/setup.bash" ]]; then
  source "$EGO_WS/install/setup.bash"
else
  log "WARN: $EGO_WS/install/setup.bash not found"
fi

if [[ -f "$PX4_WS/install/setup.bash" ]]; then
  source "$PX4_WS/install/setup.bash"
else
  log "WARN: $PX4_WS/install/setup.bash not found"
fi

SIM_ARGS=(--ros-args -p use_sim_time:=true)

log "DRONE_ID=$DRONE_ID"
log "ROS_ODOM_TOPIC=$ROS_ODOM_TOPIC"
log "EGO_ODOMETRY_TOPIC=$EGO_ODOMETRY_TOPIC"
log "ROS_CLOUD_TOPIC=$ROS_CLOUD_TOPIC"
log "ROS_CAMERA_NS=$ROS_CAMERA_NS"

### ===== 1) 清理旧进程 =====
if [[ "${CLEAN_OLD:-1}" == "1" ]]; then
  log "Cleaning old processes (pidfile-first, strict pkill fallback)..."

  # 0) 先按 pidfile 清一次（上一轮脚本留下的最干净）
  cleanup || true
  sleep 0.2

  # 1) 再兜底：只杀“明确的可执行命令行”，不要按节点名/泛字符串杀
  pkill -TERM -f -- "ros2 run ros_gz_bridge parameter_bridge" 2>/dev/null || true
  pkill -TERM -f -- "ros2 run px4_odom_bridge px4_odom_to_ros" 2>/dev/null || true
  pkill -TERM -f -- "ros2 run px4_offboard_bridge pose_to_px4_offboard" 2>/dev/null || true
  pkill -TERM -f -- "ros2 run local_sensing pcl_render_node" 2>/dev/null || true
  pkill -TERM -f -- "python3 .*ego_input_adapter\.py" 2>/dev/null || true

  # 只杀你的 launch 进程本体（它会带走 ego_planner_node / traj_server 等子节点）
  pkill -TERM -f -- "ros2 launch ${EGO_LAUNCH_PKG} ${EGO_LAUNCH_FILE}" 2>/dev/null || true

  # !!! 下面三条不要再用：太宽，最容易误杀“刚启动的新进程”
  # pkill -f "ego_planner_node"
  # pkill -f "traj_server"
  # pkill -f "static_transform_publisher"

  # 2) 给点时间优雅退出；不退再 KILL（可选但建议）
  sleep 0.6
  pkill -KILL -f -- "ros2 run ros_gz_bridge parameter_bridge" 2>/dev/null || true
  pkill -KILL -f -- "ros2 run local_sensing pcl_render_node" 2>/dev/null || true
  pkill -KILL -f -- "python3 .*ego_input_adapter\.py" 2>/dev/null || true
  pkill -KILL -f -- "ros2 launch ${EGO_LAUNCH_PKG} ${EGO_LAUNCH_FILE}" 2>/dev/null || true
fi

### ===== 2) Gazebo -> ROS：clock + 点云 + camera_info + 两路 image（统一用 ros_gz_bridge）=====
BRIDGE_YAML="$LOG_DIR/gz_bridge.yaml"
cat > "$BRIDGE_YAML" <<EOF
- ros_topic_name: /clock
  gz_topic_name: /clock
  ros_type_name: rosgraph_msgs/msg/Clock
  gz_type_name: gz.msgs.Clock
  direction: GZ_TO_ROS
  lazy: false

- ros_topic_name: ${ROS_RGB_INFO_TOPIC}
  gz_topic_name: ${GZ_RGB_INFO_TOPIC}
  ros_type_name: sensor_msgs/msg/CameraInfo
  gz_type_name: gz.msgs.CameraInfo
  direction: GZ_TO_ROS
  lazy: false

- ros_topic_name: ${ROS_DEPTH_INFO_TOPIC}
  gz_topic_name: ${GZ_DEPTH_INFO_TOPIC}
  ros_type_name: sensor_msgs/msg/CameraInfo
  gz_type_name: gz.msgs.CameraInfo
  direction: GZ_TO_ROS
  lazy: false

- ros_topic_name: ${ROS_RGB_IMAGE_TOPIC}
  gz_topic_name: ${GZ_RGB_IMAGE_TOPIC}
  ros_type_name: sensor_msgs/msg/Image
  gz_type_name: gz.msgs.Image
  direction: GZ_TO_ROS
  lazy: false

- ros_topic_name: ${ROS_DEPTH_IMAGE_TOPIC}
  gz_topic_name: ${GZ_DEPTH_IMAGE_TOPIC}
  ros_type_name: sensor_msgs/msg/Image
  gz_type_name: gz.msgs.Image
  direction: GZ_TO_ROS
  lazy: false

- ros_topic_name: ${ROS_CLOUD_TOPIC}
  gz_topic_name: ${GZ_PC2_TOPIC}
  ros_type_name: sensor_msgs/msg/PointCloud2
  gz_type_name: gz.msgs.PointCloudPacked
  direction: GZ_TO_ROS
  lazy: false
EOF

start_bg "gz_bridge" ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:="$BRIDGE_YAML" -p use_sim_time:=true

# ===== 自检：clock & cloud 优先（比 image 更关键）=====
if [[ "$WAIT_CLOCK" == "1" ]]; then
  wait_topic_once /clock 15 || true
fi
wait_sensor_once "${ROS_CLOUD_TOPIC}" 10 || true
wait_sensor_once "${ROS_DEPTH_IMAGE_TOPIC}" 10 || true
wait_sensor_once "${ROS_DEPTH_INFO_TOPIC}" 10 || true

### ===== 3) TF =====
# world -> odom（让 world frame 存在；否则 occupancy_inflate 用 world 会一直空）
if [[ "$ENABLE_WORLD_ODOM_TF" == "1" ]]; then
  start_bg "tf_world_odom" ros2 run tf2_ros static_transform_publisher \
    --frame-id world --child-frame-id odom \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    "${SIM_ARGS[@]}"
else
  log "SKIP: world->odom static TF disabled (ENABLE_WORLD_ODOM_TF=0)"
fi

# base_link -> camera_link（仅兜底）
if [[ "$ENABLE_BASE_CAMERA_TF" == "1" ]]; then
  start_bg "tf_base_camera" ros2 run tf2_ros static_transform_publisher \
    --frame-id base_link --child-frame-id camera_link \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    "${SIM_ARGS[@]}"
fi

### ===== 4) PX4 -> ROS odom =====
start_bg "px4_odom_to_ros" ros2 run px4_odom_bridge px4_odom_to_ros \
  --ros-args \
  -p odom_topic:="/drone_${DRONE_ID}_odom_sync" \
  -p odom_frame:=odom \
  -p base_frame:=base_link \
  -p use_sim_time:=true

wait_topic_once "${ROS_ODOM_TOPIC}" 10 || true

log "TF publishers:"
ros2 topic info -v /tf || true
ros2 topic info -v /tf_static || true

### ===== 4.8) ROS -> PX4 offboard bridge =====
start_bg "px4_offboard" ros2 run px4_offboard_bridge pose_to_px4_offboard \
  --ros-args \
  -p use_sim_time:=true \
  -r /position_cmd:="${POS_CMD_TOPIC}" \
  -r position_cmd:="${POS_CMD_TOPIC}" \
  -r /pos_cmd:="${POS_CMD_TOPIC}" \
  -r pos_cmd:="${POS_CMD_TOPIC}" \
  -r /planning/pos_cmd:="${POS_CMD_TOPIC}" \
  -r planning/pos_cmd:="${POS_CMD_TOPIC}"


### ===== 4.9) 起飞到 2m 悬停 =====
TAKEOFF_Z="${TAKEOFF_Z:-2.0}"
TAKEOFF_TOL="${TAKEOFF_TOL:-0.12}"      # 到 2m ±12cm 算到位
TAKEOFF_TIMEOUT="${TAKEOFF_TIMEOUT:-40}" # 最多等 40s
TAKEOFF_RATE="${TAKEOFF_RATE:-20.0}"    # 20Hz setpoint

TAKEOFF_PY="$LOG_DIR/takeoff_to_height.py"
cat > "$TAKEOFF_PY" <<'PY'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Point
from quadrotor_msgs.msg import PositionCommand
from traj_utils.msg import Bspline

def zeros_vec3():
    v = Vector3()
    v.x = v.y = v.z = 0.0
    return v

class TakeoffHold(Node):
    def __init__(self):
        super().__init__("takeoff_hold")

        self.odom_topic    = self.declare_parameter("odom_topic", "/drone_0_odom").value
        self.cmd_topic     = self.declare_parameter("cmd_topic",  "/drone_0_planning/pos_cmd").value
        self.bspline_topic = self.declare_parameter("bspline_topic", "/drone_0_planning/bspline").value

        self.target_z = float(self.declare_parameter("target_z", 2.0).value)
        self.tol_z    = float(self.declare_parameter("tol_z", 0.12).value)
        self.rate_hz  = float(self.declare_parameter("rate_hz", 20.0).value)
        self.handover_grace_s = float(self.declare_parameter("handover_grace_s", 0.5).value)

        self.have_odom = False
        self.home_x = 0.0
        self.home_y = 0.0
        self.cur_z = 0.0

        self.reached_count = 0
        self.have_bspline = False
        self.grace_count = 0
        self.announced_reached = False

        self.pub = self.create_publisher(PositionCommand, self.cmd_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.cb_odom, 20)
        self.create_subscription(Bspline, self.bspline_topic, self.cb_bspline, 10)

        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)

        self.get_logger().info(
            f"hold: odom={self.odom_topic}, cmd={self.cmd_topic}, bspline={self.bspline_topic}, "
            f"target_z={self.target_z}, tol_z={self.tol_z}, rate={self.rate_hz}Hz"
        )

    def cb_bspline(self, _msg: Bspline):
        if not self.have_bspline:
            self.have_bspline = True
            self.get_logger().info("Got first bspline -> stop takeoff_hold")
            rclpy.shutdown()

    def cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        if not self.have_odom:
            self.home_x = float(p.x)
            self.home_y = float(p.y)
            self.have_odom = True
            self.get_logger().info(f"home_xy=({self.home_x:.2f},{self.home_y:.2f})")
        self.cur_z = float(p.z)

    def tick(self):
        if not self.have_odom:
            return

        # publish hover setpoint
        cmd = PositionCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "odom"
        cmd.position = Point(x=self.home_x, y=self.home_y, z=self.target_z)

        if hasattr(cmd, "velocity"):     cmd.velocity = zeros_vec3()
        if hasattr(cmd, "acceleration"): cmd.acceleration = zeros_vec3()
        if hasattr(cmd, "jerk"):         cmd.jerk = zeros_vec3()
        if hasattr(cmd, "yaw"):          cmd.yaw = 0.0
        if hasattr(cmd, "yaw_dot"):      cmd.yaw_dot = 0.0

        self.pub.publish(cmd)

        # reached counting
        if abs(self.cur_z - self.target_z) <= self.tol_z:
            self.reached_count += 1
        else:
            self.reached_count = 0
            self.grace_count = 0

        # 1s stable reached
        if self.reached_count >= int(self.rate_hz * 1.0) and not self.announced_reached:
            self.announced_reached = True
            self.get_logger().info(f"Reached target_z={self.target_z} (cur_z={self.cur_z:.2f}), waiting for bspline...")

        # handover: reached + have_bspline -> keep grace then exit
        if self.reached_count >= int(self.rate_hz * 1.0) and self.have_bspline:
            self.grace_count += 1
            if self.grace_count >= int(self.rate_hz * self.handover_grace_s):
                self.get_logger().info("Handover done -> stop takeoff_hold publisher")
                rclpy.shutdown()

def main():
    rclpy.init()
    node = TakeoffHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
PY
chmod +x "$TAKEOFF_PY"

log "TAKEOFF: start hold publisher (keeps pos_cmd alive until planner outputs bspline)..."
start_bg "takeoff_to_height" python3 "$TAKEOFF_PY" --ros-args \
  -p use_sim_time:=true \
  -p odom_topic:="${ROS_ODOM_TOPIC}" \
  -p cmd_topic:="/drone_${DRONE_ID}_planning/pos_cmd" \
  -p bspline_topic:="/drone_${DRONE_ID}_planning/bspline" \
  -p target_z:="${TAKEOFF_Z}" \
  -p tol_z:="${TAKEOFF_TOL}" \
  -p rate_hz:="${TAKEOFF_RATE}" \
  -p handover_grace_s:=0.5

### ===== EGO 输入适配器：把 /drone_${DRONE_ID}_camera/* + /drone_${DRONE_ID}_cloud + /drone_${DRONE_ID}_odom
###                   转成 EGO 期望的 /drone_${DRONE_ID}_pcl_render_node/* + /odom_sync =====
ENABLE_EGO_INPUT_ADAPTER="${ENABLE_EGO_INPUT_ADAPTER:-1}"

if [[ "$ENABLE_EGO_INPUT_ADAPTER" == "1" ]]; then
  # 防止重复
  pkill -f "ego_input_adapter.py" 2>/dev/null || true

  ADAPTER_PY="$LOG_DIR/ego_input_adapter.py"
  cat > "$ADAPTER_PY" <<'PY'
#!/usr/bin/env python3
import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def qos_reliable(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )

QOS_SUB_IMG   = qos_reliable(10)
QOS_SUB_INFO  = qos_reliable(10)
QOS_SUB_CLOUD = qos_reliable(10)
QOS_SUB_ODOM  = qos_reliable(20)
QOS_PUB       = qos_reliable(10)

class Adapter(Node):
    def __init__(self):
        super().__init__("ego_input_adapter")

        # inputs
        self.in_odom  = self.declare_parameter("in_odom",  "/drone_0_odom").value
        self.in_depth = self.declare_parameter("in_depth", "/drone_0_camera/depth/image_raw").value
        self.in_info  = self.declare_parameter("in_info",  "/drone_0_camera/depth/camera_info").value
        self.in_cloud = self.declare_parameter("in_cloud", "/drone_0_cloud").value

        # outputs（给 EGO / advanced_param 兜底：drone_ns + global 两套都发）
        self.out_depth_a = self.declare_parameter("out_depth_a", "/drone_0_pcl_render_node/depth").value
        self.out_pose_a  = self.declare_parameter("out_pose_a",  "/drone_0_pcl_render_node/camera_pose").value
        self.out_info_a  = self.declare_parameter("out_info_a",  "/drone_0_pcl_render_node/camera_info").value
        self.out_cloud_a = self.declare_parameter("out_cloud_a", "/drone_0_pcl_render_node/cloud").value
        self.out_odom_a  = self.declare_parameter("out_odom_a",  "/drone_0_odom_sync").value

        self.out_depth_b = self.declare_parameter("out_depth_b", "/pcl_render_node/depth").value
        self.out_pose_b  = self.declare_parameter("out_pose_b",  "/pcl_render_node/camera_pose").value
        self.out_info_b  = self.declare_parameter("out_info_b",  "/pcl_render_node/camera_info").value
        self.out_cloud_b = self.declare_parameter("out_cloud_b", "/pcl_render_node/cloud").value
        self.out_odom_b  = self.declare_parameter("out_odom_b",  "/odom_sync").value

        self.warn_dt = float(self.declare_parameter("warn_dt", 0.10).value)

        # pubs
        self.pub_depth_a = self.create_publisher(Image,      self.out_depth_a, QOS_PUB)
        self.pub_depth_b = self.create_publisher(Image,      self.out_depth_b, QOS_PUB)
        self.pub_pose_a  = self.create_publisher(PoseStamped,self.out_pose_a,  QOS_PUB)
        self.pub_pose_b  = self.create_publisher(PoseStamped,self.out_pose_b,  QOS_PUB)
        self.pub_info_a  = self.create_publisher(CameraInfo, self.out_info_a,  QOS_PUB)
        self.pub_info_b  = self.create_publisher(CameraInfo, self.out_info_b,  QOS_PUB)
        self.pub_cloud_a = self.create_publisher(PointCloud2,self.out_cloud_a, QOS_PUB)
        self.pub_cloud_b = self.create_publisher(PointCloud2,self.out_cloud_b, QOS_PUB)
        self.pub_odom_a  = self.create_publisher(Odometry,   self.out_odom_a,  QOS_PUB)
        self.pub_odom_b  = self.create_publisher(Odometry,   self.out_odom_b,  QOS_PUB)

        # cache
        self.last_odom: Odometry | None = None
        self.last_info: CameraInfo | None = None

        # subs
        self.create_subscription(Odometry,    self.in_odom,  self.cb_odom,  QOS_SUB_ODOM)
        self.create_subscription(CameraInfo, self.in_info,  self.cb_info,  QOS_SUB_INFO)
        self.create_subscription(Image,      self.in_depth, self.cb_depth, QOS_SUB_IMG)
        self.create_subscription(PointCloud2,self.in_cloud, self.cb_cloud, QOS_SUB_CLOUD)

        self.get_logger().info(f"in_odom={self.in_odom}")
        self.get_logger().info(f"in_depth={self.in_depth}")
        self.get_logger().info(f"in_info={self.in_info}")
        self.get_logger().info(f"in_cloud={self.in_cloud}")

    def _t2s(self, t): return float(t.sec) + float(t.nanosec) * 1e-9

    def cb_odom(self, msg: Odometry):
        self.last_odom = msg

    def cb_info(self, msg: CameraInfo):
        self.last_info = msg

    def cb_cloud(self, msg: PointCloud2):
        self.pub_cloud_a.publish(msg)
        self.pub_cloud_b.publish(msg)

    def cb_depth(self, msg: Image):
        # depth 直接转发
        self.pub_depth_a.publish(msg)
        self.pub_depth_b.publish(msg)

        if self.last_odom is None:
            return

        # 用 depth 的 stamp 强制对齐 pose/odom/info（让 grid_map 能更新）
        stamp = msg.header.stamp

        o = Odometry()
        o.header = copy.deepcopy(self.last_odom.header)
        o.header.stamp = stamp
        o.child_frame_id = self.last_odom.child_frame_id
        o.pose = self.last_odom.pose
        o.twist = self.last_odom.twist
        self.pub_odom_a.publish(o)
        self.pub_odom_b.publish(o)

        ps = PoseStamped()
        ps.header.frame_id = "odom"
        ps.header.stamp = stamp
        ps.pose = self.last_odom.pose.pose
        self.pub_pose_a.publish(ps)
        self.pub_pose_b.publish(ps)

        if self.last_info is not None:
            ci = copy.deepcopy(self.last_info)
            ci.header.stamp = stamp
            self.pub_info_a.publish(ci)
            self.pub_info_b.publish(ci)

        dt = abs(self._t2s(stamp) - self._t2s(self.last_odom.header.stamp))
        if dt > self.warn_dt:
            self.get_logger().warn(f"NOTICE: raw |depth-odom|={dt:.3f}s (odom_sync aligned to depth)")

def main():
    rclpy.init()
    node = Adapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
PY
  chmod +x "$ADAPTER_PY"

  start_bg "ego_input_adapter" python3 "$ADAPTER_PY" --ros-args \
    -p use_sim_time:=true \
    -p in_odom:="${ROS_ODOM_TOPIC}" \
    -p in_depth:="${ROS_DEPTH_IMAGE_TOPIC}" \
    -p in_info:="${ROS_DEPTH_INFO_TOPIC}" \
    -p in_cloud:="${ROS_CLOUD_TOPIC}" \
    -p out_depth_a:="/drone_${DRONE_ID}_pcl_render_node/depth" \
    -p out_pose_a:="/drone_${DRONE_ID}_pcl_render_node/camera_pose" \
    -p out_info_a:="/drone_${DRONE_ID}_pcl_render_node/camera_info" \
    -p out_cloud_a:="/drone_${DRONE_ID}_pcl_render_node/cloud" \
    -p out_odom_a:="/drone_${DRONE_ID}_odom_sync" \
    -p out_depth_b:="/pcl_render_node/depth" \
    -p out_pose_b:="/pcl_render_node/camera_pose" \
    -p out_info_b:="/pcl_render_node/camera_info" \
    -p out_cloud_b:="/pcl_render_node/cloud" \
    -p out_odom_b:="/odom_sync"

  # 自检：确保 EGO 会看到 publisher
  timeout 2s ros2 topic echo "/drone_${DRONE_ID}_pcl_render_node/depth" --once >/dev/null || true
  timeout 2s ros2 topic echo "/drone_${DRONE_ID}_pcl_render_node/camera_pose" --once >/dev/null || true
  timeout 2s ros2 topic echo "/drone_${DRONE_ID}_odom_sync" --once >/dev/null || true
fi

### ===== 5) 启动 EGO =====
start_bg "ego_launch" ros2 launch "${EGO_LAUNCH_PKG}" "${EGO_LAUNCH_FILE}" \
  use_sim_time:=true \
  drone_id:="${DRONE_ID}" \
  px4_odom_topic:="/drone_${DRONE_ID}_odom_sync" \
  cloud_topic:="/drone_${DRONE_ID}_cloud" \
  odometry_topic:=odom_sync

sleep 2
log "=== LIVE GRAPH SNAPSHOT ==="
ros2 node list | grep -E "drone_${DRONE_ID}|pcl_render|ego_planner|mapping|grid" || true
ros2 topic list | grep -E "odom_sync|pcl_render_node|depth|camera_info|camera_pose|drone_${DRONE_ID}_camera" || true

for n in $(ros2 node list | grep -E "pcl_render|ego_planner" || true); do
  log "--- node info: $n"
  ros2 node info "$n" | sed -n '/Subscriptions:/,/Publishers:/p' || true
done
log "=== SNAPSHOT END ==="

### ===== 6) TF 快检 =====
if [[ "$CHECK_TF" == "1" ]]; then
  log "Quick TF check (3s): tf2_echo odom -> base_link"
  timeout 3s ros2 run tf2_ros tf2_echo odom base_link "${SIM_ARGS[@]}" || true
  log "Quick TF check (3s): tf2_echo base_link -> camera_link"
  timeout 3s ros2 run tf2_ros tf2_echo base_link camera_link "${SIM_ARGS[@]}" || true
fi

log "All started. Logs: $LOG_DIR"
log "Press Ctrl+C to stop all."
wait