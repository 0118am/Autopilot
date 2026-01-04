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
EGO_ODOM_ARG="${EGO_ODOM_ARG:-odom}"

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
  log "Waiting for ${topic} (timeout ${t}s) ..."
  timeout "${t}s" bash -lc "ros2 topic echo ${topic} --once >/dev/null" || {
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
if [[ "$CLEAN_OLD" == "1" ]]; then
  log "Cleaning old processes..."
  pkill -f "ros_gz_bridge.*parameter_bridge" 2>/dev/null || true
  pkill -f "px4_odom_to_ros" 2>/dev/null || true
  pkill -f "ego_planner_node" 2>/dev/null || true
  pkill -f "traj_server" 2>/dev/null || true
  pkill -f "pcl_render_node" 2>/dev/null || true
  pkill -f "static_transform_publisher" 2>/dev/null || true
  pkill -f "pose_to_px4_offboard" 2>/dev/null || true
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
wait_topic_once "${ROS_CLOUD_TOPIC}" 10 || true

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
  -p odom_topic:="${ROS_ODOM_TOPIC}" \
  -p odom_frame:=odom \
  -p base_frame:=base_link \
  -p use_sim_time:=true

wait_topic_once "${ROS_ODOM_TOPIC}" 10 || true

log "TF publishers:"
ros2 topic info -v /tf || true
ros2 topic info -v /tf_static || true

### ===== 4.6) EGO 输入适配器：确保 /drone_0_pcl_render_node/* 有数据 =====
if [[ "$ENABLE_EGO_INPUT_ADAPTER" == "1" ]]; then
  ADAPTER_PY="$LOG_DIR/ego_input_adapter.py"
  cat > "$ADAPTER_PY" <<'PY'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

QOS_SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
QOS_DEFAULT = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

class Adapter(Node):
    def __init__(self):
        super().__init__("ego_input_adapter")

        # inputs
        self.in_odom  = self.declare_parameter("in_odom",  "/drone_0_odom").value
        self.in_depth = self.declare_parameter("in_depth", "/drone_0_camera/depth/image_raw").value
        self.in_info  = self.declare_parameter("in_info",  "/drone_0_camera/depth/camera_info").value
        self.in_cloud = self.declare_parameter("in_cloud", "/drone_0_cloud").value

        # outputs (EGO 已经在订阅这些名字)
        self.out_pose  = self.declare_parameter("out_pose",  "/drone_0_pcl_render_node/camera_pose").value
        self.out_depth = self.declare_parameter("out_depth", "/drone_0_pcl_render_node/depth").value
        self.out_info  = self.declare_parameter("out_info",  "/drone_0_pcl_render_node/camera_info").value
        self.out_cloud = self.declare_parameter("out_cloud", "/drone_0_pcl_render_node/cloud").value

        self.pub_pose  = self.create_publisher(PoseStamped,  self.out_pose,  QOS_DEFAULT)
        self.pub_depth = self.create_publisher(Image,        self.out_depth, QOS_SENSOR)
        self.pub_info  = self.create_publisher(CameraInfo,   self.out_info,  QOS_SENSOR)
        self.pub_cloud = self.create_publisher(PointCloud2,  self.out_cloud, QOS_SENSOR)

        self.create_subscription(Odometry,   self.in_odom,  self.cb_odom,  QOS_DEFAULT)
        self.create_subscription(Image,      self.in_depth, self.cb_depth, QOS_SENSOR)
        self.create_subscription(CameraInfo, self.in_info,  self.cb_info,  QOS_SENSOR)
        self.create_subscription(PointCloud2,self.in_cloud, self.cb_cloud, QOS_SENSOR)

        self.get_logger().info(f"[odom ] {self.in_odom}  -> {self.out_pose}")
        self.get_logger().info(f"[depth] {self.in_depth} -> {self.out_depth}")
        self.get_logger().info(f"[info ] {self.in_info}  -> {self.out_info}")
        self.get_logger().info(f"[cloud] {self.in_cloud} -> {self.out_cloud}")

    def cb_odom(self, msg: Odometry):
        ps = PoseStamped()
        ps.header = msg.header          # stamp=sim_time, frame_id=odom
        ps.pose = msg.pose.pose         # base_link pose；你 base->camera 是单位变换时可直接当 camera_pose
        self.pub_pose.publish(ps)

    def cb_depth(self, msg: Image):
        self.pub_depth.publish(msg)

    def cb_info(self, msg: CameraInfo):
        self.pub_info.publish(msg)

    def cb_cloud(self, msg: PointCloud2):
        self.pub_cloud.publish(msg)

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

  # 用你脚本里的 DRONE_ID 自动填参
  start_bg "ego_input_adapter" python3 "$ADAPTER_PY" --ros-args \
    -p use_sim_time:=true \
    -p in_odom:="${ROS_ODOM_TOPIC}" \
    -p in_depth:="${ROS_DEPTH_IMAGE_TOPIC}" \
    -p in_info:="${ROS_DEPTH_INFO_TOPIC}" \
    -p in_cloud:="${ROS_CLOUD_TOPIC}" \
    -p out_pose:="${PCL_CAMERA_POSE_TOPIC}" \
    -p out_depth:="${PCL_DEPTH_TOPIC}" \
    -p out_info:="${PCL_DEPTH_INFO_TOPIC}" \
    -p out_cloud:="${PCL_CLOUD_TOPIC}"

  # 快检：确保这三路不再是“Terminated”
  wait_topic_once "${PCL_CAMERA_POSE_TOPIC}" 10 || true
  wait_topic_once "${PCL_DEPTH_TOPIC}" 10 || true
  wait_topic_once "${PCL_CLOUD_TOPIC}" 10 || true
else
  log "SKIP: ego input adapter disabled (ENABLE_EGO_INPUT_ADAPTER=0)"
fi

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

### ===== 5) 启动 EGO =====
start_bg "ego_launch" ros2 launch "${EGO_LAUNCH_PKG}" "${EGO_LAUNCH_FILE}" \
  use_sim_time:=true \
  flight_type:="${FLIGHT_TYPE}" \
  drone_id:="${DRONE_ID}" \
  cloud_topic:="${ROS_CLOUD_TOPIC}" \
  px4_odom_topic:="${ROS_ODOM_TOPIC}" \
  odometry_topic:="${EGO_ODOM_ARG}" \
  goal_z:=2.0

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
