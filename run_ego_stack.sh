#!/usr/bin/env bash
set -eo pipefail

### ===== 可配置区 =====
AUTOPILOT_DIR="${AUTOPILOT_DIR:-$HOME/Autopilot}"
EGO_WS="${EGO_WS:-$AUTOPILOT_DIR/ego_swarm_ws}"
PX4_WS="${PX4_WS:-$AUTOPILOT_DIR/px4_ros_ws}"
EGO_ODOMETRY_TOPIC="${EGO_ODOMETRY_TOPIC:-odom}"

DRONE_ID="${DRONE_ID:-0}"
GZ_PC2_TOPIC="${GZ_PC2_TOPIC:-/depth_camera/points}"

ROS_CLOUD_TOPIC="${ROS_CLOUD_TOPIC:-/drone_${DRONE_ID}_cloud}"
ROS_ODOM_TOPIC="${ROS_ODOM_TOPIC:-/drone_${DRONE_ID}_odom}"

EGO_LAUNCH_PKG="${EGO_LAUNCH_PKG:-ego_planner}"
EGO_LAUNCH_FILE="${EGO_LAUNCH_FILE:-external_gz_px4.launch.py}"

CLEAN_OLD="${CLEAN_OLD:-1}"

# 新增：是否等待 /clock
WAIT_CLOCK="${WAIT_CLOCK:-1}"
# 新增：启动后快速自检 TF（timeout 3s）
CHECK_TF="${CHECK_TF:-1}"

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
  kill_if_running "tf_world_odom"
  kill_if_running "tf_base_camera"
  kill_if_running "odom_relay_vslam"
  kill_if_running "odom_relay_odometry"
}
trap cleanup EXIT

### ===== 0) source 环境 + CycloneDDS =====
need_file "/opt/ros/humble/setup.bash"
source /opt/ros/humble/setup.bash

if [[ -f "$EGO_WS/install/setup.bash" ]]; then
  source "$EGO_WS/install/setup.bash"
else
  log "WARN: $EGO_WS/install/setup.bash not found (did you colcon build?)"
fi

if [[ -f "$PX4_WS/install/setup.bash" ]]; then
  source "$PX4_WS/install/setup.bash"
else
  log "WARN: $PX4_WS/install/setup.bash not found (px4_odom_bridge may be elsewhere)"
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0

# 统一 sim time 参数（关键：所有用时间的节点都要带）
SIM_ARGS=(--ros-args -p use_sim_time:=true)

log "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
log "ROS_CLOUD_TOPIC=$ROS_CLOUD_TOPIC"
log "ROS_ODOM_TOPIC=$ROS_ODOM_TOPIC"

### ===== 1) 清理旧进程（可选但建议）=====
if [[ "$CLEAN_OLD" == "1" ]]; then
  log "Cleaning old processes..."
  pkill -f "ros_gz_bridge.*parameter_bridge" 2>/dev/null || true
  pkill -f "px4_odom_to_ros" 2>/dev/null || true
  pkill -f "ego_planner_node" 2>/dev/null || true
  pkill -f "traj_server" 2>/dev/null || true
  pkill -f "pcl_render_node" 2>/dev/null || true
  pkill -f "odom_visualization" 2>/dev/null || true
  pkill -f "static_transform_publisher" 2>/dev/null || true
fi

### ===== 2) Gazebo -> ROS：/clock + 点云桥 =====
# 注意：每个 bridge spec 最好独立成一个参数（加引号更稳）
start_bg "gz_bridge" ros2 run ros_gz_bridge parameter_bridge \
  "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
  "${GZ_PC2_TOPIC}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
  --ros-args \
  -p use_sim_time:=true \
  -r "${GZ_PC2_TOPIC}:=${ROS_CLOUD_TOPIC}"

# 新增：等到 /clock 真来了，再启动 use_sim_time 的节点（强烈建议）
if [[ "$WAIT_CLOCK" == "1" ]]; then
  wait_topic_once /clock 15 || true
fi

### ===== 3) TF（全部加 use_sim_time）=====
start_bg "tf_world_odom" ros2 run tf2_ros static_transform_publisher \
  --frame-id world --child-frame-id odom \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  "${SIM_ARGS[@]}"

start_bg "tf_base_camera" ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 1 base_link camera_link \
  "${SIM_ARGS[@]}"

### ===== 4) PX4 -> ROS odom（务必 use_sim_time）=====
start_bg "px4_odom_to_ros" ros2 run px4_odom_bridge px4_odom_to_ros \
  --ros-args \
  -p odom_topic:="${ROS_ODOM_TOPIC}" \
  -p odom_frame:=odom \
  -p base_frame:=base_link \
  -p use_sim_time:=true

# 等到 odom 先来一帧（避免 EGO/TF 在启动瞬间读空）
wait_topic_once "${ROS_ODOM_TOPIC}" 10 || true
# 检查 TF 是否有人在发（比 tf2_echo 更直接）
log "TF publishers:"
ros2 topic info -v /tf || true
ros2 topic info -v /tf_static || true

### ===== 4.5) 兜底转发（不强制，但保留你的习惯）=====
start_bg "odom_relay_vslam" ros2 run topic_tools relay \
  "${ROS_ODOM_TOPIC}" "/drone_${DRONE_ID}_visual_slam/odom"\
  --ros-args -p use_sim_time:=true

start_bg "odom_relay_odometry" ros2 run topic_tools relay \
  "${ROS_ODOM_TOPIC}" "/odometry"\
  --ros-args -p use_sim_time:=true

### ===== 5) 启动 EGO（use_sim_time:=true 已经传了，OK）=====
start_bg "ego_launch" ros2 launch "${EGO_LAUNCH_PKG}" "${EGO_LAUNCH_FILE}" \
  use_sim_time:=true \
  drone_id:="${DRONE_ID}" \
  cloud_topic:="${ROS_CLOUD_TOPIC}" \
  px4_odom_topic:="${ROS_ODOM_TOPIC}" \
  odometry_topic:="${EGO_ODOMETRY_TOPIC}"

### ===== 6) 可选：快速自检 TF（关键：tf2_echo 也必须 use_sim_time）=====
if [[ "$CHECK_TF" == "1" ]]; then
  log "Quick TF check (3s): tf2_echo odom -> base_link"
  timeout 3s ros2 run tf2_ros tf2_echo odom base_link "${SIM_ARGS[@]}" || true
fi

log "All started. Logs: $LOG_DIR"
log "Press Ctrl+C to stop all."
wait