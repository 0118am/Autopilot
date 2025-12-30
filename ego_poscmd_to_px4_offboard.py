#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quat_xyzw(x, y, z, w) -> float:
    # ROS ENU yaw about +Z
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class EgoPosCmdToPX4(Node):
    """
    Sub:
      - /drone_0_planning/pos_cmd  (quadrotor_msgs/PositionCommand, ENU)
      - /drone_0_odom              (nav_msgs/Odometry, ENU)  用于没pos_cmd时保持/起飞
    Pub:
      - /fmu/in/offboard_control_mode
      - /fmu/in/trajectory_setpoint  (NED)
      - /fmu/in/vehicle_command      (arm + offboard)
    """

    def __init__(self):
        super().__init__("ego_poscmd_to_px4_offboard")

        self.declare_parameter("pos_cmd_topic", "/drone_0_planning/pos_cmd")
        self.declare_parameter("odom_topic", "/drone_0_odom")

        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("warmup_cycles", 30)   # 先发一段setpoint再切模式
        self.declare_parameter("takeoff_alt_m", 1.5)  # 没pos_cmd前保持在这个高度(ENU z-up)
        self.declare_parameter("min_alt_m", 0.3)

        self.declare_parameter("auto_offboard", True)
        self.declare_parameter("auto_arm", True)

        self.pos_cmd_topic = self.get_parameter("pos_cmd_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.warmup = int(self.get_parameter("warmup_cycles").value)
        self.takeoff_alt = float(self.get_parameter("takeoff_alt_m").value)
        self.min_alt = float(self.get_parameter("min_alt_m").value)
        self.auto_offboard = bool(self.get_parameter("auto_offboard").value)
        self.auto_arm = bool(self.get_parameter("auto_arm").value)

        # PX4 输入通常 best-effort
        qos_px4 = QoSProfile(depth=1)
        qos_px4.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_px4.durability = QoSDurabilityPolicy.VOLATILE

        # EGO pos_cmd 发布是 RELIABLE，你订阅用 RELIABLE 更稳
        qos_pos = QoSProfile(depth=10)
        qos_pos.reliability = QoSReliabilityPolicy.RELIABLE
        qos_pos.durability = QoSDurabilityPolicy.VOLATILE

        qos_odom = QoSProfile(depth=10)
        qos_odom.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_odom.durability = QoSDurabilityPolicy.VOLATILE

        self.pub_offboard = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_px4)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_px4)
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_px4)

        self.sub_pos = self.create_subscription(PositionCommand, self.pos_cmd_topic, self.cb_pos, qos_pos)
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.cb_odom, qos_odom)

        self.latest_pos = None
        self.latest_odom = None

        self.counter = 0
        self.sent_mode = False
        self.sent_arm = False

        self.timer = self.create_timer(1.0 / max(self.rate_hz, 1.0), self.on_timer)

        self.get_logger().info(f"Sub pos_cmd: {self.pos_cmd_topic}")
        self.get_logger().info(f"Sub odom   : {self.odom_topic}")
        self.get_logger().info(f"Pub PX4    : /fmu/in/offboard_control_mode, /fmu/in/trajectory_setpoint, /fmu/in/vehicle_command")
        self.get_logger().info(f"rate={self.rate_hz}Hz warmup={self.warmup} takeoff_alt={self.takeoff_alt}m")

    def now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def cb_pos(self, msg: PositionCommand):
        self.latest_pos = msg

    def cb_odom(self, msg: Odometry):
        self.latest_odom = msg

    def send_vehicle_command(self, command: int, p1: float = 0.0, p2: float = 0.0):
        vc = VehicleCommand()
        vc.timestamp = self.now_us()
        vc.param1 = float(p1)
        vc.param2 = float(p2)
        vc.command = int(command)
        vc.target_system = 1
        vc.target_component = 1
        vc.source_system = 1
        vc.source_component = 1
        vc.from_external = True
        self.pub_cmd.publish(vc)

    def on_timer(self):
        # 没 odom 又没 pos_cmd，就不发（避免乱arm）
        if self.latest_pos is None and self.latest_odom is None:
            return

        t_us = self.now_us()

        # 1) offboard heartbeat（必须持续发）
        hb = OffboardControlMode()
        hb.timestamp = t_us
        hb.position = True
        hb.velocity = False
        hb.acceleration = False
        hb.attitude = False
        hb.body_rate = False
        self.pub_offboard.publish(hb)

        # 2) 选择 ENU 期望（优先用 EGO pos_cmd）
        if self.latest_pos is not None:
            x_enu = float(self.latest_pos.position.x)
            y_enu = float(self.latest_pos.position.y)
            z_enu = float(self.latest_pos.position.z)
            yaw_enu = float(self.latest_pos.yaw)
        else:
            p = self.latest_odom.pose.pose.position
            q = self.latest_odom.pose.pose.orientation
            x_enu = float(p.x)
            y_enu = float(p.y)
            z_enu = float(self.takeoff_alt)
            yaw_enu = yaw_from_quat_xyzw(q.x, q.y, q.z, q.w)

        z_enu = max(z_enu, self.min_alt)

        # 3) ENU -> NED
        # N = y_enu, E = x_enu, D = -z_enu
        sp = TrajectorySetpoint()
        sp.timestamp = t_us
        sp.position = [float(y_enu), float(x_enu), float(-z_enu)]

        # yaw: ENU(0=East, CCW+) -> NED(0=North, CW+)
        sp.yaw = wrap_pi(math.pi * 0.5 - yaw_enu)

        self.pub_sp.publish(sp)

        # 4) warmup 一段 setpoint 后，切 offboard + arm（各发一次）
        if self.counter < self.warmup:
            self.counter += 1
            return

        if self.auto_offboard and not self.sent_mode:
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.sent_mode = True
            self.get_logger().info("Sent DO_SET_MODE: OFFBOARD")

        if self.auto_arm and not self.sent_arm:
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
            self.sent_arm = True
            self.get_logger().info("Sent ARM")


def main():
    rclpy.init()
    node = EgoPosCmdToPX4()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
