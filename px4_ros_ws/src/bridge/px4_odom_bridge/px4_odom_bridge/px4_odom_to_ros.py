#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.time import Time

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def quat_wxyz_to_rot(w, x, y, z):
    # Hamilton quaternion (w,x,y,z)
    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=float)
    return R


def rot_to_quat_xyzw(R):
    # rotation matrix -> ROS quaternion (x,y,z,w)
    t = float(np.trace(R))
    if t > 0.0:
        S = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    else:
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / S
            x = 0.25 * S
            y = (R[0, 1] + R[1, 0]) / S
            z = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / S
            x = (R[0, 1] + R[1, 0]) / S
            y = 0.25 * S
            z = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w = (R[1, 0] - R[0, 1]) / S
            x = (R[0, 2] + R[2, 0]) / S
            y = (R[1, 2] + R[2, 1]) / S
            z = 0.25 * S

    # normalize
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n > 1e-12:
        x, y, z, w = x/n, y/n, z/n, w/n
    return (x, y, z, w)


class Px4OdomToRos(Node):
    def __init__(self):
        super().__init__('px4_odom_to_ros')

        # topics / frames
        self.declare_parameter('px4_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('odom_topic', '/drone_0_odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # key fixes for EGO
        self.declare_parameter('require_pose_frame_ned', True)
        self.declare_parameter('use_local_origin', True)   # 首帧归零
        self.declare_parameter('z_lift', 0.30)             # 抬高 30cm，避免贴地/入地
        self.declare_parameter('z_min', 0.05)              # 最小高度钳制，避免 z<0
        self.declare_parameter('publish_tf', True)

        # time / twist
        self.declare_parameter('use_px4_timestamp', False)  # 有些环境下用 now() 更稳
        self.declare_parameter('twist_in_world', True)      # 线速度输出在 ENU 世界系

        self.px4_topic = self.get_parameter('px4_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.require_pose_frame_ned = bool(self.get_parameter('require_pose_frame_ned').value)
        self.use_local_origin = bool(self.get_parameter('use_local_origin').value)
        self.z_lift = float(self.get_parameter('z_lift').value)
        self.z_min = float(self.get_parameter('z_min').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.use_px4_timestamp = bool(self.get_parameter('use_px4_timestamp').value)
        self.twist_in_world = bool(self.get_parameter('twist_in_world').value)

        # QoS
        sub_qos = QoSProfile(depth=10,
                             reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             durability=QoSDurabilityPolicy.VOLATILE)
        pub_qos = QoSProfile(depth=10,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.VOLATILE)

        self.pub = self.create_publisher(Odometry, self.odom_topic, pub_qos)
        self.tfbr = TransformBroadcaster(self) if self.publish_tf else None
        self.sub = self.create_subscription(VehicleOdometry, self.px4_topic, self.cb, sub_qos)

        # NED -> ENU
        self.R_enu_ned = np.array([[0, 1, 0],
                                   [1, 0, 0],
                                   [0, 0,-1]], dtype=float)

        # FRD <-> FLU (self-inverse)
        self.R_frd_flu = np.array([[1, 0, 0],
                                   [0,-1, 0],
                                   [0, 0,-1]], dtype=float)

        # local origin
        self._inited = False
        self._p0 = np.zeros(3, dtype=float)
        self._printed_once = False

        self.get_logger().info(f"Sub PX4: {self.px4_topic}")
        self.get_logger().info(f"Pub Odom: {self.odom_topic} ({self.odom_frame}->{self.base_frame})")
        self.get_logger().info(
            f"require_pose_frame_ned={self.require_pose_frame_ned}, "
            f"use_local_origin={self.use_local_origin}, z_lift={self.z_lift}, z_min={self.z_min}, "
            f"use_px4_timestamp={self.use_px4_timestamp}, twist_in_world={self.twist_in_world}"
        )

    def _stamp(self, msg: VehicleOdometry):
        if self.use_px4_timestamp and msg.timestamp_sample != 0:
            # PX4 us -> ROS ns
            return Time(nanoseconds=int(msg.timestamp_sample) * 1000).to_msg()
        return self.get_clock().now().to_msg()

    def cb(self, msg: VehicleOdometry):
        if not self._printed_once:
            self.get_logger().info(f"pose_frame={msg.pose_frame}, velocity_frame={msg.velocity_frame}")
            self._printed_once = True

        if self.require_pose_frame_ned and msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            self.get_logger().warn(f"pose_frame != NED (got {msg.pose_frame}), skip")
            return

        # position NED (N,E,D) -> ENU (E,N,U)
        pn, pe, pd = float(msg.position[0]), float(msg.position[1]), float(msg.position[2])
        if any(math.isnan(v) for v in [pn, pe, pd]):
            return

        x, y, z = pe, pn, -pd

        # local origin: subtract first sample
        if self.use_local_origin:
            p = np.array([x, y, z], dtype=float)
            if not self._inited:
                self._p0 = p.copy()
                self._inited = True
            p = p - self._p0
            x, y, z = float(p[0]), float(p[1]), float(p[2])

        # lift + clamp z to avoid "in ground / in obstacle"
        z = z + self.z_lift
        if self.z_min > -1e9:
            z = max(z, self.z_min)

        # quaternion (w,x,y,z): body(FRD) -> world(NED)
        qw, qx, qy, qz = float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3])
        if any(math.isnan(v) for v in [qw, qx, qy, qz]):
            return
        qn = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        if qn < 1e-12:
            return
        qw, qx, qy, qz = qw/qn, qx/qn, qy/qn, qz/qn

        R_ned_frd = quat_wxyz_to_rot(qw, qx, qy, qz)                 # v_ned = R_ned_frd * v_frd
        R_enu_flu = self.R_enu_ned @ R_ned_frd @ self.R_frd_flu      # v_enu = R_enu_flu * v_flu
        ox, oy, oz, ow = rot_to_quat_xyzw(R_enu_flu)

        # velocities
        vx = vy = vz = 0.0
        wn = wq = wr = 0.0

        # linear velocity: depending on velocity_frame
        v0, v1, v2 = float(msg.velocity[0]), float(msg.velocity[1]), float(msg.velocity[2])
        have_v = not any(math.isnan(v) for v in [v0, v1, v2])
        if have_v:
            if msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_NED:
                v_ned = np.array([v0, v1, v2], dtype=float)
                if self.twist_in_world:
                    v_enu = self.R_enu_ned @ v_ned
                    vx, vy, vz = float(v_enu[0]), float(v_enu[1]), float(v_enu[2])
                else:
                    v_frd = R_ned_frd.T @ v_ned
                    v_flu = self.R_frd_flu @ v_frd
                    vx, vy, vz = float(v_flu[0]), float(v_flu[1]), float(v_flu[2])

            elif msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD:
                v_frd = np.array([v0, v1, v2], dtype=float)
                if self.twist_in_world:
                    v_ned = R_ned_frd @ v_frd
                    v_enu = self.R_enu_ned @ v_ned
                    vx, vy, vz = float(v_enu[0]), float(v_enu[1]), float(v_enu[2])
                else:
                    v_flu = self.R_frd_flu @ v_frd
                    vx, vy, vz = float(v_flu[0]), float(v_flu[1]), float(v_flu[2])

        # angular velocity is body FRD in PX4; convert to FLU for ROS base_link
        p, q, r = float(msg.angular_velocity[0]), float(msg.angular_velocity[1]), float(msg.angular_velocity[2])
        if not any(math.isnan(v) for v in [p, q, r]):
            w_frd = np.array([p, q, r], dtype=float)
            w_flu = self.R_frd_flu @ w_frd
            wn, wq, wr = float(w_flu[0]), float(w_flu[1]), float(w_flu[2])

        stamp = self._stamp(msg)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.x = ox
        odom.pose.pose.orientation.y = oy
        odom.pose.pose.orientation.z = oz
        odom.pose.pose.orientation.w = ow

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        odom.twist.twist.angular.x = wn
        odom.twist.twist.angular.y = wq
        odom.twist.twist.angular.z = wr

        self.pub.publish(odom)

        if self.tfbr is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = z
            tf.transform.rotation.x = ox
            tf.transform.rotation.y = oy
            tf.transform.rotation.z = oz
            tf.transform.rotation.w = ow
            self.tfbr.sendTransform(tf)


def main():
    rclpy.init()
    node = Px4OdomToRos()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
