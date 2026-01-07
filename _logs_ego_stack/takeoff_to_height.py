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
