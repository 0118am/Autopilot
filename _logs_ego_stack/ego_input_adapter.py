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
