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
