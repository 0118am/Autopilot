import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

class ScanToCloud(Node):
    def __init__(self):
        super().__init__('scan_to_cloud')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cloud_topic', '/cloud')
        self.declare_parameter('frame_id', 'lidar')
        self.declare_parameter('use_sim_time', True)

        scan_topic = self.get_parameter('scan_topic').value
        cloud_topic = self.get_parameter('cloud_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub = self.create_publisher(PointCloud2, cloud_topic, 10)
        self.sub = self.create_subscription(LaserScan, scan_topic, self.cb, 10)

        self.get_logger().info(f"Sub: {scan_topic}  Pub: {cloud_topic}  frame_id: {self.frame_id}")

    def cb(self, msg: LaserScan):
        pts = []
        a = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                pts.append((r * math.cos(a), r * math.sin(a), 0.0))
            a += msg.angle_increment

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.frame_id
        cloud = point_cloud2.create_cloud_xyz32(header, pts)
        self.pub.publish(cloud)

def main():
    rclpy.init()
    node = ScanToCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
