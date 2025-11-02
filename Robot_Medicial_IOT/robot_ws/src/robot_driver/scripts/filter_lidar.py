#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import copy

class LidarFilter(Node):
    def __init__(self):
        super().__init__("filter_lidar_node")

        self.sub_lidar_scan = self.create_subscription(
            LaserScan,
            "/scan",
            self.callback_laser,
            10
        )

        self.publish_lidarscan = self.create_publisher(
            LaserScan,
            "/scan_filter",
            10
        )
    
    def callback_laser(self, msg: LaserScan):
        filter_msg = copy.deepcopy(msg)
        filter_msg.ranges = [
            float("inf") if r <= 0.2 else r for r in msg.ranges
        ]

        self.publish_lidarscan.publish(filter_msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = LidarFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
