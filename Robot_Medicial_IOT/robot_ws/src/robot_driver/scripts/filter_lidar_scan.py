#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarFilterNode(Node):
    def __init__(self):
        super().__init__("lidar_filter_node")

        # Subscribe to /rplidar_scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/rplidar_scan',  # Input topic
            self.lidar_scan_callback,
            10
        )
        
        # Angle ranges in [-π, π] format
        self.angle_ranges = [
            (0.0, 0.576),           # 0° to 33°
            (0.698, 2.094),         # 40° to 120°  
            (2.217, -2.356),        # 127° to 224° (crosses -π boundary)
            (-2.234, -0.681),       # 231° to 321°
            (-0.559, 0.0)           # 328° to 360°
        ]
        
        # Publish to /scan topic
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

    def normalize_angle(self, angle):
        """Normalize angle to [-π, π] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def is_angle_in_ranges(self, angle):
        """Check if angle is within any of the valid ranges"""
        angle = self.normalize_angle(angle)
        
        for start, end in self.angle_ranges:
            if start <= end:
                # Normal range (doesn't cross boundary)
                if start <= angle <= end:
                    return True
            else:
                # Range crosses -π/π boundary (e.g., 127° to 224°)
                if angle >= start or angle <= end:
                    return True
        return False

    def lidar_scan_callback(self, msg: LaserScan):
        filter_msg = LaserScan()
        filter_msg.header = msg.header
        filter_msg.angle_min = msg.angle_min
        filter_msg.angle_max = msg.angle_max
        filter_msg.angle_increment = msg.angle_increment
        filter_msg.time_increment = msg.time_increment
        filter_msg.scan_time = msg.scan_time
        filter_msg.range_min = msg.range_min
        filter_msg.range_max = msg.range_max

        filter_msg.ranges = []

        for i in range(len(msg.ranges)):
            angle = msg.angle_min + i * msg.angle_increment
            
            if self.is_angle_in_ranges(angle):
                filter_msg.ranges.append(float(msg.ranges[i]))
            else:
                filter_msg.ranges.append(float('nan'))

        self.publisher.publish(filter_msg)



def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create the LidarFilterNode
    node = LidarFilterNode()

    # Spin the node to keep it alive
    rclpy.spin(node)

    # Destroy the node and shut down ROS
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
