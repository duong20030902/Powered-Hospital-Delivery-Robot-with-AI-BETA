#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import json
import os
from ament_index_python.packages import get_package_share_directory
save_file = os.path.join(get_package_share_directory('omni_base_driver'), 'pose_json', 'last_pose.json')

class PoseLoader(Node):
    def __init__(self):
        super().__init__('pose_loader')
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10)
        self.load_pose()

    def load_pose(self):
        pose_data = None
        try:
            if os.path.exists(save_file):
                with open(save_file, 'r') as f:
                    pose_data = json.load(f)
                    self.set_initial_pose(pose_data)
            else:
                self.get_logger().warning(f'Pose file not found: {save_file}')
                self.set_default_pose()
        except (json.JSONDecodeError, KeyError, Exception) as e:
            self.get_logger().warning(f'Error loading pose: {e}')
            self.set_default_pose()

    def set_default_pose(self):
        """Set default pose at origin (0, 0, 0)"""
        default_pose_data = {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            'covariance': [0.0] * 36
        }
        self.set_initial_pose(default_pose_data)
        self.get_logger().info('Set default initial pose at origin (0, 0, 0)')

    def set_initial_pose(self, pose_data):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = pose_data['position']['x']
        msg.pose.pose.position.y = pose_data['position']['y']
        msg.pose.pose.position.z = pose_data['position']['z']
        msg.pose.pose.orientation.x = pose_data['orientation']['x']
        msg.pose.pose.orientation.y = pose_data['orientation']['y']
        msg.pose.pose.orientation.z = pose_data['orientation']['z']
        msg.pose.pose.orientation.w = pose_data['orientation']['w']
        msg.pose.covariance = pose_data['covariance']
        self.publisher.publish(msg)
        self.get_logger().info(f'Initial pose set: ({pose_data["position"]["x"]}, {pose_data["position"]["y"]}, {pose_data["position"]["z"]})')

def main(args=None):
    rclpy.init(args=args)
    pose_loader = PoseLoader()
    rclpy.spin(pose_loader)
    pose_loader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()