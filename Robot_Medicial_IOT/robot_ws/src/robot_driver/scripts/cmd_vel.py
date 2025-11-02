#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import time
import os
from ament_index_python.packages import get_package_share_directory
import yaml
class Source:
    def __init__(self, name, priority):
        self.name = name
        self.priority = priority
        self.last_msg = None
        self.last_time = 0
    
class CmdVelPriority(Node):
    def __init__(self):
        super().__init__('cmd_vel_priority')
        config_file = os.path.join(
            os.path.dirname(get_package_share_directory('omni_base_driver'), 'config', 'priority_cmd.yaml')
        )
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        self.sources = []

        for src in config['sources']:
            source = Source(src['name'], src['priority'])
            self.sources.append(source)
            self.create_subscription(
                Twist,
                f"/{source.name}/cmd_vel",
                self.create_callback(source),
                10
            )
        
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.time_out = 1.0
    
    def create_callback(self, source):
        def call_back(msg):
            source.last_msg = msg
            source.last_time = time.time()
        return call_back
    
    def timer_callback(self):
        now = time.time()
        active_souce = [s for s in self.sources if now - self.last_time < self.time_out]
        if not active_souce:
            return
        
        best_source = max(active_souce, key=lambda s:s.priority)
        self.cmd_pub.publish(best_source.last_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPriority()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    

if __name__ == "__main__":
    main()
