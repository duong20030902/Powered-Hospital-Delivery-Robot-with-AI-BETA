#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import serial
import time
import threading
import math
from tf2_ros import TransformBroadcaster

class STM32Driver(Node):
    def __init__(self):
        super().__init__('stm32_controller')
        
        self.declare_parameter('wheel_radius', 0.029)  # 29mm radius
        self.declare_parameter('wheel_base', 0.24)     # 240mm wheel separation
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        
        # Publishers and Subscribers
        self.velocity_subscriber = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # Odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
           
        self.get_logger().info('STM32 driver initialized')
        
            
    def inverse_kinematics(self, vx, vy, omega):
        """Convert robot velocities to wheel velocities for differential drive"""
        if abs(vy) > 0.01:
            self.get_logger().warn(f'Differential drive cannot move sideways, ignoring vy={vy}')
        
        v_left = (vx - (omega * self.wheel_base / 2.0)) / self.wheel_radius
        v_right = (vx + (omega * self.wheel_base / 2.0)) / self.wheel_radius
        
        return [v_left, v_right]
        
    def forward_kinematics(self, v_left, v_right):
        """Convert wheel velocities to robot velocities for differential drive"""
        vx = (v_left + v_right) * self.wheel_radius / 2.0
        vy = 0.0  # Differential drive cannot move sideways
        omega = (v_right - v_left) * self.wheel_radius / self.wheel_base
        
        return vx, vy, omega
    
    def cmd_vel_callback(self, msg):
        if not hasattr(self, 'serial_port') or not self.serial_port.is_open:
            self.get_logger().warn('Serial port not open, attempting to reconnect')
            if not self.connect_to_serial():
                return
                
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z
        
        wheel_velocities = self.inverse_kinematics(vx, vy, omega)
        
        # Convert to duty cycles
        v_max = 5.973  # Maximum velocity (rad/s)
        duty_max = 60  # Maximum duty cycle
        
        duty_cycles = []
        max_abs_vel = max(abs(vel) for vel in wheel_velocities)
        if max_abs_vel > v_max:
            scale_factor = v_max / max_abs_vel
            wheel_velocities = [vel * scale_factor for vel in wheel_velocities]
        
        for vel in wheel_velocities:
            duty = int((vel / v_max) * duty_max)
            duty = max(-duty_max, min(duty_max, duty))
            duty_cycles.append(duty)
        
        # Send command: "left_duty right_duty"
        command = f"{duty_cycles[0]} {duty_cycles[1]}\n"
        
        try:
            self.serial_port.write(command.encode('utf-8'))
            self.serial_port.flush()
            self.get_logger().info(command)
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to send command: {e}')
            self.connect_to_serial()
    
    
    
    def destroy_node(self):
        """Clean up"""
        self.running = False
        if hasattr(self, 'serial_port') and self.serial_port:
            try:
                self.serial_port.write(b"0 0\n")  # Stop command
                self.serial_port.close()
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = STM32Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()