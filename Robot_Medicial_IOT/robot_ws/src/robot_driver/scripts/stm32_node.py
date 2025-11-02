#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu  # Add IMU import
import serial
import time
import threading
import math
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

class STM32Driver(Node):
    def __init__(self):
        super().__init__('stm32_driver')
        
        # Parameters
        self.declare_parameter('port', '/dev/stm32_uart')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius', 0.029)  # 29mm radius
        self.declare_parameter('wheel_base', 0.24)     # 240mm wheel separation
        self.declare_parameter('debug_mode', True)    # Enable debug logging
        
        self.port_name = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # Publishers - IMU và Odometry hoàn toàn riêng biệt
        self.cmd_vel_subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)
        
        # TF broadcaster cho odometry
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Odometry variables - chỉ dựa trên encoder
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # Initialize counter for debug logging
        self.count = 0
        
        # Data monitoring timeout
        self.data_timeout = 2.0  # seconds
        
        # Serial connection
        self.serial_port = None
        self.connect_to_serial()
        
        # Start reading thread
        self.running = True
        self.reader_thread = threading.Thread(target=self.read_data)
        self.reader_thread.daemon = True
        self.reader_thread.start()
        
        # Data monitoring for TIM10 interrupt
        self.create_timer(1.0, self.monitor_data_flow)  # Check data flow every second
        self.last_data_time = time.time()
        self.data_count = 0
        
        self.get_logger().info('STM32 driver initialized - separate IMU and Odometry')
        
    def connect_to_serial(self):
        try:
            self.serial_port = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                timeout=3.0
            )
            time.sleep(1)
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            self.get_logger().info(f'Connected to STM32 on {self.port_name}')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to STM32: {e}')
            return False
            
    def inverse_kinematics(self, vx, vy, omega):
        """Convert robot velocities to wheel velocities for differential drive"""
        if abs(vy) > 0.01:
            self.get_logger().warn(f'Differential drive cannot move sideways, ignoring vy={vy}')
        
        v_left = (vx - (omega * self.wheel_base / 2.0)) / self.wheel_radius
        v_right = (vx + (omega * self.wheel_base / 2.0)) / self.wheel_radius
        
        return [v_left, v_right]
        
    def forward_kinematics(self, v_left, v_right):
        """Convert wheel velocities to robot velocities for differential drive"""
        vx = (v_left + v_right)/ 2.0
        vy = 0.0  # Differential drive cannot move sideways
        omega = (v_right - v_left) / self.wheel_base
        
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
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to send command: {e}')
            self.connect_to_serial()
    
    def read_data(self):
        """Read data from STM32 - process IMU and odometry separately"""
        consecutive_errors = 0
        max_errors = 5
        
        while self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    self.count = self.count + 1
                    if line:
                        try:
                            # Parse data: "v_left v_right ax ay az gx gy gz roll pitch yaw"
                            # Note: v_left is already negated in STM32 code
                            data = line.split()
                            if len(data) == 11:  # Changed from 8 to 11
                                v_left, v_right, ax, ay, az, gx, gy, gz, roll, pitch, yaw = map(float, data)
                                
                                # Debug log if enabled
                                if self.debug_mode:
                                    if self.count%20 == 0:
                                        self.get_logger().info(f'Raw data: v_left={v_left:.3f}, v_right={v_right:.3f}')
                                        self.get_logger().info(f'RPY: roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}')
                                
                                # Process odometry data separately (no IMU involvement)
                                self.process_odometry_data(v_left, v_right)
                                
                                # Process IMU data separately with orientation
                                self.process_imu_data(ax, ay, az, gx, gy, gz, roll, pitch, yaw)
                                
                                # Update monitoring
                                self.last_data_time = time.time()
                                self.data_count += 1
                            elif len(data) == 8:  # Fallback for old format without RPY
                                v_left, v_right, ax, ay, az, gx, gy, gz = map(float, data)
                                
                                if self.debug_mode:
                                    if self.count%20 == 0:
                                        self.get_logger().info(f'Raw data: v_left={v_left:.3f}, v_right={v_right:.3f}')
                                
                                self.process_odometry_data(v_left, v_right)
                                self.process_imu_data(ax, ay, az, gx, gy, gz)
                                
                                self.last_data_time = time.time()
                                self.data_count += 1
                            else:
                                self.get_logger().warn(f'Warning: Unexpected data format: {data}')
                                
                            consecutive_errors = 0
                        except ValueError as e:
                            self.get_logger().warn(f'Invalid data format: {line} - {e}')
                
                time.sleep(0.01)
                
            except serial.SerialException as e:
                consecutive_errors += 1
                self.get_logger().error(f'Serial read error: {e}')
                if consecutive_errors >= max_errors:
                    self.get_logger().warn('Too many consecutive errors, attempting to reconnect')
                    self.connect_to_serial()
                    consecutive_errors = 0
                time.sleep(1.0)
    
    def process_odometry_data(self, v_left, v_right):
        """Process odometry data only - no IMU involvement"""
      
        vx, vy, omega = self.forward_kinematics(v_left, v_right)
        
        # Update odometry (pure encoder-based)
        self.update_odometry(vx, vy, omega)
    
    def process_imu_data(self, ax, ay, az, gx, gy, gz, roll=None, pitch=None, yaw=None):
        """Process IMU data with optional orientation"""
        # Publish raw IMU data (completely independent)
        self.publish_imu(ax, ay, az, gx, gy, gz, roll, pitch, yaw)
    

    def publish_imu(self, ax, ay, az, gx, gy, gz, roll=None, pitch=None, yaw=None):
        """Publish IMU data with orientation if available"""
        imu_msg = Imu()
        
        # Header
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu'  # Use IMU frame from URDF
        
        # Linear acceleration (m/s^2)
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        
        # Angular velocity (rad/s)
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz
        
        # Orientation - use provided RPY if available
        if roll is not None and pitch is not None and yaw is not None:
            # Convert RPY to quaternion
            quaternion = quaternion_from_euler(roll, pitch, yaw, 'sxyz')
            imu_msg.orientation.w = quaternion[0]
            imu_msg.orientation.x = quaternion[1]
            imu_msg.orientation.y = quaternion[2]
            imu_msg.orientation.z = quaternion[3]
            
            # Lower orientation covariance since we have actual orientation data
            imu_msg.orientation_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]
        else:
            # No orientation data available
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0
            imu_msg.orientation.w = 1.0
            
            # High orientation covariance since we don't provide orientation
            imu_msg.orientation_covariance = [
                99999.0, 0.0, 0.0,
                0.0, 99999.0, 0.0,
                0.0, 0.0, 99999.0
            ]
        
        # Covariance matrices for acceleration and angular velocity
        imu_msg.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        
        imu_msg.angular_velocity_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        ]
        
        self.imu_publisher.publish(imu_msg)
    
   
    
    def update_odometry(self, vx, vy, omega):
        """Update and publish odometry - pure encoder-based, no IMU"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        # self.get_logger().info(f"dt: {dt}")
        
        # Pure wheel odometry - no IMU sensor fusion
        # Use current theta for position calculation, not omega
        delta_x = vx * math.cos(self.theta) * dt
        delta_y = vx * math.sin(self.theta) * dt
        delta_theta = omega  * dt
        
        if self.count % 20 == 0:
            self.get_logger().info(f"delta theta: {delta_theta}, current theta: {self.theta}")
            
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta
        
        if self.theta > math.pi:
            self.theta -= 2*math.pi
        elif self.theta < -math.pi:
            self.theta += 2*math.pi
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        quaternion = quaternion_from_euler(0, 0, self.theta, 'sxyz')
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (pure encoder-based)
        odom.pose.pose.orientation.w = quaternion[0]
        odom.pose.pose.orientation.x = quaternion[1]
        odom.pose.pose.orientation.y = quaternion[2]
        odom.pose.pose.orientation.z = quaternion[3]
        
        # Velocity
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0  # Differential drive
        odom.twist.twist.angular.z = omega
        
        # Covariance - encoder uncertainty only
        pose_cov = [0.1] * 36
        pose_cov[35] = 0.2  # Higher yaw uncertainty without IMU
        odom.pose.covariance = pose_cov
        
        odom.twist.covariance = [0.1] * 36
        
        self.odom_publisher.publish(odom)
        
        # Publish TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
            
        self.last_time = current_time
    
    def monitor_data_flow(self):
        """Monitor data flow"""
        current_time = time.time()
        time_since_data = current_time - self.last_data_time
        
        if time_since_data > self.data_timeout:
            self.get_logger().warn(f'No data received for {time_since_data:.1f}s - checking STM32 connection')
            if hasattr(self, 'serial_port') and self.serial_port:
                if not self.serial_port.is_open:
                    self.connect_to_serial()
        
        # Log data rate every 10 seconds
        if hasattr(self, '_last_rate_log'):
            if current_time - self._last_rate_log > 10.0:
                rate = self.data_count / 10.0
                self.get_logger().info(f'Data rate: {rate:.1f} Hz (IMU and Odometry separate)')
                self.data_count = 0
                self._last_rate_log = current_time
        else:
            self._last_rate_log = current_time
    
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