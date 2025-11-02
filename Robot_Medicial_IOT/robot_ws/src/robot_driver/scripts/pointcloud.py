#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class MinZFinder(Node):
    def __init__(self):
        super().__init__('min_z_finder')

        # Subscribe vào topic PointCloud2
        self.subscription = self.create_subscription(
            PointCloud2,
            '/input_pointcloud',  # Chỉnh lại theo tên topic của bạn
            self.callback,
            10
        )

    def callback(self, msg):
        # Chuyển PointCloud2 message thành danh sách các điểm (x, y, z)
        pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        # Khởi tạo min_z với một giá trị lớn
        min_z = float('inf')
        
        # Duyệt qua các điểm để tìm giá trị min Z
        for point in pc_data:
            z = point[2]  # Lấy giá trị z
            if z < min_z:
                min_z = z

        self.get_logger().info(f'Min Z value: {min_z}')

def main(args=None):
    rclpy.init(args=args)

    min_z_finder = MinZFinder()

    rclpy.spin(min_z_finder)

    min_z_finder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
