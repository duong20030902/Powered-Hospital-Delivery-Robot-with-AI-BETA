from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    teleop_node = Node(
	package="robot_driver",
	executable="teleop.py",
        name="teleop_node",
	output="screen"
    )
    web_server_node = Node(
	package="robot_driver",
	executable="web_control.py",
	name="web_server_control_node",
	output="screen"
    )
    
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_driver'),
                'launch',
                'bringup_launch.py'
            )
        )
    )

    ld.add_action(bringup_launch)
    ld.add_action(web_server_node)
  
    return ld
