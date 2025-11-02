from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   
    teleop_node = Node(
        package='omni_base_driver',
        executable='teleop.py',
        name='teleop_node',
        output='screen'
            
    ) 
    web_server_node = Node(
        package='omni_base_driver',
        executable='web_control.py',
        name='web_server_node',
        output='screen'
    )
    ld = LaunchDescription()
    ld.add_action(teleop_node)
    ld.add_action(web_server_node)

    return ld
    
