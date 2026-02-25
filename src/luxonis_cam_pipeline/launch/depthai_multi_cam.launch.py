from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='luxonis_cam_pipeline',
            executable='depthai_multi_cam_node',
            name='depthai_multi_cam_node',
            output='screen',
            parameters=[
                # Add any parameters for your node here if needed
                # Example: {'device_ip': '192.168.1.200'}
            ]
        )
    ])