from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='threat_detector',
            executable='threat_detection_node', # This is the name of the console script
            name='threat_detection_node',
            output='screen',
            emulate_tty=True, # Required for output to be visible in terminal
            parameters=[
                # Add any parameters for your node here if needed
            ]
        )
    ])
