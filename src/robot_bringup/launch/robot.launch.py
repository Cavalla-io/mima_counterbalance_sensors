from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get package share directories
    luxonis_cam_pipeline_dir = get_package_share_directory('luxonis_cam_pipeline')
    
    # Launch Luxonis Camera Pipeline
    cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                luxonis_cam_pipeline_dir,
                'launch',
                'multi_oak.launch.py' # Assuming this is the correct launch file
            )
        )
    )

    # Launch Threat Detector Node (using venv)
    threat_detector_node = Node(
        package='threat_detector',
        executable='threat_detection_node',
        name='threat_detector',
        # Use prefix to run this node with the virtual environment's Python
        prefix='/opt/venv/bin/python3',
        output='screen',
    )

    # Launch Inductive Sensor Node
    inductive_sensor_node = Node(
        package='inductive_sensor',
        executable='ime12_serial_node.py', # Assuming this is the correct executable
        name='inductive_sensor',
        output='screen'
    )

    return LaunchDescription([
        cam_launch,
        threat_detector_node,
        inductive_sensor_node,
        # Add other nodes here as needed (e.g., pull_wire_encoder, turn_assist)
    ])
