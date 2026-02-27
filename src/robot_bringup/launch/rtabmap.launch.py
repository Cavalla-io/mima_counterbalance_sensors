from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'subscribe_rgb': True,
                'rgb_topic': '/oak/rgb/image',
                'camera_info_topic': '/oak/rgb/camera_info',
                'depth_topic': '/camera/depth/image_raw',
                'frame_id': 'base_link',
                'subscribe_odom': False,
                'database_path': '~/.ros/rtabmap.db',
                'queue_size': 10,
                'Vis/MinInliers': '12',
            }],
            remappings=[
                ('rgb/image', '/oak/rgb/image'),
                ('rgb/camera_info', '/oak/rgb/camera_info'),
            ],
        ),
        Node(
            package='rtabmap_ros',
            executable='rtabmap_gui',
            name='rtabmap_gui',
            output='screen',
        )
    ])
