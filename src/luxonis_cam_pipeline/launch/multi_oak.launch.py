from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
import yaml

def _load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}

def _spawn_nodes(context):
    config_path = LaunchConfiguration("config").perform(context)
    data = _load_yaml(config_path)
    cameras = data.get("cameras", [])
    if not cameras:
        print(f"[multi_oak.launch] No 'cameras' entries found in {config_path}")
        return []

    nodes = []
    for i, cam in enumerate(cameras, start=1):
        ip           = str(cam.get("ip", ""))
        topic_prefix = str(cam.get("topic_prefix", f"oak{i}"))
        rgb_width    = int(cam.get("rgb_width", 1280))
        rgb_height   = int(cam.get("rgb_height", 720))
        mono_height  = int(cam.get("mono_height", 720))
        fps          = float(cam.get("fps", 20.0))
        nodes.append(
            Node(
                package="luxonis_cam_pipeline",
                executable="depthai_multi_cam_node",
                name=f"oak_node_{i}",
                output="screen",
                parameters=[{
                    "device_ip": ip,
                    "topic_prefix": topic_prefix,
                    "rgb_width": rgb_width,
                    "rgb_height": rgb_height,
                    "mono_height": mono_height,
                    "fps": fps,
                }],
            )
        )
    return nodes

def generate_launch_description():
    pkg_share = get_package_share_directory("luxonis_cam_pipeline")
    default_config = os.path.join(pkg_share, "config", "oaks.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Path to YAML listing OAK cameras (IPs and topic prefixes).",
        ),
        OpaqueFunction(function=_spawn_nodes),
    ])
