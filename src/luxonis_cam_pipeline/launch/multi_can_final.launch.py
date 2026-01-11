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
    streams = data.get("/**", {}).get("ros__parameters", {}).get("streams", {})
    if not streams:
        print(f"[multi_oak] No 'streams' entries found in {config_path}")
        return []

    nodes = []
    names = streams.get("names", [])
    topics = streams.get("topics", [])
    camera_ips = streams.get("camera_ips", [])
    widths = streams.get("widths", [])
    heights = streams.get("heights", [])
    fps_list = streams.get("fps", [])
    bitrates = streams.get("bitrates", [])

    if not (len(names) == len(topics) == len(camera_ips) == len(widths) == len(heights) == len(fps_list) == len(bitrates)):
        print("[multi_oak] Mismatched list lengths in streams.yaml configuration.")
        return []

    for i, name in enumerate(names):
        ip = camera_ips[i]
        topic_prefix = name
        rgb_width = widths[i]
        rgb_height = heights[i]
        fps = fps_list[i]
        bitrate_kbps = bitrates[i]

        # Default to H.264 compression
        exe = "depthai_multi_cam_node"
        params = {
            "device_ip": ip,
            "topic_prefix": topic_prefix,
            "rgb_width": rgb_width,
            "rgb_height": rgb_height,
            # "fps": fps,
            "bitrate_kbps": bitrate_kbps // 1000,  # Convert to kbps
            "keyframe_interval": 60,
            "high_profile": False,
        }

        nodes.append(Node(
            package="luxonis_cam_pipeline",
            executable=exe,
            name=f"oak_node_{i + 1}",
            namespace=None,
            output="screen",
            parameters=[params],
        ))

    return nodes

def generate_launch_description():
    pkg_share = get_package_share_directory("luxonis_cam_pipeline")
    default_config = os.path.join(pkg_share, "config", "streams.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Path to YAML listing streams with camera settings."
        ),
        OpaqueFunction(function=_spawn_nodes),
    ])
