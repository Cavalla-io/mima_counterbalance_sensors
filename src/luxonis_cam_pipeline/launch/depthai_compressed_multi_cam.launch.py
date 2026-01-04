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
    cams = data.get("cameras", [])
    if not isinstance(cams, list) or not cams:
        print(f"[multi_oak] No 'cameras' entries found in {config_path}")
        return []

    nodes = []
    for i, cam in enumerate(cams, start=1):
        ip            = str(cam.get("ip", ""))
        topic_prefix  = str(cam.get("topic_prefix", f"oak{i}"))
        rgb_width     = int(cam.get("rgb_width", 1280))
        rgb_height    = int(cam.get("rgb_height", 720))
        fps           = float(cam.get("fps", 30.0))
        compression   = str(cam.get("compression", "raw")).lower()
        jpeg_quality  = int(cam.get("jpeg_quality", 90))
        bitrate_kbps  = int(cam.get("bitrate_kbps", 8000))     # for h265
        keyframe_int  = int(cam.get("keyframe_interval", 60))  # for h265
        ns            = str(cam.get("namespace", "")).strip()   # optional

        if compression == "mjpeg":
            exe = "depthai_rgb_mjpeg_node"   # publishes <prefix>/rgb/image/compressed
            params = {
                "device_ip": ip,
                "topic_prefix": topic_prefix,
                "rgb_width": rgb_width,
                "rgb_height": rgb_height,
                "fps": fps,
                "jpeg_quality": jpeg_quality,
            }
        elif compression == "h265":
            exe = "depthai_rgb_h265_node"    # publishes <prefix>/rgb/h265 (CompressedImage format="h265")
            params = {
                "device_ip": ip,
                "topic_prefix": topic_prefix,
                "rgb_width": rgb_width,
                "rgb_height": rgb_height,
                "fps": fps,
                "bitrate_kbps": bitrate_kbps,
                "keyframe_interval": keyframe_int,
            }
        elif compression == "h264":
            exe = "depthai_rgb_h264_node"
            params = {
                "device_ip": ip,
                "topic_prefix": topic_prefix,
                "rgb_width": rgb_width,
                "rgb_height": rgb_height,
                "fps": fps,
                "bitrate_kbps": int(cam.get("bitrate_kbps", 8000)),
                "keyframe_interval": int(cam.get("keyframe_interval", 60)),
                "high_profile": bool(cam.get("high_profile", False)),
            }
        else:
            # default: raw sensor_msgs/Image using your RGB node
            exe = "depthai_multi_cam_node"
            params = {
                "device_ip": ip,
                "topic_prefix": topic_prefix,
                "rgb_width": rgb_width,
                "rgb_height": rgb_height,
                "mono_height": 720,   # kept for compatibility; unused in RGB-only path
                "fps": fps,
            }

        nodes.append(Node(
            package="luxonis_cam_pipeline",
            executable=exe,
            name=f"oak_node_{i}",
            namespace=ns if ns else None,
            output="screen",
            parameters=[params],
        ))

    return nodes

def generate_launch_description():
    pkg_share = get_package_share_directory("luxonis_cam_pipeline")
    default_config = os.path.join(pkg_share, "config", "oaks.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Path to YAML listing cameras with compression settings (raw|mjpeg|h265)."
        ),
        OpaqueFunction(function=_spawn_nodes),
    ])
