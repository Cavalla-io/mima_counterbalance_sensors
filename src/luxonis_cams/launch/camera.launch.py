#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Change IP to your PoE device IP
    camera_ip = "192.168.1.222"

    return LaunchDescription([
        Node(
            package="depthai_ros_driver",
            executable="driver",
            name="depthai_poe_rgb",
            output="screen",
            parameters=[{
                # Connect by IP
                "camera.i_ip": camera_ip,
                # Only enable RGB
                "rgb.i_publish_topic": True,
                "rgb.i_enable_preview": True,
                # Optional: compressed for lower bandwidth (PoE)
                "rgb.i_low_bandwidth": True,
                "rgb.i_low_bandwidth_quality": 50,
                # Disable depth/stereo if not needed
                "stereo.i_publish_topic": False,
                "depth.i_publish_topic": False,
            }],
        ),
    ])

