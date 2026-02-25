#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash

tmux new-session -d -s robot

# Launch the entire robot system using the robot_bringup launch file
tmux rename-window -t robot:0 'robot bringup'
tmux send-keys -t robot:0 'ros2 launch robot_bringup robot.launch.py' Enter

tmux attach -t robot