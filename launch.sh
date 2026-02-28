#!/usr/bin/env bash
set -e

# ROS environment
source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash

# Sanity check
if [ ! -x /opt/threat_venv/bin/python3 ]; then
  echo "ERROR: /opt/threat_venv not found"
  exit 1
fi

# Restart tmux session cleanly
tmux has-session -t robot 2>/dev/null && tmux kill-session -t robot
tmux new-session -d -s robot

# ROS-native node
tmux rename-window -t robot:0 'inductive'
tmux send-keys -t robot:0 \
  'exec ros2 run inductive_sensor ime12_serial_node' Enter

# ML node
tmux new-window -t robot:1 -n 'threat_detector'
tmux send-keys -t robot:1 \
  "exec /opt/threat_venv/bin/python3 \
   \$(ros2 pkg prefix threat_detector)/lib/threat_detector/threat_detection_node" Enter

# Attach for interactive use
exec tmux attach -t robot