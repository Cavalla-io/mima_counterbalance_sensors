#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash

tmux new-session -d -s robot

# Launch inductive_sensor in the default ROS Python environment
tmux rename-window -t robot:0 'inductive launch'
tmux send-keys -t robot:0 'ros2 run inductive_sensor ime12_serial_node.py' Enter

# Launch threat_detector using the virtual environment's Python
tmux new-window -t robot:1 -n 'threat_detector launch'
tmux send-keys -t robot:1 '/opt/venv/bin/python3 /ros_ws/install/threat_detector/lib/threat_detector/threat_detection_node.py' Enter

tmux attach -t robot
```

**docker/.tmux.conf:**
```
set -g mouse on
set -g history-limit 10000
set -g default-terminal "screen-256color"
bind | split-window -h
bind - split-window -v
bind r source-file ~/.tmux.conf \; display "Reloaded!"
set -g status-bg black
set -g status-fg white
set -g status-left '#[fg=green]#S '
set -g status-right '#[fg=yellow]%H:%M'