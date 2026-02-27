#!/usr/bin/env bash
set -e

# -----------------------------
# ROS environment
# -----------------------------
source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash

# -----------------------------
# Basic sanity checks
# -----------------------------
echo "[INFO] Python:"
python3 --version

echo "[INFO] Torch:"
python3 - <<'EOF'
import torch
print("Torch version:", torch.__version__)
print("CUDA available:", torch.cuda.is_available())
EOF

# -----------------------------
# Restart tmux session cleanly
# -----------------------------
tmux has-session -t robot 2>/dev/null && tmux kill-session -t robot
tmux new-session -d -s robot

# -----------------------------
# ROS-native node
# -----------------------------
tmux rename-window -t robot:0 'inductive'
tmux send-keys -t robot:0 \
  'exec ros2 run inductive_sensor ime12_serial_node' Enter

# -----------------------------
# ML node (system Python, GPU-enabled)
# -----------------------------
tmux new-window -t robot:1 -n 'threat_detector'
tmux send-keys -t robot:1 \
  "exec python3 \
   \$(ros2 pkg prefix threat_detector)/lib/threat_detector/threat_detection_node" Enter

# -----------------------------
# Attach for interactive use
# -----------------------------
exec tmux attach -t robot