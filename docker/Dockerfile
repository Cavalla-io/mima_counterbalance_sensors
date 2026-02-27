FROM ros:jazzy-ros-base

# -----------------------------
# System / ROS dependencies
# -----------------------------
RUN apt-get update && apt-get install -y \
    tmux \
    python3-venv \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-depthai-ros \
    curl \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

# DepthAI deps (external script – keep but be aware it's fragile)
RUN curl -fsSL https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_deps.sh | bash

# -----------------------------
# ML-only virtual environment
# (runtime only)
# -----------------------------
RUN python3 -m venv /opt/threat_venv && \
    /opt/threat_venv/bin/pip install --upgrade pip && \
    /opt/threat_venv/bin/pip install --no-cache-dir \
        torch torchvision ultralytics \
        "numpy==1.26.4"

# -----------------------------
# ROS workspace
# -----------------------------
WORKDIR /ros_ws
COPY src/ src/

# IMPORTANT: build with system Python
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# -----------------------------
# Runtime
# -----------------------------
COPY docker/.tmux.conf /root/.tmux.conf
COPY launch.sh /launch.sh
RUN chmod +x /launch.sh

COPY docker/fastrtps.xml /ros_ws/fastrtps.xml

ENTRYPOINT ["/launch.sh"]