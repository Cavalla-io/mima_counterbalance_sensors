# ==============================
# Base ROS image
# ==============================
FROM ros:jazzy-ros-base

# -----------------------------
# System / ROS dependencies
# -----------------------------
RUN apt-get update && apt-get install -y \
    tmux \
    python3-pip \
    python3-venv \
    python3-setuptools \
    python3-pyyml \
    python3-can \
    python3-serial \
    build-essential \
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
# Split steps for better error visibility
RUN python3 -m venv /opt/threat_venv
RUN /opt/threat_venv/bin/python3 -m pip install --upgrade pip setuptools wheel

# Install Jetson-compatible PyTorch + ML stack
# NOTE: Replace URL/version below with your JetPack version if needed
RUN /opt/threat_venv/bin/pip install --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v51 \
    torch==2.1.0+nv23.05 torchvision==0.16.0+nv23.05 \
    && /opt/threat_venv/bin/pip install --no-cache-dir ultralytics numpy==1.26.4

# Sanity check: venv exists
RUN [ -x /opt/threat_venv/bin/python3 ] || (echo "VENV not created" && exit 1)

# -----------------------------
# ROS workspace
# -----------------------------
WORKDIR /ros_ws
COPY src/ src/

# Build workspace with system Python
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# -----------------------------
# Runtime setup
# -----------------------------
COPY docker/.tmux.conf /root/.tmux.conf
COPY launch.sh /launch.sh
RUN chmod +x /launch.sh
COPY docker/fastrtps.xml /ros_ws/fastrtps.xml

# -----------------------------
# Default entrypoint
# -----------------------------
ENTRYPOINT ["/launch.sh"]