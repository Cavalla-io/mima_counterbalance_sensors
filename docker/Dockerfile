# ==============================
# Base image: NVIDIA Jetson PyTorch (GPU-enabled)
# ==============================
FROM nvcr.io/nvidia/l4t-pytorch:r35.1-pth2.1-py3

# -----------------------------
# System / ROS dependencies
# -----------------------------
RUN apt-get update && apt-get install -y \
    tmux \
    python3-pip \
    build-essential \
    python3-yaml \
    python3-can \
    python3-serial \
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
# ML dependencies (system Python)
# -----------------------------
# Torch and torchvision already installed with GPU support in base image
# Only install additional Python packages needed (Ultralytics, Numpy)
RUN python3 -m pip install --upgrade pip setuptools wheel && \
    python3 -m pip install --no-cache-dir ultralytics numpy==1.26.4

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