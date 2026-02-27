FROM nvcr.io/nvidia/l4t-pytorch:r35.2.1-pth2.0-py3

ENV DEBIAN_FRONTEND=noninteractive

# -----------------------------
# Base system
# -----------------------------
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    ca-certificates \
    build-essential \
    python3-pip \
    python3-venv \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# ROS 2 Jazzy repository
# -----------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# -----------------------------
# ROS install
# -----------------------------
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-base \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-depthai-ros \
    python3-yaml \
    python3-can \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# Jetson PyTorch
# -----------------------------
RUN pip3 install --upgrade pip setuptools wheel && \
    pip3 install --no-cache-dir \
      --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v51 \
      torch torchvision ultralytics numpy==1.26.4

# -----------------------------
# ROS workspace
# -----------------------------
WORKDIR /ros_ws
COPY src/ src/

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# -----------------------------
# Runtime
# -----------------------------
COPY docker/.tmux.conf /root/.tmux.conf
COPY launch.sh /launch.sh
RUN chmod +x /launch.sh

ENTRYPOINT ["/launch.sh"]