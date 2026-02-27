FROM nvcr.io/nvidia/pytorch:23.09-py3

# Install ROS Jazzy
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y     ros-jazzy-ros-base     tmux     python3-pip     python3-serial     ros-jazzy-nav2-bringup     ros-jazzy-slam-toolbox     ros-jazzy-robot-localization     ros-jazzy-rtabmap-ros     && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    && curl -fsSL https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_deps.sh | bash

RUN apt-get update && apt-get install -y \
    ros-jazzy-depthai-ros \
    && rm -rf /var/lib/apt/lists/*

# Install ultralytics (PyTorch and torchvision are already in the base image)
RUN pip install --no-cache-dir ultralytics "numpy==1.26.4"

COPY docker/.tmux.conf /root/.tmux.conf

# Create workspace
WORKDIR /ros_ws
COPY src/ /ros_ws/src/


# Build workspace
RUN . /opt/ros/jazzy/setup.sh &&     python3 -m colcon build --symlink-install

# Copy launch script
COPY launch.sh /launch.sh
RUN chmod +x /launch.sh

# Check if any tmux session exists
HEALTHCHECK --interval=30s --timeout=10s --retries=3 \
  CMD tmux list-sessions 2>/dev/null || exit 1


ENTRYPOINT ["/launch.sh"]
COPY docker/fastrtps.xml /ros_ws/fastrtps.xml

