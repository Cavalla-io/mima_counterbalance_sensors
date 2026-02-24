FROM ros:jazzy-ros-base

# System dependencies
RUN apt-get update && apt-get install -y \
    tmux \
    python3-pip \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    && rm -rf /var/lib/apt/lists/*
    
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    && curl -fsSL https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_deps.sh | bash

RUN apt-get update && apt-get install -y \
    ros-jazzy-depthai-ros \
    && rm -rf /var/lib/apt/lists/*# # Python packages - Method 1: requirements.txt

RUN pip3 install --no-cache-dir --break-system-packages numpy pyyaml pyserial






# Copy tmux config
COPY docker/.tmux.conf /root/.tmux.conf

# Create workspace
WORKDIR /ros_ws
COPY src/ /ros_ws/src/


# Build workspace
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# Copy launch script
COPY launch.sh /launch.sh
RUN chmod +x /launch.sh

# Check if any tmux session exists
HEALTHCHECK --interval=30s --timeout=10s --retries=3 \
  CMD tmux list-sessions 2>/dev/null || exit 1


ENTRYPOINT ["/launch.sh"]
COPY docker/fastrtps.xml /ros_ws/fastrtps.xml
