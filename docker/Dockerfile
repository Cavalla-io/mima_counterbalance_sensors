FROM ros:jazzy-ros-base

# System dependencies
RUN apt-get update && apt-get install -y     tmux     python3-pip     python3-venv     ros-jazzy-nav2-bringup     ros-jazzy-slam-toolbox     ros-jazzy-robot-localization     && rm -rf /var/lib/apt/lists/*
    
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    && curl -fsSL https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_deps.sh | bash

RUN apt-get update && apt-get install -y \
    ros-jazzy-depthai-ros \
    && rm -rf /var/lib/apt/lists/*# # Python packages - Method 1: requirements.txt








# Copy tmux config
# Setup Python virtual environment and install dependencies
RUN python3 -m venv /opt/venv --system-site-packages
ENV PATH="/opt/venv/bin:$PATH"
RUN pip install --upgrade pip
RUN pip install --no-cache-dir torch torchvision ultralytics typeguard numpy pyyaml pyserial

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
