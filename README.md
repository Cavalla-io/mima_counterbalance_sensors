###Unified Launch Steps for ROS + ML Node


###Step 1 — (bare-metal only) Create ML virtual environment

###Skip in Docker — already created in /opt/threat_venv.

python3 -m venv ~/threat_venv
source ~/threat_venv/bin/activate
pip install --upgrade pip
pip install torch torchvision ultralytics numpy==1.26.4
deactivate

###Bare-metal path: ~/threat_venv
###Docker path: /opt/threat_venv (already exists)


###Step 2 — Build ROS workspace

cd /ros_ws
colcon build --symlink-install

###This generates install/setup.bash used by launch.sh
###Run once after workspace changes

###Step 3 — Ensure launch.sh is executable

chmod +x /ros_ws/launch.sh  # or wherever your launch.sh lives

###Step 4 — Launch everything via launch.sh
./launch.sh

###ROS-native nodes run in system Python
###ML node runs in venv (threat_venv)
###tmux opens two windows: one per node
###You can detach/attach freely

###Step 5 — Optional checks / troubleshooting
# Check venv exists
ls ~/threat_venv/bin/python3        # bare-metal
ls /opt/threat_venv/bin/python3    # Docker

# Activate venv manually to test ML node
source ~/threat_venv/bin/activate
python -m threat_detector.threat_detection_node
deactivate

# Rebuild ROS workspace if you change packages
cd /ros_ws
colcon build --symlink-install