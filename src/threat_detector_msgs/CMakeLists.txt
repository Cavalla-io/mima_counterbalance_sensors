cmake_minimum_required(VERSION 3.8)
project(threat_detector_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED) # Add std_msgs dependency for Header

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ThreatAlert.msg"
  DEPENDENCIES std_msgs # Explicitly declare dependency on std_msgs
)

ament_export_dependencies(rosidl_default_runtime std_msgs) # Export std_msgs
ament_package()