cmake_minimum_required(VERSION 3.8)
project(threat_detector)

if(CMAKE_COMPILER_ID STREQUAL "GNU")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(threat_detector_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(python-opencv REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the Python linter
  # ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  rclpy
  sensor_msgs
  cv_bridge
  python-opencv
  std_msgs
  rosidl_default_runtime
)

ament_package()
