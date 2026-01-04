cmake_minimum_required(VERSION 3.8)
project(luxonis_cam_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(depthai_bridge REQUIRED)   # <— add
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)


# DepthAI C++ SDK
find_package(depthai CONFIG REQUIRED)

add_executable(depthai_multi_cam_node src/depthai_multi_cam_node.cpp)
ament_target_dependencies(depthai_multi_cam_node
  rclcpp sensor_msgs camera_info_manager depthai_bridge image_transport cv_bridge
)
target_link_libraries(depthai_multi_cam_node
  depthai::core
  depthai::opencv
  ${OpenCV_LIBS}
)

add_executable(depthai_rgb_mjpeg_node src/depthai_rgb_mjpeg_node.cpp)
ament_target_dependencies(depthai_rgb_mjpeg_node
 rclcpp sensor_msgs 
)
target_link_libraries(depthai_rgb_mjpeg_node 
  depthai::core
)

add_executable(depthai_rgb_h265_node src/depthai_rgb_h265_node.cpp)
ament_target_dependencies(depthai_rgb_h265_node
 rclcpp sensor_msgs
)
target_link_libraries(depthai_rgb_h265_node
 depthai::core
)

add_executable(depthai_rgb_h264_node src/depthai_rgb_h264_node.cpp)
ament_target_dependencies(depthai_rgb_h264_node 
  rclcpp sensor_msgs
)
target_link_libraries(depthai_rgb_h264_node 
  depthai::core
)

target_include_directories(depthai_multi_cam_node PUBLIC ${OpenCV_INCLUDE_DIRS})

install(TARGETS 
  depthai_multi_cam_node
  depthai_rgb_mjpeg_node
  depthai_rgb_h265_node
  depthai_rgb_h264_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python nodes
install(
  PROGRAMS
    src/camera.py
    src/camera_new.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
