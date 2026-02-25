cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)
find_package(launch_ros REQUIRED)

# Launch files are installed via setup.py for ament_python packages

ament_package()