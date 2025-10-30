cmake_minimum_required(VERSION 3.5)
project(uw)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY
    camera_intrinsics
    camera_settings
    config
    launch
    rviz
    urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/sys_enable_node.py
    scripts/bag_data.bash
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
