cmake_minimum_required(VERSION 3.5)
project(oxts_ntrip)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/GgaGenerator.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<INSTALL_INTERFACE:include>
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "${PROJECT_NAME}::GgaGenerator"
  EXECUTABLE ${PROJECT_NAME}_gga
)

install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_gga
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/ntrip_forwarding.py
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
