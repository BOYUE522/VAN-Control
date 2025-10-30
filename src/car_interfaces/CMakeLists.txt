cmake_minimum_required(VERSION 3.8)
project(car_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(MSG_FILES
  "msg/SurroundingInfoInterface.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
