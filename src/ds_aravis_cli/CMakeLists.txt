cmake_minimum_required(VERSION 3.5)
project(ds_aravis_cli)
string(REPLACE "_" "::" PROJECT_NAMESPACE ${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-maybe-uninitialized)
endif()

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

FILE(GLOB_RECURSE SOURCE_FILES src/*)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_include_directories(
  ${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/include/aravis-0.8
  /usr/include/glib-2.0
  /usr/lib/x86_64-linux-gnu/glib-2.0/include
)

target_link_libraries(${PROJECT_NAME}
  /usr/lib/x86_64-linux-gnu/libaravis-0.8.so
  /lib/x86_64-linux-gnu/libgobject-2.0.so
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
