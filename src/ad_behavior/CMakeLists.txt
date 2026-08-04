cmake_minimum_required(VERSION 3.8)
project(ad_behavior)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ad_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(behavior_node
  src/behavior_node.cpp
)

target_include_directories(behavior_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../ad_common/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(behavior_node
  rclcpp
  ad_msgs
  nav_msgs
)

install(
  TARGETS behavior_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
