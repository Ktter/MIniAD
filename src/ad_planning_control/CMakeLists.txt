cmake_minimum_required(VERSION 3.5)
project(ad_planning_control)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ad_msgs REQUIRED)
add_executable(planning_control_node src/planning_control_node.cpp)
target_include_directories(planning_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(planning_control_node rclcpp ad_msgs)
install(TARGETS planning_control_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_package()
