cmake_minimum_required(VERSION 3.5)
project(ad_localization)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ad_msgs REQUIRED)
add_executable(localization_node src/localization_node.cpp)
target_include_directories(localization_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(localization_node rclcpp ad_msgs)
install(TARGETS localization_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_package()
