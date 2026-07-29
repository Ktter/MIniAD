cmake_minimum_required(VERSION 3.5)
project(ad_gazebo_sim)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ad_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
add_executable(cpu_simulator src/cpu_simulator.cpp)
target_include_directories(cpu_simulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(cpu_simulator rclcpp ad_msgs sensor_msgs)
install(TARGETS cpu_simulator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch worlds models DESTINATION share/${PROJECT_NAME})
ament_package()
