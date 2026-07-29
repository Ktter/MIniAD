cmake_minimum_required(VERSION 3.5)
project(ad_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EgoPose.msg" "msg/VehicleState.msg" "msg/DetectedObject.msg"
  "msg/DetectedObjectArray.msg" "msg/TrajectoryPoint.msg" "msg/Trajectory.msg"
  "msg/ControlCommand.msg" "msg/BehaviorState.msg" "msg/ScenarioStatus.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
