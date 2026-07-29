#ifndef AD_PLANNING_CONTROL__PLANNING_CONTROL_NODE_HPP_
#define AD_PLANNING_CONTROL__PLANNING_CONTROL_NODE_HPP_

#include <string>

#include "ad_msgs/msg/behavior_state.hpp"
#include "ad_msgs/msg/control_command.hpp"
#include "ad_msgs/msg/detected_object_array.hpp"
#include "ad_msgs/msg/trajectory.hpp"
#include "ad_msgs/msg/vehicle_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace MiniAD {

class PlanningControlNode final : public rclcpp::Node {
 public:
  PlanningControlNode();

 private:
  void Tick();

  ad_msgs::msg::VehicleState vehicle_;
  ad_msgs::msg::DetectedObjectArray objects_;
  std::string state_ = "STANDBY";
  bool autonomous_enabled_ = false;
  bool have_vehicle_ = false;

  rclcpp::Publisher<ad_msgs::msg::Trajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<ad_msgs::msg::ControlCommand>::SharedPtr command_publisher_;
  rclcpp::Publisher<ad_msgs::msg::ControlCommand>::SharedPtr
      simulator_command_publisher_;
  rclcpp::Subscription<ad_msgs::msg::VehicleState>::SharedPtr
      vehicle_subscription_;
  rclcpp::Subscription<ad_msgs::msg::BehaviorState>::SharedPtr
      behavior_subscription_;
  rclcpp::Subscription<ad_msgs::msg::DetectedObjectArray>::SharedPtr
      objects_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace MiniAD

#endif  // AD_PLANNING_CONTROL__PLANNING_CONTROL_NODE_HPP_
