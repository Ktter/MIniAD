#ifndef AD_BEHAVIOR__BEHAVIOR_NODE_HPP_
#define AD_BEHAVIOR__BEHAVIOR_NODE_HPP_

#include <string>

#include "ad_msgs/msg/behavior_state.hpp"
#include "ad_msgs/msg/detected_object_array.hpp"
#include "ad_msgs/msg/vehicle_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace MiniAD {

class BehaviorNode final : public rclcpp::Node {
 public:
  BehaviorNode();

 private:
  void Tick();
  std::string ScenarioToState() const;

  std::string scenario_;
  rclcpp::Time last_vehicle_;
  rclcpp::Time last_objects_;
  rclcpp::Publisher<ad_msgs::msg::BehaviorState>::SharedPtr publisher_;
  rclcpp::Subscription<ad_msgs::msg::VehicleState>::SharedPtr
      vehicle_subscription_;
  rclcpp::Subscription<ad_msgs::msg::DetectedObjectArray>::SharedPtr
      objects_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace MiniAD

#endif  // AD_BEHAVIOR__BEHAVIOR_NODE_HPP_
