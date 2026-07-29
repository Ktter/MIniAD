#ifndef AD_LOCALIZATION__LOCALIZATION_NODE_HPP_
#define AD_LOCALIZATION__LOCALIZATION_NODE_HPP_

#include "ad_msgs/msg/ego_pose.hpp"
#include "ad_msgs/msg/vehicle_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace MiniAD {

class LocalizationNode final : public rclcpp::Node {
 public:
  LocalizationNode();

 private:
  rclcpp::Publisher<ad_msgs::msg::EgoPose>::SharedPtr publisher_;
  rclcpp::Subscription<ad_msgs::msg::VehicleState>::SharedPtr subscription_;
};

}  // namespace MiniAD

#endif  // AD_LOCALIZATION__LOCALIZATION_NODE_HPP_
