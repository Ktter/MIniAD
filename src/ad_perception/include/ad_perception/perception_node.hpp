#ifndef AD_PERCEPTION__PERCEPTION_NODE_HPP_
#define AD_PERCEPTION__PERCEPTION_NODE_HPP_

#include "ad_msgs/msg/detected_object_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace MiniAD {

class PerceptionNode final : public rclcpp::Node {
 public:
  PerceptionNode();

 private:
  rclcpp::Publisher<ad_msgs::msg::DetectedObjectArray>::SharedPtr publisher_;
  rclcpp::Subscription<ad_msgs::msg::DetectedObjectArray>::SharedPtr
      subscription_;
};

}  // namespace MiniAD

#endif  // AD_PERCEPTION__PERCEPTION_NODE_HPP_
