#include "ad_perception/perception_node.hpp"

#include <algorithm>

namespace MiniAD {

PerceptionNode::PerceptionNode() : Node("perception_node") {
  publisher_ = create_publisher<ad_msgs::msg::DetectedObjectArray>(
      "/perception/objects", 10);
  subscription_ = create_subscription<ad_msgs::msg::DetectedObjectArray>(
      "/sim/ground_truth/objects", 10,
      [this](const ad_msgs::msg::DetectedObjectArray::SharedPtr message) {
        if (!message) {
          return;
        }
        auto output = *message;
        for (auto &object : output.objects) {
          object.confidence = std::clamp(object.confidence, 0.0, 1.0);
        }
        publisher_->publish(output);
      });
}

}  // namespace MiniAD

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MiniAD::PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
