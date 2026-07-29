#include "ad_localization/localization_node.hpp"

namespace MiniAD {

LocalizationNode::LocalizationNode() : Node("localization_node") {
  publisher_ =
      create_publisher<ad_msgs::msg::EgoPose>("/localization/ego_pose", 10);
  subscription_ = create_subscription<ad_msgs::msg::VehicleState>(
      "/sim/vehicle_state", 10,
      [this](const ad_msgs::msg::VehicleState::SharedPtr message) {
        if (!message) {
          return;
        }
        ad_msgs::msg::EgoPose pose;
        pose.header = message->header;
        pose.x = message->x;
        pose.y = message->y;
        pose.z = 0.0;
        pose.yaw = message->yaw;
        pose.valid = message->valid;
        publisher_->publish(pose);
      });
}

}  // namespace MiniAD

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MiniAD::LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
