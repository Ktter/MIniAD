#include "ad_behavior/behavior_node.hpp"

#include <chrono>
#include <functional>
using namespace std::chrono_literals;

namespace MiniAD {

BehaviorNode::BehaviorNode() : Node("behavior_node") {
  scenario_ = declare_parameter<std::string>("scenario", "acc");
  publisher_ =
      create_publisher<ad_msgs::msg::BehaviorState>("/behavior/state", 10);
  vehicle_subscription_ = create_subscription<ad_msgs::msg::VehicleState>(
      "/sim/vehicle_state", 10,
      [this](const ad_msgs::msg::VehicleState::SharedPtr) {
        last_vehicle_ = now();
      });
  objects_subscription_ =
      create_subscription<ad_msgs::msg::DetectedObjectArray>(
          "/perception/objects", 10,
          [this](const ad_msgs::msg::DetectedObjectArray::SharedPtr) {
            last_objects_ = now();
          });
  timer_ = create_wall_timer(100ms, std::bind(&BehaviorNode::Tick, this));
}

void BehaviorNode::Tick() {
  ad_msgs::msg::BehaviorState message;
  message.header.stamp = now();
  message.state = scenario_.empty() ? "STANDBY" : ScenarioToState();
  message.autonomous_enabled = message.state != "EMERGENCY";
  message.reason = "scenario_selected";

  if ((now() - last_vehicle_).seconds() > 1.0 &&
      last_vehicle_.nanoseconds() > 0) {
    message.state = "EMERGENCY";
    message.autonomous_enabled = false;
    message.reason = "vehicle_state_timeout";
  }

  publisher_->publish(message);
}

std::string BehaviorNode::ScenarioToState() const {
  if (scenario_ == "noa") {
    return "NOA";
  }
  if (scenario_ == "apa") {
    return "APA";
  }
  if (scenario_ == "avp") {
    return "AVP";
  }
  return "ACC";
}

}  // namespace MiniAD

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MiniAD::BehaviorNode>());
  rclcpp::shutdown();
  return 0;
}
