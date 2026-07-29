#include "ad_planning_control/planning_control_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace MiniAD {

PlanningControlNode::PlanningControlNode() : Node("planning_control_node") {
  trajectory_publisher_ =
      create_publisher<ad_msgs::msg::Trajectory>("/planning/trajectory", 10);
  command_publisher_ =
      create_publisher<ad_msgs::msg::ControlCommand>("/control/command", 10);
  simulator_command_publisher_ = create_publisher<ad_msgs::msg::ControlCommand>(
      "/sim/vehicle/control", 10);

  vehicle_subscription_ = create_subscription<ad_msgs::msg::VehicleState>(
      "/sim/vehicle_state", 10,
      [this](const ad_msgs::msg::VehicleState::SharedPtr message) {
        if (message) {
          vehicle_ = *message;
          have_vehicle_ = true;
        }
      });
  behavior_subscription_ = create_subscription<ad_msgs::msg::BehaviorState>(
      "/behavior/state", 10,
      [this](const ad_msgs::msg::BehaviorState::SharedPtr message) {
        if (message) {
          state_ = message->state;
          autonomous_enabled_ = message->autonomous_enabled;
        }
      });
  objects_subscription_ =
      create_subscription<ad_msgs::msg::DetectedObjectArray>(
          "/perception/objects", 10,
          [this](const ad_msgs::msg::DetectedObjectArray::SharedPtr message) {
            if (message) {
              objects_ = *message;
            }
          });
  timer_ =
      create_wall_timer(100ms, std::bind(&PlanningControlNode::Tick, this));
}

void PlanningControlNode::Tick() {
  ad_msgs::msg::ControlCommand command;
  command.header.stamp = now();
  command.emergency_stop = !autonomous_enabled_ || state_ == "EMERGENCY";

  double target_speed = state_ == "APA" || state_ == "AVP" ? 1.5 : 6.0;
  if (state_ == "ACC" && !objects_.objects.empty() && have_vehicle_) {
    const double distance = objects_.objects.front().x - vehicle_.x;
    target_speed = std::clamp((distance - 8.0) * 0.7, 0.0, 8.0);
  }

  command.target_speed = target_speed;
  command.acceleration =
      command.emergency_stop
          ? -3.0
          : std::clamp((target_speed - vehicle_.speed) * 0.8, -2.0, 1.5);
  command.steering_angle = state_ == "APA" ? 0.12 : 0.0;
  command_publisher_->publish(command);
  simulator_command_publisher_->publish(command);

  ad_msgs::msg::Trajectory trajectory;
  trajectory.header.stamp = now();
  trajectory.valid = !command.emergency_stop;
  trajectory.source = "traditional_baseline";
  for (int i = 1; i <= 10; ++i) {
    ad_msgs::msg::TrajectoryPoint point;
    point.x = vehicle_.x + i * target_speed * 0.1;
    point.y = vehicle_.y;
    point.yaw = vehicle_.yaw;
    point.speed = target_speed;
    trajectory.points.push_back(point);
  }
  trajectory_publisher_->publish(trajectory);
}

}  // namespace MiniAD

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MiniAD::PlanningControlNode>());
  rclcpp::shutdown();
  return 0;
}
