#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "ad_msgs/msg/control_command.hpp"
#include "geometry_msgs/msg/twist.hpp"

class GazeboControlAdapter final : public rclcpp::Node {
public:
  GazeboControlAdapter() : Node("gazebo_control_adapter") {
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("/sim/vehicle/cmd_vel", 10);
    subscription_ = create_subscription<ad_msgs::msg::ControlCommand>(
      "/sim/vehicle/control", 10,
      [this](const ad_msgs::msg::ControlCommand::SharedPtr message) {
        if (!message) return;
        geometry_msgs::msg::Twist twist;
        twist.linear.x = message->emergency_stop ? 0.0 : std::clamp(message->target_speed, -2.0, 10.0);
        twist.angular.z = message->emergency_stop ? 0.0 : std::clamp(message->steering_angle, -0.6, 0.6);
        publisher_->publish(twist);
      });
  }
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<ad_msgs::msg::ControlCommand>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboControlAdapter>());
  rclcpp::shutdown();
  return 0;
}
