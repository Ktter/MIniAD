#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "ad_msgs/msg/vehicle_state.hpp"
#include "nav_msgs/msg/odometry.hpp"

class GazeboStateAdapter final : public rclcpp::Node {
public:
  GazeboStateAdapter() : Node("gazebo_state_adapter") {
    publisher_ = create_publisher<ad_msgs::msg::VehicleState>("/sim/vehicle_state", 10);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/sim/vehicle/odom", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        if (!message) return;
        const auto &q = message->pose.pose.orientation;
        const double sin_yaw = 2.0 * (q.w * q.z + q.x * q.y);
        const double cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        ad_msgs::msg::VehicleState state;
        state.header = message->header;
        state.x = message->pose.pose.position.x;
        state.y = message->pose.pose.position.y;
        state.yaw = std::atan2(sin_yaw, cos_yaw);
        state.speed = std::hypot(message->twist.twist.linear.x, message->twist.twist.linear.y);
        state.acceleration = 0.0;
        state.steering_angle = 0.0;
        state.valid = true;
        publisher_->publish(state);
      });
  }
private:
  rclcpp::Publisher<ad_msgs::msg::VehicleState>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboStateAdapter>());
  rclcpp::shutdown();
  return 0;
}
