#ifndef AD_GAZEBO_SIM__CPU_SIMULATOR_HPP_
#define AD_GAZEBO_SIM__CPU_SIMULATOR_HPP_

#include <string>

#include "ad_msgs/msg/control_command.hpp"
#include "ad_msgs/msg/detected_object_array.hpp"
#include "ad_msgs/msg/vehicle_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace MiniAD {

class CpuSimulator final : public rclcpp::Node {
 public:
  CpuSimulator();

 private:
  void Tick();
  void PublishVehicleState(const rclcpp::Time &stamp, double acceleration);
  ad_msgs::msg::DetectedObjectArray CreateObjects(
      const rclcpp::Time &stamp) const;
  void PublishSensors(const rclcpp::Time &stamp);

  std::string scenario_;
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  double speed_ = 0.0;
  double elapsed_ = 0.0;
  ad_msgs::msg::ControlCommand command_;

  rclcpp::Publisher<ad_msgs::msg::VehicleState>::SharedPtr vehicle_publisher_;
  rclcpp::Publisher<ad_msgs::msg::DetectedObjectArray>::SharedPtr
      truth_publisher_;
  rclcpp::Publisher<ad_msgs::msg::DetectedObjectArray>::SharedPtr
      radar_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_publisher_;
  rclcpp::Subscription<ad_msgs::msg::ControlCommand>::SharedPtr
      control_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace MiniAD

#endif  // AD_GAZEBO_SIM__CPU_SIMULATOR_HPP_
