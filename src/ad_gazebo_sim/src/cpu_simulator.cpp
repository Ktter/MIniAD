#include "ad_gazebo_sim/cpu_simulator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

using namespace std::chrono_literals;

namespace MiniAD {

CpuSimulator::CpuSimulator() : Node("cpu_simulator") {
  scenario_ = declare_parameter<std::string>("scenario", "acc");
  vehicle_publisher_ =
      create_publisher<ad_msgs::msg::VehicleState>("/sim/vehicle_state", 10);
  truth_publisher_ = create_publisher<ad_msgs::msg::DetectedObjectArray>(
      "/sim/ground_truth/objects", 10);
  radar_publisher_ = create_publisher<ad_msgs::msg::DetectedObjectArray>(
      "/sim/radar/targets", 10);
  image_publisher_ =
      create_publisher<sensor_msgs::msg::Image>("/sim/camera/front/image", 2);
  lidar_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/sim/lidar/points", 2);
  control_subscription_ = create_subscription<ad_msgs::msg::ControlCommand>(
      "/sim/vehicle/control", 10,
      [this](const ad_msgs::msg::ControlCommand::SharedPtr message) {
        if (message) {
          command_ = *message;
        }
      });
  timer_ = create_wall_timer(50ms, std::bind(&CpuSimulator::Tick, this));
  RCLCPP_INFO(get_logger(), "CPU fallback simulator started, scenario=%s",
              scenario_.c_str());
}

void CpuSimulator::Tick() {
  constexpr double kTimeStep = 0.05;
  const double acceleration =
      command_.emergency_stop ? -3.0
                              : std::clamp(command_.acceleration, -3.0, 2.0);
  speed_ = std::max(0.0, speed_ + acceleration * kTimeStep);
  x_ += speed_ * std::cos(yaw_) * kTimeStep;
  y_ += speed_ * std::sin(yaw_) * kTimeStep;
  yaw_ +=
      std::clamp(command_.steering_angle, -0.5, 0.5) * speed_ * kTimeStep / 2.7;
  elapsed_ += kTimeStep;

  const auto stamp = now();
  PublishVehicleState(stamp, acceleration);
  auto objects = CreateObjects(stamp);
  truth_publisher_->publish(objects);
  radar_publisher_->publish(objects);
  PublishSensors(stamp);
}

void CpuSimulator::PublishVehicleState(const rclcpp::Time &stamp,
                                       double acceleration) {
  ad_msgs::msg::VehicleState vehicle;
  vehicle.header.stamp = stamp;
  vehicle.header.frame_id = "map";
  vehicle.x = x_;
  vehicle.y = y_;
  vehicle.yaw = yaw_;
  vehicle.speed = speed_;
  vehicle.acceleration = acceleration;
  vehicle.steering_angle = command_.steering_angle;
  vehicle.valid = true;
  vehicle_publisher_->publish(vehicle);
}

ad_msgs::msg::DetectedObjectArray CpuSimulator::CreateObjects(
    const rclcpp::Time &stamp) const {
  ad_msgs::msg::DetectedObjectArray objects;
  objects.header.stamp = stamp;
  objects.header.frame_id = "map";

  if (scenario_ == "acc") {
    ad_msgs::msg::DetectedObject lead;
    lead.header = objects.header;
    lead.class_id = ad_msgs::msg::DetectedObject::VEHICLE;
    lead.id = "lead_0";
    lead.x = x_ + 15.0 - std::min(elapsed_ * 0.6, 5.0);
    lead.y = 0.0;
    lead.vx = elapsed_ > 4.0 ? 3.0 : 7.0;
    lead.length = 4.5;
    lead.width = 1.8;
    lead.confidence = 1.0;
    lead.valid = true;
    objects.objects.push_back(lead);
  } else if (scenario_ == "noa") {
    ad_msgs::msg::DetectedObject obstacle;
    obstacle.header = objects.header;
    obstacle.class_id = ad_msgs::msg::DetectedObject::STATIC;
    obstacle.id = "obstacle_0";
    obstacle.x = 25.0;
    obstacle.y = 0.0;
    obstacle.length = 1.0;
    obstacle.width = 1.0;
    obstacle.confidence = 1.0;
    obstacle.valid = true;
    objects.objects.push_back(obstacle);
  } else if (scenario_ == "apa" || scenario_ == "avp") {
    ad_msgs::msg::DetectedObject slot;
    slot.header = objects.header;
    slot.class_id = ad_msgs::msg::DetectedObject::STATIC;
    slot.id = "parking_slot";
    slot.x = 12.0;
    slot.y = 3.0;
    slot.length = 5.0;
    slot.width = 2.4;
    slot.confidence = 1.0;
    slot.valid = true;
    objects.objects.push_back(slot);
  }
  return objects;
}

void CpuSimulator::PublishSensors(const rclcpp::Time &stamp) {
  sensor_msgs::msg::Image image;
  image.header.stamp = stamp;
  image.header.frame_id = "camera_front";
  image.height = 180;
  image.width = 320;
  image.encoding = "rgb8";
  image.step = image.width * 3;
  image.data.assign(image.step * image.height, 20);
  image_publisher_->publish(image);

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = "lidar";
  cloud.height = 1;
  cloud.width = 0;
  cloud.is_dense = true;
  lidar_publisher_->publish(cloud);
}

}  // namespace MiniAD

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MiniAD::CpuSimulator>());
  rclcpp::shutdown();
  return 0;
}
