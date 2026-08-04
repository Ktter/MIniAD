#ifndef AD_BEHAVIOR__BEHAVIOR_NODE_HPP_
#define AD_BEHAVIOR__BEHAVIOR_NODE_HPP_

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>

#include "ad_msgs/msg/behavior_event.hpp"
#include "ad_msgs/msg/behavior_state.hpp"
#include "ad_msgs/msg/detected_object_array.hpp"
#include "ad_msgs/msg/vehicle_state.hpp"
#include "ad_common.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace MiniAD {

using TransitionKey = std::tuple<
    std::uint8_t,  // current system state
    std::uint8_t,  // current function state
    std::uint8_t>; // requested state

enum class TransitionAction : std::uint8_t {
  kInitToStandby,
  kStandbyToSwitching,
  kSwitchingToStandby,
  kSwitchingToPilotStandby,
  kPilotStandbyToSwitching,
  kPilotStandbyToNoaActive,
  kNoaActiveToPilotFinished,
  kPilotFinishedToPilotStandby,
};

using TransitionTable = std::map<TransitionKey, TransitionAction>;

struct BehaviorContext {
  ad_msgs::msg::VehicleState::SharedPtr vehicle_data;
  ad_msgs::msg::DetectedObjectArray::SharedPtr object_data;
  nav_msgs::msg::Odometry::SharedPtr odometry_data;
};

struct TransitionResult {
  bool accepted{false};
  bool state_changed{false};
  std::string reason{"none"};
};

class BehaviorNode final : public rclcpp::Node {
 public:
  BehaviorNode();

 private:
  void Tick();
  void EventCallback(
      const ad_msgs::msg::BehaviorEvent::SharedPtr event);
  void PublishState(const std::string &reason);

  TransitionResult HandleEvent(
      TransitionAction action,
      const ad_msgs::msg::BehaviorEvent::SharedPtr event);

  TransitionResult InitToStandby(
      const ad_msgs::msg::BehaviorEvent::SharedPtr event);
  TransitionResult StandbyToSwitching(
      const ad_msgs::msg::BehaviorEvent::SharedPtr event);
  TransitionResult SwitchingToStandby(
      const ad_msgs::msg::BehaviorEvent::SharedPtr event);
  TransitionResult SwitchingToPilotStandby(
      const ad_msgs::msg::BehaviorEvent::SharedPtr event);
  TransitionResult PilotStandbyToSwitching(
      const ad_msgs::msg::BehaviorEvent::SharedPtr event);
  TransitionResult PilotStandbyToNoaActive(
      const ad_msgs::msg::BehaviorEvent::SharedPtr event);
  TransitionResult NoaActiveToPilotFinished(
      const ad_msgs::msg::BehaviorEvent::SharedPtr event);
  TransitionResult PilotFinishedToPilotStandby(
      const ad_msgs::msg::BehaviorEvent::SharedPtr event);

  std::string scenario_;
  rclcpp::Time last_vehicle_message_time_;
  bool have_vehicle_ = false;

  rclcpp::Publisher<ad_msgs::msg::BehaviorState>::SharedPtr state_publisher_;
  rclcpp::Subscription<ad_msgs::msg::BehaviorEvent>::SharedPtr event_subscription_;
  rclcpp::Subscription<ad_msgs::msg::VehicleState>::SharedPtr vehicle_subscription_;
  rclcpp::Subscription<ad_msgs::msg::DetectedObjectArray>::SharedPtr object_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<BehaviorContext> context_ =
      std::make_shared<BehaviorContext>();
  std::mutex context_mutex_;
  ad_msgs::msg::BehaviorState::SharedPtr state_ =
      std::make_shared<ad_msgs::msg::BehaviorState>();
  std::mutex state_mutex_;
};

}  // namespace MiniAD

#endif  // AD_BEHAVIOR__BEHAVIOR_NODE_HPP_
