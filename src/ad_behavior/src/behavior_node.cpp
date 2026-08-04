#include "ad_behavior/behavior_node.hpp"

#include <chrono>
#include <utility>

using namespace std::chrono_literals;

namespace MiniAD {
namespace {

TransitionKey MakeKey(SystemStateMachine system_state,
                      std::uint8_t function_state,
                      std::uint8_t requested_state) {
  return {
      static_cast<std::uint8_t>(system_state),
      function_state,
      requested_state};
}

const TransitionTable kTransitionTable = {
    {MakeKey(SystemStateMachine::INIT,
             static_cast<std::uint8_t>(SystemStateMachine::INIT),
             static_cast<std::uint8_t>(SystemStateMachine::STANDBY)),
     TransitionAction::kInitToStandby},

    {MakeKey(SystemStateMachine::STANDBY,
             static_cast<std::uint8_t>(SystemStateMachine::STANDBY),
             static_cast<std::uint8_t>(SystemStateMachine::SWITCHING)),
     TransitionAction::kStandbyToSwitching},

    {MakeKey(SystemStateMachine::SWITCHING,
             static_cast<std::uint8_t>(SystemStateMachine::SWITCHING),
             static_cast<std::uint8_t>(SystemStateMachine::STANDBY)),
     TransitionAction::kSwitchingToStandby},
    {MakeKey(SystemStateMachine::SWITCHING,
             static_cast<std::uint8_t>(SystemStateMachine::SWITCHING),
             static_cast<std::uint8_t>(SystemStateMachine::PILOT)),
     TransitionAction::kSwitchingToPilotStandby},

    {MakeKey(SystemStateMachine::PILOT,
             static_cast<std::uint8_t>(DrivingStateMachine::STANDBY),
             static_cast<std::uint8_t>(SystemStateMachine::SWITCHING)),
     TransitionAction::kPilotStandbyToSwitching},
    {MakeKey(SystemStateMachine::PILOT,
             static_cast<std::uint8_t>(DrivingStateMachine::STANDBY),
             static_cast<std::uint8_t>(DrivingStateMachine::NOA_ACTIVE)),
     TransitionAction::kPilotStandbyToNoaActive},
    {MakeKey(SystemStateMachine::PILOT,
             static_cast<std::uint8_t>(DrivingStateMachine::NOA_ACTIVE),
             static_cast<std::uint8_t>(DrivingStateMachine::PILOT_FINISHED)),
     TransitionAction::kNoaActiveToPilotFinished},
    {MakeKey(SystemStateMachine::PILOT,
             static_cast<std::uint8_t>(DrivingStateMachine::PILOT_FINISHED),
             static_cast<std::uint8_t>(DrivingStateMachine::STANDBY)),
     TransitionAction::kPilotFinishedToPilotStandby},
};

}  // namespace

BehaviorNode::BehaviorNode() : Node("behavior_node") {
  scenario_ = declare_parameter<std::string>("scenario", "acc");
  state_publisher_ =
      create_publisher<ad_msgs::msg::BehaviorState>("/behavior/state", 10);

  state_->vehicle_state =
      static_cast<std::uint8_t>(SystemStateMachine::INIT);
  state_->function_state =
      static_cast<std::uint8_t>(SystemStateMachine::INIT);
  state_->header.stamp = now();

  vehicle_subscription_ = create_subscription<ad_msgs::msg::VehicleState>(
      "/sim/vehicle_state", 10,
      [this](const ad_msgs::msg::VehicleState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(context_mutex_);
        context_->vehicle_data = message;
        last_vehicle_message_time_ = now();
        have_vehicle_ = true;
      });

  object_subscription_ =
      create_subscription<ad_msgs::msg::DetectedObjectArray>(
          "/perception/objects", 10,
          [this](const ad_msgs::msg::DetectedObjectArray::SharedPtr message) {
            std::lock_guard<std::mutex> lock(context_mutex_);
            context_->object_data = message;
          });

  odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/sim/odometry", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        std::lock_guard<std::mutex> lock(context_mutex_);
        context_->odometry_data = message;
      });

  event_subscription_ = create_subscription<ad_msgs::msg::BehaviorEvent>(
      "/behavior/event", 10,
      std::bind(&BehaviorNode::EventCallback, this, std::placeholders::_1));

  timer_ = create_wall_timer(100ms, std::bind(&BehaviorNode::Tick, this));
  PublishState("initialized");
}

void BehaviorNode::EventCallback(
    const ad_msgs::msg::BehaviorEvent::SharedPtr event) {
  if (!event) {
    RCLCPP_WARN(get_logger(), "Ignoring null behavior event");
    return;
  }

  TransitionKey key;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    key = {
        state_->vehicle_state,
        state_->function_state,
        event->request_state};
  }

  const auto iter = kTransitionTable.find(key);
  if (iter == kTransitionTable.end()) {
    RCLCPP_WARN(
        get_logger(),
        "No transition for system=%u function=%u requested_state=%u",
        std::get<0>(key), std::get<1>(key), std::get<2>(key));
    return;
  }

  const TransitionResult result = HandleEvent(iter->second, event);
  if (!result.accepted || !result.state_changed) {
    RCLCPP_WARN(get_logger(), "Transition rejected: %s",
                result.reason.c_str());
    return;
  }

  PublishState(result.reason);
}

TransitionResult BehaviorNode::HandleEvent(
    const TransitionAction action,
    const ad_msgs::msg::BehaviorEvent::SharedPtr event) {
  switch (action) {
    case TransitionAction::kInitToStandby:
      return InitToStandby(event);
    case TransitionAction::kStandbyToSwitching:
      return StandbyToSwitching(event);
    case TransitionAction::kSwitchingToStandby:
      return SwitchingToStandby(event);
    case TransitionAction::kSwitchingToPilotStandby:
      return SwitchingToPilotStandby(event);
    case TransitionAction::kPilotStandbyToSwitching:
      return PilotStandbyToSwitching(event);
    case TransitionAction::kPilotStandbyToNoaActive:
      return PilotStandbyToNoaActive(event);
    case TransitionAction::kNoaActiveToPilotFinished:
      return NoaActiveToPilotFinished(event);
    case TransitionAction::kPilotFinishedToPilotStandby:
      return PilotFinishedToPilotStandby(event);
  }

  return {false, false, "unknown_transition_action"};
}

void BehaviorNode::Tick() {
  // 保留定时器作为后续超时事件入口；状态跳转只由 /behavior/event 驱动。
}

void BehaviorNode::PublishState(const std::string &reason) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_->header.stamp = now();
  state_->reason = reason;
  const auto system_state =
      static_cast<SystemStateMachine>(state_->vehicle_state);
  state_->autonomous_enabled =
      system_state == SystemStateMachine::PILOT ||
      system_state == SystemStateMachine::PARKING;
  state_publisher_->publish(*state_);
}

TransitionResult BehaviorNode::InitToStandby(
    const ad_msgs::msg::BehaviorEvent::SharedPtr event) {
  if (!event) return {false, false, "null_event"};
  std::lock_guard<std::mutex> lock(context_mutex_);
  if (!context_->odometry_data) return {true, false, "odometry_not_ready"};

  const double age = (now() - context_->odometry_data->header.stamp).seconds();
  if (age > 0.5) return {true, false, "odometry_timeout"};

  std::lock_guard<std::mutex> state_lock(state_mutex_);
  state_->vehicle_state =
      static_cast<std::uint8_t>(SystemStateMachine::STANDBY);
  state_->function_state =
      static_cast<std::uint8_t>(SystemStateMachine::STANDBY);
  return {true, true, "init_data_ready"};
}

TransitionResult BehaviorNode::StandbyToSwitching(
    const ad_msgs::msg::BehaviorEvent::SharedPtr event) {
  if (!event) return {false, false, "null_event"};
  std::lock_guard<std::mutex> context_lock(context_mutex_);
  if (!context_->vehicle_data || !context_->odometry_data) {
    return {true, false, "vehicle_data_not_ready"};
  }

  std::lock_guard<std::mutex> state_lock(state_mutex_);
  state_->vehicle_state =
      static_cast<std::uint8_t>(SystemStateMachine::SWITCHING);
  state_->function_state =
      static_cast<std::uint8_t>(SystemStateMachine::SWITCHING);
  return {true, true, "switching_started"};
}

TransitionResult BehaviorNode::SwitchingToStandby(
    const ad_msgs::msg::BehaviorEvent::SharedPtr event) {
  if (!event) return {false, false, "null_event"};
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  state_->vehicle_state =
      static_cast<std::uint8_t>(SystemStateMachine::STANDBY);
  state_->function_state =
      static_cast<std::uint8_t>(SystemStateMachine::STANDBY);
  return {true, true, "switching_cancelled"};
}

TransitionResult BehaviorNode::SwitchingToPilotStandby(
    const ad_msgs::msg::BehaviorEvent::SharedPtr event) {
  if (!event) return {false, false, "null_event"};
  std::lock_guard<std::mutex> context_lock(context_mutex_);
  if (!context_->vehicle_data ||
      context_->vehicle_data->speed > kVehicleStopSpeedMps) {
    return {true, false, "vehicle_not_stopped"};
  }

  std::lock_guard<std::mutex> state_lock(state_mutex_);
  state_->vehicle_state =
      static_cast<std::uint8_t>(SystemStateMachine::PILOT);
  state_->function_state =
      static_cast<std::uint8_t>(DrivingStateMachine::STANDBY);
  return {true, true, "pilot_standby"};
}

TransitionResult BehaviorNode::PilotStandbyToSwitching(
    const ad_msgs::msg::BehaviorEvent::SharedPtr event) {
  if (!event) return {false, false, "null_event"};
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  state_->vehicle_state =
      static_cast<std::uint8_t>(SystemStateMachine::SWITCHING);
  state_->function_state =
      static_cast<std::uint8_t>(SystemStateMachine::SWITCHING);
  return {true, true, "parking_switching_started"};
}

TransitionResult BehaviorNode::PilotStandbyToNoaActive(
    const ad_msgs::msg::BehaviorEvent::SharedPtr event) {
  if (!event) return {false, false, "null_event"};
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  state_->vehicle_state =
      static_cast<std::uint8_t>(SystemStateMachine::PILOT);
  state_->function_state =
      static_cast<std::uint8_t>(DrivingStateMachine::NOA_ACTIVE);
  return {true, true, "noa_active"};
}

TransitionResult BehaviorNode::NoaActiveToPilotFinished(
    const ad_msgs::msg::BehaviorEvent::SharedPtr event) {
  if (!event) return {false, false, "null_event"};
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  state_->vehicle_state =
      static_cast<std::uint8_t>(SystemStateMachine::PILOT);
  state_->function_state =
      static_cast<std::uint8_t>(DrivingStateMachine::PILOT_FINISHED);
  return {true, true, "pilot_finished"};
}

TransitionResult BehaviorNode::PilotFinishedToPilotStandby(
    const ad_msgs::msg::BehaviorEvent::SharedPtr event) {
  if (!event) return {false, false, "null_event"};
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  state_->vehicle_state =
      static_cast<std::uint8_t>(SystemStateMachine::PILOT);
  state_->function_state =
      static_cast<std::uint8_t>(DrivingStateMachine::STANDBY);
  return {true, true, "pilot_standby"};
}

}  // namespace MiniAD

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MiniAD::BehaviorNode>());
  rclcpp::shutdown();
  return 0;
}
