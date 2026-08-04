#ifndef AD_COMMON_HPP_
#define AD_COMMON_HPP_

#include <cstdint>

namespace MiniAD {

constexpr float kVehicleStopSpeedMps = 0.1F;
constexpr float kSwitching2ParkingSpeedKmh = 15.0F;

enum class SystemStateMachine : std::uint8_t {
  INVALID = 0,
  INIT = 1,
  STANDBY = 2,
  SWITCHING = 3,
  PILOT = 4,
  PARKING = 5,
  ACTIVE = 6,
  FINISHED = 7,
  EMERGENCY = 8,
};

enum class DrivingStateMachine : std::uint8_t {
  INVALID = 40,
  IDLE = 41,
  STANDBY = 42,
  ACC_ACTIVE = 43,
  NOA_ACTIVE = 44,
  PILOT_FINISHED = 45,
};

enum class ParkingStateMachine : std::uint8_t {
  INVALID = 50,
  IDLE = 51,
  STANDBY = 52,
  APA_ACTIVE = 53,
  AVP_ACTIVE = 54,
  PARKING_FINISHED = 55,
};

}  // namespace MiniAD

#endif  // AD_COMMON_HPP_
