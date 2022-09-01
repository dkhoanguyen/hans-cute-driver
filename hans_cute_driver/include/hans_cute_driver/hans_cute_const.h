#ifndef _HANS_CUTE_CONST_H_
#define _HANS_CUTE_CONST_H_

#include <string>
#include <map>

namespace HansCuteRobot
{
enum class ControlTableConstant
{
  MODEL_NUMBER_L = 0,
  MODEL_NUMBER_H = 1,
  VERSION = 2,
  ID = 3,
  BAUD_RATE = 4,
  RETURN_DELAY_TIME = 5,
  CW_ANGLE_LIMIT_L = 6,
  CW_ANGLE_LIMIT_H = 7,
  CCW_ANGLE_LIMIT_L = 8,
  CCW_ANGLE_LIMIT_H = 9,
  DRIVE_MODE = 10,
  LIMIT_TEMPERATURE = 11,
  DOWN_LIMIT_VOLTAGE = 12,
  UP_LIMIT_VOLTAGE = 13,
  MAX_TORQUE_L = 14,
  MAX_TORQUE_H = 15,
  RETURN_LEVEL = 16,
  ALARM_LED = 17,
  ALARM_SHUTDOWN = 18,
  OPERATING_MODE = 19,
  DOWN_CALIBRATION_L = 20,
  DOWN_CALIBRATION_H = 21,
  UP_CALIBRATION_L = 22,
  UP_CALIBRATION_H = 23,
  TORQUE_ENABLE = 24,
  LED = 25,
  CW_COMPLIANCE_MARGIN = 26,
  CCW_COMPLIANCE_MARGIN = 27,
  CW_COMPLIANCE_SLOPE = 28,
  CCW_COMPLIANCE_SLOPE = 29,
  D_GAIN = 26,
  I_GAIN = 27,
  P_GAIN = 28,
  GOAL_POSITION_L = 30,
  GOAL_POSITION_H = 31,
  GOAL_SPEED_L = 32,
  GOAL_SPEED_H = 33,
  TORQUE_LIMIT_L = 34,
  TORQUE_LIMIT_H = 35,
  PRESENT_POSITION_L = 36,
  PRESENT_POSITION_H = 37,
  PRESENT_SPEED_L = 38,
  PRESENT_SPEED_H = 39,
  PRESENT_LOAD_L = 40,
  PRESENT_LOAD_H = 41,
  PRESENT_VOLTAGE = 42,
  PRESENT_TEMPERATURE = 43,
  REGISTERED_INSTRUCTION = 44,
  PAUSE_TIME = 45,
  MOVING = 46,
  LOCK = 47,
  PUNCH_L = 48,
  PUNCH_H = 49,
  SENSED_CURRENT_L = 56,
  SENSED_CURRENT_H = 57,
  CURRENT_L = 68,
  CURRENT_H = 69,
  TORQUE_CONTROL_MODE = 70,
  GOAL_TORQUE_L = 71,
  GOAL_TORQUE_H = 72,
  GOAL_ACCELERATION = 73,
};

enum class StatusReturnLevel
{
  RETURN_NONE = 0,
  RETURN_READ = 1,
  RETURN_ALL = 2,

};

enum class InstructionSet
{
  PING = 1,
  READ_DATA = 2,
  WRITE_DATA = 3,
  REG_WRITE = 4,
  ACTION = 5,
  RESET = 6,
  SYNC_WRITE = 131,
};

enum class BroadcastConstant
{
  BROADCAST = 254,
};

enum class ErrorCode
{
  INSTRUCTION_ERROR = 64,
  OVERLOAD_ERROR = 32,
  CHECKSUM_ERROR = 16,
  RANGE_ERROR = 8,
  OVERHEATING_ERROR = 4,
  ANGLE_LIMIT_ERROR = 2,
  INPUT_VOLTAGE_ERROR = 1,
  NO_ERROR = 0,
};

enum class StaticParameter
{
  MIN_COMPLIANCE_MARGIN = 0,
  MAX_COMPLIANCE_MARGIN = 255,

  MIN_COMPLIANCE_SLOPE = 1,
  MAX_COMPLIANCE_SLOPE = 254,

  MIN_TORQUE = 0,
  MAX_TORQUE = 1
};

enum class UtilsParameter
{
  MIN_PUNCH = 0,
  MAX_PUNCH = 255,

  MAX_SPEED_TICK = 1023,   // maximum speed in encoder units
  MAX_TORQUE_TICK = 1023,  // maximum torque in encoder units
};

constexpr double KGCM_TO_NM  = 0.0980665;       // 1 kg-cm is that many N-m
constexpr double RPM_TO_RADSEC = 0.104719755;  // 1 RPM is that many rad/sec

// Servo model
struct ServoModel
{
  std::string name;
  unsigned int encoder_resolution;
  double range_degrees;
  double torque_per_volt;
  double velocity_per_volt;
  double rpm_per_tick;
};

// static const ServoModel AX12{}''
static const ServoModel MX28{ "MX-28", 4096, 360.0, 6.0 / 12.0, (36 * RPM_TO_RADSEC) / 12.0, 0.114 / 2.5 };
static const ServoModel MX64{ "MX-64", 4096, 360.0, 6.0 / 12.0, (32 * RPM_TO_RADSEC) / 12.0, 0.114 / 2.8 };

static const std::map<unsigned int, ServoModel> ModelToParams = {
  {29,MX28},
  {310, MX64}
}; // Replication of Python dictionary


};  // namespace HansCuteRobot
#endif