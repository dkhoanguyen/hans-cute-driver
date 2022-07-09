#ifndef _HANS_CUTE_CONST_H_
#define _HANS_CUTE_CONST_H_

#include <string>
#include <map>

namespace HansCuteRobot
{
enum class ControlTableConstant
{
  DXL_MODEL_NUMBER_L = 0,
  DXL_MODEL_NUMBER_H = 1,
  DXL_VERSION = 2,
  DXL_ID = 3,
  DXL_BAUD_RATE = 4,
  DXL_RETURN_DELAY_TIME = 5,
  DXL_CW_ANGLE_LIMIT_L = 6,
  DXL_CW_ANGLE_LIMIT_H = 7,
  DXL_CCW_ANGLE_LIMIT_L = 8,
  DXL_CCW_ANGLE_LIMIT_H = 9,
  DXL_DRIVE_MODE = 10,
  DXL_LIMIT_TEMPERATURE = 11,
  DXL_DOWN_LIMIT_VOLTAGE = 12,
  DXL_UP_LIMIT_VOLTAGE = 13,
  DXL_MAX_TORQUE_L = 14,
  DXL_MAX_TORQUE_H = 15,
  DXL_RETURN_LEVEL = 16,
  DXL_ALARM_LED = 17,
  DXL_ALARM_SHUTDOWN = 18,
  DXL_OPERATING_MODE = 19,
  DXL_DOWN_CALIBRATION_L = 20,
  DXL_DOWN_CALIBRATION_H = 21,
  DXL_UP_CALIBRATION_L = 22,
  DXL_UP_CALIBRATION_H = 23,
  DXL_TORQUE_ENABLE = 24,
  DXL_LED = 25,
  DXL_CW_COMPLIANCE_MARGIN = 26,
  DXL_CCW_COMPLIANCE_MARGIN = 27,
  DXL_CW_COMPLIANCE_SLOPE = 28,
  DXL_CCW_COMPLIANCE_SLOPE = 29,
  DXL_D_GAIN = 26,
  DXL_I_GAIN = 27,
  DXL_P_GAIN = 28,
  DXL_GOAL_POSITION_L = 30,
  DXL_GOAL_POSITION_H = 31,
  DXL_GOAL_SPEED_L = 32,
  DXL_GOAL_SPEED_H = 33,
  DXL_TORQUE_LIMIT_L = 34,
  DXL_TORQUE_LIMIT_H = 35,
  DXL_PRESENT_POSITION_L = 36,
  DXL_PRESENT_POSITION_H = 37,
  DXL_PRESENT_SPEED_L = 38,
  DXL_PRESENT_SPEED_H = 39,
  DXL_PRESENT_LOAD_L = 40,
  DXL_PRESENT_LOAD_H = 41,
  DXL_PRESENT_VOLTAGE = 42,
  DXL_PRESENT_TEMPERATURE = 43,
  DXL_REGISTERED_INSTRUCTION = 44,
  DXL_PAUSE_TIME = 45,
  DXL_MOVING = 46,
  DXL_LOCK = 47,
  DXL_PUNCH_L = 48,
  DXL_PUNCH_H = 49,
  DXL_SENSED_CURRENT_L = 56,
  DXL_SENSED_CURRENT_H = 57,
  DXL_CURRENT_L = 68,
  DXL_CURRENT_H = 69,
  DXL_TORQUE_CONTROL_MODE = 70,
  DXL_GOAL_TORQUE_L = 71,
  DXL_GOAL_TORQUE_H = 72,
  DXL_GOAL_ACCELERATION = 73,
};

enum class StatusReturnLevel
{
  DXL_RETURN_NONE = 0,
  DXL_RETURN_READ = 1,
  DXL_RETURN_ALL = 2,

};

enum class InstructionSet
{
  DXL_PING = 1,
  DXL_READ_DATA = 2,
  DXL_WRITE_DATA = 3,
  DXL_REG_WRITE = 4,
  DXL_ACTION = 5,
  DXL_RESET = 6,
  DXL_SYNC_WRITE = 131,
};

enum class BroadcastConstant
{
  DXL_BROADCAST = 254,
};

enum class ErrorCode
{
  DXL_INSTRUCTION_ERROR = 64,
  DXL_OVERLOAD_ERROR = 32,
  DXL_CHECKSUM_ERROR = 16,
  DXL_RANGE_ERROR = 8,
  DXL_OVERHEATING_ERROR = 4,
  DXL_ANGLE_LIMIT_ERROR = 2,
  DXL_INPUT_VOLTAGE_ERROR = 1,
  DXL_NO_ERROR = 0,
};

enum class StaticParameter
{
  DXL_MIN_COMPLIANCE_MARGIN = 0,
  DXL_MAX_COMPLIANCE_MARGIN = 255,

  DXL_MIN_COMPLIANCE_SLOPE = 1,
  DXL_MAX_COMPLIANCE_SLOPE = 254,
};

enum class UtilsParameter
{
  DXL_MIN_PUNCH = 0,
  DXL_MAX_PUNCH = 255,

  DXL_MAX_SPEED_TICK = 1023,   // maximum speed in encoder units
  DXL_MAX_TORQUE_TICK = 1023,  // maximum torque in encoder units
};

#define KGCM_TO_NM 0.0980665       // 1 kg-cm is that many N-m
#define RPM_TO_RADSEC 0.104719755  // 1 RPM is that many rad/sec

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

static const ServoModel MX28{ "MX-28", 4096, 360.0, 6.0 / 12.0, (36 * RPM_TO_RADSEC) / 12.0, 0.114 / 2.5 };
static const ServoModel MX64{ "MX-64", 4096, 360.0, 6.0 / 12.0, (32 * RPM_TO_RADSEC) / 12.0, 0.114 / 2.8 };

static const std::map<unsigned int, ServoModel> ModelToParams = {
  {29,MX28},
  {310, MX64}
};


};  // namespace HansCuteRobot
#endif