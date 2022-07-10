#include "hans_cute_driver/hans_cute_driver.h"

HansCuteRobot::ServoDriver::ServoDriver(const std::string port, const long baudrate)
  : HansCuteRobot::ServoSerialComms(port, baudrate)
{
}

HansCuteRobot::ServoDriver::~ServoDriver()
{
  std::cout << "ServoDriver destructor" << std::endl;
}

//====================================================================//
// These function modify EEPROM data which persists after power cycle //
//====================================================================//

// These can be left until the end as they are relatively unimportant
bool HansCuteRobot::ServoDriver::setID(const uint8_t& old_id, const uint8_t& new_id)
{
}
bool HansCuteRobot::ServoDriver::setBaudrate(const uint8_t& servo_id, const long& baudrate)
{
}

bool HansCuteRobot::ServoDriver::setReturnDelayTime()
{
}
bool HansCuteRobot::ServoDriver::setAngleLimits(const uint8_t& servo_id, const unsigned int& min_limit, const unsigned int& max_limit)
{
  std::vector<uint8_t> raw_limit = HansCuteRobot::AngleLimits::getRawData(min_limit, max_limit);
  std::vector<uint8_t> returned_data;
  unsigned long timestamp;
  write(servo_id, (uint8_t)ControlTableConstant::DXL_CW_ANGLE_LIMIT_L, raw_limit, returned_data, timestamp);
  return true;
}

bool HansCuteRobot::ServoDriver::setVoltageLimits()
{
}

//===============================================================//
// These functions can send multiple commands to a single servo  //
//===============================================================//
bool HansCuteRobot::ServoDriver::setTorqueEnable(const uint8_t& servo_id, const bool& enabled)
{
  std::vector<uint8_t> data({(uint8_t)enabled});
  std::vector<uint8_t> returned_data;
  unsigned long timestamp;
  write(servo_id, (uint8_t)ControlTableConstant::DXL_TORQUE_ENABLE, data, returned_data, timestamp);
  for (uint8_t data : returned_data)
  {
    std::cout << "Received data:" << std::hex << (int)data << std::endl;
  }
  return true;
}
bool HansCuteRobot::ServoDriver::setComplianceMargin()
{
}
bool HansCuteRobot::ServoDriver::setComplianceSlope()
{
}
bool HansCuteRobot::ServoDriver::setDGain()
{
}
bool HansCuteRobot::ServoDriver::setIGain()
{
}
bool HansCuteRobot::ServoDriver::setPGain()
{
}

bool HansCuteRobot::ServoDriver::setAcceleration()
{
}
bool HansCuteRobot::ServoDriver::setPosition(const uint8_t& servo_id, const unsigned int& position)
{
  std::vector<uint8_t> raw_postion = HansCuteRobot::ServoPosition::getRawPosition(position);
  std::vector<uint8_t> returned_data;
  unsigned long timestamp;
  write(servo_id, (uint8_t)ControlTableConstant::DXL_GOAL_POSITION_L, raw_postion, returned_data, timestamp);
  for (uint8_t data : returned_data)
  {
    std::cout << "Received data:" << std::hex << (int)data << std::endl;
  }
  return true;
}
bool HansCuteRobot::ServoDriver::setSpeed(const uint8_t& servo_id, const unsigned int& speed)
{
  std::vector<uint8_t> raw_speed = HansCuteRobot::ServoSpeed::getRawSpeed(speed);
  std::vector<uint8_t> returned_data;
  unsigned long timestamp;
  write(servo_id, (uint8_t)ControlTableConstant::DXL_GOAL_SPEED_L, raw_speed, returned_data, timestamp);
  return true;
}

bool HansCuteRobot::ServoDriver::setTorqueLimit()
{
}
bool HansCuteRobot::ServoDriver::setGoalTorque()
{
}

bool HansCuteRobot::ServoDriver::setPositionAndSpeed()
{
}

//===============================================================//
// These functions can send multiple commands to multiple servos //
//===============================================================//

bool HansCuteRobot::ServoDriver::setMultiTorqueEnabled()
{
}

// Range is between 0 -> 255
bool HansCuteRobot::ServoDriver::setMultiComplianceMargin()
{
}
bool HansCuteRobot::ServoDriver::setMultiComplianceSlope()
{
}

// Position value ranges from 0 -> 4095 (0xFFF), unit is 0.088 degree
bool HansCuteRobot::ServoDriver::setMultiPosition()
{
}
bool HansCuteRobot::ServoDriver::setMultiSpeed()
{
}

bool HansCuteRobot::ServoDriver::setMultiTorqueLimit()
{
}
bool HansCuteRobot::ServoDriver::setMultiPositionAndSpeed()
{
}

//===============================//
// Servo status access functions //
//===============================//

bool HansCuteRobot::ServoDriver::getModelNumber(const int& servo_id, unsigned int& model_number)
{
  std::vector<uint8_t> response;
  unsigned long timestamp = 0;
  if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::DXL_MODEL_NUMBER_L, 2, response, timestamp))
  {
    return false;
  }
  model_number = HansCuteRobot::ModelNumber::getData(response);
  return true;
}
bool HansCuteRobot::ServoDriver::getFirmwareVersion()
{
}
bool HansCuteRobot::ServoDriver::getReturnDelayTime()
{
}

bool HansCuteRobot::ServoDriver::getAngleLimits(const int& servo_id, HansCuteRobot::AngleLimits& angle_limit)
{
  std::vector<uint8_t> response;
  unsigned long timestamp = 0;
  if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::DXL_CW_ANGLE_LIMIT_L, 4, response, timestamp))
  {
    return false;
  }
  angle_limit = HansCuteRobot::AngleLimits::getData(response);
  return true;
}

bool HansCuteRobot::ServoDriver::getTorqueEnabled(const int& servo_id, bool &enabled)
{
  std::vector<uint8_t> response;
  unsigned long timestamp = 0;
  if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::DXL_TORQUE_ENABLE, 1, response, timestamp))
  {
    return false;
  }
  for (uint8_t data : response)
  {
    std::cout << "Received data:" << std::hex << (int)data << std::endl;
  }
  return true;
}

bool HansCuteRobot::ServoDriver::getVoltageLimits()
{
}
bool HansCuteRobot::ServoDriver::getPosition(const int& servo_id)
{
  std::vector<uint8_t> response;
  unsigned long timestamp = 0;
  if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::DXL_PRESENT_POSITION_L, 2, response, timestamp))
  {
    return false;
  }
  std::cout << std::dec << (int)HansCuteRobot::ServoPosition::getPosition(response) << std::endl;
  return true;
}
bool HansCuteRobot::ServoDriver::getSpeed(const int& servo_id)
{
  std::vector<uint8_t> response;
  unsigned long timestamp = 0;
  if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::DXL_PRESENT_SPEED_L, 2, response, timestamp))
  {
    return false;
  }
  std::cout << std::dec << (int)HansCuteRobot::ServoSpeed::getSpeed(response) << std::endl;
  return true;
}

bool HansCuteRobot::ServoDriver::getVoltage()
{
}
bool HansCuteRobot::ServoDriver::getCurrent()
{
}

bool HansCuteRobot::ServoDriver::getFeedback(const int& servo_id, ServoFeedback& feedback)
{
  std::vector<uint8_t> response;
  unsigned long timestamp = 0;
  if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::DXL_GOAL_POSITION_L, 17, response, timestamp))
  {
    return false;
  }

  feedback = ServoFeedback::getData((uint8_t)servo_id, response, timestamp);
  return true;
}