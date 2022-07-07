#include "hans_cute_driver/hans_cute_driver.h"

HansCuteRobot::RobotDriver::RobotDriver(const std::string port, const long baudrate)
  : HansCuteRobot::SerialComms(port, baudrate)
{
}

HansCuteRobot::RobotDriver::~RobotDriver()
{
  std::cout << "RobotDriver destructor" << std::endl;
}

//====================================================================//
// These function modify EEPROM data which persists after power cycle //
//====================================================================//

// These can be left until the end as they are relatively unimportant
bool HansCuteRobot::RobotDriver::setID(const uint8_t& old_id, const uint8_t& new_id)
{
}
bool HansCuteRobot::RobotDriver::setBaudrate(const uint8_t& servo_id, const long baudrate)
{
}

bool HansCuteRobot::RobotDriver::setReturnDelayTime()
{
}
bool HansCuteRobot::RobotDriver::setAngleLimits()
{
}
bool HansCuteRobot::RobotDriver::setDriveMode()
{
}
bool HansCuteRobot::RobotDriver::setVoltageLimits()
{
}

//===============================================================//
// These functions can send multiple commands to a single servo  //
//===============================================================//
bool HansCuteRobot::RobotDriver::setTorqueEnable()
{
}
bool HansCuteRobot::RobotDriver::setComplianceMargin()
{
}
bool HansCuteRobot::RobotDriver::setComplianceSlope()
{
}
bool HansCuteRobot::RobotDriver::setDGain()
{
}
bool HansCuteRobot::RobotDriver::setIGain()
{
}
bool HansCuteRobot::RobotDriver::setPGain()
{
}

bool HansCuteRobot::RobotDriver::setAcceleration()
{
}
bool HansCuteRobot::RobotDriver::setPosition(const uint8_t& servo_id, const unsigned int& position)
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
bool HansCuteRobot::RobotDriver::setSpeed(const uint8_t& servo_id, const unsigned int& speed)
{
  std::vector<uint8_t> raw_speed = HansCuteRobot::ServoSpeed::getRawSpeed(speed);
  std::vector<uint8_t> returned_data;
  unsigned long timestamp;
  write(servo_id, (uint8_t)ControlTableConstant::DXL_GOAL_SPEED_L, raw_speed, returned_data, timestamp);
  for (uint8_t data : returned_data)
  {
    std::cout << "Received data:" << std::hex << (int)data << std::endl;
  }
  return true;
}

bool HansCuteRobot::RobotDriver::setTorqueLimit()
{
}
bool HansCuteRobot::RobotDriver::setGoalTorque()
{
}

bool HansCuteRobot::RobotDriver::setPositionAndSpeed()
{
}

//===============================================================//
// These functions can send multiple commands to multiple servos //
//===============================================================//

bool HansCuteRobot::RobotDriver::setMultiTorqueEnabled()
{
}

// Range is between 0 -> 255
bool HansCuteRobot::RobotDriver::setMultiComplianceMargin()
{
}
bool HansCuteRobot::RobotDriver::setMultiComplianceSlope()
{
}

// Position value ranges from 0 -> 4095 (0xFFF), unit is 0.088 degree
bool HansCuteRobot::RobotDriver::setMultiPosition()
{
}
bool HansCuteRobot::RobotDriver::setMultiSpeed()
{
}

bool HansCuteRobot::RobotDriver::setMultiTorqueLimit()
{
}
bool HansCuteRobot::RobotDriver::setMultiPositionAndSpeed()
{
}

//===============================//
// Servo status access functions //
//===============================//

bool HansCuteRobot::RobotDriver::getModelNumber(const int& servo_id, unsigned int& model_number)
{
  std::vector<uint8_t> response;
  unsigned long timestamp = 0;
  if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::DXL_MODEL_NUMBER_L, 2, response, timestamp))
  {
    return false;
  }
  for (uint8_t data : response)
  {
    std::cout << "Received data:" << std::hex << (int)data << std::endl;
  }
  model_number = HansCuteRobot::ModelNumber::getData(response);
  return true;
}
bool HansCuteRobot::RobotDriver::getFirmwareVersion()
{
}
bool HansCuteRobot::RobotDriver::getReturnDelayTime()
{
}

bool HansCuteRobot::RobotDriver::getAngleLimits()
{
}
bool HansCuteRobot::RobotDriver::getDriveMode()
{
}

bool HansCuteRobot::RobotDriver::getVoltageLimits()
{
}
bool HansCuteRobot::RobotDriver::getPosition(const int& servo_id)
{
  std::vector<uint8_t> response;
  unsigned long timestamp = 0;
  if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::DXL_PRESENT_POSITION_L, 2, response, timestamp))
  {
    return false;
  }
  std::cout <<std::dec<< (int)HansCuteRobot::ServoPosition::getPosition(response) << std::endl;
  return true;
}
bool HansCuteRobot::RobotDriver::getSpeed(const int& servo_id)
{
  std::vector<uint8_t> response;
  unsigned long timestamp = 0;
  if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::DXL_PRESENT_SPEED_L, 2, response, timestamp))
  {
    return false;
  }
  std::cout <<std::dec<< (int)HansCuteRobot::ServoSpeed::getSpeed(response) << std::endl;
  return true;
}

bool HansCuteRobot::RobotDriver::getVoltage()
{
}
bool HansCuteRobot::RobotDriver::getCurrent()
{
}

bool HansCuteRobot::RobotDriver::getFeedback(const int& servo_id, ServoFeedback& feedback)
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