#include "hans_cute_driver/hans_cute_driver.h"

const SamplePacket HansCuteRobot::ServoDriver::DXL_PACKET{ std::vector<uint8_t>({ 0xFF, 0xFF }), 2, 3, 4 };

HansCuteRobot::ServoDriver::ServoDriver(const std::string port, const long baudrate)
  : HansCuteRobot::ServoSerialComms(port, baudrate)
{
  // Update paket format at initialisation
  setSamplePacket(HansCuteRobot::ServoDriver::DXL_PACKET);
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
  return true;
}
bool HansCuteRobot::ServoDriver::setBaudrate(const uint8_t& servo_id, const long& baudrate)
{
  return true;
}

bool HansCuteRobot::ServoDriver::setReturnDelayTime()
{
  return true;
}
bool HansCuteRobot::ServoDriver::setAngleLimits(const uint8_t& servo_id, const unsigned int& min_limit,
                                                const unsigned int& max_limit)
{
  std::vector<uint8_t> raw_limit = HansCuteRobot::AngleLimits::getRawData(min_limit, max_limit);
  std::vector<uint8_t> returned_data;
  unsigned long timestamp;
  write(servo_id, (uint8_t)ControlTableConstant::CW_ANGLE_LIMIT_L, raw_limit, returned_data, timestamp);
  return true;
}

bool HansCuteRobot::ServoDriver::setVoltageLimits()
{
  return true;
}

//===============================================================//
// These functions can send multiple commands to a single servo  //
//===============================================================//
bool HansCuteRobot::ServoDriver::setTorqueEnable(const uint8_t& servo_id, const bool& enabled)
{
  std::vector<uint8_t> data({ (uint8_t)enabled });
  std::vector<uint8_t> returned_data;
  unsigned long timestamp;
  write(servo_id, (uint8_t)ControlTableConstant::TORQUE_ENABLE, data, returned_data, timestamp);
  return true;
}
bool HansCuteRobot::ServoDriver::setComplianceMargin()
{
  return true;
}
bool HansCuteRobot::ServoDriver::setComplianceSlope()
{
  return true;
}
bool HansCuteRobot::ServoDriver::setDGain()
{
  return true;
}
bool HansCuteRobot::ServoDriver::setIGain()
{
  return true;
}
bool HansCuteRobot::ServoDriver::setPGain()
{
  return true;
}

bool HansCuteRobot::ServoDriver::setAcceleration()
{
  return true;
}
bool HansCuteRobot::ServoDriver::setPosition(const uint8_t& servo_id, const unsigned int& position)
{
  std::vector<uint8_t> raw_postion = HansCuteRobot::ServoPosition::getRawData(position);
  std::vector<uint8_t> returned_data;
  unsigned long timestamp;
  write(servo_id, (uint8_t)ControlTableConstant::GOAL_POSITION_L, raw_postion, returned_data, timestamp);
  return true;
}
bool HansCuteRobot::ServoDriver::setSpeed(const uint8_t& servo_id, const unsigned int& speed)
{
  std::vector<uint8_t> raw_speed = HansCuteRobot::ServoSpeed::getRawData(speed);
  std::vector<uint8_t> returned_data;
  unsigned long timestamp;
  write(servo_id, (uint8_t)ControlTableConstant::GOAL_SPEED_L, raw_speed, returned_data, timestamp);
  return true;
}

bool HansCuteRobot::ServoDriver::setTorqueLimit()
{
  return true;
}
bool HansCuteRobot::ServoDriver::setGoalTorque()
{
  return true;
}

bool HansCuteRobot::ServoDriver::setPositionAndSpeed()
{
  return true;
}

//===============================================================//
// These functions can send multiple commands to multiple servos //
//===============================================================//

bool HansCuteRobot::ServoDriver::setMultiTorqueEnabled()
{
  return true;
}

// Range is between 0 -> 255
bool HansCuteRobot::ServoDriver::setMultiComplianceMargin()
{
  return true;
}
bool HansCuteRobot::ServoDriver::setMultiComplianceSlope()
{
  return true;
}

// Position value ranges from 0 -> 4095 (0xFFF), unit is 0.088 degree
bool HansCuteRobot::ServoDriver::setMultiPosition(const std::vector<unsigned int>& servo_ids,
                                                  const std::vector<unsigned int>& positions)
{
  // Prepare data for sync_write
  std::vector<std::vector<uint8_t>> data_lists;
  for (int indx = 0; indx < servo_ids.size(); indx++)
  {
    std::vector<uint8_t> data;
    data.push_back((uint8_t)servo_ids.at(indx));
    std::vector<uint8_t> position = HansCuteRobot::ServoPosition::getRawData(positions.at(indx));
    data.insert(data.end(), position.begin(), position.end());
    data_lists.push_back(data);
  }
  syncWrite((uint8_t)ControlTableConstant::GOAL_POSITION_L, data_lists);
  return true;
}
bool HansCuteRobot::ServoDriver::setMultiSpeed(const std::vector<unsigned int>& servo_ids,
                                               const std::vector<unsigned int>& speeds)
{
  // Prepare data for sync_write
  std::vector<std::vector<uint8_t>> data_lists;
  for (int indx = 0; indx < servo_ids.size(); indx++)
  {
    std::vector<uint8_t> data;
    data.push_back((uint8_t)servo_ids.at(indx));
    std::vector<uint8_t> speed = HansCuteRobot::ServoSpeed::getRawData(speeds.at(indx));
    data.insert(data.end(), speed.begin(), speed.end());
    data_lists.push_back(data);
  }
  syncWrite((uint8_t)ControlTableConstant::GOAL_POSITION_L, data_lists);
  return true;
}

bool HansCuteRobot::ServoDriver::setMultiTorqueLimit()
{
  return true;
}
bool HansCuteRobot::ServoDriver::setMultiPositionAndSpeed(const std::vector<unsigned int>& servo_ids,
                                                          const std::vector<unsigned int>& positions,
                                                          const std::vector<unsigned int>& speeds)
{
  std::vector<std::vector<uint8_t>> data_lists;
  for (int indx = 0; indx < servo_ids.size(); indx++)
  {
    std::vector<uint8_t> data;
    data.push_back((uint8_t)servo_ids.at(indx));
    std::vector<uint8_t> position = HansCuteRobot::ServoPosition::getRawData(positions.at(indx));
    data.insert(data.end(), position.begin(), position.end());
    std::vector<uint8_t> speed = HansCuteRobot::ServoSpeed::getRawData(speeds.at(indx));
    data.insert(data.end(), speed.begin(), speed.end());
    data_lists.push_back(data);
  }
  syncWrite((uint8_t)ControlTableConstant::GOAL_POSITION_L, data_lists);
  return true;
}

//===============================//
// Servo status access functions //
//===============================//

bool HansCuteRobot::ServoDriver::getModelNumber(const int& servo_id, unsigned int& model_number)
{
  std::vector<uint8_t> response;
  unsigned long timestamp = 0;
  if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::MODEL_NUMBER_L, 2, response, timestamp))
  {
    return false;
  }
  model_number = HansCuteRobot::ModelNumber::getData(response);
  return true;
}
bool HansCuteRobot::ServoDriver::getFirmwareVersion()
{
  return true;
}
bool HansCuteRobot::ServoDriver::getReturnDelayTime()
{
  return true;
}

bool HansCuteRobot::ServoDriver::getAngleLimits(const int& servo_id, HansCuteRobot::AngleLimits& angle_limit)
{
  std::vector<uint8_t> response;
  unsigned long timestamp = 0;
  if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::CW_ANGLE_LIMIT_L, 4, response, timestamp))
  {
    return false;
  }
  angle_limit = HansCuteRobot::AngleLimits::getData(response);
  return true;
}

bool HansCuteRobot::ServoDriver::getTorqueEnabled(const int& servo_id, bool& enabled)
{
  std::vector<uint8_t> response;
  unsigned long timestamp = 0;
  if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::TORQUE_ENABLE, 1, response, timestamp))
  {
    return false;
  }
  return true;
}

bool HansCuteRobot::ServoDriver::getVoltageLimits()
{
  return true;
}
bool HansCuteRobot::ServoDriver::getPosition(const int& servo_id, unsigned int& position)
{
  std::vector<uint8_t> response;
  unsigned long timestamp = 0;
  if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::PRESENT_POSITION_L, 2, response, timestamp))
  {
    return false;
  }
  position = (unsigned int)HansCuteRobot::ServoPosition::getData(response);
  return true;
}
bool HansCuteRobot::ServoDriver::getSpeed(const int& servo_id, unsigned int& speed)
{
  std::vector<uint8_t> response;
  unsigned long timestamp = 0;
  if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::PRESENT_SPEED_L, 2, response, timestamp))
  {
    return false;
  }
  speed = (unsigned int)HansCuteRobot::ServoSpeed::getData(response);
  return true;
}

bool HansCuteRobot::ServoDriver::getVoltage()
{
  return true;
}
bool HansCuteRobot::ServoDriver::getCurrent()
{
  return true;
}

bool HansCuteRobot::ServoDriver::getFeedback(const int& servo_id, ServoFeedback& feedback)
{
  std::vector<uint8_t> response;
  unsigned long timestamp = 0;
  if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::GOAL_POSITION_L, 17, response, timestamp))
  {
    return false;
  }

  feedback = ServoFeedback::getData((uint8_t)servo_id, response, timestamp);
  return true;
}