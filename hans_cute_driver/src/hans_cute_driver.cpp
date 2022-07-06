#include "hans_cute_driver/hans_cute_driver.h"

HansCuteRobot::RobotDriver::RobotDriver(const std::string port, const long baudrate) : HansCuteRobot::SerialComms(port, baudrate)
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
bool HansCuteRobot::RobotDriver::setID(const uint8_t &old_id, const uint8_t &new_id)
{
}
bool HansCuteRobot::RobotDriver::setBaudrate(const uint8_t &servo_id, const long baudrate)
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
bool HansCuteRobot::RobotDriver::setPosition() {}
bool HansCuteRobot::RobotDriver::setSpeed() {}

bool HansCuteRobot::RobotDriver::setTorqueLimit() {}
bool HansCuteRobot::RobotDriver::setGoalTorque() {}

bool HansCuteRobot::RobotDriver::setPositionAndSpeed() {}

//===============================================================//
// These functions can send multiple commands to multiple servos //
//===============================================================//

bool HansCuteRobot::RobotDriver::setMultiTorqueEnabled() {}

// Range is between 0 -> 255
bool HansCuteRobot::RobotDriver::setMultiComplianceMargin() {}
bool HansCuteRobot::RobotDriver::setMultiComplianceSlope() {}

// Position value ranges from 0 -> 4095 (0xFFF), unit is 0.088 degree
bool HansCuteRobot::RobotDriver::setMultiPosition() {}
bool HansCuteRobot::RobotDriver::setMultiSpeed() {}

bool HansCuteRobot::RobotDriver::setMultiTorqueLimit() {}
bool HansCuteRobot::RobotDriver::setMultiPositionAndSpeed() {}

//===============================//
// Servo status access functions //
//===============================//

bool HansCuteRobot::RobotDriver::getModelNumber() {}
bool HansCuteRobot::RobotDriver::getFirmwareVersion() {}
bool HansCuteRobot::RobotDriver::getReturnDelayTime() {}

bool HansCuteRobot::RobotDriver::getAngleLimits() {}
bool HansCuteRobot::RobotDriver::getDriveMode() {}

bool HansCuteRobot::RobotDriver::getVoltageLimits() {}
bool HansCuteRobot::RobotDriver::getPosition() {}
bool HansCuteRobot::RobotDriver::getSpeed() {}

bool HansCuteRobot::RobotDriver::getVoltage() {}
bool HansCuteRobot::RobotDriver::getCurrent() {}