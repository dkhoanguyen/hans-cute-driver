#ifndef _HANS_CUTE_DRIVER_H_
#define _HANS_CUTE_DRIVER_H_

#include "hans_cute_const.h"
#include "hans_cute_serial.h"

namespace HansCuteRobot
{
class RobotDriver : public SerialComms
{
public:
  RobotDriver(const std::string port, const long baudrate);
  ~RobotDriver();

  //====================================================================//
  // These function modify EEPROM data which persists after power cycle //
  //====================================================================//

  // These can be left until the end as they are relatively unimportant
  bool setID(const uint8_t& old_id, const uint8_t& new_id);
  bool setBaudrate(const uint8_t& servo_id, const long baudrate);
  
  bool setReturnDelayTime();
  bool setAngleLimits();
  bool setDriveMode();
  bool setVoltageLimits();

  //===============================================================//
  // These functions can send multiple commands to a single servo  //
  //===============================================================//
  bool setTorqueEnable();
  bool setComplianceMargin();
  bool setComplianceSlope();

  bool setDGain();
  bool setIGain();
  bool setPGain();

  bool setAcceleration();
  bool setPosition();
  bool setSpeed();

  bool setTorqueLimit();
  bool setGoalTorque();

  bool setPositionAndSpeed();

  //===============================================================//
  // These functions can send multiple commands to multiple servos //
  //===============================================================//

  bool setMultiTorqueEnabled();

  // Range is between 0 -> 255
  bool setMultiComplianceMargin();
  bool setMultiComplianceSlope();

  // Position value ranges from 0 -> 4095 (0xFFF), unit is 0.088 degree
  bool setMultiPosition();
  bool setMultiSpeed();

  bool setMultiTorqueLimit();
  bool setMultiPositionAndSpeed();

  //===============================//
  // Servo status access functions //
  //===============================//

  bool getModelNumber();
  bool getFirmwareVersion();
  bool getReturnDelayTime();

  bool getAngleLimits();
  bool getDriveMode();

  bool getVoltageLimits();
  bool getPosition();
  bool getSpeed();

  bool getVoltage();
  bool getCurrent();
};
};  // namespace HansCuteRobot

#endif