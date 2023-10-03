#ifndef HANS_CUTE_DRIVER__HANS_COMMAND_HPP_
#define HANS_CUTE_DRIVER__HANS_COMMAND_HPP_

#include <iostream>
#include <vector>
#include "hans_cute_const.h"

namespace HansCuteRobot
{
  struct DataPacket
  {
    uint8_t control_id;
    std::vector<uint8_t> data;
  };

  class HansCommand
  {
  public:
    //====================================================================//
    // These function modify EEPROM data which persists after power cycle //
    //====================================================================//

    // These can be left until the end as they are relatively unimportant
    static DataPacket setID(const uint8_t &old_id, const uint8_t &new_id);
    static DataPacket setBaudrate(const uint8_t &servo_id, const long &baudrate);

    static DataPacket setReturnDelayTime(const uint8_t &servo_id, const unsigned int &delay_time);
    static DataPacket setAngleLimits(const uint8_t &servo_id, const unsigned int &min_limit, const unsigned int &max_limit);
    static DataPacket setVoltageLimits(const uint8_t &servo_id, const double &min, const double &max);
    static DataPacket setMaxTorque(const uint8_t &servo_id, const unsigned int &max_torque);

    //===============================================================//
    // These functions can send multiple commands to a single servo  //
    //===============================================================//
    static DataPacket setTorqueEnable(const uint8_t &servo_id, const bool &enabled);
    static DataPacket setComplianceMargin();
    static DataPacket setComplianceSlope();

    static DataPacket setDGain();
    static DataPacket setIGain();
    static DataPacket setPGain();

    static DataPacket setAcceleration(const uint8_t &servo_id, const unsigned int &acceleration);
    static DataPacket setPosition(const uint8_t &servo_id, const unsigned int &position);
    static DataPacket setSpeed(const uint8_t &servo_id, const unsigned int &speed);

    static DataPacket setTorqueLimit(const uint8_t &servo_id, const unsigned int &torque_limit);
    static DataPacket setGoalTorque();

    static DataPacket setPositionAndSpeed();

    //===============================================================//
    // These functions can send multiple commands to multiple servos //
    //===============================================================//

    static DataPacket setMultiTorqueEnabled();

    // Range is between 0 -> 255
    static DataPacket setMultiComplianceMargin();
    static DataPacket setMultiComplianceSlope();

    // Position value ranges from 0 -> 4095 (0xFFF), unit is 0.088 degree
    static DataPacket setMultiPosition(const std::vector<unsigned int> &servo_ids, const std::vector<unsigned int> &positions);
    static DataPacket setMultiSpeed(const std::vector<unsigned int> &servo_ids, const std::vector<unsigned int> &speeds);

    static DataPacket setMultiTorqueLimit(const std::vector<unsigned int> &servo_ids,
                                          const std::vector<double> &torque_limits);
    static DataPacket setMultiPositionAndSpeed(const std::vector<unsigned int> &servo_ids, const std::vector<unsigned int> &positions,
                                               const std::vector<unsigned int> &speeds);
  };
};

#endif