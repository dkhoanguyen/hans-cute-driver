#ifndef HANS_CUTE_DRIVER__HANS_COMMUNICATION_HPP_
#define HANS_CUTE_DRIVER__HANS_COMMUNICATION_HPP_

#include <string>
#include <vector>
#include <iostream>
#include <mutex>
#include <cstring>
#include <limits>
#include <cmath>
#include <thread>
#include <atomic>
#include <csignal>

#include "hans_cute_datatype.h"
#include "hans_cute_const.h"
#include "serial_port.hpp"

namespace HansCuteRobot
{
  class ServoComms
  {
  public:
    ServoComms();
    ~ServoComms();

    bool open(const std::string &port);
    bool close();
    bool ping(const uint8_t &id, std::vector<uint8_t> &returned_data);

    //====================================================================//
    // These function modify EEPROM data which persists after power cycle //
    //====================================================================//

    // These can be left until the end as they are relatively unimportant
    bool setID(const uint8_t &old_id, const uint8_t &new_id);
    bool setBaudrate(const uint8_t &servo_id, const long &baudrate);

    bool setReturnDelayTime(const uint8_t &servo_id, const unsigned int &delay_time);
    bool setAngleLimits(const uint8_t &servo_id, const unsigned int &min_limit, const unsigned int &max_limit);
    bool setVoltageLimits(const uint8_t &servo_id, const double &min, const double &max);
    bool setMaxTorque(const uint8_t &servo_id, const unsigned int &max_torque);

    //===============================================================//
    // These functions can send multiple commands to a single servo  //
    //===============================================================//
    bool setTorqueEnable(const uint8_t &servo_id, const bool &enabled);
    bool setComplianceMargin();
    bool setComplianceSlope();

    bool setDGain();
    bool setIGain();
    bool setPGain();

    bool setAcceleration(const uint8_t &servo_id, const unsigned int &acceleration);
    bool setPosition(const uint8_t &servo_id, const unsigned int &position);
    bool setSpeed(const uint8_t &servo_id, const unsigned int &speed);

    bool setTorqueLimit(const uint8_t &servo_id, const unsigned int &torque_limit);
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
    bool setMultiPosition(const std::vector<unsigned int> &servo_ids, const std::vector<unsigned int> &positions);
    bool setMultiSpeed(const std::vector<unsigned int> &servo_ids, const std::vector<unsigned int> &speeds);

    bool setMultiTorqueLimit(const std::vector<unsigned int> &servo_ids,
                             const std::vector<double> &torque_limits);
    bool setMultiPositionAndSpeed(const std::vector<unsigned int> &servo_ids, const std::vector<unsigned int> &positions,
                                  const std::vector<unsigned int> &speeds);

    //===============================//
    // Servo status access functions //
    //===============================//
    bool getModelNumber(const int &servo_id, unsigned int &model_number);
    bool getFirmwareVersion();
    bool getReturnDelayTime();

    bool getMaxTorque(const int &servo_id, unsigned int &torque);
    bool getTorqueEnabled(const int &servo_id, bool &enabled);
    bool getAngleLimits(const int &servo_id, unsigned int &angle_min, unsigned int &angle_max);
    bool getVoltageLimits();

    bool getPosition(const int &servo_id, unsigned int &position);
    bool getSpeed(const int &servo_id, unsigned int &speed);
    bool getAcceleration(const int &servo_id, unsigned int &acceleration);

    bool getVoltage();
    bool getCurrent();
    bool getLock(const int &servo_id, bool &lock);

    bool getFeedback(const int &servo_id, ServoFeedback &feedback);

  protected:
    bool read(const uint8_t &id, const uint8_t &address, const uint8_t &size, std::vector<uint8_t> &returned_data,
              unsigned long &timestamp);
    bool write(const uint8_t &id, const uint8_t &address, const std::vector<uint8_t> &data,
               std::vector<uint8_t> &returned_data, unsigned long &timestamp);
    bool syncWrite(const uint8_t &address, const std::vector<std::vector<uint8_t>> &data);

    uint8_t calcCheckSum(std::vector<uint8_t> &data) const;

    int readResponse(std::vector<uint8_t> &response);
    int writeCommand(const std::vector<uint8_t> &command);

    std::shared_ptr<SerialPort> serial_port_ptr_;
  };

  enum class SerialError
  {
    NO_ERROR,

    OPEN_ERROR,
    CLOSE_ERROR,

    READ_ERROR,
    WRITE_ERROR,

    NO_RESPONSE,
    WRONG_HEADER,
    WRONG_CHECKSUM,
  };

  struct ServoParams
  {
    unsigned int id;
    unsigned int model_number;
    std::string model_name;
    std::string joint_name;

    // Raw data
    unsigned int raw_origin;
    unsigned int raw_min;
    unsigned int raw_max;
    static const unsigned int raw_full_sweep = 4096;

    unsigned int speed;
    unsigned int acceleration;

    // Radian
    double min;
    double max;

    // Utils
    double torque_per_volt;
    double max_torque;

    double velocity_per_volt;
    double max_velocity;
    double radians_second_per_encoder_tick;
    double rpm_per_tick;

    double encoder_resolution;
    double range_radian;
    double rad_per_enc_tick;
    double enc_tick_per_rad;

    friend std::ostream &operator<<(std::ostream &stream, const ServoParams &param)
    {
      std::cout << "Servo ID: " << param.id << std::endl;
      std::cout << "Model Number: " << param.model_number << std::endl;
      std::cout << "Model Name: " << param.model_name << std::endl;
      std::cout << "Raw Origin: " << param.raw_origin << std::endl;
      std::cout << "Raw Min: " << param.raw_min << std::endl;
      std::cout << "Raw Max Name: " << param.raw_max << std::endl;
      std::cout << "Raw Full Sweep: " << param.raw_full_sweep << std::endl;
      std::cout << "Min: " << param.min << std::endl;
      std::cout << "Max: " << param.max << std::endl;
      std::cout << "Radian per encoder tick: " << param.rad_per_enc_tick << std::endl;
      std::cout << "Encoder tick per radian: " << param.enc_tick_per_rad << std::endl;
    };
  };
};
#endif