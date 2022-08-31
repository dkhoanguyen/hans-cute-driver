#ifndef _HANS_CUTE_DRIVER_H_
#define _HANS_CUTE_DRIVER_H_

#include "hans_cute_const.h"
#include "hans_cute_serial.h"
#include "hans_cute_datatype.h"

namespace HansCuteRobot
{
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
  class ServoDriver : public ServoSerialComms
  {
  public:
    static const SamplePacket DXL_PACKET;

    ServoDriver();
    ~ServoDriver();

    bool getJointPosition(const std::vector<unsigned int> &joint_ids,
                          std::vector<unsigned int> &positions);

    bool setJointPosition(const std::vector<unsigned int> &joint_ids,
                          const std::vector<unsigned int> &positions);

    bool getJointSpeed(const std::vector<unsigned int> &joint_ids,
                       std::vector<unsigned int> &speeds);
    bool setJointSpeed(const std::vector<unsigned int> &joint_ids,
                       const std::vector<unsigned int> &speeds);

    bool getJointAccceleration(const std::vector<unsigned int> &joint_ids,
                               std::vector<unsigned int> &accelerations);
    bool setJointAcceleration(const std::vector<unsigned int> &joint_ids,
                              const std::vector<unsigned int> &accelerations);

    //====================================================================//
    // These function modify EEPROM data which persists after power cycle //
    //====================================================================//

    // These can be left until the end as they are relatively unimportant
    bool setID(const uint8_t &old_id, const uint8_t &new_id);
    bool setBaudrate(const uint8_t &servo_id, const long &baudrate);

    bool setReturnDelayTime(const uint8_t &servo_id, const unsigned int &delay_time);
    bool setAngleLimits(const uint8_t &servo_id, const unsigned int &min_limit, const unsigned int &max_limit);
    bool setVoltageLimits(const uint8_t &servo_id, const VoltageLimits &voltage_limits);
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
    bool getAngleLimits(const int &servo_id, HansCuteRobot::AngleLimits &angle_limit);

    bool getVoltageLimits();
    bool getPosition(const int &servo_id, unsigned int &position);
    bool getSpeed(const int &servo_id, unsigned int &speed);
    bool getAcceleration(const int &servo_id, unsigned int &acceleration);

    bool getVoltage();
    bool getCurrent();
    bool getLock(const int &servo_id, bool &lock);

    bool getFeedback(const int &servo_id, ServoFeedback &feedback);
  };
}; // namespace HansCuteRobot

#endif