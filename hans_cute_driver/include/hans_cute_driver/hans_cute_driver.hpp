#ifndef HANS_CUTE_DRIVER__HANS_CUTE_DRIVER_HPP_
#define HANS_CUTE_DRIVER__HANS_CUTE_DRIVER_HPP_

#include <unordered_map>
#include "servo_communication.hpp"

namespace HansCuteRobot
{
  class HansCuteDriver
  {
  public:
    HansCuteDriver();
    virtual ~HansCuteDriver();

    bool setJointLimits(
        const std::string &current_joint_name,
        const std::string &new_joint_name,
        const unsigned int &raw_min,
        const unsigned int &raw_max,
        const unsigned int &raw_origin,
        const unsigned int &max_speed,
        const unsigned int &max_acceleration);

    bool init(const std::string &port);
    bool start();
    bool halt();

    // Status check
    bool isHalted();
    bool getJointStates(std::unordered_map<std::string, double> &joint_states);

    // Control
    bool setJointPTP(const std::unordered_map<std::string, double> &joint_pos,
                     const double &vel_per,
                     const double &accel_per);
    // Terminology borrowed from the TM Robots
    // PVT - Point Velocity Time
    bool setJointPVT(const std::unordered_map<std::string, double> &joint_pos,
                     const std::unordered_map<std::string, double> &joint_vel);

  protected:
    ServoComms servo_comms_;
    std::unordered_map<unsigned int, ServoParams> servo_params_;

    bool findServo(const unsigned int &servo_id);
    bool updateServo(const unsigned int &servo_id,
                     ServoParams &output_servo_param);

    void posRadToRaw(
        const double &rad, unsigned int &raw, const ServoParams &params);
    void posRawToRad(
        double &rad, const unsigned int &raw, const ServoParams &params);
    void spdRadToRaw(
        const double &rad, unsigned int &raw, const ServoParams &params);
    void spdRawToRad(
        double &rad, const unsigned int &raw, const ServoParams &params);
  };
}; // namespace HansCuteRobot

#endif