#ifndef _STATUS_MANAGER_DATATYPE_H_
#define _STATUS_MANAGER_DATATYPE_H_

#include <iostream>
#include <string>
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

  friend std::ostream& operator<<(std::ostream& stream, const ServoParams& param)
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
#endif