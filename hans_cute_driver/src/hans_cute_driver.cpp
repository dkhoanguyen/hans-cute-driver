#include "hans_cute_driver/hans_cute_driver.hpp"

namespace HansCuteRobot
{
  HansCuteDriver::HansCuteDriver()
  {
    // Initialise Servo Params with default values
    // Assuming the hans doesn't change
    for (unsigned int joint_id = 0; joint_id <= 6; joint_id++)
    {
      ServoParams joint_params;
      joint_params.id = joint_id;
      joint_params.joint_name = "joint_" + std::to_string(joint_id);
      joint_params.raw_origin = 2048;
      joint_params.raw_min = 342;
      joint_params.raw_max = 3754;
      joint_params.speed = 300;
      joint_params.acceleration = 20;

      servo_params_[joint_id] = joint_params;
    }
    // Gripper
    gripper_params_.id = 7;
    gripper_params_.joint_name = "gripper";
    gripper_params_.raw_origin = 300;
    gripper_params_.raw_min = 200;
    gripper_params_.raw_max = 500;
    gripper_params_.speed = 300;
  }

  HansCuteDriver::~HansCuteDriver()
  {
    // Try to close port 5 times
    for (unsigned int num = 0; num < 5; num++)
    {
      if (servo_comms_.close())
      {
        break;
      }
    }
  }

  bool HansCuteDriver::setJointLimits(
      const std::string &current_joint_name,
      const std::string &new_joint_name,
      const unsigned int &raw_min,
      const unsigned int &raw_max,
      const unsigned int &raw_origin,
      const unsigned int &max_speed,
      const unsigned int &max_acceleration)
  {
    // Find the name and update the param
    unsigned int target_joint_id = -1;
    for (auto it = servo_params_.begin(); it != servo_params_.end(); ++it)
    {
      if (strcmp(current_joint_name.c_str(), it->second.joint_name.c_str()) == 0)
      {
        target_joint_id = it->first;
        break;
      }
    }

    if (target_joint_id == -1)
    {
      return false;
    }

    ServoParams params;
    params.joint_name = new_joint_name;
    params.id = target_joint_id;
    params.raw_min = raw_min;
    params.raw_max = raw_max;
    params.raw_origin = raw_origin;
    params.speed = max_speed;
    params.acceleration = max_acceleration;

    if (findServo(target_joint_id))
    {
      updateServo(target_joint_id, params);
      servo_params_[target_joint_id] = params;
      return true;
    }
    return false;
  }

  bool HansCuteDriver::init(const std::string &port)
  {
    // Try to open port 5 times
    bool port_opened = false;
    for (unsigned int num = 0; num < 5; num++)
    {
      if (servo_comms_.open(port))
      {
        port_opened = true;
        break;
      }
    }
    if (!port_opened)
    {
      return false;
    }
  }

  bool HansCuteDriver::start()
  {
    // Find servo based on params
    for (auto it : servo_params_)
    {
      unsigned int joint_id = it.first;
      if (findServo(joint_id))
      {
        // We found the current servo
        // let's load parameters
        // Angle limits
        servo_comms_.setAngleLimits(
            joint_id, servo_params_[joint_id].raw_min, servo_params_[joint_id].raw_max);
        // Max speed
        servo_comms_.setSpeed(joint_id, servo_params_[joint_id].speed);
        // Max acceleration
        servo_comms_.setAcceleration(joint_id, servo_params_[joint_id].acceleration);

        // Enable torque
        if (!servo_comms_.setTorqueEnable(joint_id, true))
        {
          return false;
        }
        unsigned int position = 0;
        servo_comms_.getPosition(it.second.id, position);
        servo_comms_.setPosition(it.second.id, position);
      }
      else
      {
        return false;
      }
    }

    // Find gripper
    if (findServo(gripper_params_.id))
    {
      servo_comms_.setAngleLimits(
          gripper_params_.id, gripper_params_.raw_min, gripper_params_.raw_max);
      servo_comms_.setSpeed(gripper_params_.id, gripper_params_.speed);
      servo_comms_.setAcceleration(gripper_params_.id, gripper_params_.acceleration);
      if (!servo_comms_.setTorqueEnable(gripper_params_.id, true))
      {
        return false;
      }
      unsigned int position = 0;
      servo_comms_.getPosition(gripper_params_.id, position);
      servo_comms_.setPosition(gripper_params_.id, position);
    }
    return true;
  }

  bool HansCuteDriver::halt()
  {
    for (auto it = servo_params_.begin(); it != servo_params_.end(); ++it)
    {
      unsigned int joint_id = it->first;
      if (!servo_comms_.setTorqueEnable(joint_id, false))
      {
        return false;
      }
    }
    return true;
  }

  bool HansCuteDriver::isHalted()
  {
    std::vector<uint8_t> response;
    for (auto it = servo_params_.begin(); it != servo_params_.end(); ++it)
    {
      unsigned int joint_id = it->first;
      bool enabled = false;
      if (!servo_comms_.getTorqueEnabled(joint_id, enabled) || enabled)
      {
        return false;
      }
    }
    return true;
  }

  bool HansCuteDriver::getJointStates(
      std::unordered_map<std::string, double> &joint_states)
  {
    joint_states.clear();
    for (auto it = servo_params_.begin(); it != servo_params_.end(); ++it)
    {
      unsigned int raw_position = 0;
      unsigned int joint_id = it->first;
      joint_states[it->second.joint_name] = 0.0;
      if (!servo_comms_.getPosition(joint_id, raw_position))
      {
        return false;
      }
      double position = 0.0;
      posRawToRad(position, raw_position, it->second);
      joint_states[it->second.joint_name] = position;
    }
    return true;
  }

  bool HansCuteDriver::findServo(const unsigned int &servo_id)
  {
    unsigned int num_retries = 5;
    for (int ping_try = 0; ping_try < num_retries; ping_try++)
    {
      if (ping_try == num_retries)
      {
        return false;
      }

      // We should wrap this ping function in driver maybe
      std::vector<uint8_t> response;
      if (!servo_comms_.ping((uint8_t)servo_id, response) ||
          response.size() == 0)
      {
        continue;
      }
      break;
    }
    return true;
  }

  bool HansCuteDriver::updateServo(const unsigned int &servo_id,
                                   ServoParams &output_servo_param)
  {
    unsigned int model_number = 0;
    if (!servo_comms_.getModelNumber(servo_id, model_number))
    {
      return false;
    }
    output_servo_param.model_number = model_number;

    ServoModel model = ModelToParams.at(model_number);

    double range_radians = model.range_degrees * (M_PI / 180);
    double rad_per_enc_tick = range_radians / model.encoder_resolution;
    double enc_tick_per_rad = model.encoder_resolution / range_radians;

    // Radians
    output_servo_param.min = (output_servo_param.raw_min - output_servo_param.raw_origin) * rad_per_enc_tick;
    output_servo_param.max = (output_servo_param.raw_max - output_servo_param.raw_origin) * rad_per_enc_tick;

    output_servo_param.rad_per_enc_tick = rad_per_enc_tick;
    output_servo_param.enc_tick_per_rad = enc_tick_per_rad;

    output_servo_param.rpm_per_tick = model.rpm_per_tick;
    output_servo_param.radians_second_per_encoder_tick = model.rpm_per_tick * RPM_TO_RADSEC;

    return true;
  }

  // Status

  bool HansCuteDriver::getJointNames(std::vector<std::string> &joint_names)
  {
    joint_names.clear();
    for (auto param : servo_params_)
    {
      // Ignore gripper
      if (param.second.joint_name == "gripper")
      {
        continue;
      }
      joint_names.push_back(param.second.joint_name);
    }
    return true;
  }

  // Control
  bool HansCuteDriver::setJointPTP(const std::unordered_map<std::string, double> &joint_pos,
                                   const double &vel_per,
                                   const double &accel_per)
  {
    unsigned int vel = 1023 * vel_per;
    unsigned int accel = 253 * accel_per;
    for (auto joint_pos_it = joint_pos.begin(); joint_pos_it != joint_pos.end(); ++joint_pos_it)
    {
      std::string joint_name = joint_pos_it->first;
      // Find joint id based on name
      unsigned int joint_id = -1;
      for (auto it = servo_params_.begin(); it != servo_params_.end(); ++it)
      {
        if (strcmp(joint_name.c_str(), it->second.joint_name.c_str()) == 0)
        {
          joint_id = it->first;
          break;
        }
      }
      // Max speed
      servo_comms_.setSpeed(joint_id, vel);
      // Max acceleration
      servo_comms_.setAcceleration(joint_id, accel);
      // Send command
      double position = joint_pos.at(joint_name);
      unsigned int raw_position = 0;
      posRadToRaw(joint_pos.at(joint_name), raw_position, servo_params_.at(joint_id));
      // This should be updated to sync write to ensure all motors receive commands at the same time
      if (!servo_comms_.setPosition(joint_id, raw_position))
      {
        return false;
      }
    }
    servo_comms_.setPosition(7, 200);
    return true;
  }

  bool HansCuteDriver::setJointPVT(const std::unordered_map<std::string, double> &joint_pos,
                                   const std::unordered_map<std::string, double> &joint_vel)
  {
    // New temp joint id -> position map to ensure all joints move at roughly the same tim
    std::unordered_map<unsigned int, unsigned int> raw_joint_pos;
    // Configure steps
    for (auto joint_pos_it = joint_pos.begin(); joint_pos_it != joint_pos.end(); ++joint_pos_it)
    {
      std::string joint_name = joint_pos_it->first;
      // Find joint id based on name
      unsigned int joint_id = -1;
      for (auto it = servo_params_.begin(); it != servo_params_.end(); ++it)
      {
        if (strcmp(joint_name.c_str(), it->second.joint_name.c_str()) == 0)
        {
          joint_id = it->first;
          break;
        }
      }
      double goal_position_rad = joint_pos.at(joint_name);
      unsigned int goal_position_raw = 0;
      posRadToRaw(goal_position_rad, goal_position_raw, servo_params_[joint_id]);

      // Obtain raw_velocity
      double goal_vel_rad = joint_vel.at(joint_name);
      unsigned int goal_vel_raw = 0;
      spdRadToRaw(goal_vel_rad, goal_vel_raw, servo_params_.at(joint_id));
      servo_comms_.setSpeed(joint_id, goal_vel_raw);
      // Set max accel
      servo_comms_.setAcceleration(joint_id, 254);
      // Send command
      raw_joint_pos[joint_id] = goal_position_raw;
    }
    for (auto raw_joint : raw_joint_pos)
    {
      // This should be updated to sync write to ensure all motors receive commands at the same time
      if (!servo_comms_.setPosition(raw_joint.first, raw_joint.second))
      {
        return false;
      }
    }
    return true;
  }

  bool HansCuteDriver::setGripperCommand(const double &pos)
  {
    // Clamp input
    double input = pos;
    if (pos > 0.0285)
    {
      input = 0.0285;
    }
    else if (pos < 0.0)
    {
      input = 0.0;
    }
    unsigned int raw_pos = 200;
    // Linear interpolation
    double input_range = 0.0285 - 0.0;
    double output_range = 500 - 200;
    raw_pos = 200 + (unsigned int)(((input - 0.0) / input_range) * output_range);
    if (!servo_comms_.setPosition(gripper_params_.id, raw_pos))
    {
      return false;
    }
    return true;
  }

  bool HansCuteDriver::getGripperPos(double &pos)
  {
    unsigned int raw_pos = 0;
    if (!servo_comms_.getPosition(gripper_params_.id, raw_pos))
    {
      return false;
    }
    double input_range = 500 - 200;
    double output_range = 0.0285 - 0.0;
    pos = 0.0 + (double)(((raw_pos - 200) / input_range) * output_range);
    return true;
  }

  void HansCuteDriver::posRadToRaw(
      const double &rad, unsigned int &raw, const ServoParams &params)
  {
    raw = (unsigned int)round(params.raw_origin + (rad * params.enc_tick_per_rad));
  }

  void HansCuteDriver::posRawToRad(
      double &rad, const unsigned int &raw, const ServoParams &params)
  {
    rad = ((int)(raw - params.raw_origin)) * params.rad_per_enc_tick;
  }

  void HansCuteDriver::spdRadToRaw(
      const double &rad, unsigned int &raw, const ServoParams &params)
  {
    raw = (unsigned int)round(rad / params.radians_second_per_encoder_tick);
  }

  void HansCuteDriver::spdRawToRad(
      double &rad, const unsigned int &raw, const ServoParams &params)
  {
    rad = raw * params.radians_second_per_encoder_tick;
  }
}