#include "hans_cute_driver/hans_cute_robot.h"

HansCuteRobot::HansCuteRobot::HansCuteRobot(const std::string &port_name, const std::string &port_namespace,
                                            const long &baud_rate, const unsigned int &min_joint_id,
                                            const unsigned int &max_joint_id, const unsigned int &gripper_id)
    : port_name_(port_name),
      port_namespace_(port_namespace),
      baud_rate_(baud_rate),
      min_joint_id_(min_joint_id),
      max_joint_id_(max_joint_id),
      gripper_id_(gripper_id),
      running_(false),
      robot_driver_ptr_(std::make_shared<ServoDriver>()),
      SerialCommandRobotInterface()
{
}

HansCuteRobot::HansCuteRobot::~HansCuteRobot()
{
}

void HansCuteRobot::HansCuteRobot::setSerialPort(const std::shared_ptr<SerialPortInterface> &serial_port)
{
  robot_driver_ptr_->setSerialPort(serial_port);
}

bool HansCuteRobot::HansCuteRobot::initialise()
{
  findServos();
  // Set Speed and Acceleration
  for (unsigned int id = 0; id < joint_ids_.size(); id++)
  {
    robot_driver_ptr_->setAcceleration(joint_ids_.at(id), servos_params_.at(id).acceleration);
    robot_driver_ptr_->setSpeed(joint_ids_.at(id), servos_params_.at(id).speed);
  }
}

bool HansCuteRobot::HansCuteRobot::start()
{
  running_ = true;
  for (const unsigned int id : joint_ids_)
  {
    robot_driver_ptr_->setTorqueEnable(id, true);
  }
}

bool HansCuteRobot::HansCuteRobot::stop()
{
  running_ = false;
}

void HansCuteRobot::HansCuteRobot::updateJointParams(const std::vector<ServoParams> &servo_params)
{
  for (unsigned int id = 0; id < servo_params.size(); id++)
  {
    servos_params_.at(id).joint_name = servo_params.at(id).joint_name;
    servos_params_.at(id).raw_min = servo_params.at(id).raw_min;
    servos_params_.at(id).raw_max = servo_params.at(id).raw_max;
    servos_params_.at(id).raw_origin = servo_params.at(id).raw_origin;

    servos_params_.at(id).speed = servo_params.at(id).speed;
    servos_params_.at(id).acceleration = servo_params.at(id).acceleration;

    // Update hardware with new values
    robot_driver_ptr_->setAngleLimits(joint_ids_.at(id), servos_params_.at(id).raw_min, servos_params_.at(id).raw_max);
    robot_driver_ptr_->setSpeed(joint_ids_.at(id), servos_params_.at(id).speed);
    robot_driver_ptr_->setAcceleration(joint_ids_.at(id), servos_params_.at(id).acceleration);
  }
}

bool HansCuteRobot::HansCuteRobot::getJointPosition(std::vector<double> &positions)
{
  positions.clear();
  for (unsigned int idx = 0; idx < joint_ids_.size(); idx++)
  {
    // Obtain the raw position
    unsigned int raw_position = 0;
    robot_driver_ptr_->getPosition(joint_ids_.at(idx), raw_position);

    // Convert raw position to radian
    double position = 0.0;
    posRawToRad(position, raw_position, servos_params_.at(idx));
    positions.push_back(position);
  }
  return true;
}

bool HansCuteRobot::HansCuteRobot::setJointPosition(const std::vector<double> &positions)
{
  for (unsigned int idx = 0; idx < joint_ids_.size(); idx++)
  {
    unsigned int processed_pos = 2048;
    posRadToRaw(positions.at(idx), processed_pos, servos_params_.at(idx));

    robot_driver_ptr_->setSpeed(joint_ids_.at(idx), servos_params_.at(idx).speed);
    robot_driver_ptr_->setAcceleration(joint_ids_.at(idx), servos_params_.at(idx).acceleration);
    robot_driver_ptr_->setPosition(joint_ids_.at(idx), processed_pos);
  }
  return true;
}

bool HansCuteRobot::HansCuteRobot::getJointVelocity(std::vector<double> &velocities)
{
  velocities.clear();
  std::vector<unsigned int> raw_speeds;
  getJointRawSpeed(raw_speeds);

  for (unsigned int idx = 0; idx < joint_ids_.size(); idx++)
  {
    double velocity = 0;
    spdRawToRad(velocity, raw_speeds.at(idx), servos_params_.at(idx));
    velocities.push_back(velocity);
  }

  return true;
}

bool HansCuteRobot::HansCuteRobot::setJointVelocity(const std::vector<double> &velocities)
{
  return true;
}

bool HansCuteRobot::HansCuteRobot::getJointEffort(std::vector<double> &efforts)
{
  return true;
}

bool HansCuteRobot::HansCuteRobot::setJointEffort(const std::vector<double> &efforts)
{
  return true;
}

bool HansCuteRobot::HansCuteRobot::setGripperState(const unsigned int &state)
{
  unsigned int gripper_pose = state;
  if(state > gripper_params_.raw_max)
  {
    gripper_pose = gripper_params_.raw_max;
  }
  else if (state < gripper_params_.raw_min)
  {
    gripper_pose = gripper_params_.raw_min;
  }
  robot_driver_ptr_->setPosition(gripper_id_,gripper_pose);
  return true;
}

bool HansCuteRobot::HansCuteRobot::getGripperState(unsigned int &state)
{
  robot_driver_ptr_->setPosition(gripper_id_,state);
  return true;
}

//
bool HansCuteRobot::HansCuteRobot::getJointRawSpeed(std::vector<unsigned int> &speeds)
{
  speeds.clear();
  for (const unsigned int id : joint_ids_)
  {
    unsigned int speed = 0;
    robot_driver_ptr_->getSpeed(id, speed);
    speeds.push_back(speed);
  }

  return true;
}
bool HansCuteRobot::HansCuteRobot::setJointRawSpeed(const std::vector<unsigned int> &speeds)
{
  for (const unsigned int id : joint_ids_)
  {
    robot_driver_ptr_->setSpeed(joint_ids_.at(id), speeds.at(id));
  }
  return true;
}

bool HansCuteRobot::HansCuteRobot::getJointRawAccceleration(std::vector<unsigned int> &accelerations)
{
  accelerations.clear();
  for (const unsigned int id : joint_ids_)
  {
    unsigned int acceleration = 0;
    robot_driver_ptr_->getAcceleration(id, acceleration);
    accelerations.push_back(acceleration);
  }
  return true;
}
bool HansCuteRobot::HansCuteRobot::setJointRawAcceleration(const std::vector<unsigned int> &accelerations)
{
  for (unsigned int id = 0; id < joint_ids_.size(); id++)
  {
    robot_driver_ptr_->setAcceleration(joint_ids_.at(id), accelerations.at(id));
  }
  return true;
}

bool HansCuteRobot::HansCuteRobot::findServos()
{
  std::vector<unsigned int> motor_ids_list;
  unsigned int num_retries = 5;
  servos_params_.clear();
  joint_ids_.clear();
  for (int servo_id = min_joint_id_; servo_id <= max_joint_id_; servo_id++)
  {
    for (int ping_try = 1; ping_try <= num_retries; ping_try++)
    {
      if (ping_try == num_retries)
      {
        std::cout << "Failed to ping motor: " << servo_id << std::endl;
        break;
      }

      // We should wrap this ping function in driver maybe
      std::vector<uint8_t> response;
      robot_driver_ptr_->ping((uint8_t)servo_id, response);
      if (response.size() == 0)
      {
        continue;
      }

      // IF we can ping this servo then, we should be able to retrieve the servo params
      std::cout << "Found servo " << servo_id << std::endl;
      joint_ids_.push_back(servo_id);
      bool servo_params_filled = false;
      for (int query_param_try = 1; query_param_try <= num_retries; query_param_try++)
      {
        if (query_param_try == num_retries)
        {
          std::cout << "Unable to retrieve servo params for servo after 5 tries: " << servo_id << std::endl;
          break;
        }

        unsigned int model_number = 0;
        if (!robot_driver_ptr_->getModelNumber(servo_id, model_number))
        {
          std::cout << "Unable to retrieve servo params for servo: " << servo_id << std::endl;
          continue;
        }

        ServoParams servo_param;
        fillServoParams(servo_id, model_number,servo_param);
        servos_params_.push_back(servo_param);
        servo_params_filled = true;
        break;
      }

      // Only add to id list if we can retrieve params of this servo
      if (servo_params_filled)
      {
        motor_ids_list.push_back(servo_id);
      }

      break;
    }
  }

  // Find gripper
  for (int ping_try = 1; ping_try <= num_retries; ping_try++)
  {
    if (ping_try == num_retries)
    {
      std::cout << "Failed to ping motor: " << gripper_id_ << std::endl;
      break;
    }

    // We should wrap this ping function in driver maybe
    std::vector<uint8_t> response;
    robot_driver_ptr_->ping((uint8_t)gripper_id_, response);
    if (response.size() == 0)
    {
      continue;
    }

    std::cout << "Found servo " << gripper_id_ << std::endl;
    unsigned int model_number = 0;
    if (!robot_driver_ptr_->getModelNumber(gripper_id_, model_number))
    {
      std::cout << "Unable to retrieve servo params for servo: " << gripper_id_ << std::endl;
      continue;
    }
    fillServoParams(gripper_id_, model_number,gripper_params_);
    break;
  }
  return true;
}

bool HansCuteRobot::HansCuteRobot::fillServoParams(const unsigned int &servo_id, const unsigned int &model_number, ServoParams &servo_param)
{
  servo_param.id = servo_id;
  servo_param.model_number = model_number;

  // Joint Limits
  AngleLimits angle_lims;
  if (!robot_driver_ptr_->getAngleLimits(servo_id, angle_lims))
  {
    return false;
  }
  ServoModel model = ModelToParams.at(model_number);

  double range_radians = model.range_degrees * (M_PI / 180);
  double rad_per_enc_tick = range_radians / model.encoder_resolution;
  double enc_tick_per_rad = model.encoder_resolution / range_radians;

  servo_param.raw_min = angle_lims.min;
  servo_param.raw_max = angle_lims.max;
  servo_param.raw_origin = angle_lims.min + (angle_lims.max - angle_lims.min) / 2;

  // Radians
  servo_param.min = (servo_param.raw_min - servo_param.raw_origin) * rad_per_enc_tick;
  servo_param.max = (servo_param.raw_max - servo_param.raw_origin) * rad_per_enc_tick;

  servo_param.rad_per_enc_tick = rad_per_enc_tick;
  servo_param.enc_tick_per_rad = enc_tick_per_rad;

  servo_param.rpm_per_tick = model.rpm_per_tick;
  servo_param.radians_second_per_encoder_tick = model.rpm_per_tick * RPM_TO_RADSEC;

  servo_param.speed = SERVO_DEFAULT_SPEED;
  servo_param.acceleration = SERVO_DEFAULT_ACCELERATION;
  return true;
}

void HansCuteRobot::HansCuteRobot::posRadToRaw(const double &rad, unsigned int &raw, const ServoParams &params)
{
  raw = (unsigned int)round(params.raw_origin + (rad * params.enc_tick_per_rad));
}

void HansCuteRobot::HansCuteRobot::posRawToRad(double &rad, const unsigned int &raw, const ServoParams &params)
{
  rad = ((int)(raw - params.raw_origin)) * params.rad_per_enc_tick;
}

void HansCuteRobot::HansCuteRobot::spdRadToRaw(const double &rad, unsigned int &raw, const ServoParams &params)
{
  raw = (unsigned int)round(raw / params.radians_second_per_encoder_tick);
}

void HansCuteRobot::HansCuteRobot::spdRawToRad(double &rad, const unsigned int &raw, const ServoParams &params)
{
  rad = raw * params.radians_second_per_encoder_tick;
}