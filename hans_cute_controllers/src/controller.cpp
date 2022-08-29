#include "hans_cute_controllers/controller.h"

namespace HansCuteController
{
  Controller::Controller(const std::shared_ptr<SerialCommandRobot> &servo_driver_ptr,
                         const std::string &controller_namespace, const std::string &port_namespace)
      : robot_driver_ptr_(servo_driver_ptr),
        controller_namespace_(controller_namespace),
        port_namespace_(port_namespace),
        running_(false),
        joint_names_(std::vector<std::string>()),
        joint_speeds_(std::vector<unsigned int>()),
        compliance_slopes_(std::vector<unsigned int>()),
        compliance_margins_(std::vector<unsigned int>()),
        compliance_punches_(std::vector<unsigned int>()),
        torque_limits_(std::vector<double>())
  {
    // For now, assume that the ids range from 0 to 6 - hardcoded
    for (unsigned int motor_id = 0; motor_id < 6; motor_id++)
    {
      joint_ids_.push_back(motor_id);
    }
  }

  Controller::~Controller()
  {
  }

  void Controller::initialise()
  {
  }

  void Controller::start()
  {
    running_ = true;
  }

  void Controller::stop()
  {
  }

  void Controller::setJointIds(const std::vector<unsigned int> &joint_ids)
  {
    joint_ids_ = joint_ids;
  }

  void Controller::setServoParams(const std::vector<ServoParams> &servo_params)
  {
    joint_params_ = servo_params;
  }

  void Controller::setJointNames(const std::vector<std::string> &joint_names)
  {
    joint_names_ = joint_names;
  }
  void Controller::setJointSpeeds(const std::vector<unsigned int> &joint_speeds)
  {
    joint_speeds_ = joint_speeds;
    robot_driver_ptr_->setJointSpeed(joint_ids_, joint_speeds_);
  }
} // namespace HansCuteController
