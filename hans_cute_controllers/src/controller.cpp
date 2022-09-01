#include "hans_cute_controllers/controller.h"

namespace HansCuteController
{
  Controller::Controller(const std::shared_ptr<SerialCommandRobotInterface> &robot_driver_ptr,
                         const std::string &controller_namespace, const std::string &port_namespace)
      : robot_driver_ptr_(robot_driver_ptr),
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

  // void Controller::setServoParams(const std::vector<ServoParams> &servo_params)
  // {
  //   joint_params_ = servo_params;
  // }

  void Controller::setJointNames(const std::vector<std::string> &joint_names)
  {
    joint_names_ = joint_names;
  }
  void Controller::setJointSpeeds(const std::vector<unsigned int> &joint_speeds)
  {
    joint_speeds_ = joint_speeds;
  }
} // namespace HansCuteController
