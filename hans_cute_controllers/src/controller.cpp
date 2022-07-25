#include "hans_cute_controllers/controller.h"

namespace HansCuteController
{
  Controller::Controller(const std::shared_ptr<HansCuteRobot::ServoDriver> &servo_driver_ptr,
                         const std::string &controller_namespace, const std::string &port_namespace)
      : servo_driver_ptr_(servo_driver_ptr),
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

    servo_driver_ptr_->setMultiSpeed(joint_ids_, joint_speeds_);
  }
  void Controller::setComplianceSlopes(const std::vector<unsigned int> &compliance_slopes)
  {
    compliance_slopes_.clear();
    for (const unsigned int compliance_slope : compliance_slopes)
    {
      if (compliance_slope < (unsigned int)HansCuteRobot::StaticParameter::MIN_COMPLIANCE_SLOPE)
      {
        compliance_slopes_.push_back((unsigned int)HansCuteRobot::StaticParameter::MIN_COMPLIANCE_SLOPE);
      }
      else if (compliance_slope > (unsigned int)HansCuteRobot::StaticParameter::MAX_COMPLIANCE_SLOPE)
      {
        compliance_slopes_.push_back((unsigned int)HansCuteRobot::StaticParameter::MAX_COMPLIANCE_SLOPE);
      }
      else
      {
        compliance_slopes_.push_back(compliance_slope);
      }
    }
    servo_driver_ptr_->setMultiComplianceSlope();
  }
  void Controller::setComplianceMargins(const std::vector<unsigned int> &compliance_margins)
  {
    compliance_margins_.clear();
    for (const unsigned int compliance_margin : compliance_margins)
    {
      if (compliance_margin < (unsigned int)HansCuteRobot::StaticParameter::MIN_COMPLIANCE_MARGIN)
      {
        compliance_margins_.push_back((unsigned int)HansCuteRobot::StaticParameter::MIN_COMPLIANCE_MARGIN);
      }
      else if (compliance_margin > (unsigned int)HansCuteRobot::StaticParameter::MAX_COMPLIANCE_MARGIN)
      {
        compliance_margins_.push_back((unsigned int)HansCuteRobot::StaticParameter::MAX_COMPLIANCE_MARGIN);
      }
      else
      {
        compliance_margins_.push_back(compliance_margin);
      }
    }
    servo_driver_ptr_->setMultiComplianceMargin();
  }
  void Controller::setCompliancePunches(const std::vector<unsigned int> &compliance_punches)
  {
    compliance_punches_.clear();
    for (const unsigned int compliance_punch : compliance_punches)
    {
      if (compliance_punch < (unsigned int)HansCuteRobot::UtilsParameter::MIN_PUNCH)
      {
        compliance_punches_.push_back((unsigned int)HansCuteRobot::UtilsParameter::MIN_PUNCH);
      }
      else if (compliance_punch > (unsigned int)HansCuteRobot::UtilsParameter::MAX_PUNCH)
      {
        compliance_punches_.push_back((unsigned int)HansCuteRobot::UtilsParameter::MAX_PUNCH);
      }
      else
      {
        compliance_punches_.push_back(compliance_punch);
      }
    }
  }
  void Controller::setTorqueLimits(const std::vector<double> &torque_limits)
  {
    torque_limits_.clear();
    for (const double torque_limit : torque_limits_)
    {
      if (torque_limit < (double)HansCuteRobot::StaticParameter::MIN_TORQUE)
      {
        torque_limits_.push_back((double)HansCuteRobot::StaticParameter::MIN_TORQUE);
      }
      else if (torque_limit > (double)HansCuteRobot::StaticParameter::MAX_TORQUE)
      {
        torque_limits_.push_back((double)HansCuteRobot::StaticParameter::MAX_TORQUE);
      }
      else
      {
        torque_limits_.push_back(torque_limit);
      }
    }

    servo_driver_ptr_->setMultiTorqueLimit(joint_ids_, torque_limits_);
  }

  void Controller::ensureLimits()
  {
  }

} // namespace HansCuteController
