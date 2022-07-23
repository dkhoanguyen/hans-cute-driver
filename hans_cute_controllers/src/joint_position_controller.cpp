#include "hans_cute_controllers/joint_position_controller.h"

namespace HansCuteController
{
  JointPositionController::JointPositionController(const std::shared_ptr<HansCuteRobot::ServoDriver> &servo_driver_ptr,
                                                   const std::string &controller_namespace, const std::string &port_namespace)
      : Controller(servo_driver_ptr, controller_namespace, port_namespace)
  {
  }

  JointPositionController::~JointPositionController()
  {
  }

  void JointPositionController::initialise()
  {
    // Configure angle sweep
    joint_params_.clear();
    for (unsigned int idx = 0; idx < joint_ids_.size(); idx++)
    {
      HansCuteRobot::AngleLimits angle_lims;
      servo_driver_ptr_->getAngleLimits(joint_ids_.at(idx), angle_lims);
      HansCuteRobot::ServoModel model = models_.at(idx);
      double range_radians = model.range_degrees * (M_PI / 180);
      double rad_per_enc_tick = range_radians / model.encoder_resolution;
      double enc_tick_per_rad = model.encoder_resolution / range_radians;

      JointParams joint_params;
      // Raw values
      joint_params.raw_min = angle_lims.min;
      joint_params.raw_max = angle_lims.max;
      joint_params.raw_origin = angle_lims.min + (angle_lims.max - angle_lims.min) / 2;

      // Radians
      joint_params.min = (joint_params.raw_min - joint_params.raw_origin) * rad_per_enc_tick;
      joint_params.max = (joint_params.raw_min - joint_params.raw_origin) * rad_per_enc_tick;
      joint_params_.push_back(joint_params);

      // Utils params
      joint_params.rad_per_enc_tick = rad_per_enc_tick;
      joint_params.enc_tick_per_rad = enc_tick_per_rad;
    }
  }

  void JointPositionController::start()
  {
  }

  void JointPositionController::stop()
  {
  }

  void JointPositionController::processCommand(Data &data)
  {
    // First maybe convert the data
    std::vector<double> raw_positions;
    data.get(raw_positions);

    std::vector<unsigned int> processed_data;
    for (unsigned int idx = 0; idx < raw_positions.size(); idx++)
    {
      unsigned int processed_pos = 2048;
      posRadToRaw(raw_positions.at(idx), processed_pos, joint_params_.at(idx));
      processed_data.push_back(processed_pos);
    }
    servo_driver_ptr_->setMultiPosition(joint_ids_, processed_data);
  }

  void JointPositionController::posRadToRaw(const double &rad, unsigned int &raw, const JointParams &params)
  {
    raw = (unsigned int)round(params.raw_origin + (rad * params.enc_tick_per_rad));
  }
}