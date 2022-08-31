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
    for (unsigned int joint_id : joint_ids_)
    {
      unsigned int position = 0;
      servo_driver_ptr_->getPosition(joint_id, position);
      servo_driver_ptr_->setPosition(joint_id, position);
    }
  }

  void JointPositionController::start()
  {
    for (unsigned int joint_id : joint_ids_)
    {
      servo_driver_ptr_->setTorqueEnable(joint_id, true);
    }
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

      servo_driver_ptr_->setAcceleration((uint8_t)joint_ids_.at(idx), 20);
      servo_driver_ptr_->setSpeed((uint8_t)joint_ids_.at(idx), 300);
      servo_driver_ptr_->setPosition((uint8_t)joint_ids_.at(idx), processed_pos);
    }
  }

  void JointPositionController::posRadToRaw(const double &rad, unsigned int &raw, const ServoParams &params)
  {
    raw = (unsigned int)round(params.raw_origin + (rad * params.enc_tick_per_rad));
  }
}