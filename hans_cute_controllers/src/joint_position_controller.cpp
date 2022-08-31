#include "hans_cute_controllers/joint_position_controller.h"

namespace HansCuteController
{
  JointPositionController::JointPositionController(const std::shared_ptr<SerialCommandRobotInterface> &robot_driver_ptr,
                                                   const std::string &controller_namespace, const std::string &port_namespace)
      : Controller(robot_driver_ptr, controller_namespace, port_namespace)
  {
  }

  JointPositionController::~JointPositionController()
  {
  }

  void JointPositionController::initialise()
  {
    std::vector<double> positions;
    robot_driver_ptr_->getJointPosition(positions);
    robot_driver_ptr_->setJointPosition(positions);
  }

  void JointPositionController::start()
  {
    for (unsigned int joint_id : joint_ids_)
    {
      // robot_driver_ptr_->setTorqueEnable(joint_id, true);
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

    std::vector<double> processed_data;
    std::vector<unsigned int> speeds;
    std::vector<unsigned int> accelerations;
    for (unsigned int idx = 0; idx < raw_positions.size(); idx++)
    {
      double processed_pos = 0.0;
      // posRadToRaw(raw_positions.at(idx), processed_pos, joint_params_.at(idx));
      processed_data.push_back(processed_pos);
      speeds.push_back(300);
      accelerations.push_back(30);
    }
    robot_driver_ptr_->setJointSpeed(speeds);
    robot_driver_ptr_->setJointAcceleration(accelerations);
    robot_driver_ptr_->setJointPosition(processed_data);
  }
}