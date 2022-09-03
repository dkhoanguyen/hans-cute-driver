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
    robot_driver_ptr_->setJointPosition(raw_positions);
  }
}