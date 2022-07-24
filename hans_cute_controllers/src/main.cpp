
#include "hans_cute_driver/hans_cute_driver.h"
#include "hans_cute_driver/hans_cute_const.h"

#include "hans_cute_status_manager/hans_cute_status_manager.h"
#include "hans_cute_controllers/joint_position_controller.h"

int main()
{
  // Shared pointer for hardware driver
  std::shared_ptr<HansCuteRobot::ServoDriver> servo_driver_ptr =
      std::make_shared<HansCuteRobot::ServoDriver>("/dev/ttyUSB0", 250000);
  servo_driver_ptr->open();

  // Common params
  std::string controller_namespace = "";
  std::string port_namespace = "ttyUSB0";

  // Start status manager
  std::shared_ptr<HansCuteStatusManager> status_manager = std::make_shared<HansCuteStatusManager>();
  status_manager->setServoDriver(servo_driver_ptr);
  status_manager->initialise();

  // Start controller
  std::shared_ptr<HansCuteController::JointPositionController> joint_position_controller =
      std::make_shared<HansCuteController::JointPositionController>(servo_driver_ptr,
                                                                    controller_namespace,
                                                                    port_namespace);

  // Joint Names
  std::vector<std::string> joint_names;
  joint_names.push_back("joint_0");
  joint_names.push_back("joint_1");
  joint_names.push_back("joint_2");
  joint_names.push_back("joint_3");
  joint_names.push_back("joint_4");
  joint_names.push_back("joint_5");
  joint_names.push_back("joint_6");

  joint_position_controller->setJointNames(joint_names);

  // Joint IDS
  std::vector<unsigned int> joint_ids;
  for (unsigned int id = 0; id <= 6 ; id++) 
  {
    joint_ids.push_back(id);
  }

  joint_position_controller->setJointIds(joint_ids);

  // Joint Models
  // This requires status manager

  return 0;
}