
#include "hans_cute_status_manager/hans_cute_status_manager.h"

int main()
{
  std::shared_ptr<HansCuteRobot::ServoDriver> servo_driver_ptr =
      std::make_shared<HansCuteRobot::ServoDriver>("/dev/ttyUSB0", 250000);
  servo_driver_ptr->open();
  
  // Start status manager
  std::shared_ptr<HansCuteStatusManager> status_manager = std::make_shared<HansCuteStatusManager>();
  status_manager->setServoDriver(servo_driver_ptr);

  status_manager->initialise();

  return 0;
}