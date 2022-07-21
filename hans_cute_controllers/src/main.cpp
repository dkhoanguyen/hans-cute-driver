
#include "hans_cute_driver/hans_cute_driver.h"
#include "hans_cute_driver/hans_cute_const.h"
#include "hans_cute_controllers/joint_position_controller.h"

int main()
{
  std::shared_ptr<HansCuteRobot::ServoDriver> servo_driver_ptr = std::make_shared<HansCuteRobot::ServoDriver>("/dev/ttyUSB0", 250000);
  return 0;
}