/*
 * Allows to set arbitrary speed for the serial device on Linux.
 *
 */
#include <iostream>

#include "hans_cute_driver/hans_cute_driver.h"
#include "hans_cute_driver/hans_cute_datatype.h"

int main(int argc, char* argv[])
{
  HansCuteRobot::ServoDriver hans_robot("/dev/ttyUSB0", 250000);
  hans_robot.open();

  for (int i = 0; i <= 6; i++)
  {
    hans_robot.setTorqueEnable(i,true);
    unsigned int position = 0;
    hans_robot.getPosition(i,position);
    hans_robot.setPosition(i,position);
    hans_robot.setPosition(i,2048);
  }

  hans_robot.close();
  return 0;
}