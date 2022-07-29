#include <iostream>
#include "controller_manager.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hans_cute_controller_manager");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(3);

  HansCuteControllerManager controller_manager(nh);
  controller_manager.initialise();

  ROS_INFO("HansCuteControllerManager: Starting Controller Manager.");
  controller_manager.start();

  spinner.start();
  ros::Rate rate(10);
  while (ros::ok)
  {
    rate.sleep();
  }

  spinner.stop();
  ros::waitForShutdown();
  return 0;
}