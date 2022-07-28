#include <iostream>
#include "controller_manager.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hans_cute_controller_manager");
  ros::NodeHandle nh;
  std::cout << "HEY" << std::endl;
  std::string port;
  double origin = 0;
  
  std::cout << nh.hasParam("controller_manager/port") << std::endl;
  if (!(nh.getParam("controller_manager/port", port)))
  {
    ROS_ERROR("DobotRosWrapper: port not specified");
    // exit(1);
  }
  std::cout << port << std::endl;
  std::cout << ros::this_node::getName().substr(ros::this_node::getNamespace().size()) << std::endl;
  return 0;
}