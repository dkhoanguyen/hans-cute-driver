#include "controller_manager.h"

HansCuteControllerManager::HansCuteControllerManager(ros::NodeHandle &nh, const std::string &port) : nh_(nh), rate_(10)
{
  
}

HansCuteControllerManager::~HansCuteControllerManager()
{
}