#ifndef _JOINT_CONTROLLER_INTERFACE_H_
#define _JOINT_CONTROLLER_INTERFACE_H_

#include "hans_cute_driver/hans_cute_driver.h"

namespace HansCuteController
{
  class JointControllerInterface
  {
    public:
      JointControllerInterface(){};
      virtual ~JointControllerInterface(){};

      virtual void initialise() = 0;
      virtual void start() = 0;
      virtual void stop() = 0;

      virtual void processCommand() = 0
  };
};

#endif