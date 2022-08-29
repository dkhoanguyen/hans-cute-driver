#ifndef _JOINT_CONTROLLER_INTERFACE_H_
#define _JOINT_CONTROLLER_INTERFACE_H_

#include "serial_command_robot/serial_command_robot.h"
#include "controller_datatype.h"

namespace HansCuteController
{
  class ControllerInterfaces
  {
    public:
      ControllerInterfaces(){};
      virtual ~ControllerInterfaces(){};

      virtual void initialise() = 0;
      virtual void start() = 0;
      virtual void stop() = 0;

      virtual void processCommand(Data &data) = 0;
  };
};

#endif