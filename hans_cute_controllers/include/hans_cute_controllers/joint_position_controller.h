#ifndef _JOINT_POSITION_CONTROLLER_H_
#define _JOINT_POSITION_CONTROLLER_H_

#include "joint_controller.h"

namespace HansCuteController
{
  class JointPositionController : public JointController
  {
    public:
      JointPositionController();
      ~JointPositionController();

      void initialise();
      void start();
      void stop();

      void processCommand();
  };
};

#endif