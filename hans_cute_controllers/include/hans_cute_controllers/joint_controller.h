#ifndef _JOINT_CONTROLLER_H_
#define _JOINT_CONTROLLER_H_

#include <thread>
#include <string>
#include <vector>

#include "hans_cute_driver/hans_cute_driver.h"
#include "joint_controller_interface.h"

namespace HansCuteController
{
  class JointController : public JointControllerInterface
  {
  public:
    JointController(const std::unique_ptr<HansCuteRobot::ServoDriver> &servo_driver_ptr,
                    std::string &controller_namespace, const std::string &port_namespace);
    ~JointController();

    virtual void initialise() override;
    virtual void start() override;
    virtual void stop() override;

    virtual void processCommand() = 0;

  protected:
    std::unique_ptr<HansCuteRobot::ServoDriver> servo_driver_ptr_;

    std::string controller_namespace_;
    std::string port_namespace_;
  };

};

#endif