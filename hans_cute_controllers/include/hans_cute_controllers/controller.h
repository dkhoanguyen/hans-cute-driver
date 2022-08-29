#ifndef _JOINT_CONTROLLER_H_
#define _JOINT_CONTROLLER_H_

#include <thread>
#include <string>
#include <vector>

#include "serial_command_robot/serial_command_robot.h"

#include "hans_cute_driver/hans_cute_driver.h"
#include "hans_cute_driver/hans_cute_const.h"
#include "hans_cute_driver/hans_cute_datatype.h"

#include "hans_cute_status_manager/status_manager_datatype.h"

#include "controller_interface.h"

namespace HansCuteController
{
  class Controller : public ControllerInterfaces
  {
  public:
    Controller(const std::shared_ptr<SerialCommandRobot> &servo_driver_ptr,
               const std::string &controller_namespace, const std::string &port_namespace);
    virtual ~Controller();

    // Setters
    void setJointIds(const std::vector<unsigned int> &joint_ids);
    void setServoParams(const std::vector<ServoParams> &servo_params);

    void setJointNames(const std::vector<std::string> &joint_names);
    void setJointSpeeds(const std::vector<unsigned int> &joint_speeds);

    virtual void initialise() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;

    virtual void processCommand(Data &data) = 0;

  protected:
    std::shared_ptr<SerialCommandRobot> robot_driver_ptr_;

    std::string controller_namespace_;
    std::string port_namespace_;

    bool running_;

    std::vector<unsigned int> joint_ids_;
    std::vector<ServoParams> joint_params_;
    std::vector<std::string> joint_names_;
    std::vector<unsigned int> joint_speeds_;
    std::vector<unsigned int> compliance_slopes_;
    std::vector<unsigned int> compliance_margins_;
    std::vector<unsigned int> compliance_punches_;
    std::vector<double> torque_limits_;

    std::vector<HansCuteRobot::ServoModel> models_;

  private:
    void ensureLimits();
  };

}; // namespace HansCuteController

#endif