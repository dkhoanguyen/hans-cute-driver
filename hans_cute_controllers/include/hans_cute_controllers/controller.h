#ifndef _JOINT_CONTROLLER_H_
#define _JOINT_CONTROLLER_H_

#include <thread>
#include <string>
#include <vector>

#include "hans_cute_driver/hans_cute_driver.h"
#include "hans_cute_driver/hans_cute_const.h"
#include "hans_cute_driver/hans_cute_datatype.h"

#include "controller_interface.h"

namespace HansCuteController
{
  class Controller : public ControllerInterfaces
  {
  public:
    Controller(const std::shared_ptr<HansCuteRobot::ServoDriver> &servo_driver_ptr,
               const std::string &controller_namespace, const std::string &port_namespace);
    virtual ~Controller();

    // Setters
    void setJointIds(const std::vector<unsigned int> &joint_ids);
    void setServoModels(const std::vector<HansCuteRobot::ServoModel> &models);

    void setJointNames(const std::vector<std::string> &joint_names);
    void setJointSpeeds(const std::vector<unsigned int> &joint_speeds);
    void setComplianceSlopes(const std::vector<unsigned int> &compliance_slopes);
    void setComplianceMargins(const std::vector<unsigned int> &compliance_margins);
    void setCompliancePunches(const std::vector<unsigned int> &compliance_punches);
    void setTorqueLimits(const std::vector<double> &torque_limits);

    virtual void initialise() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;

    virtual void processCommand(Data &data) = 0;

  protected:
    std::shared_ptr<HansCuteRobot::ServoDriver> servo_driver_ptr_;

    std::string controller_namespace_;
    std::string port_namespace_;

    bool running_;

    std::vector<unsigned int> joint_ids_;
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