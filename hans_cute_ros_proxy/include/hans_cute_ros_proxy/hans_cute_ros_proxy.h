#ifndef _HANS_CUTE_ROS_PROXY_H_
#define _HANS_CUTE_ROS_PROXY_H_

#include <thread>

#include "hans_cute_driver/hans_cute_driver.h"

class HansCuteRosProxy
{
public:
  HansCuteRosProxy(const std::string& port_name, const std::string& port_namespace, const long& baud_rate,
                   const unsigned int& min_motor_id=0, const unsigned int& max_motor_id=7);
  HansCuteRosProxy();
  ~HansCuteRosProxy();

  void setServoDriver(std::shared_ptr<HansCuteRobot::ServoDriver> servo_driver_ptr);

  bool connect();
  bool disconnect();

private:
  bool findMotors();
  bool fillMotorParameters(const unsigned int& servo_id, const unsigned int& model_number);

  void updateMotorState();
  void publishDiagnosticInformation();

  std::string port_name_;
  std::string port_namespace_;
  long baud_rate_;
  double update_rate_;
  double diagnostics_rate_;

  unsigned int min_motor_id_;
  unsigned int max_motor_id_;

  bool running_;

  std::shared_ptr<HansCuteRobot::ServoDriver> servo_driver_ptr_;

  std::shared_ptr<std::thread> update_motor_state_thred_;
  std::shared_ptr<std::thread> diagnostic_thread_;
};

#endif