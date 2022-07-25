#ifndef _HANS_CUTE_STATUS_MANAGER_H_
#define _HANS_CUTE_STATUS_MANAGER_H_

#include <thread>
#include "status_manager_datatype.h"
#include "hans_cute_driver/hans_cute_driver.h"

class HansCuteStatusManager
{
public:
  HansCuteStatusManager(const std::string &port_name, const std::string &port_namespace, const long &baud_rate,
                        const unsigned int &min_motor_id = 0, const unsigned int &max_motor_id = 7);
  HansCuteStatusManager();
  ~HansCuteStatusManager();

  void setServoDriver(std::shared_ptr<HansCuteRobot::ServoDriver> servo_driver_ptr);

  void getJointParameters(std::vector<ServoParams> &servo_params);
  void getJointIds(std::vector<unsigned int> &joint_ids);

  bool initialise();
  bool disconnect();

private:
  bool findMotors();
  bool fillServoParams(const unsigned int &servo_id, const unsigned int &model_number);

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

  std::vector<ServoParams> servos_params_;
  std::vector<unsigned int> joint_ids_;

  std::shared_ptr<HansCuteRobot::ServoDriver> servo_driver_ptr_;

  std::shared_ptr<std::thread> update_motor_state_thred_;
  std::shared_ptr<std::thread> diagnostic_thread_;
};

#endif