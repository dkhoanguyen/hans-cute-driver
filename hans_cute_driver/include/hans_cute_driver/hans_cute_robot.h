#ifndef _HANS_CUTE_ROBOT_H_
#define _HANS_CUTE_ROBOT_H_

#include <thread>

#include "hans_cute_driver/hans_cute_driver.h"
#include "serial_command_robot/serial_command_robot_interface.h"

namespace HansCuteRobot
{
  class HansCuteRobot : public SerialCommandRobotInterface
  {
  public:
    static const unsigned int SERVO_DEFAULT_SPEED = 400;
    static const unsigned int SERVO_DEFAULT_ACCELERATION = 40;

    HansCuteRobot(const std::string &port_name, const std::string &port_namespace, const long &baud_rate,
                  const unsigned int &min_motor_id = 0, const unsigned int &max_motor_id = 7);
    ~HansCuteRobot();

    bool initialise();
    bool start();
    bool stop();

    void updateJointParams(const std::vector<ServoParams> &servo_params);

    bool getJointPosition(std::vector<double> &positions);
    bool setJointPosition(const std::vector<double> &positions);

    bool getJointSpeed(std::vector<unsigned int> &speeds);
    bool setJointSpeed(const std::vector<unsigned int> &speeds);

    bool getJointAccceleration(std::vector<unsigned int> &accelerations);
    bool setJointAcceleration(const std::vector<unsigned int> &accelerations);

  private:
    bool findServos();
    bool fillServoParams(const unsigned int &servo_id, const unsigned int &model_number);

    void posRadToRaw(const double &rad, unsigned int &raw, const ServoParams &params);
    void posRawToRad(double &rad, const unsigned int &raw, const ServoParams &params);

    std::string port_name_;
    std::string port_namespace_;
    long baud_rate_;
    double update_rate_;
    double diagnostics_rate_;

    unsigned int min_motor_id_;
    unsigned int max_motor_id_;

    bool running_;
    bool servo_found_;

    std::vector<ServoParams> servos_params_;
    std::vector<unsigned int> joint_ids_;

    std::shared_ptr<ServoDriver> robot_driver_ptr_;

    std::shared_ptr<std::thread> update_motor_state_thred_;
    std::shared_ptr<std::thread> diagnostic_thread_;
  };
};

#endif