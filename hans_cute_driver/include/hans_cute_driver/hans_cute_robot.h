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
                  const unsigned int &min_joint_id = 0, const unsigned int &max_joint_id = 6,
                  const unsigned int &gripper_id = 7);
    ~HansCuteRobot();

    bool initialise();
    bool start();
    bool stop();

    void setSerialPort(const std::shared_ptr<SerialPortInterface> &serial_port);
    void updateJointParams(const std::vector<ServoParams> &servo_params);

    // Functions for using with controllers
    // For Position Control
    bool setJointPosition(const std::vector<double> &positions);
    bool getJointPosition(std::vector<double> &positions);

    // For Velocity Control
    bool setJointVelocity(const std::vector<double> &velocities);
    bool getJointVelocity(std::vector<double> &velocities);

    // For Effort Control
    bool setJointEffort(const std::vector<double> &efforts);
    bool getJointEffort(std::vector<double> &efforts);

    // For Gripper Control
    bool setGripperState(const unsigned int &state);
    bool getGripperState(unsigned int &state);

    // Others
    bool getJointRawSpeed(std::vector<unsigned int> &speeds);
    bool setJointRawSpeed(const std::vector<unsigned int> &speeds);

    bool getJointRawAccceleration(std::vector<unsigned int> &accelerations);
    bool setJointRawAcceleration(const std::vector<unsigned int> &accelerations);

  private:
    bool findServos();
    bool fillServoParams(const unsigned int &servo_id, const unsigned int &model_number, ServoParams &servo_param);

    void posRadToRaw(const double &rad, unsigned int &raw, const ServoParams &params);
    void posRawToRad(double &rad, const unsigned int &raw, const ServoParams &params);

    void spdRadToRaw(const double &rad, unsigned int &raw, const ServoParams &params);
    void spdRawToRad(double &rad, const unsigned int &raw, const ServoParams &params);

    std::string port_name_;
    std::string port_namespace_;
    long baud_rate_;
    double update_rate_;
    double diagnostics_rate_;

    unsigned int min_joint_id_;
    unsigned int max_joint_id_;
    unsigned int gripper_id_;

    bool running_;
    bool servo_found_;

    ServoParams gripper_params_;

    std::vector<ServoParams> servos_params_;
    std::vector<unsigned int> joint_ids_;

    std::shared_ptr<ServoDriver> robot_driver_ptr_;

    std::shared_ptr<std::thread> update_motor_state_thred_;
    std::shared_ptr<std::thread> diagnostic_thread_;
  };
};

#endif