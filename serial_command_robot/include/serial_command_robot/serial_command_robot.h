#ifndef _SERIAL_COMMAND_ROBOT_H_
#define _SERIAL_COMMAND_ROBOT_H_

#include <iostream>
#include <mutex>
#include <string>
#include <vector>
#include <cstring>
#include <limits>
#include <cmath>
#include <thread>
#include <atomic>
#include <csignal>

// New native serial library
#include "serial_port/serial_port_interface.h"

#include "serial_command.h"

class SerialCommandRobot : public SerialCommand
{
public:
  SerialCommandRobot();
  ~SerialCommandRobot();

  virtual bool getJointPosition(const std::vector<unsigned int> &joint_ids,
                                std::vector<unsigned int> &positions) = 0;
  virtual bool setJointPosition(const std::vector<unsigned int> &joint_ids,
                                const std::vector<unsigned int> &positions) = 0;

  virtual bool getJointSpeed(const std::vector<unsigned int> &joint_ids,
                             std::vector<unsigned int> &speeds) = 0;
  virtual bool setJointSpeed(const std::vector<unsigned int> &joint_ids,
                             const std::vector<unsigned int> &speeds) = 0;

  virtual bool getJointAccceleration(const std::vector<unsigned int> &joint_ids,
                                     std::vector<unsigned int> &accelerations) = 0;
  virtual bool setJointAcceleration(const std::vector<unsigned int> &joint_ids,
                                    const std::vector<unsigned int> &accelerations) = 0;
};

#endif