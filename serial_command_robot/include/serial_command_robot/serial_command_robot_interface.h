#ifndef _SERIAL_COMMAND_ROBOT_INTERFACE_H_
#define _SERIAL_COMMAND_ROBOT_INTERFACE_H_

#include <string>
#include <vector>

#include "serial_port/custom_serial_port.h"
#include "serial_port/serial_port_interface.h"

#include "serial_command_robot/serial_command.h"

class SerialCommandRobotInterface
{
public:
  SerialCommandRobotInterface();
  virtual ~SerialCommandRobotInterface();

  virtual bool initialise() = 0;
  virtual bool start() = 0;
  virtual bool stop() = 0;

  void setSerialPort(const std::shared_ptr<SerialPortInterface> &serial_port);

  virtual bool getJointPosition(std::vector<unsigned int> &positions) = 0;
  virtual bool setJointPosition(const std::vector<unsigned int> &positions) = 0;

  virtual bool getJointSpeed(std::vector<unsigned int> &speeds) = 0;
  virtual bool setJointSpeed(const std::vector<unsigned int> &speeds) = 0;

  virtual bool getJointAccceleration(std::vector<unsigned int> &accelerations) = 0;
  virtual bool setJointAcceleration(const std::vector<unsigned int> &accelerations) = 0;

protected:
  std::shared_ptr<SerialCommand> serial_command_;
};

#endif