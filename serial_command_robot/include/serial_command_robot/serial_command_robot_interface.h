#ifndef _SERIAL_COMMAND_ROBOT_INTERFACE_H_
#define _SERIAL_COMMAND_ROBOT_INTERFACE_H_

#include <string>
#include <vector>

#include "serial_port/custom_serial_port.h"
#include "serial_port/serial_port_interface.h"

#include "serial_command_robot/serial_command.h"

struct RobotParamsInterface
{
  
};

class SerialCommandRobotInterface
{
public:
  SerialCommandRobotInterface();
  virtual ~SerialCommandRobotInterface();

  virtual bool initialise() = 0;
  virtual bool start() = 0;
  virtual bool stop() = 0;

  virtual void setSerialPort(const std::shared_ptr<SerialPortInterface> &serial_port);

  virtual bool getJointPosition(std::vector<double> &positions) = 0;
  virtual bool setJointPosition(const std::vector<double> &positions) = 0;

  virtual bool getJointVelocity(std::vector<double> &velocities) = 0;
  virtual bool setJointVelocity(const std::vector<double> &velocities) = 0;

  virtual bool getJointEffort(std::vector<double> &efforts) = 0;
  virtual bool setJointEffort(const std::vector<double> &efforts) = 0;

protected:
  std::shared_ptr<SerialCommand> serial_command_;
};

#endif