#include "serial_command_robot/serial_command_robot_interface.h"

SerialCommandRobotInterface::SerialCommandRobotInterface()
{
}

SerialCommandRobotInterface::~SerialCommandRobotInterface()
{
  
}

void SerialCommandRobotInterface::setSerialPort(const std::shared_ptr<SerialPortInterface> &serial_port)
{
  serial_command_->setSerialPort(serial_port);
}