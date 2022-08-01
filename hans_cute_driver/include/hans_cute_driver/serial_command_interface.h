#ifndef _SERIAL_COMMAND_INTERFACE_H_
#define _SERIAL_COMMAND_INTERFACE_H_

#include <vector>
#include "custom_serial_port/serial_port_interface.h"

class SerialCommandInterface
{
public:
  SerialCommandInterface(){};
  virtual ~SerialCommandInterface(){};

  virtual void setSerialPort(const std::shared_ptr<SerialPortInterface> &serial_port) = 0;

  virtual void open() const = 0;
  virtual void close() const = 0;

  virtual bool readResponse(std::vector<uint8_t> &response) = 0;
  virtual bool writeCommand(const std::vector<uint8_t> &command) = 0;

  virtual uint8_t calcCheckSum(std::vector<uint8_t> &data) const = 0;
};

#endif