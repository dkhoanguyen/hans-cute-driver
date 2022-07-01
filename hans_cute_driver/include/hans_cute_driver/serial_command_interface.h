#ifndef _SERIAL_COMMAND_INTERFACE_H_
#define _SERIAL_COMMAND_INTERFACE_H_

#include <vector>

class SerialCommandInterface
{
public:
  SerialCommandInterface(){};
  virtual ~SerialCommandInterface(){};

  virtual void open() = 0;
  virtual void close() = 0;

  virtual void readResponse(std::vector<uint8_t>& response) = 0;
  virtual void writeCommand(const std::vector<uint8_t>& command) = 0;
};

#endif