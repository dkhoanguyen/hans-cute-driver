#ifndef _SERIAL_COMMAND_INTERFACE_H_
#define _SERIAL_COMMAND_INTERFACE_H_

#include <vector>

class SerialCommandInterface
{
public:
  SerialCommandInterface(){};
  virtual ~SerialCommandInterface(){};

  virtual void open() const = 0;
  virtual void close() const = 0;

  virtual bool readResponse(std::vector<uint8_t>& response) = 0;
  virtual bool writeCommand(const std::vector<uint8_t>& command) = 0;
};

#endif