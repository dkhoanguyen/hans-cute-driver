#ifndef _HANS_CUTE_SERIAL_H_
#define _HANS_CUTE_SERIAL_H_

#include "serial_command.h"

class HansCuteSerial : public SerialCommand
{
public:
  HansCuteSerial(const std::string port, const long baudrate);
  ~HansCuteSerial();

  bool readResponse(std::vector<uint8_t>& response) const;
  bool writeCommand(const std::vector<uint8_t>& command) const;

private:
  uint8_t uint8_t calcCheckSum(std::vector<uint8_t>& data) const;
};

#endif