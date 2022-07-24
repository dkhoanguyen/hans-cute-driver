#ifndef _HANS_CUTE_SERIAL_H_
#define _HANS_CUTE_SERIAL_H_

#include "serial_command.h"
#include "hans_cute_const.h"

namespace HansCuteRobot
{
class ServoSerialComms : public SerialCommand
{
public:
  ServoSerialComms(const std::string port, const long baudrate);
  ~ServoSerialComms();

  bool readResponse(std::vector<uint8_t>& response);
  bool writeCommand(const std::vector<uint8_t>& command);

  bool read(const uint8_t& id, const uint8_t& address, const uint8_t& size, std::vector<uint8_t>& returned_data,
            unsigned long& timestamp);
  bool write(const uint8_t& id, const uint8_t& address, const std::vector<uint8_t>& data,
             std::vector<uint8_t>& returned_data, unsigned long& timestamp);
  bool syncWrite(const uint8_t& address, const std::vector<std::vector<uint8_t> >& data);

  bool ping(const uint8_t& id, std::vector<uint8_t>& returned_data);

  uint8_t calcCheckSum(std::vector<uint8_t>& data) const;
};
};  // namespace HansCuteRobot

#endif