#include "hans_cute_driver/hans_cute_serial.h"

HansCuteSerial::HansCuteSerial(const std::string port, const long baudrate) : SerialCommand(port, baudrate)
{
}

HansCuteSerial::~HansCuteSerial()
{
}

bool HansCuteSerial::readResponse(std::vector<uint8_t>& response)
{
  return SerialCommand::readResponse(response);
}

bool HansCuteSerial::writeCommand(const std::vector<uint8_t>& command)
{
  return SerialCommand::writeCommand(command);
}

uint8_t HansCuteSerial::calcCheckSum(std::vector<uint8_t>& data) const
{
  return 0;
}