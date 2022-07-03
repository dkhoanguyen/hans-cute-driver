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
  unsigned int checksum = 0;
  unsigned int payload_sum = 0;
  for (int i = 2; i < data.size() - 1; i++)
  {
    payload_sum += (int)data.at(i);
  }
  checksum = 255 - (payload_sum % 256);
  return (uint8_t)checksum;
}