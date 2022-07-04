#include "hans_cute_driver/hans_cute_serial.h"

HansCuteSerial::HansCuteSerial(const std::string port, const long baudrate) : SerialCommand(port, baudrate)
{
}

HansCuteSerial::~HansCuteSerial()
{
  std::cout << "HansCuteSerial Destructor" << std::endl;
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
  unsigned int bytes_to_read = data.at(_sample_packet.length); // Add ID and length as well

  for (int i = 2; i < 2 + bytes_to_read; i++)
  {
    payload_sum += (int)data.at(i);
  }
  checksum = 255 - (payload_sum % 256);
  return (uint8_t)checksum;
}

//
bool HansCuteSerial::read(const uint8_t& id, const uint8_t& address, const uint8_t& size,
                          std::vector<uint8_t>& returned_data, unsigned long& timestamp)
{
  // Number of bytes following standard header (0xFF, 0xFF, id, length)
  uint8_t length = 4;
  std::vector<uint8_t> packet = { 0xFF, 0xFF, id, length, (uint8_t)InstructionSet::DXL_READ_DATA, address, size };
  
  // directly from AX-12 manual:
  // Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
  // If the calculated value is > 255, the lower byte is the check sum.
  uint8_t checksum = 255 -((id + length + (uint8_t)InstructionSet::DXL_READ_DATA + address + size) % 256);
  packet.push_back(checksum);

  // Thread safe execution of
  if(!writeCommand(packet))
  {
    return false;
  }
  // Read response ans return data
  return readResponse(returned_data);
}

bool HansCuteSerial::write(const uint8_t& id, const uint8_t& address, const std::vector<uint8_t>& data,
                           std::vector<uint8_t>& returned_data, unsigned long& timestamp)
{
  // Number of bytes following standard header (0xFF, 0xFF, id, length)
  uint8_t length = 3 + (uint8_t)data.size();
  std::vector<uint8_t> packet = { 0xFF, 0xFF, id, length, (uint8_t)InstructionSet::DXL_WRITE_DATA, address };
  packet.insert(std::end(packet), std::begin(data), std::end(data));
  
  // directly from AX-12 manual:
  // Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
  // If the calculated value is > 255, the lower byte is the check sum.
  unsigned int sum = 0;
  for(uint8_t data_point : data)
  {
    sum += data_point;
  }
  sum += (id + length + (uint8_t)InstructionSet::DXL_WRITE_DATA + address);
  uint8_t checksum = 255 - (sum % 256);
  packet.push_back(checksum);

  // Thread safe execution of
  if(!writeCommand(packet))
  {
    return false;
  }
  // Read response ans return data
  return readResponse(returned_data);
}

bool HansCuteSerial::syncWrite(const uint8_t& address, const std::vector<std::vector<uint8_t> >& data)
{
  return false;
}

bool HansCuteSerial::ping(const uint8_t& id, std::vector<uint8_t>& returned_data)
{
  uint8_t length = 2;
  std::vector<uint8_t> packet = { 0xFF, 0xFF, id, length, (uint8_t)InstructionSet::DXL_PING };
  
  // directly from AX-12 manual:
  // Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
  // If the calculated value is > 255, the lower byte is the check sum.
  // Don't ask me why
  uint8_t checksum = 255 - ((id + length + (uint8_t)InstructionSet::DXL_PING) % 256);
  packet.push_back(checksum);

  // Thread safe execution of
  if(!writeCommand(packet))
  {
    return false;
  }
  // Read response ans return data
  if(!readResponse(returned_data))
  {
    return false;
  }
  // We need to handle error code returned from the robot
  return true;
}