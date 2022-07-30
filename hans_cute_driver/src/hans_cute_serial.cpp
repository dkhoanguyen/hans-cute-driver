#include "hans_cute_driver/hans_cute_serial.h"

HansCuteRobot::ServoSerialComms::ServoSerialComms(const std::string port, const long baudrate) : SerialCommand(port, baudrate)
{
}

HansCuteRobot::ServoSerialComms::~ServoSerialComms()
{
  std::cout << "ServoSerialComms Destructor" << std::endl;
}

bool HansCuteRobot::ServoSerialComms::readResponse(std::vector<uint8_t> &response)
{
  return SerialCommand::readResponse(response);
}

bool HansCuteRobot::ServoSerialComms::writeCommand(const std::vector<uint8_t> &command)
{
  return SerialCommand::writeCommand(command);
}

uint8_t HansCuteRobot::ServoSerialComms::calcCheckSum(std::vector<uint8_t> &data) const
{
  unsigned int checksum = 0;
  unsigned int payload_sum = 0;
  unsigned int bytes_to_read = data.at(sample_packet_.length); // Add ID and length as well

  for (int i = 2; i <= 2 + bytes_to_read; i++)
  {
    payload_sum += (int)data.at(i);
  }
  checksum = 255 - (payload_sum % 256);
  return (uint8_t)checksum;
}

//
bool HansCuteRobot::ServoSerialComms::read(const uint8_t &id, const uint8_t &address, 
                                           const uint8_t &size, unsigned long &timestamp)
{
  // Number of bytes following standard header (0xFF, 0xFF, id, length)
  uint8_t length = 4;
  std::vector<uint8_t> packet = {0xFF, 0xFF, id, length, (uint8_t)InstructionSet::READ_DATA, address, size};

  // directly from AX-12 manual:
  // Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
  // If the calculated value is > 255, the lower byte is the check sum.
  uint8_t checksum = 255 - ((id + length + (uint8_t)InstructionSet::READ_DATA + address + size) % 256);
  packet.push_back(checksum);

  return writeCommand(packet);
}

bool HansCuteRobot::ServoSerialComms::write(const uint8_t &id, const uint8_t &address,
                                            const std::vector<uint8_t> &data, unsigned long &timestamp)
{
  // Number of bytes following standard header (0xFF, 0xFF, id, length)
  uint8_t length = 3 + (uint8_t)data.size();
  std::vector<uint8_t> packet = {0xFF, 0xFF, id, length, (uint8_t)InstructionSet::WRITE_DATA, address};
  packet.insert(std::end(packet), std::begin(data), std::end(data));

  // directly from AX-12 manual:
  // Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
  // If the calculated value is > 255, the lower byte is the check sum.
  unsigned int sum = 0;
  for (uint8_t data_point : data)
  {
    sum += data_point;
  }
  sum += (id + length + (uint8_t)InstructionSet::WRITE_DATA + address);
  uint8_t checksum = 255 - (sum % 256);
  packet.push_back(checksum);

  return writeCommand(packet);
}

bool HansCuteRobot::ServoSerialComms::syncWrite(const uint8_t &address, const std::vector<std::vector<uint8_t>> &data)
{
  // First flatten the input data
  std::vector<uint8_t> flatten_data;
  unsigned int sum = 0;
  for (std::vector<uint8_t> servo : data)
  {
    for (uint8_t data_point : servo)
    {
      flatten_data.push_back(data_point);
      sum += data_point;
    }
  }
  // Number of bytes following standard header (0xFF, 0xFF, id, length) plus data
  uint8_t length = 4 + flatten_data.size();
  uint8_t servo_data_length = data.at(0).size() - 1;
  std::vector<uint8_t> packet = {0xFF, 0xFF, (uint8_t)BroadcastConstant::BROADCAST, length, (uint8_t)InstructionSet::SYNC_WRITE, address, servo_data_length};
  packet.insert(std::end(packet), std::begin(flatten_data), std::end(flatten_data));

  sum += (uint8_t)BroadcastConstant::BROADCAST + length + (uint8_t)InstructionSet::SYNC_WRITE + address + servo_data_length;
  uint8_t checksum = 255 - (sum % 256);
  packet.push_back(checksum);

  // Thread safe execution of
  if (!writeCommand(packet))
  {
    return false;
  }
  // Read response ans return data
  std::vector<uint8_t> returned_data;
  return readResponse(returned_data);
}

bool HansCuteRobot::ServoSerialComms::ping(const uint8_t &id)
{
  uint8_t length = 2;
  std::vector<uint8_t> packet = {0xFF, 0xFF, id, length, (uint8_t)InstructionSet::PING};

  // directly from AX-12 manual:
  // Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
  // If the calculated value is > 255, the lower byte is the check sum.
  // Don't ask me why
  uint8_t checksum = 255 - ((id + length + (uint8_t)InstructionSet::PING) % 256);
  packet.push_back(checksum);

  return writeCommand(packet);
}