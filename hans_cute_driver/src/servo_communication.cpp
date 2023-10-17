#include "hans_cute_driver/servo_communication.hpp"

namespace HansCuteRobot
{
  ServoCommunication::ServoCommunication()
  {
  }

  ServoCommunication::~ServoCommunication()
  {
  }

  bool ServoCommunication::open(const std::string &port, const unsigned int &baudrate)
  {
    // Open a new one
    serial_port_ptr_ = std::make_shared<SerialPort>(port, baudrate, 50);
    if (!serial_port_ptr_->openPort())
    {
      return false;
    }

    return true;
  }

  bool ServoCommunication::close()
  {
  }

  bool ServoCommunication::read(const uint8_t &id, const uint8_t &address,
                        const uint8_t &size, std::vector<uint8_t> &returned_data,
                        unsigned long &timestamp)
  {
    // Number of bytes following standard header (0xFF, 0xFF, id, length)
    uint8_t length = 4;
    std::vector<uint8_t> packet = {0xFF, 0xFF, id, length, (uint8_t)InstructionSet::READ_DATA, address, size};

    // directly from AX-12 manual:
    // Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
    // If the calculated value is > 255, the lower byte is the check sum.
    uint8_t checksum = 255 - ((id + length + (uint8_t)InstructionSet::READ_DATA + address + size) % 256);
    packet.push_back(checksum);

    int write_status = writeCommand(packet);
    if (write_status != (int)SerialError::NO_ERROR)
    {
      return false;
    }
    // Read response ans return data
    int status = readResponse(returned_data);
    if (status != 0)
    {
      return false;
    }
    return true;
  }
  bool ServoCommunication::write(const uint8_t &id, const uint8_t &address,
                         const std::vector<uint8_t> &data,
                         std::vector<uint8_t> &returned_data,
                         unsigned long &timestamp)
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

    int write_status = writeCommand(packet);
    if (write_status != (int)SerialError::NO_ERROR)
    {
      return false;
    }
    // Read response ans return data
    int status = readResponse(returned_data);
    if (status != 0)
    {
      return false;
    }
    return true;
  }
  bool ServoCommunication::syncWrite(const uint8_t &address,
                             const std::vector<std::vector<uint8_t>> &data)
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

    int write_status = writeCommand(packet);
    if (write_status != (int)SerialError::NO_ERROR)
    {
      return false;
    }
    // Read response ans return data
    std::vector<uint8_t> returned_data;
    int status = readResponse(returned_data);
    if (status != 0)
    {
      return false;
    }
    return true;
  }

  bool ServoCommunication::ping(
      const uint8_t &id, std::vector<uint8_t> &returned_data)
  {
    uint8_t length = 2;
    std::vector<uint8_t> packet = {0xFF, 0xFF, id, length, (uint8_t)InstructionSet::PING};

    // directly from AX-12 manual:
    // Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
    // If the calculated value is > 255, the lower byte is the check sum.
    // Don't ask me why
    uint8_t checksum = 255 - ((id + length + (uint8_t)InstructionSet::PING) % 256);
    packet.push_back(checksum);

    int write_status = writeCommand(packet);
    if (write_status != (int)SerialError::NO_ERROR)
    {
      return false;
    }
    // Read response ans return data
    int status = readResponse(returned_data);
    if (status != 0)
    {
      return false;
    }
    // We need to handle error code returned from the robot
    return true;
  }

  uint8_t ServoCommunication::calcCheckSum(std::vector<uint8_t> &data) const
  {
    unsigned int checksum = 0;
    unsigned int payload_sum = 0;
    unsigned int bytes_to_read = data.at(3); // Position of the length byte for total length of the data

    for (int i = 2; i <= 2 + bytes_to_read; i++)
    {
      payload_sum += (int)data.at(i);
    }
    checksum = 255 - (payload_sum % 256);
    return (uint8_t)checksum;
  }

  int ServoCommunication::readResponse(std::vector<uint8_t> &response)
  {
    std::vector<uint8_t> returned_data;
    if (serial_port_ptr_->available() == 0)
    {
      return (int)SerialError::NO_RESPONSE;
    }

    // To support testing this should be a separate function
    unsigned int num_byte_read = 0;
    for (int i = 0; i < 5; i++)
    {
      num_byte_read = serial_port_ptr_->read(returned_data);
      // If we actually receive data
      if (num_byte_read > 0)
      {
        break;
      }
    }
    // Unable to get a response
    if (num_byte_read == 0)
    {
      return (int)SerialError::READ_ERROR;
    }
    // Verify header first
    std::vector<uint8_t> headers = {0xFF, 0xFF};
    for (int i = 0; i < headers.size(); i++)
    {
      if (returned_data.at(i) != headers.at(i))
      {
        return (int)SerialError::WRONG_HEADER;
      }
    }
    // Then verify checksum
    if (calcCheckSum(returned_data) != returned_data.at(returned_data.size() - 1))
    {
      return (int)SerialError::WRONG_CHECKSUM;
    }
    response = returned_data;
    return (int)SerialError::NO_ERROR;
  }

  int ServoCommunication::writeCommand(const std::vector<uint8_t> &command)
  {
    try
    {
      serial_port_ptr_->write(command);
      serial_port_ptr_->wait();
      return (int)SerialError::NO_ERROR;
    }
    catch (const std::exception &se)
    {
      // Handle errors
      return (int)SerialError::WRITE_ERROR;
    }
  }

  //====================================================================//
  // These function modify EEPROM data which persists after power cycle //
  //====================================================================//

  bool ServoCommunication::setID(
      const uint8_t &old_id, const uint8_t &new_id)
  {
    return true;
  }

  bool ServoCommunication::setBaudrate(
      const uint8_t &servo_id, const long &baudrate)
  {
    return true;
  }

  bool ServoCommunication::setReturnDelayTime(
      const uint8_t &servo_id, const unsigned int &delay_time)
  {
    return true;
  }
  bool ServoCommunication::setAngleLimits(
      const uint8_t &servo_id, const unsigned int &min_limit,
      const unsigned int &max_limit)
  {
    uint8_t min_angle_low = (min_limit % 256);
    uint8_t min_angle_high = (min_limit >> 8);

    uint8_t max_angle_low = (max_limit % 256);
    uint8_t max_angle_high = (max_limit >> 8);

    std::vector<uint8_t> data({min_angle_low, min_angle_high, max_angle_low, max_angle_high});
    std::vector<uint8_t> returned_data;
    unsigned long timestamp;
    return write(servo_id, (uint8_t)ControlTableConstant::CW_ANGLE_LIMIT_L, data, returned_data, timestamp);
  }
  bool ServoCommunication::setVoltageLimits(
      const uint8_t &servo_id, const double &min, const double &max)
  {
    return true;
  }
  bool ServoCommunication::setMaxTorque(
      const uint8_t &servo_id, const unsigned int &max_torque)
  {
    return true;
  }

  //===============================================================//
  // These functions can send multiple commands to a single servo  //
  //===============================================================//
  bool ServoCommunication::setTorqueEnable(
      const uint8_t &servo_id, const bool &enabled)
  {
    std::vector<uint8_t> data({(uint8_t)enabled});
    std::vector<uint8_t> returned_data;
    unsigned long timestamp;
    return write(servo_id, (uint8_t)ControlTableConstant::TORQUE_ENABLE, data, returned_data, timestamp);
  }
  bool ServoCommunication::setComplianceMargin() {}
  bool ServoCommunication::setComplianceSlope() {}

  bool ServoCommunication::setDGain() {}
  bool ServoCommunication::setIGain() {}
  bool ServoCommunication::setPGain() {}

  bool ServoCommunication::setAcceleration(
      const uint8_t &servo_id, const unsigned int &acceleration)
  {
    std::vector<uint8_t> raw_data({(uint8_t)(acceleration % 256), (uint8_t)(acceleration >> 8)});
    std::vector<uint8_t> returned_data;
    unsigned long timestamp;
    return write(servo_id, (uint8_t)ControlTableConstant::GOAL_ACCELERATION, raw_data, returned_data, timestamp);
  }
  bool ServoCommunication::setPosition(
      const uint8_t &servo_id, const unsigned int &position)
  {
    std::vector<uint8_t> raw_postion({(uint8_t)(position % 256), (uint8_t)(position >> 8)});
    std::vector<uint8_t> returned_data;
    unsigned long timestamp;
    return write(servo_id, (uint8_t)ControlTableConstant::GOAL_POSITION_L, raw_postion, returned_data, timestamp);
  }
  bool ServoCommunication::setSpeed(
      const uint8_t &servo_id, const unsigned int &speed)
  {
    std::vector<uint8_t> raw_speed({(uint8_t)(speed % 256), (uint8_t)(speed >> 8)});
    std::vector<uint8_t> returned_data;
    unsigned long timestamp;
    return write(servo_id, (uint8_t)ControlTableConstant::GOAL_SPEED_L, raw_speed, returned_data, timestamp);
  }

  bool ServoCommunication::setTorqueLimit(
      const uint8_t &servo_id, const unsigned int &torque_limit)
  {
    return true;
  }
  bool ServoCommunication::setGoalTorque()
  {
    return true;
  }

  bool ServoCommunication::setPositionAndSpeed()
  {
    return true;
  }

  //===============================================================//
  // These functions can send multiple commands to multiple servos //
  //===============================================================//

  bool ServoCommunication::setMultiTorqueEnabled()
  {
    return true;
  }

  // Range is between 0 -> 255
  bool ServoCommunication::setMultiComplianceMargin()
  {
    return true;
  }
  bool ServoCommunication::setMultiComplianceSlope()
  {
    return true;
  }

  // Position value ranges from 0 -> 4095 (0xFFF), unit is 0.088 degree
  bool ServoCommunication::setMultiPosition(
      const std::vector<unsigned int> &servo_ids,
      const std::vector<unsigned int> &positions)
  {
  }
  bool ServoCommunication::setMultiSpeed(
      const std::vector<unsigned int> &servo_ids,
      const std::vector<unsigned int> &speeds) {}

  bool ServoCommunication::setMultiTorqueLimit(const std::vector<unsigned int> &servo_ids,
                                       const std::vector<double> &torque_limits) {}
  bool ServoCommunication::setMultiPositionAndSpeed(
      const std::vector<unsigned int> &servo_ids,
      const std::vector<unsigned int> &positions,
      const std::vector<unsigned int> &speeds)
  {
    std::vector<std::vector<uint8_t>> data_lists;
    for (int indx = 0; indx < servo_ids.size(); indx++)
    {
      std::vector<uint8_t> data;
      data.push_back((uint8_t)servo_ids.at(indx));
      std::vector<uint8_t> position = std::vector<uint8_t>({(uint8_t)(positions.at(indx) % 256), (uint8_t)(positions.at(indx) >> 8)});
      data.insert(data.end(), position.begin(), position.end());
      std::vector<uint8_t> speed = std::vector<uint8_t>({(uint8_t)(speeds.at(indx) % 256), (uint8_t)(speeds.at(indx) >> 8)});
      data.insert(data.end(), speed.begin(), speed.end());
      data_lists.push_back(data);
    }
    return syncWrite((uint8_t)ControlTableConstant::GOAL_POSITION_L, data_lists);
  }

  //===============================//
  // Servo status access functions //
  //===============================//
  bool ServoCommunication::getModelNumber(const int &servo_id, unsigned int &model_number)
  {
    std::vector<uint8_t> response;
    unsigned long timestamp = 0;
    if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::MODEL_NUMBER_L, 2, response, timestamp))
    {
      return false;
    }
    model_number = HansCuteRobot::ModelNumber::getData(response);
    return true;
  }
  bool ServoCommunication::getFirmwareVersion() {}
  bool ServoCommunication::getReturnDelayTime() {}

  bool ServoCommunication::getMaxTorque(const int &servo_id, unsigned int &torque) {}
  bool ServoCommunication::getTorqueEnabled(const int &servo_id, bool &enabled)
  {
    std::vector<uint8_t> response;
    unsigned long timestamp = 0;
    if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::TORQUE_ENABLE, 1, response, timestamp))
    {
      return false;
    }
    return true;
  }
  bool ServoCommunication::getAngleLimits(
      const int &servo_id, unsigned int &angle_min, unsigned int &angle_max)
  {
    std::vector<uint8_t> response;
    unsigned long timestamp = 0;
    if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::CW_ANGLE_LIMIT_L, 4, response, timestamp))
    {
      return false;
    }
    angle_min = (unsigned int)(response.at(5) + (response.at(6) << 8));
    angle_max = (unsigned int)(response.at(7) + (response.at(8) << 8));
    return true;
  }
  bool ServoCommunication::getVoltageLimits() {}

  bool ServoCommunication::getPosition(const int &servo_id, unsigned int &position)
  {
    std::vector<uint8_t> response;
    unsigned long timestamp = 0;
    if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::PRESENT_POSITION_L, 2, response, timestamp))
    {
      return false;
    }
    position = (unsigned int)(response.at(5) + (response.at(6) << 8));
    return true;
  }
  bool ServoCommunication::getSpeed(const int &servo_id, unsigned int &speed)
  {
    std::vector<uint8_t> response;
    unsigned long timestamp = 0;
    if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::GOAL_SPEED_L, 2, response, timestamp))
    {
      return false;
    }
    speed = (unsigned int)(unsigned int)(response.at(5) + (response.at(6) << 8));
    return true;
  }
  bool ServoCommunication::getAcceleration(const int &servo_id, unsigned int &acceleration) {}

  bool ServoCommunication::getVoltage() {}
  bool ServoCommunication::getCurrent() {}
  bool ServoCommunication::getLock(const int &servo_id, bool &lock) {}

  bool ServoCommunication::getFeedback(const int &servo_id, ServoFeedback &feedback)
  {
    return true;
  }

  void unpackFloats(const std::vector<uint8_t>::iterator &it, float &output)
  {
    uint8_t b[] = {*it, *(it + 1), *(it + 2), *(it + 3)};
    std::memcpy(&output, &b, sizeof(output)); // convert to float from bytes[4]
                                              //     printf("%f\n", temp);
  }

  void floatToByte(float float_variable, uint8_t temp_bytes[])
  {
    union
    {
      float a;
      uint8_t bytes[4];
    } link;
    link.a = float_variable;
    std::memcpy(temp_bytes, link.bytes, 4);
  }

  void doubleToByte(double double_variable, uint8_t temp_bytes[])
  {
    union
    {
      double a;
      uint8_t bytes[8];
    } link;

    link.a = double_variable;
    std::memcpy(temp_bytes, link.bytes, 8);
  }

  void packFromFloats(const std::vector<float> &value_to_pack, std::vector<uint8_t> &packed_floats)
  {
    for (const float value : value_to_pack)
    {
      uint8_t bytes_temp[4];
      floatToByte(value, bytes_temp);
      for (int j = 0; j < 4; ++j)
      {
        packed_floats.push_back(bytes_temp[j]);
      }
    }
  }

  void packFromDoubles(const std::vector<double> &value_to_pack, std::vector<uint8_t> &packed_doubles)
  {
    for (const double value : value_to_pack)
    {
      uint8_t bytes_temp[8];
      doubleToByte(value, bytes_temp);
      for (int j = 0; j < 8; j++)
      {
        packed_doubles.push_back(bytes_temp[j]);
      }
    }
  }
}