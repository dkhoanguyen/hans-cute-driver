#include "hans_cute_driver/servo_command.hpp"

namespace HansCuteRobot
{
  DataPacket HansCommand::setID(
      const uint8_t &old_id, const uint8_t &new_id)
  {
    DataPacket packet;
    return packet;
  }

  DataPacket HansCommand::setBaudrate(
      const uint8_t &servo_id, const long &baudrate)
  {
  }

  DataPacket HansCommand::setReturnDelayTime(
      const uint8_t &servo_id, const unsigned int &delay_time)
  {
  }
  DataPacket HansCommand::setAngleLimits(
      const uint8_t &servo_id, const unsigned int &min_limit,
      const unsigned int &max_limit)
  {
    DataPacket packet;
    packet.control_id = (uint8_t)ControlTableConstant::CW_ANGLE_LIMIT_L;
    uint8_t min_angle_low = (min_limit % 256);
    uint8_t min_angle_high = (min_limit >> 8);

    uint8_t max_angle_low = (max_limit % 256);
    uint8_t max_angle_high = (max_limit >> 8);

    packet.data = std::vector<uint8_t>({min_angle_low, min_angle_high, max_angle_low, max_angle_high});
    return packet
  }
  DataPacket HansCommand::setVoltageLimits(
      const uint8_t &servo_id, const double &min, const double &max)
  {
    DataPacket packet;
    return packet;
  }
  DataPacket HansCommand::setMaxTorque(
      const uint8_t &servo_id, const unsigned int &max_torque)
  {
    DataPacket packet;
    return packet;
  }

  //===============================================================//
  // These functions can send multiple commands to a single servo  //
  //===============================================================//
  DataPacket HansCommand::setTorqueEnable(
      const uint8_t &servo_id, const bool &enabled) {}
  DataPacket HansCommand::setComplianceMargin() {}
  DataPacket HansCommand::setComplianceSlope() {}

  DataPacket HansCommand::setDGain() {}
  DataPacket HansCommand::setIGain() {}
  DataPacket HansCommand::setPGain() {}

  DataPacket HansCommand::setAcceleration(
      const uint8_t &servo_id, const unsigned int &acceleration)
  {
    DataPacket packet;
    packet.control_id = (uint8_t)ControlTableConstant::GOAL_ACCELERATION;
    packet.data = std::vector<uint8_t>({(uint8_t)(acceleration % 256), (uint8_t)(acceleration >> 8)});
    return packet;
  }
  DataPacket HansCommand::setPosition(
      const uint8_t &servo_id, const unsigned int &position)
  {
    DataPacket packet;
    packet.control_id = (uint8_t)ControlTableConstant::GOAL_POSITION_L;
    packet.data = std::vector<uint8_t>({(uint8_t)(position % 256), (uint8_t)(position >> 8)});
    return packet;
  }
  DataPacket HansCommand::setSpeed(
      const uint8_t &servo_id, const unsigned int &speed)
  {
    DataPacket packet;
    packet.control_id = (uint8_t)ControlTableConstant::GOAL_SPEED_L;
    packet.data = std::vector<uint8_t>({(uint8_t)(speed % 256), (uint8_t)(speed >> 8)});
    return packet;
  }

  DataPacket HansCommand::setTorqueLimit(
      const uint8_t &servo_id, const unsigned int &torque_limit)
  {
    DataPacket packet;
    packet.control_id = (uint8_t)ControlTableConstant::GOAL_SPEED_L;
    packet.data = std::vector<uint8_t>({(uint8_t)(torque_limit % 256), (uint8_t)(torque_limit >> 8)});
    return packet;
  }
  DataPacket HansCommand::setGoalTorque() {}

  DataPacket HansCommand::setPositionAndSpeed() {}

  //===============================================================//
  // These functions can send multiple commands to multiple servos //
  //===============================================================//

  DataPacket HansCommand::setMultiTorqueEnabled() {}

  // Range is between 0 -> 255
  DataPacket HansCommand::setMultiComplianceMargin() {}
  DataPacket HansCommand::setMultiComplianceSlope() {}

  // Position value ranges from 0 -> 4095 (0xFFF), unit is 0.088 degree
  DataPacket HansCommand::setMultiPosition(
      const std::vector<unsigned int> &servo_ids,
      const std::vector<unsigned int> &positions)
  {
  }
  DataPacket HansCommand::setMultiSpeed(const std::vector<unsigned int> &servo_ids, const std::vector<unsigned int> &speeds) {}

  DataPacket HansCommand::setMultiTorqueLimit(const std::vector<unsigned int> &servo_ids,
                                              const std::vector<double> &torque_limits) {}
  DataPacket HansCommand::setMultiPositionAndSpeed(
      const std::vector<unsigned int> &servo_ids,
      const std::vector<unsigned int> &positions,
      const std::vector<unsigned int> &speeds)
  {
    DataPacket packet;
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
    std::vector<uint8_t> flatten_data;
    unsigned int sum = 0;
    for (std::vector<uint8_t> servo : data_lists)
    {
      for (uint8_t data_point : servo)
      {
        flatten_data.push_back(data_point);
        sum += data_point;
      }
    }
    packet.data = flatten_data;
    return packet;
  }
}