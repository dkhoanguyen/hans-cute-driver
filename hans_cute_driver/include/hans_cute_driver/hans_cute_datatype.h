#ifndef _HANS_CUTE_DATATYPE_H_
#define _HANS_CUTE_DATATYPE_H_

#include <cmath>

#include "serial_command.h"
namespace HansCuteRobot
{
struct ModelNumber
{
  static unsigned int getData(const std::vector<uint8_t>& raw_data)
  {
    return (unsigned int)(raw_data.at(5) + (raw_data.at(6) << 8));
  }
};
struct VoltageLimits
{
  double min;
  double max;

  static VoltageLimits getData(const std::vector<uint8_t>& raw_data)
  {
    VoltageLimits voltage_limits;
    voltage_limits.min = (double)raw_data.at(5) / 10;
    voltage_limits.max = (double)raw_data.at(6) / 10;
    return voltage_limits;
  };
};
struct AngleLimits
{
  unsigned int min;
  unsigned int max;

  static AngleLimits getData(const std::vector<uint8_t>& raw_data)
  {
    AngleLimits angle_limits;
    // TODO: we should ensure that min is less than max
    angle_limits.min = (unsigned int)(raw_data.at(5) + (raw_data.at(6) << 8));
    angle_limits.max = (unsigned int)(raw_data.at(7) + (raw_data.at(8) << 8));
    return angle_limits;
  };

  static std::vector<uint8_t> getRawData(const unsigned int& min_angle, const unsigned int& max_angle)
  {
    uint8_t min_angle_low = (min_angle % 256);
    uint8_t min_angle_high = (min_angle >> 8);

    uint8_t max_angle_low = (max_angle % 256);
    uint8_t max_angle_high = (max_angle >> 8);

    return std::vector<uint8_t>({ min_angle_low, min_angle_high, max_angle_low, max_angle_high });
  };
};
struct ServoPosition
{
  unsigned int position;
  static unsigned int getData(const std::vector<uint8_t>& raw_data)
  {
    return (unsigned int)(raw_data.at(5) + (raw_data.at(6) << 8));
  };

  static std::vector<uint8_t> getRawData(const unsigned int& position)
  {
    return std::vector<uint8_t>({ (uint8_t)(position % 256), (uint8_t)(position >> 8) });
  };
};

struct ServoSpeed
{
  unsigned int speed;
  static unsigned int getData(const std::vector<uint8_t>& raw_data)
  {
    unsigned int speed = (unsigned int)(raw_data.at(5) + (raw_data.at(6) << 8));
    if (speed > 1023)
    {
      speed = 1023;
    }
    return speed;
  };
  static std::vector<uint8_t> getRawData(const unsigned int& speed)
  {
    return std::vector<uint8_t>({ (uint8_t)(speed % 256), (uint8_t)(speed >> 8) });
  };
};

struct ServoAcceleration
{
  unsigned int acceleration;
  static unsigned int getData(const std::vector<uint8_t>& raw_data)
  {
    return (unsigned int)(raw_data.at(5) + (raw_data.at(6) << 8));
  }

  static std::vector<uint8_t> getRawData(const unsigned int& acceleration)
  {
    return std::vector<uint8_t>({ (uint8_t)(acceleration % 256), (uint8_t)(acceleration >> 8) });
  }
};

struct TorqueLimit
{
  unsigned int limit;
  static unsigned int getData(const std::vector<uint8_t>& raw_data)
  {
    return (unsigned int)(raw_data.at(5) + (raw_data.at(6) << 8));
  }
  static std::vector<uint8_t> getRawData(const unsigned int& torque)
  {
    return std::vector<uint8_t>({ (uint8_t)(torque % 256), (uint8_t)(torque >> 8) });
  }
};

struct ServoFeedback
{
  unsigned long timestamp;
  unsigned int id;
  unsigned int goal;
  unsigned int position;
  int error;
  unsigned int speed;

  double voltage;
  unsigned int temperature;
  bool moving;

  static ServoFeedback getData(const uint8_t& servo_id, const std::vector<uint8_t>& raw_data, const unsigned long& timestamp)
  {
    ServoFeedback processed_data;
    std::cout << raw_data.size() << std::endl;
    if (raw_data.size() != 23)
    {
      return processed_data;
    }

    processed_data.timestamp = timestamp;
    processed_data.id = (unsigned int)servo_id;
    processed_data.goal = (unsigned int)(raw_data.at(5) + (raw_data.at(6) << 8));
    processed_data.position = (unsigned int)(raw_data.at(11) + (raw_data.at(12) << 8));
    processed_data.error = (processed_data.position - processed_data.goal);
    processed_data.speed = (unsigned int)(raw_data.at(13) + (raw_data.at(14) << 8));
    if (processed_data.speed > 1023)
    {
      processed_data.speed = 1023;
    }
    processed_data.voltage = (double)raw_data.at(17) / 10.0;
    processed_data.temperature = (unsigned int)raw_data.at(18);
    processed_data.moving = (bool)raw_data.at(21);

    return processed_data;
  };

  friend std::ostream& operator<<(std::ostream& stream, const ServoFeedback& feedback)
  {
    std::cout << "Servo ID: " << feedback.id << std::endl;
    std::cout << "Goal: " << feedback.goal << std::endl;
    std::cout << "Position: " << feedback.position << std::endl;
    std::cout << "Error: " << feedback.error << std::endl;
    std::cout << "Speed: " << feedback.speed << std::endl;
    std::cout << "Voltage: " << feedback.voltage << "V" << std::endl;
    std::cout << "Temperature: " << feedback.temperature << "°C" << std::endl;
    std::cout << "Moving: " << feedback.moving;
  };
};
};  // namespace HansCuteRobot

#endif