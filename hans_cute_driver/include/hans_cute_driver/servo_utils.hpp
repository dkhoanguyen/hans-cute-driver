#ifndef HANS_CUTE_DRIVER__SERIAL_UTILS_HPP_
#define HANS_CUTE_DRIVER__SERIAL_UTILS_HPP_

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>

namespace HansCuteRobot
{
  void unpackFloats(const std::vector<uint8_t>::iterator &it, float &output);
  void floatToByte(float float_variable, uint8_t temp_bytes[]);
  void doubleToByte(double double_variable, uint8_t temp_bytes[]);
  void packFromFloats(const std::vector<float> &value_to_pack, std::vector<uint8_t> &packed_floats);
  void packFromDoubles(const std::vector<double> &value_to_pack, std::vector<uint8_t> &packed_doubles);
}

#endif