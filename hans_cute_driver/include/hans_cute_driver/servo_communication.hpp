#ifndef HANS_CUTE_DRIVER__HANS_COMMUNICATION_HPP_
#define HANS_CUTE_DRIVER__HANS_COMMUNICATION_HPP_

#include <string>
#include <vector>
#include <libusb.h>

#include "hans_cute_datatype.h"
#include "serial_port.hpp"

namespace HansCuteRobot
{
  class ServoComms
  {
  public:
    ServoComms();
    ~ServoComms();

    bool open(const uint16_t &vendor_id, const uint16_t &product_id);
    bool isOpen();
    
  protected:
    bool read(const uint8_t &id, const uint8_t &address, const uint8_t &size, std::vector<uint8_t> &returned_data,
              unsigned long &timestamp);
    bool write(const uint8_t &id, const uint8_t &address, const std::vector<uint8_t> &data,
               std::vector<uint8_t> &returned_data, unsigned long &timestamp);
    bool syncWrite(const uint8_t &address, const std::vector<std::vector<uint8_t>> &data);

    bool ping(const uint8_t &id, std::vector<uint8_t> &returned_data);

    uint8_t calcCheckSum(std::vector<uint8_t> &data) const;

    SerialPort port_;
  };
};

#endif