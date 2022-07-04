#ifndef _HANS_CUTE_DRIVER_H_
#define _HANS_CUTE_DRIVER_H_

#include "hans_cute_const.h"
#include "hans_cute_serial.h"

class HansCuteDriver : public HansCuteSerial
{
public:
  HansCuteDriver(const std::string port, const long baudrate);
  ~HansCuteDriver();
};

#endif