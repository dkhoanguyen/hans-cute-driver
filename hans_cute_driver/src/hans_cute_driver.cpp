#include "hans_cute_driver/hans_cute_driver.h"

HansCuteDriver::HansCuteDriver(const std::string port, const long baudrate) : HansCuteSerial(port, baudrate)
{
}

HansCuteDriver::~HansCuteDriver()
{
  std::cout << "HansCuteDriver destructor" << std::endl;
}