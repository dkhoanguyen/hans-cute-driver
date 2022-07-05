#include "hans_cute_driver/hans_cute_driver.h"

HansCuteRobot::Driver::Driver(const std::string port, const long baudrate) : HansCuteRobot::SerialComms(port, baudrate)
{
}

HansCuteRobot::Driver::~Driver()
{
  std::cout << "Driver destructor" << std::endl;
}