#include <iostream>
#include "hans_cute_driver/serial_port_manager.hpp"
#include "hans_cute_driver/servo_communication.hpp"

int main()
{
  SerialPortManager manager;
  manager.startMonitoring();
  while(manager.serialPortAvailable("0403","6001").empty());
  std::string port = manager.serialPortAvailable("0403","6001");
  HansCuteRobot::ServoComms servo_comms;
  std::cout << servo_comms.open(port) << std::endl;;
  for (int i = 0; i <= 0; i++)
  {
    std::vector<uint8_t> response;
    servo_comms.ping(i,response);
    unsigned int position = 0;
    std::cout << servo_comms.getPosition(i, position) << std::endl;
  //   std::cout << position << std::endl;
  //   servo_comms.setPosition(i, position);
  //   servo_comms.setPosition(i, 2048);
  }
  servo_comms.close();
  return 0;
}
