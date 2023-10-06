#include <iostream>
#include <unordered_map>
#include "hans_cute_driver/serial_port_manager.hpp"
#include "hans_cute_driver/servo_communication.hpp"
#include "hans_cute_driver/hans_cute_driver.hpp"

int main()
{
  SerialPortManager manager;
  manager.startMonitoring();
  while (manager.serialPortAvailable("0403", "6001").empty());
  std::string port = manager.serialPortAvailable("0403", "6001");
  HansCuteRobot::HansCuteDriver hans_driver;
  hans_driver.init(port);
  hans_driver.start();
  while(true)
  {
    std::unordered_map<std::string, double> joint_state;
    hans_driver.getJointStates(joint_state);
    std::cout << "Joint States" << std::endl;
    for (auto it = joint_state.begin(); it != joint_state.end(); ++it)
    {
      std::cout << it->first << ": " << it->second << std::endl;
    }
    std::cout << "==" << std::endl;
    
  }
  // servo_comms.open(port);
  // for (int i = 0; i <= 6; i++)
  // {
  //   std::vector<uint8_t> response;
  //   unsigned int position = 0;
  //   servo_comms.setTorqueEnable(i,true);
  //   servo_comms.getPosition(i, position);
  //   servo_comms.setPosition(i, position);
  //   servo_comms.setPosition(i, 2048);
  // }
  // servo_comms.close();
  return 0;
}
