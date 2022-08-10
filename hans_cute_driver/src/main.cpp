#include <iostream>

#include "hans_cute_driver/hans_cute_driver.h"
#include "hans_cute_driver/hans_cute_datatype.h"

#include "custom_serial_port/custom_serial_port.h"
#include "custom_serial_port/serial_port_interface.h"

int main(int argc, char *argv[])
{
  std::shared_ptr<SerialPortInterface> serial_port = std::make_shared<SerialPort>("/dev/ttyUSB0", 250000, 50);
  HansCuteRobot::ServoDriver hans_robot;
  hans_robot.setSerialPort(serial_port);
  hans_robot.open();

  // std::vector<unsigned int> servo_ids;
  // std::vector<unsigned int> positions;
  // std::vector<unsigned int> speeds;

  // for (int i = 0; i <= 6; i++)
  // {
  //   servo_ids.push_back(i);
  //   positions.push_back(2048);
  //   speeds.push_back(1023);

  //   unsigned int max_torque = 0;
  //   hans_robot.getMaxTorque(i,max_torque);
  //   std::cout << max_torque << std::endl;

  //   hans_robot.setTorqueEnable(i, true);

  //   unsigned int position = 0;
  //   hans_robot.getPosition(i, position);
  //   hans_robot.setPosition(i, position);
  //   hans_robot.setSpeed(i,512);
  //   unsigned int speed = 0;
  //   hans_robot.getSpeed(i, speed);
  //   // std::cout << speed << std::endl;
  //   hans_robot.setTorqueLimit(i,1);
  //   hans_robot.setPosition(i, 2048);
  // }

  // std::this_thread::sleep_for(std::chrono::milliseconds((2500)));
  // hans_robot.setMultiSpeed(servo_ids,speeds);
  // hans_robot.setPosition(5, 3000);

  // std::this_thread::sleep_for(std::chrono::milliseconds((2500)));
  // hans_robot.setMultiSpeed(servo_ids,speeds);
  // hans_robot.setPosition(5, 900);

  // hans_robot.close();
  return 0;
}