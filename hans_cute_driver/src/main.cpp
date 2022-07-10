/*
 * Allows to set arbitrary speed for the serial device on Linux.
 *
 */
#include <iostream>

#include "hans_cute_driver/hans_cute_driver.h"
#include "hans_cute_driver/hans_cute_datatype.h"

int main(int argc, char* argv[])
{
  HansCuteRobot::ServoDriver hans_robot("/dev/ttyUSB0", 250000);
  hans_robot.open();

  SamplePacket packet;
  packet.headers = std::vector<uint8_t>({ 0xFF, 0xFF });
  packet.id = 2;
  packet.length = 3;
  hans_robot.setSamplePacket(packet);
  
  for (int i = 6; i <= 6; i++)
  {
    // std::vector<uint8_t> outData;
    // hans_robot.ping(i, outData);
    // std::cout << "Received data size:" << outData.size() << std::endl;

    // for (uint8_t data : outData)
    // {
    //   std::cout << "Received data:" << std::hex << (int)data << std::endl;
    // }
    HansCuteRobot::ServoFeedback feedback;
    hans_robot.getFeedback(i,feedback);
    std::cout << feedback << std::endl;
    std::cout << "=======" << std::endl;
    hans_robot.setAngleLimits(i,853,3243);
    hans_robot.setTorqueEnable(i,true);
    bool test = false;
    hans_robot.getTorqueEnabled(i,test);
    // hans_robot.getPosition(i);
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    hans_robot.setPosition(i,2077);
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // hans_robot.setPosition(i,2400);
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // hans_robot.getSpeed(i);
    // unsigned int model_number = 0;
    // hans_robot.getModelNumber(i,model_number);
    // std::cout <<(int) model_number << std::endl;
  }

  hans_robot.close();
  return 0;
}
