/*
 * Allows to set arbitrary speed for the serial device on Linux.
 *
 */
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

// #include "hans_cute_driver/SerialPort.hpp"
#include "hans_cute_driver/custom_serial_port.h"
#include "hans_cute_driver/hans_cute_serial.h"

int main(int argc, char* argv[])
{
  HansCuteSerial hans_robot("/dev/ttyUSB0", 250000);
  hans_robot.open();
  
  SamplePacket packet;
  packet.headers = std::vector<uint8_t>({0xFF,0xFF});
  hans_robot.setSamplePacket(packet);

  std::vector<uint8_t> data = { 0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB };
  std::cout << hans_robot.writeCommand(data) << std::endl;

  std::vector<uint8_t> outData;
  hans_robot.readResponse(outData);

  std::cout << "Received data size:" << outData.size() << std::endl;

  for (uint8_t data : outData)
  {
    std::cout << "Received data:" << (int)data << std::endl;
  }

  hans_robot.close();
  return 0;
}
