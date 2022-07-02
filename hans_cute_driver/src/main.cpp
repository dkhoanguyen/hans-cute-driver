/*
 * Allows to set arbitrary speed for the serial device on Linux.
 *
 */
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <thread>

#include "hans_cute_driver/SerialPort.hpp"

using namespace mn::CppLinuxSerial;

int main(int argc, char* argv[])
{
  //   CustomSerialPort serial_port("/dev/ttyUSB0", 250000, 50);
  //   serial_port.openPort();
  //   std::vector<uint8_t> data = { 0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB };
  //   serial_port.writeData(data);
  //   std::vector<uint8_t> outData;
  //   serial_port.readData(outData);

  //   std::cout << "Received data size:" << outData.size() << std::endl;

  //   for (uint8_t data : outData)
  //   {
  //     std::cout << "Received data:" << (int)data << std::endl;
  //   }

  //   serial_port.closePort();
  SerialPort serialPort("/dev/ttyUSB0", 250000);
  serialPort.SetTimeout(-1);  // Block when reading until any data is received
  serialPort.Open();

  std::vector<uint8_t> data = { 0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB };
  serialPort.WriteBinary(data);
  std::vector<uint8_t> outData;
  std::string readData;
  serialPort.ReadBinary(outData);
  std::cout << "Received data size:" << outData.size() << std::endl;

  for (uint8_t data : outData)
  {
    std::cout << "Received data:" << (int)data << std::endl;
  }
  serialPort.Close();
  return 0;
}
