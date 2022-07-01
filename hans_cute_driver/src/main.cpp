/*
 * Allows to set arbitrary speed for the serial device on Linux.
 *
 */
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <thread>

#include "serial/serial.h"

int main(int argc, char* argv[])
{
  serial::Serial my_serial("/dev/ttyUSB0", 250000, serial::Timeout::simpleTimeout(1000));

  std::cout << "Is the serial port open?";
  if (my_serial.isOpen())
    std::cout << " Yes." << std::endl;
  else
    std::cout << " No." << std::endl;

  std::vector<uint8_t> data = { 0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB };
  std::string test = "FFFF010201FB";
  my_serial.write(data);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  std::cout << my_serial.available() << std::endl;

  my_serial.write(test);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  std::cout << my_serial.available() << std::endl;

  std::vector<uint8_t> out_data;
  unsigned int t = my_serial.read(out_data,my_serial.available());
  std::cout << out_data.size() << std::endl;

  for(auto val : out_data) 
  {
    std::cout << (unsigned int)val << std::endl;
  }

  std:: cout << t << std::endl;
  return 0;
}