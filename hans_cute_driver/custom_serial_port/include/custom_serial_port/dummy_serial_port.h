#ifndef _DUMMY_SERIAL_PORT_H_
#define _DUMMY_SERIAL_PORT_H_

#include <string>
#include <iostream>

#include "serial_port_interface.h"

class DummySerialPort : public SerialPortInterface
{
public:
  DummySerialPort(const std::string& port, const unsigned int& baud_rate);
  ~DummySerialPort();

  void openPort();
  void closePort();

  void write(const std::vector<uint8_t> &data);
  void wait();

  unsigned int read(std::vector<uint8_t> &data);
  int available();

private:
  std::string port_;
  unsigned int baud_rate_;
};

#endif