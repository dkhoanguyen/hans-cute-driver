#include "custom_serial_port/dummy_serial_port.h"

DummySerialPort::DummySerialPort(const std::string &port, const unsigned int &baud_rate)
{
}

DummySerialPort::~DummySerialPort()
{
}

void DummySerialPort::openPort()
{
}

void DummySerialPort::closePort()
{
}

void DummySerialPort::write(const std::vector<uint8_t> &data)
{
  for (const uint8_t data_point : data)
  {
    std::cout << (int)data_point << std::endl;
  }
}

void DummySerialPort::wait()
{
}

unsigned int DummySerialPort::read(std::vector<uint8_t> &data)
{
}

int DummySerialPort::available()
{
  return 1;
}