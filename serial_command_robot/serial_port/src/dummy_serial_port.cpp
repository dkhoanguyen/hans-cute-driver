#include "serial_port/dummy_serial_port.h"

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
  write_data_stream_ = data;
}

void DummySerialPort::wait()
{
}

unsigned int DummySerialPort::read(std::vector<uint8_t> &data)
{
  data = read_data_stream_;
  return read_data_stream_.size();
}

int DummySerialPort::available()
{
  return read_data_stream_.size();
}

void DummySerialPort::getWriteDataStream(std::vector<uint8_t> &data)
{
  data = write_data_stream_;
}

void DummySerialPort::setReadDataStream(const std::vector<uint8_t> &data)
{
  read_data_stream_ = data;
}