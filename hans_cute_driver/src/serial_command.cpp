#include "hans_cute_driver/serial_command.h"

SerialCommand::SerialCommand(const std::string port, const long baudrate)
  : _port(port), _baudrate(baudrate), _timeout(50), _num_tries(5)
{
  _serial_port = std::make_shared<SerialPort>(port, baudrate, _timeout);
}

SerialCommand::SerialCommand() : SerialCommand("/dev/ttyUSB0",115200)
{
}

SerialCommand::~SerialCommand()
{
  close();
}

void SerialCommand::open() const
{
  try
  {
    _serial_port->openPort();
  }
  catch (const std::exception& e)
  {
    // We should catch errors here
  }
}

void SerialCommand::close() const
{
  _serial_port->closePort();
}

bool SerialCommand::readResponse(std::vector<uint8_t>& response)
{
  std::unique_lock<std::mutex> lck(_comms_mtx);
  std::vector<uint8_t> returned_data;
  if (_serial_port->available() == 0)
  {
    return false;
  }

  // To support testing this should be a separate function
  unsigned int num_byte_read = 0;
  for (int i = 0; i < _num_tries; i++)
  {
    num_byte_read = _serial_port->readData(returned_data);
    // If we actually receive data
    if (num_byte_read > 0)
    {
      break;
    }
  }
  // Unable to get a response
  if (num_byte_read == 0)
  {
    return false;
  }

  // Verify header first
  for (int i = 0; i < _sample_packet.headers.size(); i++)
  {
    if (returned_data.at(i) != _sample_packet.headers.at(i))
    {
      return false;
    }
  }

  // Then verify checksum
  if (calcCheckSum(returned_data) != returned_data.at(returned_data.size() - 1))
  {
    return false;
  }

  response = returned_data;

  return true;
}

bool SerialCommand::writeCommand(const std::vector<uint8_t>& command)
{
  std::unique_lock<std::mutex> lck(_comms_mtx);
  try
  {
    _serial_port->writeData(command);
    _serial_port->waitData();
    return true;
  }
  catch (const std::exception &se)
  {
    // Handle errors
    return false;
  }
}

void SerialCommand::setSamplePacket(const SamplePacket sample_packet)
{
  _sample_packet = sample_packet;
}