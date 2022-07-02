#include "hans_cute_driver/serial_command.h"

SerialCommand::SerialCommand(const std::string port, const long baudrate)
  : _port(port), _baudrate(baudrate), _timeout(1), _num_tries(5)
{
  _serial_port = std::make_shared<serial::Serial>(port, baudrate);
}

SerialCommand::SerialCommand() : _port("/dev/ttyUSB0"), _baudrate(115200), _timeout(1), _num_tries(5)
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
    _serial_port->open();
  }
  catch (const serial::SerialException& se)
  {
    // We should catch errors here
  }
}

void SerialCommand::close() const
{
  _serial_port->close();
}

bool SerialCommand::readResponse(std::vector<uint8_t>& response) const
{
  std::vector<uint8_t> returned_data;
  if (_serial_port->available() == 0)
  {
    return false;
  }

  unsigned int num_byte_read = 0;
  for (int i = 0; i < _num_tries; i++)
  {
    num_byte_read = _serial_port->read(returned_data, _serial_port->available());
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
  for (int i = 0; i < _sample_packet->headers.size(); i++)
  {
    if (returned_data.at(i) != _sample_packet->headers.at(i))
    {
      return false;
    }
  }

  // Then verify checksum
  if (calcCheckSum(returned_data) != returned_data.at(returned_data.size() - 1))
  {
    return false;
  }

  returned_data = response;

  return true;
}

bool SerialCommand::writeCommand(const std::vector<uint8_t>& command) const
{
  try
  {
    _serial_port->write(command);
    _serial_port->waitReadable();
    return true;
  }
  catch (const serial::SerialException &se)
  {
    // Handle errors
    return false;
  }
}

void SerialCommand::setPacket(const std::shared_ptr<SamplePacket> sample_packet)
{
  _sample_packet = sample_packet;
}