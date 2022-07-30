#include "hans_cute_driver/serial_command.h"

SerialCommand::SerialCommand(const std::string port, const long baudrate)
    : port_(port), baudrate_(baudrate), timeout_(30), num_tries_(5)
{
  // serial_port_ = std::make_shared<SerialPort>(port, baudrate, timeout_);

  // TODO:Register sigterm handler
  //  signal(SIGTERM,SerialCommand::sigTermHandler);
}

SerialCommand::SerialCommand() : SerialCommand("/dev/ttyUSB0", 115200)
{
}

SerialCommand::~SerialCommand()
{
  std::cout << "SerialCommand Destructor" << std::endl;
  close();
}

void SerialCommand::open() const
{
  try
  {
    serial_port_->openPort();
  }
  catch (const std::exception &e)
  {
    // We should catch errors here
  }
}

void SerialCommand::close() const
{
  serial_port_->closePort();
}

bool SerialCommand::readResponse(std::vector<uint8_t> &response)
{
  std::unique_lock<std::mutex> lck(comms_mtx_);
  std::vector<uint8_t> returned_data;
  if (serial_port_->available() == 0)
  {
    return false;
  }

  // To support testing this should be a separate function
  unsigned int num_byte_read = 0;
  for (int i = 0; i < num_tries_; i++)
  {
    num_byte_read = serial_port_->read(returned_data);
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
  for (int i = 0; i < sample_packet_.headers.size(); i++)
  {
    if (returned_data.at(i) != sample_packet_.headers.at(i))
    {
      return false;
    }
  }
  // Then verify checksum
  if (calcCheckSum(returned_data) != returned_data.at(returned_data.size() - 1))
  {
    std::cout << "ChecksumError" << std::endl;
    return false;
  }

  response = returned_data;

  return true;
}

bool SerialCommand::writeCommand(const std::vector<uint8_t> &command)
{
  std::unique_lock<std::mutex> lck(comms_mtx_);
  try
  {
    serial_port_->write(command);
    serial_port_->wait();
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
  sample_packet_ = sample_packet;
}

void SerialCommand::setSerialPort(const std::shared_ptr<SerialPortInterface> &serial_port)
{
  serial_port_ = serial_port;
}