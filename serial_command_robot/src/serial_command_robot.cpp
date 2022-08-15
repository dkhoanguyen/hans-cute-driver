#include "serial_command_robot/serial_command_robot.h"

SerialCommandRobot::SerialCommandRobot(const unsigned int &timeout, const unsigned int &num_tries)
    : timeout_(timeout), num_tries_(num_tries)
{
  // serial_port_ = std::make_shared<SerialPort>(port, baudrate, timeout_);

  // TODO:Register sigterm handler
  //  signal(SIGTERM,SerialCommandRobot::sigTermHandler);
}

SerialCommandRobot::SerialCommandRobot() : SerialCommandRobot(30,5)
{
}

SerialCommandRobot::~SerialCommandRobot()
{
  // std::cout << "SerialCommandRobot Destructor" << std::endl;
  close();
}

int SerialCommandRobot::open() const
{
  try
  {
    serial_port_->openPort();
    return (int)SerialCommandError::NO_ERROR;
  }
  catch (const std::exception &e)
  {
    return (int)SerialCommandError::OPEN_ERROR;
  }
}

int SerialCommandRobot::close() const
{
  serial_port_->closePort();
}

int SerialCommandRobot::readResponse(std::vector<uint8_t> &response)
{
  std::unique_lock<std::mutex> lck(comms_mtx_);
  std::vector<uint8_t> returned_data;
  if (serial_port_->available() == 0)
  {
    return (int)SerialCommandError::NO_RESPONSE;
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
    return (int)SerialCommandError::READ_ERROR;
  }
  // Verify header first
  for (int i = 0; i < sample_packet_.headers.size(); i++)
  {
    if (returned_data.at(i) != sample_packet_.headers.at(i))
    {
      return (int)SerialCommandError::WRONG_HEADER;
    }
  }
  // Then verify checksum
  if (calcCheckSum(returned_data) != returned_data.at(returned_data.size() - 1))
  {
    return (int)SerialCommandError::WRONG_CHECKSUM;
  }

  response = returned_data;

  return (int)SerialCommandError::NO_ERROR;
}

int SerialCommandRobot::writeCommand(const std::vector<uint8_t> &command)
{
  std::unique_lock<std::mutex> lck(comms_mtx_);
  try
  {
    serial_port_->write(command);
    serial_port_->wait();
    return (int)SerialCommandError::WRONG_CHECKSUM;
  }
  catch (const std::exception &se)
  {
    // Handle errors
    return (int)SerialCommandError::WRITE_ERROR;
  }
}

void SerialCommandRobot::setSamplePacket(const SamplePacket sample_packet)
{
  sample_packet_ = sample_packet;
}

void SerialCommandRobot::setSerialPort(const std::shared_ptr<SerialPortInterface> &serial_port)
{
  serial_port_ = serial_port;
}