// System includes
#include <iostream>
#include <sstream>
#include <stdio.h>   // Standard input/output definitions
#include <string.h>  // String function definitions
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
// #include <termios.h> 	// POSIX terminal control definitions (struct termios)
#include <system_error>  // For throwing std::system_error
#include <sys/ioctl.h>   // Used for TCGETS2, which is required for custom baud rates
#include <cassert>
// #include <asm/termios.h> // Terminal control definitions (struct termios)
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <algorithm>
#include <iterator>

#include "hans_cute_driver/custom_serial_port.h"

CustomSerialPort::CustomSerialPort() : CustomSerialPort("/dev/ttyUSB0", 1152300, 50)
{
}

CustomSerialPort::CustomSerialPort(const std::string& port, const speed_t& baud_rate, int32_t timeout)
  : port_(port)
  , baud_rate_(baud_rate)
  , timeout_(timeout)
  , num_data_bits_(NumDataBits::EIGHT)
  , parity_(Parity::NONE)
  , num_stop_bits_(NumStopBits::ONE)
  , is_open_(false)
{
  read_buffer_size_B_ = 255;
  read_buffer_.reserve(read_buffer_size_B_);
}

CustomSerialPort::~CustomSerialPort()
{
  try
  {
    closePort();
  }
  catch (const std::exception& e)
  {
  }
}

void CustomSerialPort::setPort(const std::string& port)
{
}

void CustomSerialPort::setBaudRate(const speed_t& baud_rate)
{
}

void CustomSerialPort::setNumDataBits(const NumDataBits& num_data_bits)
{
}

void CustomSerialPort::setParity(const Parity& parity)
{
}

void CustomSerialPort::setNumStopBits(const NumStopBits& num_stop_bits)
{
}

void CustomSerialPort::setTimeout(const int32_t& timeout)
{
}

void CustomSerialPort::openPort()
{
  if (port_.empty())
  {
    // Thow empty error here
  }

  file_desc_ = open(port_.c_str(), O_RDWR);

  if (file_desc_ == -1)
  {
    // Throw error here
    return;
  }
  configureTermios();
  is_open_ = true;
}

void CustomSerialPort::closePort()
{
  if (file_desc_ != -1)
  {
    int retVal = close(file_desc_);
    if (retVal != 0)
      return;

    file_desc_ = -1;
  }
}

bool CustomSerialPort::isOpen() const
{
  return is_open_;
}

void CustomSerialPort::writeData(const std::vector<uint8_t>& data)
{
  int writeResult = write(file_desc_, data.data(), data.size());
}

void CustomSerialPort::readData(std::vector<uint8_t>& data)
{
  data.clear();
  if (file_desc_ == 0)
  {
    return;
  }
  ssize_t n = read(file_desc_, &read_buffer_[0], read_buffer_size_B_);
  if (n > 0)
  {
    copy(read_buffer_.begin(), read_buffer_.begin() + n, back_inserter(data));
  }
}

// Private
void CustomSerialPort::configureTermios()
{
  termios2 tty = getTermios2();

  //================= (.c_cflag) ===============//
  // Set num data bits
  tty.c_cflag &= ~CSIZE;  // CSIZE is a mask for the number of bits per character
  switch (num_data_bits_)
  {
    case NumDataBits::FIVE:
      tty.c_cflag |= CS5;
      break;
    case NumDataBits::SIX:
      tty.c_cflag |= CS6;
      break;
    case NumDataBits::SEVEN:
      tty.c_cflag |= CS7;
      break;
    case NumDataBits::EIGHT:
      tty.c_cflag |= CS8;
      break;
    default:
      return;
      //   THROW_EXCEPT("numDataBits_ value not supported!");
  }
  // Set parity
  // See https://man7.org/linux/man-pages/man3/tcflush.3.html
  switch (parity_)
  {
    case Parity::NONE:
      tty.c_cflag &= ~PARENB;
      break;
    case Parity::EVEN:
      tty.c_cflag |= PARENB;
      tty.c_cflag &= ~PARODD;  // Clearing PARODD makes the parity even
      break;
    case Parity::ODD:
      tty.c_cflag |= PARENB;
      tty.c_cflag |= PARODD;
      break;
    default:
      return;
      //   THROW_EXCEPT("parity_ value not supported!");
  }

  // Set num. stop bits
  switch (num_stop_bits_)
  {
    case NumStopBits::ONE:
      tty.c_cflag &= ~CSTOPB;
      break;
    case NumStopBits::TWO:
      tty.c_cflag |= CSTOPB;
      break;
    default:
      return;
      //   THROW_EXCEPT("numStopBits_ value not supported!");
  }

  tty.c_cflag &= ~CRTSCTS;        // Disable hadrware flow control (RTS/CTS)
  tty.c_cflag |= CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_cflag &= ~CBAUD;
  tty.c_cflag |= CBAUDEX;
  // tty.c_cflag |= BOTHER;
  tty.c_ispeed = baud_rate_;
  tty.c_ospeed = baud_rate_;

  //===================== (.c_oflag) =================//

  tty.c_oflag = 0;        // No remapping, no delays
  tty.c_oflag &= ~OPOST;  // Make raw

  if (timeout_ == -1)
  {
    // Always wait for at least one byte, this could
    // block indefinitely
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 1;
  }
  else if (timeout_ == 0)
  {
    // Setting both to 0 will give a non-blocking read
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 0;
  }
  else if (timeout_ > 0)
  {
    tty.c_cc[VTIME] = (cc_t)(timeout_ / 100);  // 0.5 seconds read timeout
    tty.c_cc[VMIN] = 0;
  }

  //======================== (.c_iflag) ====================//

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  //=========================== LOCAL MODES (c_lflag) =======================//

  tty.c_lflag &= ~ICANON;  // Turn off canonical input, which is suitable for pass-through
                           // Configure echo depending on echo_ boolean
  if (echo_)
  {
    tty.c_lflag |= ECHO;
  }
  else
  {
    tty.c_lflag &= ~(ECHO);
  }
  tty.c_lflag &= ~ECHOE;   // Turn off echo erase (echo erase only relevant if canonical input is active)
  tty.c_lflag &= ~ECHONL;  //
  tty.c_lflag &= ~ISIG;    // Disables recognition of INTR (interrupt), QUIT and SUSP (suspend) characters

  setTermios2(tty);
}

termios2 CustomSerialPort::getTermios2()
{
  struct termios2 term2;
  ioctl(file_desc_, TCGETS2, &term2);
  return term2;
}

void CustomSerialPort::setTermios2(termios2 tty)
{
  ioctl(file_desc_, TCSETS2, &tty);
}