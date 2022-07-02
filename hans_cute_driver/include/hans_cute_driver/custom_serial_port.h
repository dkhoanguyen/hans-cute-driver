#ifndef _CUSTOM_SERIAL_PORT_H_
#define _CUSTOM_SERIAL_PORT_H_

#include <string>
#include <fstream>  // For file I/O (reading/writing to COM port)
#include <sstream>

#include <vector>
#include <asm/ioctls.h>
#include <asm/termbits.h>

enum class NumDataBits
{
  FIVE,
  SIX,
  SEVEN,
  EIGHT,
};

enum class Parity
{
  NONE,
  EVEN,
  ODD,
};

enum class NumStopBits
{
  ONE,
  TWO,
};

class CustomSerialPort
{
public:
  CustomSerialPort();
  CustomSerialPort(const std::string& port, const speed_t& baud_rate, int32_t timeout);
  ~CustomSerialPort();

  // Setters
  void setPort(const std::string& port);
  void setBaudRate(const speed_t& baud_rate);
  void setNumDataBits(const NumDataBits& num_data_bits);
  void setParity(const Parity& parity);
  void setNumStopBits(const NumStopBits& num_stop_bits);
  void setTimeout(const int32_t& timeout);

  //
  void openPort();
  void closePort();

  bool isOpen() const;

  void writeData(const std::vector<uint8_t>& data);
  void readData(std::vector<uint8_t>& data);

private:
  void configureTermios();
  termios2 getTermios2();
  void setTermios2(termios2 tty);

  std::string port_;
  speed_t baud_rate_;

  NumDataBits num_data_bits_;
  Parity parity_;
  NumStopBits num_stop_bits_;

  int file_desc_;

  int32_t timeout_;

  bool is_open_;
  bool echo_;

  std::vector<char> read_buffer_;
  unsigned char read_buffer_size_B_;
};

#endif
