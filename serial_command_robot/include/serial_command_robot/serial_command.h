#ifndef _SERIAL_COMMAND_H_
#define _SERIAL_COMMAND_H_

#include <iostream>
#include <mutex>
#include <string>
#include <vector>
#include <cstring>
#include <limits>
#include <cmath>
#include <thread>
#include <atomic>
#include <csignal>

// New native serial library
#include "serial_port/serial_port_interface.h"

#include "serial_command_interface.h"

struct SamplePacket
{
  std::vector<uint8_t> headers; // Headers in HEX
  unsigned int id;              // Position of the id byte
  unsigned int length;          // Position of length byte
  unsigned int data;            // Start position of data byte
};

enum class SerialCommandError
{
  NO_ERROR,

  OPEN_ERROR,
  CLOSE_ERROR,
  
  READ_ERROR,
  WRITE_ERROR,

  NO_RESPONSE,
  WRONG_HEADER,
  WRONG_CHECKSUM,
};

class SerialCommand : public SerialCommandInterface
{
  public:
  SerialCommand(const unsigned int &timeout, const unsigned int &num_tries);
  SerialCommand();
  virtual ~SerialCommand();

  virtual int open() const;
  virtual int close() const;

  void setSerialPort(const std::shared_ptr<SerialPortInterface> &serial_port);
  void setSamplePacket(const SamplePacket sample_packet);

  virtual int readResponse(std::vector<uint8_t> &response) override;
  virtual int writeCommand(const std::vector<uint8_t> &command) override;

  virtual uint8_t calcCheckSum(std::vector<uint8_t> &data) const = 0;

  static void packFromFloats(const std::vector<float> &value_to_pack, std::vector<uint8_t> &packed_doubles);
  static void packFromDoubles(const std::vector<double> &value_to_pack, std::vector<uint8_t> &packed_doubles);

  static void unpackFloats(const std::vector<uint8_t>::iterator &it, float &output);

  static void floatToByte(float float_variable, uint8_t temp_bytes[]);
  static void doubleToByte(double double_variable, uint8_t temp_bytes[]);

protected:
  std::shared_ptr<SerialPortInterface> serial_port_;
  unsigned int timeout_;
  unsigned int num_tries_;
  SamplePacket sample_packet_;
  std::mutex comms_mtx_;
};

#endif
