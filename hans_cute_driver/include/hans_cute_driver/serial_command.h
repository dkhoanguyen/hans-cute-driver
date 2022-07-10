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

// New native serial library
#include "custom_serial_port.h"

#include "serial_command_interface.h"

struct SamplePacket
{
  std::vector<uint8_t> headers;  // Headers in HEX
  unsigned int id;               // Position of the id byte
  unsigned int length;           // Position of length byte
  unsigned int data;             // Start position of data byte
};

class SerialCommand : public SerialCommandInterface
{
public:
  SerialCommand(const std::string port, const long baudrate);
  /**
   * @brief Default constructor
   *
   */
  SerialCommand();
  virtual ~SerialCommand();

  virtual void open() const;
  virtual void close() const;

  void setSamplePacket(const SamplePacket sample_packet);

  virtual bool readResponse(std::vector<uint8_t>& response) override;
  virtual bool writeCommand(const std::vector<uint8_t>& command) override;

  virtual uint8_t calcCheckSum(std::vector<uint8_t>& data) const = 0;

protected:
  std::shared_ptr<SerialPort> _serial_port;
  std::string _port;
  unsigned int _baudrate;
  unsigned int _timeout;

  unsigned int _num_tries;

  SamplePacket _sample_packet;

  std::mutex _comms_mtx;
};

#endif