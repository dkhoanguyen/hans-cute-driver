#ifndef _SERIAL_PORT_INTERFACE_H_
#define _SERIAL_PORT_INTERFACE_H_

#include <vector>

class SerialPortInterface
{
  public:
    SerialPortInterface(){};
    virtual ~SerialPortInterface(){};

    virtual void openPort() = 0;
    virtual void closePort() = 0;

    virtual void write(const std::vector<uint8_t>& data) = 0;
    virtual void wait() = 0;

    virtual unsigned int read(std::vector<uint8_t>& data) = 0;

    virtual int available() = 0;

};

#endif