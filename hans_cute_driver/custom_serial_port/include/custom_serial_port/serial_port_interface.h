#ifndef _SERIAL_PORT_INTERFACE_H_
#define _SERIAL_PORT_INTERFACE_H_

#include <vector>

class SerialPortInterfaces
{
  public:
    SerialPortInterfaces(){};
    virtual ~SerialPortInterfaces(){};

    virtual void openPort() = 0;
    virtual void closePort() = 0;

    virtual void write(const std::vector<uint8_t>& data) = 0;
    virtual void wait() = 0;

    virtual unsigned int read(std::vector<uint8_t>& data) = 0;

};

#endif