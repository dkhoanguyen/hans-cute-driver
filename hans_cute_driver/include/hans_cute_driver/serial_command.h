#ifndef _SERIAL_COMMAND_H_
#define _SERIAL_COMMAND_H_

#include <iostream>
#include <mutex>
#include <string>
#include <vector>
#include <cstring>
#include <limits>
#include <cmath>

#include <signal.h>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <libserial/SerialStreamBuf.h>

#include "serial/serial.h"

#include "serial_command_interface.h"

class SerialCommand : public SerialCommandInterface 
{
    public:
        SerialCommand(const std::string port, const long baudrate);
        ~SerialCommand();

        void open();
        void close();

        void readResponse();
        void write();
};

#endif