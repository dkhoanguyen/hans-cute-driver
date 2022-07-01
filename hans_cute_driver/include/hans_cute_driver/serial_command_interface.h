#ifndef _SERIAL_COMMAND_INTERFACE_H_
#define _SERIAL_COMMAND_INTERFACE_H_

class SerialCommandInterface 
{
    public:
        SerialCommandInterface(){};
        virtual ~SerialCommandInterface(){};

        virtual void open() = 0;
        virtual void close() = 0;

        virtual void readResponse() = 0;
        virtual void writeCommand() = 0;

};


#endif