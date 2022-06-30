/*
 * Allows to set arbitrary speed for the serial device on Linux.
 * 
 */
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include "serial/serial.h"

int main(int argc, char* argv[]) 
{
    serial::Serial my_serial("/dev/ttyUSB0", 250000, serial::Timeout::simpleTimeout(1000));

    std::cout << "Is the serial port open?";
    if(my_serial.isOpen())
        std::cout << " Yes." << std::endl;
    else
        std::cout << " No." << std::endl;
    return 0;
}