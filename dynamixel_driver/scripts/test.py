#!/usr/bin/env python3

from dynamixel_driver.dynamixel_driver import DynamixelDriver

if __name__ == '__main__':
    port = "/dev/ttyUSB0"
    baud_rate = 1000000

    driver = DynamixelDriver(baud_rate=250000)
    print(driver._baud_rate)
    driver.connect()

    print(driver.connected_motors)