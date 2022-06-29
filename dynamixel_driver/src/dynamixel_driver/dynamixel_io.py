#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010-2011, Cody Jorgensen, Antons Rebguns.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import time
import serial
from array import array
from threading import RLock

from . dynamixel_const import *
from . dynamixel_exception import *


class DynamixelIO(object):
    """ Provides low level IO with the Dynamixel servos through pyserial. Has the
    ability to write instruction packets, request and read register value
    packets, send and receive a response to a ping packet, and send a SYNC WRITE
    multi-servo instruction packet.
    """

    def __init__(self, port: str, baudrate: int, readback_echo: bool = False):
        """ Constructor takes serial port and baudrate as arguments. """
        try:
            self._serial_lock = RLock()
            self._serial = serial.Serial(port, baudrate, timeout=0.015)
            self._port = port
            self._baudrate = baudrate
            self._readback_echo = readback_echo
        except SerialOpenError:
            raise SerialOpenError(port, baudrate)

    def close(self):
        if self._serial:
            self._serial.flushInput()
            self._serial.flushOutput()
            self._serial.close()

    def _write(self, data: bytearray):
        self._serial.flushInput()
        self._serial.flushOutput()
        self._serial.write(data)
        if self._readback_echo:
            self._serial.read(len(data))

    def _read_response(self, servo_id: int) -> list:
        data = list()
        try:
            data.extend(self._serial.read(4))
            if not data[0:2] == ['\xff', '\xff']:
                raise Exception(f"Wrong packet prefix {data[0:2]}")
            data.extend(self._serial.read(ord(data[3])))
            # [int(b2a_hex(byte), 16) for byte in data]
            data = array('B', ''.join(data)).tolist()
        except Exception as e:
            raise DroppedPacketError(
                f"Invalid response received from motor {servo_id}. {e}")
        # verify checksum
        checksum = 255 - sum(data[2:-1]) % 256
        if not checksum == data[-1]:
            raise ChecksumError(servo_id, data, checksum)

        return data

    def read(self, servo_id: int, address: int, size: int) -> list:
        """ Read "size" bytes of data from servo with "servo_id" starting at the
        register with "address". "address" is an integer between 0 and 57. It is
        recommended to use the constants in module dynamixel_const for readability.

        To read the position from servo with id 1, the method should be called
        like:
            read(1, DXL_GOAL_POSITION_L, 2)
        """
        # Number of bytes following standard header (0xFF, 0xFF, id, length)
        length = 4  # instruction, address, size, checksum

        # directly from AX-12 manual:
        # Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
        # If the calculated value is > 255, the lower byte is the check sum.
        checksum = 255 - \
            ((servo_id + length + DXL_READ_DATA + address + size) % 256)

        # packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
        packet = [0xFF, 0xFF, servo_id, length,
                  DXL_READ_DATA, address, size, checksum]
        # same as: packetStr = ''.join([chr(byte) for byte in packet])
        packet_str = array('B', packet).tostring()

        with self._serial_lock:
            self._write(packet_str)

            # wait for response packet from the motor
            timestamp = time.time()
            time.sleep(0.0013)  # 0.00235)

            # read response
            data = self._read_response(servo_id)
            data.append(timestamp)

        return data

    def write(self, servo_id: int, address: int, data: list) -> list:
        """ Write the values from the "data" list to the servo with "servo_id"
        starting with data[0] at "address", continuing through data[n-1] at
        "address" + (n-1), where n = len(data). "address" is an integer between
        0 and 49. It is recommended to use the constants in module dynamixel_const
        for readability. "data" is a list/tuple of integers.

        To set servo with id 1 to position 276, the method should be called
        like:
            write(1, DXL_GOAL_POSITION_L, (20, 1))
        """
        # Number of bytes following standard header (0xFF, 0xFF, id, length)
        length = 3 + len(data)  # instruction, address, len(data), checksum

        # directly from AX-12 manual:
        # Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
        # If the calculated value is > 255, the lower byte is the check sum.
        checksum = 255 - \
            ((servo_id + length + DXL_WRITE_DATA + address + sum(data)) % 256)

        # packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
        packet = [0xFF, 0xFF, servo_id, length, DXL_WRITE_DATA, address]
        packet.extend(data)
        packet.append(checksum)

        # packetStr = ''.join([chr(byte) for byte in packet])
        packet_str = array('B', packet).tostring()

        with self._serial_lock:
            self._write(packet_str)

            # wait for response packet from the motor
            timestamp = time.time()
            time.sleep(0.0013)

            # read response
            data = self._read_response(servo_id)
            data.append(timestamp)

        return data

    def sync_write(self, address: int, data: list):
        """ Use Broadcast message to send multiple servos instructions at the
        same time. No "status packet" will be returned from any servos.
        "address" is an integer between 0 and 49. It is recommended to use the
        constants in module dynamixel_const for readability. "data" is a tuple of
        tuples. Each tuple in "data" must contain the servo id followed by the
        data that should be written from the starting address. The amount of
        data can be as long as needed.

        To set servo with id 1 to position 276 and servo with id 2 to position
        550, the method should be called like:
            sync_write(DXL_GOAL_POSITION_L, ( (1, 20, 1), (2 ,38, 2) ))
        """
        # Calculate length and sum of all data
        flattened = [value for servo in data for value in servo]

        # Number of bytes following standard header (0xFF, 0xFF, id, length) plus data
        length = 4 + len(flattened)

        checksum = 255 - ((DXL_BROADCAST + length +
                          DXL_SYNC_WRITE + address + len(data[0][1:]) +
                          sum(flattened)) % 256)

        # packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
        packet = [0xFF, 0xFF, DXL_BROADCAST, length,
                  DXL_SYNC_WRITE, address, len(data[0][1:])]
        packet.extend(flattened)
        packet.append(checksum)

        # packetStr = ''.join([chr(byte) for byte in packet])
        packet_str = array('B', packet).tostring()

        with self._serial_lock:
            self._write(packet_str)

    def ping(self, servo_id: int):
        """ Ping the servo with "servo_id". This causes the servo to return a
        "status packet". This can tell us if the servo is attached and powered,
        and if so, if there are any errors.
        """
        # Number of bytes following standard header (0xFF, 0xFF, id, length)
        length = 2  # instruction, checksum

        # directly from AX-12 manual:
        # Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
        # If the calculated value is > 255, the lower byte is the check sum.
        checksum = 255 - ((servo_id + length + DXL_PING) % 256)

        # packet: FF  FF  ID LENGTH INSTRUCTION CHECKSUM
        packet = [0xFF, 0xFF, servo_id, length, DXL_PING, checksum]
        packet_str = array('B', packet).tostring()

        with self._serial_lock:
            self._write(packet_str)

            # wait for response packet from the motor
            timestamp = time.time()
            time.sleep(0.0013)

            # read response
            try:
                response = self._read_response(servo_id)
                print(response)
                response.append(timestamp)
            except Exception as e:
                response = []

        if response:
            self.exception_on_error(response[4], servo_id, 'ping')
        return response

    def test_bit(self, number, offset):
        mask = 1 << offset
        return (number & mask)

    def exception_on_error(self, error_code: int, servo_id: int, command_failed: str):
        exception = None
        ex_message = '[servo #%d on %s@%sbps]: %s failed' % (servo_id, self._port, self._baudrate, command_failed)

        if not isinstance(error_code, int):
            msg = 'Communcation Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, 0)
            return
        if not error_code & DXL_OVERHEATING_ERROR == 0:
            msg = 'Overheating Error ' + ex_message
            exception = FatalErrorCodeError(msg, error_code)
        if not error_code & DXL_OVERLOAD_ERROR == 0:
            msg = 'Overload Error ' + ex_message
            exception = FatalErrorCodeError(msg, error_code)
        if not error_code & DXL_INPUT_VOLTAGE_ERROR == 0:
            msg = 'Input Voltage Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & DXL_ANGLE_LIMIT_ERROR == 0:
            msg = 'Angle Limit Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & DXL_RANGE_ERROR == 0:
            msg = 'Range Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & DXL_CHECKSUM_ERROR == 0:
            msg = 'Checksum Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & DXL_INSTRUCTION_ERROR == 0:
            msg = 'Instruction Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)

