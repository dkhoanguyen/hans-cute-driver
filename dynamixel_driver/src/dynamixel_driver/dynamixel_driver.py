#!/usr/bin/python3
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

import math
import sys
import errno
import time
from collections import deque
from threading import RLock, Thread
from collections import defaultdict

from . dynamixel_protocol import DynamixelProtocol
from . dynamixel_exception import *
from . dynamixel_const import *


class MotorState(object):
    def __init__(self,
                 timestamp: float = 0.0,
                 id: int = 0,
                 goal: int = 0,
                 position: int = 0,
                 error: int = 0,
                 speed: int = 0,
                 load: float = 0.0,
                 voltage: float = 0.0,
                 temperature: int = 0.0,
                 moving: bool = False):

        self._timestamp = timestamp
        self._id = id
        self._goal = goal
        self._position = position
        self._error = error
        self._speed = speed
        self._load = load
        self._voltage = voltage
        self._temperature = temperature
        self._moving = moving

    @property
    def timestamp(self):
        return self._timestamp

    @timestamp.setter
    def timestamp(self, timestamp):
        self._timestamp = timestamp

    @property
    def id(self):
        return self._id

    @id.setter
    def id(self, id):
        self._id = id

    @property
    def goal(self):
        return self._goal

    @goal.setter
    def goal(self, goal):
        self._goal = goal

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, position):
        self._position = position

    @property
    def error(self):
        return self._timestamp

    @error.setter
    def error(self, error):
        self._error = error

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, speed):
        self._speed = speed

    @property
    def load(self):
        return self._load

    @load.setter
    def load(self, load):
        self._load = load

    @property
    def voltage(self):
        return self._voltage

    @voltage.setter
    def load(self, voltage):
        self._voltage = voltage

    @property
    def temperature(self):
        return self._temperature

    @temperature.setter
    def temperature(self, temperature):
        self._temperature = temperature

    @property
    def moving(self):
        return self._moving

    @moving.setter
    def moving(self, moving):
        self._moving = moving


class DynamixelDriver(object):
    def __init__(self,
                 port_name='/dev/ttyUSB0',
                 port_namespace='ttyUSB0',
                 baud_rate=1000000,
                 min_motor_id=1,
                 max_motor_id=25,
                 update_rate=5,
                 diagnostics_rate=1,
                 error_level_temp=75,
                 warn_level_temp=70,
                 readback_echo=False):
        self._port_name = port_name
        self._port_namespace = port_namespace
        self._baud_rate = baud_rate
        self._min_motor_id = min_motor_id
        self._max_motor_id = max_motor_id
        self._update_rate = update_rate
        self._diagnostics_rate = diagnostics_rate
        self._error_level_temp = error_level_temp
        self._warn_level_temp = warn_level_temp
        self._readback_echo = readback_echo

        self._actual_rate = update_rate
        self._error_counts = {"non_fatal": 0, "checksum": 0, "dropped": 0}
        self._num_ping_retries = 5
        self._running = True

        self._motor_params = {self._port_namespace: ""}

        self._motor_states = list()
        self._motor_states_lock = RLock()

    @property
    def motor_states(self):
        with self._motor_states_lock:
            return self._motor_states

    def connect(self):
        try:
            self._dxl_protocol = DynamixelProtocol(
                self._port_name, self._baud_rate)
            self._find_motors()
        except SerialOpenError as e:
            raise e

        self._running = True

    def disconnect(self):
        self._running = False

    def _fill_motor_parameters(self, motor_id, model_number):
        """
        Stores some extra information about each motor on the parameter server.
        Some of these paramters are used in joint controller implementation.
        """
        angles = self._dxl_protocol.get_angle_limits(motor_id)
        voltage = self._dxl_protocol.get_voltage(motor_id)
        voltages = self._dxl_protocol.get_voltage_limits(motor_id)

        self._motor_params[str(motor_id)] = dict()
        self._motor_params[str(motor_id)]["model_number"] = model_number
        self._motor_params[str(
            motor_id)]["model_name"] = DXL_MODEL_TO_PARAMS[model_number]["name"]
        self._motor_params[str(motor_id)]["min_angle"] = angles["min"]
        self._motor_params[str(motor_id)]["max_angle"] = angles["max"]

        torque_per_volt = DXL_MODEL_TO_PARAMS[model_number]['torque_per_volt']

        self._motor_params[str(motor_id)]["torque_per_volt"] = torque_per_volt
        self._motor_params[str(
            motor_id)]["max_torque"] = torque_per_volt * voltage

        velocity_per_volt = DXL_MODEL_TO_PARAMS[model_number]['velocity_per_volt']
        rpm_per_tick = DXL_MODEL_TO_PARAMS[model_number]['rpm_per_tick']

        self._motor_params[str(
            motor_id)]["velocity_per_volt"] = velocity_per_volt
        self._motor_params[str(
            motor_id)]["max_velocity"] = velocity_per_volt * voltage
        self._motor_params[str(
            motor_id)]["radians_second_per_encoder_tick"] = rpm_per_tick * RPM_TO_RADSEC

        encoder_resolution = DXL_MODEL_TO_PARAMS[model_number]['encoder_resolution']
        range_degrees = DXL_MODEL_TO_PARAMS[model_number]['range_degrees']
        range_radians = math.radians(range_degrees)

        self._motor_params[str(
            motor_id)]["encoder_resolution"] = encoder_resolution
        self._motor_params[str(motor_id)]["range_degrees"] = range_degrees
        self._motor_params[str(motor_id)]["range_radians"] = range_radians
        self._motor_params[str(
            motor_id)]["encoder_ticks_per_degree"] = encoder_resolution / range_degrees
        self._motor_params[str(
            motor_id)]["encoder_ticks_per_radian"] = encoder_resolution / range_radians
        self._motor_params[str(
            motor_id)]["degrees_per_encoder_tick"] = range_degrees / encoder_resolution
        self._motor_params[str(
            motor_id)]["radians_per_encoder_tick"] = range_radians / encoder_resolution

        # keep some parameters around for diagnostics
        self._motor_static_info[str(motor_id)] = {}
        self._motor_static_info[str(
            motor_id)]['model'] = DXL_MODEL_TO_PARAMS[model_number]['name']
        self._motor_static_info[str(
            motor_id)]['firmware'] = self._dxl_protocol.get_firmware_version(motor_id)
        self._motor_static_info[str(
            motor_id)]['delay'] = self._dxl_protocol.get_return_delay_time(motor_id)
        self._motor_static_info[str(motor_id)]['min_angle'] = angles['min']
        self._motor_static_info[str(motor_id)]['max_angle'] = angles['max']
        self._motor_static_info[str(motor_id)]['min_voltage'] = voltages['min']
        self._motor_static_info[str(motor_id)]['max_voltage'] = voltages['max']

    def _find_motors(self):
        self._motors = list()
        self._motor_static_info = dict()

        for motor_id in range(self._min_motor_id, self._max_motor_id + 1):
            for trial in range(self._num_ping_retries):
                try:
                    result = self._dxl_protocol.ping(motor_id)
                except Exception as ex:
                    continue

                if result:
                    self._motors.append(motor_id)
                    break

        if not self._motors:
            raise

        counts = defaultdict(int)

        to_delete_if_error = []
        for motor_id in self._motors:
            for trial in range(self._num_ping_retries):
                try:
                    model_number = self._dxl_protocol.get_model_number(
                        motor_id)
                    self. _fill_motor_parameters(motor_id, model_number)
                except Exception as ex:
                    if trial == self._num_ping_retries - 1:
                        to_delete_if_error.append(motor_id)
                    continue

                counts[model_number] += 1
                break

        for motor_id in to_delete_if_error:
            self._motors.remove(motor_id)

        status_str = '%s: Found %d motors - ' % (
            self._port_namespace, len(self._motors))
        for model_number, count in counts.items():
            if count:
                model_name = DXL_MODEL_TO_PARAMS[model_number]['name']
                status_str += '%d %s [' % (count, model_name)

                for motor_id in self._motors:
                    if self._motor_static_info[motor_id]['model'] == model_name:
                        status_str += '%d, ' % motor_id

                status_str = status_str[:-2] + '], '

    def _update_motor_states(self):
        if self._running:
            motor_states = list()

            for motor_id in self._motors:
                try:
                    state = self._dxl_protocol.get_feedback(motor_id)
                    if state:
                        motor_states.append(MotorState(**state))
                except FatalErrorCodeError as fece:
                    raise fece
                except NonfatalErrorCodeError as nfece:
                    raise nfece
                except ChecksumError as cse:
                    raise cse
                except DroppedPacketError as dpe:
                    raise dpe
        with self._motor_states_lock:
            self._motor_states = motor_states

        # Calculate actual update rate
        # Note: need to see if this affects the logic in general
        current_time = time.time()
