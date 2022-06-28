#!/usr/bin/env python3

# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010-2011, Antons Rebguns.
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


__author__ = 'Antons Rebguns'
__copyright__ = 'Copyright (c) 2010-2011 Antons Rebguns'

__license__ = 'BSD'
__maintainer__ = 'Antons Rebguns'
__email__ = 'anton@email.arizona.edu'

import math
import rospy

from abc import ABC, abstractmethod

from dynamixel_driver.dynamixel_const import *

from dynamixel_controllers.srv import SetSpeed
from dynamixel_controllers.srv import TorqueEnable
from dynamixel_controllers.srv import SetComplianceSlope
from dynamixel_controllers.srv import SetComplianceMargin
from dynamixel_controllers.srv import SetCompliancePunch
from dynamixel_controllers.srv import SetTorqueLimit

from std_msgs.msg import Float64
from dynamixel_msgs.msg import MotorStateList
from dynamixel_msgs.msg import JointState

# TODO: Decouple all controllers from ROS


class JointController(ABC):
    def __init__(self, dxl_protocol, controller_namespace, port_namespace):
        self._running = False
        self._dxl_protocol = dxl_protocol
        self._controller_namespace = controller_namespace
        self._port_namespace = port_namespace

        self._joint_name = rospy.get_param(
            self._controller_namespace + '/joint_name')
        self._joint_speed = rospy.get_param(
            self._controller_namespace + '/joint_speed', 1.0)
        self._compliance_slope = rospy.get_param(
            self._controller_namespace + '/joint_compliance_slope', None)
        self._compliance_margin = rospy.get_param(
            self._controller_namespace + '/joint_compliance_margin', None)
        self._compliance_punch = rospy.get_param(
            self._controller_namespace + '/joint_compliance_punch', None)
        self._torque_limit = rospy.get_param(
            self._controller_namespace + '/joint_torque_limit', None)

        self._ensure_limits()

        self._speed_service = rospy.Service(
            self._controller_namespace + '/set_speed', SetSpeed, self._process_set_speed)
        self._torque_service = rospy.Service(
            self._controller_namespace + '/torque_enable', TorqueEnable, self._process_torque_enable)
        self._compliance_slope_service = rospy.Service(
            self._controller_namespace + '/set_compliance_slope', SetComplianceSlope, self._process_set_compliance_slope)
        self._compliance_marigin_service = rospy.Service(
            self._controller_namespace + '/set_compliance_margin', SetComplianceMargin, self._process_set_compliance_margin)
        self._compliance_punch_service = rospy.Service(
            self._controller_namespace + '/set_compliance_punch', SetCompliancePunch, self._process_set_compliance_punch)
        self._torque_limit_service = rospy.Service(
            self._controller_namespace + '/set_torque_limit', SetTorqueLimit, self._process_set_torque_limit)

    def _ensure_limits(self):
        if self._compliance_slope is not None:
            if self._compliance_slope < DXL_MIN_COMPLIANCE_SLOPE:
                self._compliance_slope = DXL_MIN_COMPLIANCE_SLOPE
            elif self._compliance_slope > DXL_MAX_COMPLIANCE_SLOPE:
                self._compliance_slope = DXL_MAX_COMPLIANCE_SLOPE
            else:
                self._compliance_slope = int(self._compliance_slope)

        if self._compliance_margin is not None:
            if self._compliance_margin < DXL_MIN_COMPLIANCE_MARGIN:
                self._compliance_margin = DXL_MIN_COMPLIANCE_MARGIN
            elif self._compliance_margin > DXL_MAX_COMPLIANCE_MARGIN:
                self._compliance_margin = DXL_MAX_COMPLIANCE_MARGIN
            else:
                self._compliance_margin = int(self._compliance_margin)

        if self._compliance_punch is not None:
            if self._compliance_punch < DXL_MIN_PUNCH:
                self._compliance_punch = DXL_MIN_PUNCH
            elif self._compliance_punch > DXL_MAX_PUNCH:
                self._compliance_punch = DXL_MAX_PUNCH
            else:
                self._compliance_punch = int(self._compliance_punch)

        if self._torque_limit is not None:
            if self._torque_limit < 0:
                self._torque_limit = 0.0
            elif self._torque_limit > 1:
                self._torque_limit = 1.0

    @abstractmethod
    def initialize(self):
        raise NotImplementedError

    def start(self):
        self._running = True
        self._joint_state_pub = rospy.Publisher(
            self._controller_namespace + '/state', JointState, queue_size=1)
        self._command_sub = rospy.Subscriber(
            self._controller_namespace + '/command', Float64, self._process_command)
        self._motor_states_sub = rospy.Subscriber(
            'motor_states/%s' % self._port_namespace, MotorStateList, self._process_motor_states)

    def stop(self):
        self._running = False
        self._joint_state_pub.unregister()
        self._motor_states_sub.unregister()
        self._command_sub.unregister()
        self._speed_service.shutdown('normal shutdown')
        self._torque_service.shutdown('normal shutdown')
        self._compliance_slope_service.shutdown('normal shutdown')

    @abstractmethod
    def _set_torque_enable(self, torque_enable):
        raise NotImplementedError

    @abstractmethod
    def _set_speed(self, speed):
        raise NotImplementedError

    @abstractmethod
    def _set_compliance_slope(self, slope):
        raise NotImplementedError

    @abstractmethod
    def _set_compliance_margin(self, margin):
        raise NotImplementedError

    @abstractmethod
    def _set_compliance_punch(self, punch):
        raise NotImplementedError

    @abstractmethod
    def _set_torque_limit(self, max_torque):
        raise NotImplementedError

    def _process_set_speed(self, req):
        self.set_speed(req.speed)
        return []  # success

    def _process_torque_enable(self, req):
        self._set_torque_enable(req.torque_enable)
        return []

    def _process_set_compliance_slope(self, req):
        self._set_compliance_slope(req.slope)
        return []

    def _process_set_compliance_margin(self, req):
        self._set_compliance_margin(req.margin)
        return []

    def _process_set_compliance_punch(self, req):
        self._set_compliance_punch(req.punch)
        return []

    def _process_set_torque_limit(self, req):
        self._set_torque_limit(req.torque_limit)
        return []

    @abstractmethod
    def _process_motor_states(self, state_list):
        raise NotImplementedError

    @abstractmethod
    def _process_command(self, msg):
        raise NotImplementedError

    def rad_to_raw(self, angle, initial_position_raw, flipped, encoder_ticks_per_radian):
        """ angle is in radians """
        # print 'flipped = %s, angle_in = %f, init_raw = %d' % (str(flipped), angle, initial_position_raw)
        angle_raw = angle * encoder_ticks_per_radian
        # print 'angle = %f, val = %d' % (math.degrees(angle), int(round(initial_position_raw - angle_raw if flipped else initial_position_raw + angle_raw)))
        return int(round(initial_position_raw - angle_raw if flipped else initial_position_raw + angle_raw))

    def raw_to_rad(self, raw, initial_position_raw, flipped, radians_per_encoder_tick):
        return (initial_position_raw - raw if flipped else raw - initial_position_raw) * radians_per_encoder_tick
