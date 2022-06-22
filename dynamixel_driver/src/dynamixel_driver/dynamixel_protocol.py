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

from dynamixel_const import *
from dynamixel_exception import *

from dynamixel_io import DynamixelIO


class DynamixelProtocol(DynamixelIO):
    def __init__(self, port: str, baudrate: int):
        DynamixelIO.__init__(port=port, baudrate=baudrate)

    ######################################################################
    # These function modify EEPROM data which persists after power cycle #
    ######################################################################

    def set_id(self, old_id, new_id):
        """
        Sets a new unique number to identify a motor. The range from 1 to 253
        (0xFD) can be used.
        """
        response = self.write(old_id, DXL_ID, [new_id])
        if response:
            self.exception_on_error(response[4], old_id, 'setting id to %d' % new_id)
        return response

    def set_baud_rate(self, servo_id, baud_rate):
        """
        Sets servo communication speed. The range from 0 to 254.
        """
        response = self.write(servo_id, DXL_BAUD_RATE, [baud_rate])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting baud rate to %d' % baud_rate)
        return response

    def set_return_delay_time(self, servo_id, delay):
        """
        Sets the delay time from the transmission of Instruction Packet until
        the return of Status Packet. 0 to 254 (0xFE) can be used, and the delay
        time per data value is 2 usec.
        """
        response = self.write(servo_id, DXL_RETURN_DELAY_TIME, [delay])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting return delay time to %d' % delay)
        return response

    def set_angle_limit_cw(self, servo_id, angle_cw):
        """
        Set the min (CW) angle of rotation limit.
        """
        loVal = int(angle_cw % 256)
        hiVal = int(angle_cw >> 8)

        response = self.write(servo_id, DXL_CW_ANGLE_LIMIT_L, (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting CW angle limits to %d' % angle_cw)
        return response

    def set_angle_limit_ccw(self, servo_id, angle_ccw):
        """
        Set the max (CCW) angle of rotation limit.
        """
        loVal = int(angle_ccw % 256)
        hiVal = int(angle_ccw >> 8)

        response = self.write(servo_id, DXL_CCW_ANGLE_LIMIT_L, (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting CCW angle limits to %d' % angle_ccw)
        return response

    def set_angle_limits(self, servo_id, min_angle, max_angle):
        """
        Set the min (CW) and max (CCW) angle of rotation limits.
        """
        loMinVal = int(min_angle % 256)
        hiMinVal = int(min_angle >> 8)
        loMaxVal = int(max_angle % 256)
        hiMaxVal = int(max_angle >> 8)

        # set 4 register values with low and high bytes for min and max angles
        response = self.write(servo_id, DXL_CW_ANGLE_LIMIT_L, (loMinVal, hiMinVal, loMaxVal, hiMaxVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting CW and CCW angle limits to %d and %d' %(min_angle, max_angle))
        return response

    def set_drive_mode(self, servo_id, is_slave=False, is_reverse=False):
        """
        Sets the drive mode for EX-106 motors
        """
        drive_mode = (is_slave << 1) + is_reverse

        response = self.write(servo_id, DXL_DRIVE_MODE, [drive_mode])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting drive mode to %d' % drive_mode)
        return response

    def set_voltage_limit_min(self, servo_id, min_voltage):
        """
        Set the minimum voltage limit.
        NOTE: the absolute min is 5v
        """

        if min_voltage < 5: min_voltage = 5
        minVal = int(min_voltage * 10)

        response = self.write(servo_id, DXL_DOWN_LIMIT_VOLTAGE, [minVal])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting minimum voltage level to %d' % min_voltage)
        return response

    def set_voltage_limit_max(self, servo_id, max_voltage):
        """
        Set the maximum voltage limit.
        NOTE: the absolute min is 25v
        """

        if max_voltage > 25: max_voltage = 25
        maxVal = int(max_voltage * 10)

        response = self.write(servo_id, DXL_UP_LIMIT_VOLTAGE, [maxVal])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting maximum voltage level to %d' % max_voltage)
        return response

    def set_voltage_limits(self, servo_id, min_voltage, max_voltage):
        """
        Set the min and max voltage limits.
        NOTE: the absolute min is 5v and the absolute max is 25v
        """

        if min_voltage < 5: min_voltage = 5
        if max_voltage > 25: max_voltage = 25

        minVal = int(min_voltage * 10)
        maxVal = int(max_voltage * 10)

        response = self.write(servo_id, DXL_DOWN_LIMIT_VOLTAGE, (minVal, maxVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting min and max voltage levels to %d and %d' %(min_voltage, max_voltage))
        return response


    ###############################################################
    # These functions can send a single command to a single servo #
    ###############################################################

    def set_torque_enabled(self, servo_id, enabled):
        """
        Sets the value of the torque enabled register to 1 or 0. When the
        torque is disabled the servo can be moved manually while the motor is
        still powered.
        """
        response = self.write(servo_id, DXL_TORQUE_ENABLE, [enabled])
        if response:
            self.exception_on_error(response[4], servo_id, '%sabling torque' % 'en' if enabled else 'dis')
        return response

    def set_compliance_margin_cw(self, servo_id, margin):
        """
        The error between goal position and present position in CW direction.
        The range of the value is 0 to 255, and the unit is the same as Goal Position.
        The greater the value, the more difference occurs.
        """
        response = self.write(servo_id, DXL_CW_COMPLIANCE_MARGIN, [margin])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting CW compliance margin to %d' % margin)
        return response

    def set_compliance_margin_ccw(self, servo_id, margin):
        """
        The error between goal position and present position in CCW direction.
        The range of the value is 0 to 255, and the unit is the same as Goal Position.
        The greater the value, the more difference occurs.
        """
        response = self.write(servo_id, DXL_CCW_COMPLIANCE_MARGIN, [margin])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting CCW compliance margin to %d' % margin)
        return response

    def set_compliance_margins(self, servo_id, margin_cw, margin_ccw):
        """
        The error between goal position and present position in CCW direction.
        The range of the value is 0 to 255, and the unit is the same as Goal Position.
        The greater the value, the more difference occurs.
        """
        response = self.write(servo_id, DXL_CW_COMPLIANCE_MARGIN, (margin_cw, margin_ccw))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting CW and CCW compliance margins to values %d and %d' %(margin_cw, margin_ccw))
        return response

    def set_compliance_slope_cw(self, servo_id, slope):
        """
        Sets the level of Torque near the goal position in CW direction.
        Compliance Slope is set in 7 steps, the higher the value, the more flexibility is obtained.
        """
        response = self.write(servo_id, DXL_CW_COMPLIANCE_SLOPE, [slope])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting CW compliance slope to %d' %  slope)
        return response

    def set_compliance_slope_ccw(self, servo_id, slope):
        """
        Sets the level of Torque near the goal position in CCW direction.
        Compliance Slope is set in 7 steps, the higher the value, the more flexibility is obtained.
        """
        response = self.write(servo_id, DXL_CCW_COMPLIANCE_SLOPE, [slope])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting CCW compliance slope to %d' % slope)
        return response

    def set_compliance_slopes(self, servo_id, slope_cw, slope_ccw):
        """
        Sets the level of Torque near the goal position in CW/CCW direction.
        Compliance Slope is set in 7 steps, the higher the value, the more flexibility is obtained.
        """
        response = self.write(servo_id, DXL_CW_COMPLIANCE_SLOPE, (slope_cw, slope_ccw))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting CW and CCW compliance slopes to %d and %d' %(slope_cw, slope_ccw))
        return response

    def set_d_gain(self, servo_id, d_gain):
        """
        Sets the value of derivative action of PID controller.
        Gain value is in range 0 to 254.
        """
        response = self.write(servo_id, DXL_D_GAIN, [d_gain])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting D gain value of PID controller to %d' % d_gain)
        return response

    def set_i_gain(self, servo_id, i_gain):
        """
        Sets the value of integral action of PID controller.
        Gain value is in range 0 to 254.
        """
        response = self.write(servo_id, DXL_I_GAIN, [i_gain])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting I gain value of PID controller to %d' % i_gain)
        return response

    def set_p_gain(self, servo_id, p_gain):
        """
        Sets the value of proportional action of PID controller.
        Gain value is in range 0 to 254.
        """
        response = self.write(servo_id, DXL_P_GAIN, [p_gain])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting P gain value of PID controller to %d' % p_gain)
        return response

    def set_punch(self, servo_id, punch):
        """
        Sets the limit value of torque being reduced when the output torque is
        decreased in the Compliance Slope area. In other words, it is the mimimum
        torque. The initial value is 32 (0x20) and can be extended up to 1023
        (0x3FF). (Refer to Compliance margin & Slope)
        """
        loVal = int(punch % 256)
        hiVal = int(punch >> 8)
        response = self.write(servo_id, DXL_PUNCH_L, (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting punch to %d' % punch)
        return response

    def set_acceleration(self, servo_id, acceleration):
        """
        Sets the acceleration. The unit is 8.583 Degree / sec^2.
        0 - acceleration control disabled, 1-254 - valid range for acceleration.
        """

        model = self.get_model_number(servo_id)
        if not model in DXL_MODEL_TO_PARAMS:
            raise UnsupportedFeatureError(model, DXL_GOAL_ACCELERATION)

        if DXL_GOAL_ACCELERATION in DXL_MODEL_TO_PARAMS[model]['features']:
            response = self.write(servo_id, DXL_GOAL_ACCELERATION, (acceleration, ))
            if response:
                self.exception_on_error(response[4], servo_id, 'setting acceleration to %d' % acceleration)
            return response
        else:
            raise UnsupportedFeatureError(model, DXL_GOAL_ACCELERATION)

    def set_position(self, servo_id, position):
        """
        Set the servo with servo_id to the specified goal position.
        Position value must be positive.
        """
        loVal = int(position % 256)
        hiVal = int(position >> 8)

        response = self.write(servo_id, DXL_GOAL_POSITION_L, (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting goal position to %d' % position)
        return response

    def set_speed(self, servo_id, speed):
        """
        Set the servo with servo_id to the specified goal speed.
        Speed can be negative only if the dynamixel is in "freespin" mode.
        """
        # split speed into 2 bytes
        if speed >= 0:
            loVal = int(speed % 256)
            hiVal = int(speed >> 8)
        else:
            loVal = int((1023 - speed) % 256)
            hiVal = int((1023 - speed) >> 8)

        # set two register values with low and high byte for the speed
        response = self.write(servo_id, DXL_GOAL_SPEED_L, (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting moving speed to %d' % speed)
        return response

    def set_torque_limit(self, servo_id, torque):
        """
        Sets the value of the maximum torque limit for servo with id servo_id.
        Valid values are 0 to 1023 (0x3FF), and the unit is about 0.1%.
        For example, if the value is 512 only 50% of the maximum torque will be used.
        If the power is turned on, the value of Max Torque (Address 14, 15) is used as the initial value.
        """
        loVal = int(torque % 256)
        hiVal = int(torque >> 8)

        response = self.write(servo_id, DXL_TORQUE_LIMIT_L, (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting torque limit to %d' % torque)
        return response

    def set_goal_torque(self, servo_id, torque):
        """
        Set the servo to torque control mode (similar to wheel mode, but controlling the torque)
        Valid values are from -1023 to 1023.
        Anything outside this range or 'None' disables torque control.
        """

        model = self.get_model_number(servo_id)
        if not model in DXL_MODEL_TO_PARAMS:
            raise UnsupportedFeatureError(model, DXL_TORQUE_CONTROL_MODE)

        valid_torque = torque is not None and torque >= -1023 and torque <= 1023
        if torque is not None and torque < 0:
            torque = 1024 - torque

        if DXL_TORQUE_CONTROL_MODE in DXL_MODEL_TO_PARAMS[model]['features']:
            if valid_torque:
                loVal = int(torque % 256); hiVal = int(torque >> 8);
                response = self.write(servo_id, DXL_GOAL_TORQUE_L, (loVal, hiVal))
                if response:
                    self.exception_on_error(response[4], servo_id, 'setting goal torque to %d' % torque)
            response = self.write(servo_id, DXL_TORQUE_CONTROL_MODE, (int(valid_torque), ))
            if response:
                self.exception_on_error(response[4], servo_id, 'enabling torque mode')
            return response
        else:
            raise UnsupportedFeatureError(model, DXL_TORQUE_CONTROL_MODE)

    def set_position_and_speed(self, servo_id, position, speed):
        """
        Set the servo with servo_id to specified position and speed.
        Speed can be negative only if the dynamixel is in "freespin" mode.
        """
        # split speed into 2 bytes
        if speed >= 0:
            loSpeedVal = int(speed % 256)
            hiSpeedVal = int(speed >> 8)
        else:
            loSpeedVal = int((1023 - speed) % 256)
            hiSpeedVal = int((1023 - speed) >> 8)

        # split position into 2 bytes
        loPositionVal = int(position % 256)
        hiPositionVal = int(position >> 8)

        response = self.write(servo_id, DXL_GOAL_POSITION_L, (loPositionVal, hiPositionVal, loSpeedVal, hiSpeedVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting goal position to %d and moving speed to %d' %(position, speed))
        return response

    def set_led(self, servo_id, led_state):
        """
        Turn the LED of servo motor on/off.
        Possible boolean state values:
            True - turn the LED on,
            False - turn the LED off.
        """
        response = self.write(servo_id, DXL_LED, [led_state])
        if response:
            self.exception_on_error(response[4], servo_id,
                    'setting a LED to %s' % led_state)
        return response


    #################################################################
    # These functions can send multiple commands to multiple servos #
    # These commands are used in ROS wrapper as they don't send a   #
    # response packet, ROS wrapper gets motor states at a set rate  #
    #################################################################

    def set_multi_torque_enabled(self, valueTuples):
        """
        Method to set multiple servos torque enabled.
        Should be called as such:
        set_multi_servos_to_torque_enabled( (id1, True), (id2, True), (id3, True) )
        """
        # use sync write to broadcast multi servo message
        self.sync_write(DXL_TORQUE_ENABLE, tuple(valueTuples))

    def set_multi_compliance_margin_cw(self, valueTuples):
        """
        Set different CW compliance margin for multiple servos.
        Should be called as such:
        set_multi_compliance_margin_cw( ( (id1, margin1), (id2, margin2), (id3, margin3) ) )
        """
        self.sync_write(DXL_CW_COMPLIANCE_MARGIN, tuple(valueTuples))

    def set_multi_compliance_margin_ccw(self, valueTuples):
        """
        Set different CCW compliance margin for multiple servos.
        Should be called as such:
        set_multi_compliance_margin_ccw( ( (id1, margin1), (id2, margin2), (id3, margin3) ) )
        """
        self.sync_write(DXL_CCW_COMPLIANCE_MARGIN, tuple(valueTuples))

    def set_multi_compliance_margins(self, valueTuples):
        """
        Set different CW and CCW compliance margins for multiple servos.
        Should be called as such:
        set_multi_compliance_margins( ( (id1, cw_margin1, ccw_margin1), (id2, cw_margin2, ccw_margin2) ) )
        """
        self.sync_write(DXL_CW_COMPLIANCE_MARGIN, tuple(valueTuples))

    def set_multi_compliance_slope_cw(self, valueTuples):
        """
        Set different CW compliance slope for multiple servos.
        Should be called as such:
        set_multi_compliance_slope_cw( ( (id1, slope1), (id2, slope2), (id3, slope3) ) )
        """
        self.sync_write(DXL_CW_COMPLIANCE_SLOPE, tuple(valueTuples))

    def set_multi_compliance_slope_ccw(self, valueTuples):
        """
        Set different CCW compliance slope for multiple servos.
        Should be called as such:
        set_multi_compliance_slope_ccw( ( (id1, slope1), (id2, slope2), (id3, slope3) ) )
        """
        self.sync_write(DXL_CCW_COMPLIANCE_SLOPE, tuple(valueTuples))

    def set_multi_compliance_slopes(self, valueTuples):
        """
        Set different CW and CCW compliance slopes for multiple servos.
        Should be called as such:
        set_multi_compliance_slopes( ( (id1, cw_slope1, ccw_slope1), (id2, cw_slope2, ccw_slope2) ) )
        """
        self.sync_write(DXL_CW_COMPLIANCE_SLOPE, tuple(valueTuples))

    def set_multi_punch(self, valueTuples):
        """
        Set different punch values for multiple servos.
        NOTE: according to documentation, currently this value is not being used.
        Should be called as such:
        set_multi_punch( ( (id1, punch1), (id2, punch2), (id3, punch3) ) )
        """
        # prepare value tuples for call to syncwrite
        writeableVals = []

        for sid,punch in valueTuples:
            # split punch into 2 bytes
            loVal = int(punch % 256)
            hiVal = int(punch >> 8)
            writeableVals.append( (sid, loVal, hiVal) )

        # use sync write to broadcast multi servo message
        self.sync_write(DXL_PUNCH_L, writeableVals)

    def set_multi_position(self, valueTuples):
        """
        Set different positions for multiple servos.
        Should be called as such:
        set_multi_position( ( (id1, position1), (id2, position2), (id3, position3) ) )
        """
        # prepare value tuples for call to syncwrite
        writeableVals = []

        for vals in valueTuples:
            sid = vals[0]
            position = vals[1]
            # split position into 2 bytes
            loVal = int(position % 256)
            hiVal = int(position >> 8)
            writeableVals.append( (sid, loVal, hiVal) )

        # use sync write to broadcast multi servo message
        self.sync_write(DXL_GOAL_POSITION_L, writeableVals)

    def set_multi_speed(self, valueTuples):
        """
        Set different speeds for multiple servos.
        Should be called as such:
        set_multi_speed( ( (id1, speed1), (id2, speed2), (id3, speed3) ) )
        """
        # prepare value tuples for call to syncwrite
        writeableVals = []

        for vals in valueTuples:
            sid = vals[0]
            speed = vals[1]

            # split speed into 2 bytes
            if speed >= 0:
                loVal = int(speed % 256)
                hiVal = int(speed >> 8)
            else:
                loVal = int((1023 - speed) % 256)
                hiVal = int((1023 - speed) >> 8)

            writeableVals.append( (sid, loVal, hiVal) )

        # use sync write to broadcast multi servo message
        self.sync_write(DXL_GOAL_SPEED_L, writeableVals)

    def set_multi_torque_limit(self, valueTuples):
        """
        Set different torque limits for multiple servos.
        Should be called as such:
        set_multi_torque_limit( ( (id1, torque1), (id2, torque2), (id3, torque3) ) )
        """
        # prepare value tuples for call to syncwrite
        writeableVals = []

        for sid,torque in valueTuples:
            # split torque into 2 bytes
            loVal = int(torque % 256)
            hiVal = int(torque >> 8)
            writeableVals.append( (sid, loVal, hiVal) )

        # use sync write to broadcast multi servo message
        self.sync_write(DXL_TORQUE_LIMIT_L, writeableVals)

    def set_multi_position_and_speed(self, valueTuples):
        """
        Set different positions and speeds for multiple servos.
        Should be called as such:
        set_multi_position_and_speed( ( (id1, position1, speed1), (id2, position2, speed2), (id3, position3, speed3) ) )
        """
        # prepare value tuples for call to syncwrite
        writeableVals = []

        for vals in valueTuples:
            sid = vals[0]
            position = vals[1]
            speed = vals[2]

            # split speed into 2 bytes
            if speed >= 0:
                loSpeedVal = int(speed % 256)
                hiSpeedVal = int(speed >> 8)
            else:
                loSpeedVal = int((1023 - speed) % 256)
                hiSpeedVal = int((1023 - speed) >> 8)

            # split position into 2 bytes
            loPositionVal = int(position % 256)
            hiPositionVal = int(position >> 8)
            writeableVals.append( (sid, loPositionVal, hiPositionVal, loSpeedVal, hiSpeedVal) )

        # use sync write to broadcast multi servo message
        self.sync_write(DXL_GOAL_POSITION_L, tuple(writeableVals))


    #################################
    # Servo status access functions #
    #################################

    def get_model_number(self, servo_id):
        """ Reads the servo's model number (e.g. 12 for AX-12+). """
        response = self.read(servo_id, DXL_MODEL_NUMBER_L, 2)
        if response:
            self.exception_on_error(response[4], servo_id, 'fetching model number')
        return response[5] + (response[6] << 8)

    def get_firmware_version(self, servo_id):
        """ Reads the servo's firmware version. """
        response = self.read(servo_id, DXL_VERSION, 1)
        if response:
            self.exception_on_error(response[4], servo_id, 'fetching firmware version')
        return response[5]

    def get_return_delay_time(self, servo_id):
        """ Reads the servo's return delay time. """
        response = self.read(servo_id, DXL_RETURN_DELAY_TIME, 1)
        if response:
            self.exception_on_error(response[4], servo_id, 'fetching return delay time')
        return response[5]

    def get_angle_limits(self, servo_id):
        """
        Returns the min and max angle limits from the specified servo.
        """
        # read in 4 consecutive bytes starting with low value of clockwise angle limit
        response = self.read(servo_id, DXL_CW_ANGLE_LIMIT_L, 4)
        if response:
            self.exception_on_error(response[4], servo_id, 'fetching CW/CCW angle limits')
        # extract data valus from the raw data
        cwLimit = response[5] + (response[6] << 8)
        ccwLimit = response[7] + (response[8] << 8)

        # return the data in a dictionary
        return {'min':cwLimit, 'max':ccwLimit}

    def get_drive_mode(self, servo_id):
        """ Reads the servo's drive mode. """
        response = self.read(servo_id, DXL_DRIVE_MODE, 1)
        if response:
            self.exception_on_error(response[4], servo_id, 'fetching drive mode')
        return response[5]

    def get_voltage_limits(self, servo_id):
        """
        Returns the min and max voltage limits from the specified servo.
        """
        response = self.read(servo_id, DXL_DOWN_LIMIT_VOLTAGE, 2)
        if response:
            self.exception_on_error(response[4], servo_id, 'fetching voltage limits')
        # extract data valus from the raw data
        min_voltage = response[5] / 10.0
        max_voltage = response[6] / 10.0

        # return the data in a dictionary
        return {'min':min_voltage, 'max':max_voltage}

    def get_position(self, servo_id):
        """ Reads the servo's position value from its registers. """
        response = self.read(servo_id, DXL_PRESENT_POSITION_L, 2)
        if response:
            self.exception_on_error(response[4], servo_id, 'fetching present position')
        position = response[5] + (response[6] << 8)
        return position

    def get_speed(self, servo_id):
        """ Reads the servo's speed value from its registers. """
        response = self.read(servo_id, DXL_PRESENT_SPEED_L, 2)
        if response:
            self.exception_on_error(response[4], servo_id, 'fetching present speed')
        speed = response[5] + (response[6] << 8)
        if speed > 1023:
            return 1023 - speed
        return speed

    def get_voltage(self, servo_id):
        """ Reads the servo's voltage. """
        response = self.read(servo_id, DXL_PRESENT_VOLTAGE, 1)
        if response:
            self.exception_on_error(response[4], servo_id, 'fetching supplied voltage')
        return response[5] / 10.0

    def get_current(self, servo_id):
        """ Reads the servo's current consumption (if supported by model) """
        model = self.get_model_number(servo_id)
        if not model in DXL_MODEL_TO_PARAMS:
            raise UnsupportedFeatureError(model, DXL_CURRENT_L)

        if DXL_CURRENT_L in DXL_MODEL_TO_PARAMS[model]['features']:
            response = self.read(servo_id, DXL_CURRENT_L, 2)
            if response:
                self.exception_on_error(response[4], servo_id, 'fetching sensed current')
            current = response[5] + (response[6] << 8)
            return 0.0045 * (current - 2048)

        if DXL_SENSED_CURRENT_L in DXL_MODEL_TO_PARAMS[model]['features']:
            response = self.read(servo_id, DXL_SENSED_CURRENT_L, 2)
            if response:
                self.exception_on_error(response[4], servo_id, 'fetching sensed current')
            current = response[5] + (response[6] << 8)
            return 0.01 * (current - 512)

        else:
            raise UnsupportedFeatureError(model, DXL_CURRENT_L)


    def get_feedback(self, servo_id):
        """
        Returns the id, goal, position, error, speed, load, voltage, temperature
        and moving values from the specified servo.
        """
        # read in 17 consecutive bytes starting with low value for goal position
        response = self.read(servo_id, DXL_GOAL_POSITION_L, 17)

        if response:
            self.exception_on_error(response[4], servo_id, 'fetching full servo status')
        if len(response) == 24:
            # extract data values from the raw data
            goal = response[5] + (response[6] << 8)
            position = response[11] + (response[12] << 8)
            error = position - goal
            speed = response[13] + ( response[14] << 8)
            if speed > 1023: speed = 1023 - speed
            load_raw = response[15] + (response[16] << 8)
            load_direction = 1 if self.test_bit(load_raw, 10) else 0
            load = (load_raw & int('1111111111', 2)) / 1024.0
            if load_direction == 1: load = -load
            voltage = response[17] / 10.0
            temperature = response[18]
            moving = response[21]
            timestamp = response[-1]

            # return the data in a dictionary
            return { 'timestamp': timestamp,
                     'id': servo_id,
                     'goal': goal,
                     'position': position,
                     'error': error,
                     'speed': speed,
                     'load': load,
                     'voltage': voltage,
                     'temperature': temperature,
                     'moving': bool(moving) }

    def get_led(self, servo_id):
        """
        Get status of the LED. Boolean return values:
            True - LED is on,
            False - LED is off.
        """
        response = self.read(servo_id, DXL_LED, 1)
        if response:
            self.exception_on_error(response[4], servo_id,
                'fetching LED status')

        return bool(response[5])