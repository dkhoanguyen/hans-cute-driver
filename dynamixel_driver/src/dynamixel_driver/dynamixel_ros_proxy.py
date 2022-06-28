#!/usr/bin/env python3

from dynamixel_msgs.msg import MotorStateList
from dynamixel_msgs.msg import MotorState
from diagnostic_msgs.msg import KeyValue
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import DiagnosticArray
import rospy
import math
import sys
import errno
from collections import deque
from threading import Thread, RLock
from collections import defaultdict

from . dynamixel_driver import DynamixelDriver

import roslib
roslib.load_manifest("dynamixel_driver")


class DynamixelRosProxy(object):
    def __init__(self,driver: DynamixelDriver):

        self._dxl_driver = driver

        self._driver_config = self._dxl_driver.driver_config

        self._port_namespace = self._driver_config["port_namespace"]
        self._update_rate = self._driver_config["update_rate"]
        self._actual_rate = self._driver_config["update_rate"]
        self._diagnostics_rate = self._driver_config["diagnostics_rate"]

        self._current_state = MotorStateList()
        self._current_state_lock = RLock()

        self._motor_states_pub = rospy.Publisher(
            f"motor_states/{self._port_namespace}", MotorStateList, queue_size=1)
        self._diagnostics_pub = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=1)

        self._update_motor_states_thread = Thread(
            target=self._update_motor_states)
        self._publish_diagnostic_information_thread = Thread(
            target=self._publish_diagnostic_information)

        self._motors = list()

    def start(self):
        # Raise error if driver is not started
        if not self._dxl_driver.is_running:
            raise

        self._motors = self._dxl_driver.connected_motors
        self._motor_params = self._dxl_driver.motor_params

        rospy.set_param(
            f"dynamixel/{self._port_namespace}/connected_ids", self._motors)

        for motor_id in self._motors:
            self._fill_motor_parameters(motor_id)

    def _fill_motor_parameters(self, motor_id):
        rospy.set_param(f"dynamixel/{self._port_namespace}/{motor_id}/model_number",
                        self._motor_params[str(motor_id)["model_number"]])
        rospy.set_param("dynamixel/{self._port_namespace}/{motor_id}/model_name",
                        self._motor_params[str(motor_id)["model_name"]])
        rospy.set_param(f"dynamixel/{self._port_namespace}/{motor_id}/min_angle",
                        self._motor_params[str(motor_id)["min_angle"]])
        rospy.set_param(f"dynamixel/{self._port_namespace}/{motor_id}/max_angle",
                        self._motor_params[str(motor_id)["max_angle"]])

        rospy.set_param(f"dynamixel/{self._port_namespace}/{motor_id}/torque_per_volt",
                        self._motor_params[str(motor_id)["torque_per_volt"]])
        rospy.set_param(f"dynamixel/{self._port_namespace}/{motor_id}/max_torque",
                        self._motor_params[str(motor_id)["max_torque"]])

        rospy.set_param(f"dynamixel/{self._port_namespace}/{motor_id}/velocity_per_volt",
                        self._motor_params[str(motor_id)["velocity_per_volt"]])
        rospy.set_param(f"dynamixel/{self._port_namespace}/{motor_id}/max_velocity",
                        self._motor_params[str(motor_id)["max_velocity"]])
        rospy.set_param(f"dynamixel/{self._port_namespace}/{motor_id}/radians_second_per_encoder_tick",
                        self._motor_params[str(motor_id)["radians_second_per_encoder_tick"]])

        rospy.set_param(f"dynamixel/{self._port_namespace}/{motor_id}/encoder_resolution",
                        self._motor_params[str(motor_id)["encoder_resolution"]])
        rospy.set_param(f"dynamixel/{self._port_namespace}/{motor_id}/range_degrees",
                        self._motor_params[str(motor_id)["range_degrees"]])
        rospy.set_param(f"dynamixel/{self._port_namespace}/{motor_id}/range_radians",
                        self._motor_params[str(motor_id)["range_radians"]])
        rospy.set_param(f"dynamixel/{self._port_namespace}/{motor_id}/encoder_ticks_per_degree",
                        self._motor_params[str(motor_id)["encoder_ticks_per_degree"]])
        rospy.set_param(f"dynamixel/{self._port_namespace}/{motor_id}/encoder_ticks_per_radian",
                        self._motor_params[str(motor_id)["encoder_ticks_per_radian"]])
        rospy.set_param(f"dynamixel/{self._port_namespace}/{motor_id}/degrees_per_encoder_tick",
                        self._motor_params[str(motor_id)["degrees_per_encoder_tick"]])
        rospy.set_param(f"dynamixel/{self._port_namespace}/{motor_id}/radians_per_encoder_tick",
                        self._motor_params[str(motor_id)["radians_per_encoder_tick"]])

    def _update_motor_states(self):
        num_events = 50
        rates = deque([float(self._update_rate)]*num_events, maxlen=num_events)
        last_time = rospy.Time.now()

        rate = rospy.Rate(self._update_rate)
        while not rospy.is_shutdown() and self._dxl_driver.is_running:
            motor_states = []
            self._dxl_driver.update_motor_states()
            motor_states = self._dxl_driver.motor_states

            if motor_states:
                msl = MotorStateList()
                msl.motor_states = motor_states

                self._motor_states_pub.publish(msl)
                with self._current_state_lock:
                    self._current_state = msl

                # calculate actual update rate
                current_time = rospy.Time.now()
                rates.append(1.0 / (current_time - last_time).to_sec())
                self._actual_rate = round(sum(rates)/num_events, 2)
                last_time = current_time

            rate.sleep()

    def _publish_diagnostic_information(self):
        diag_msg = DiagnosticArray()
        motor_static_info =

        rate = rospy.Rate(self._diagnostics_rate)
        while not rospy.is_shutdown() and self._dxl_driver.is_running:
            status = DiagnosticStatus()

            error_counts = self._dxl_driver.error_counts

            status.name = f"Dynamixel Serial Bus ({self._port_namespace})"
            port_name = self._driver_config["port_name"]
            status.hardware_id = f"Dynamixel Serial Bus on port {port_name}"
            status.values.append(KeyValue("Baud Rate", str(self._driver_config["baud_rate"])))
            status.values.append(
                KeyValue("Min Motor ID", str(self._driver_config["min_motor_id"])))
            status.values.append(
                KeyValue("Max Motor ID", str(self._driver_config["max_motor_id"])))
            status.values.append(
                KeyValue("Desired Update Rate", str(self._update_rate)))
            status.values.append(
                KeyValue("Actual Update Rate", str(self._actual_rate)))
            status.values.append(
                KeyValue("# Non Fatal Errors", str(error_counts["non_fatal"])))
            status.values.append(
                KeyValue("# Checksum Errors", str(error_counts["checksum"])))
            status.values.append(
                KeyValue("# Dropped Packet Errors", str(error_counts["dropped"])))
            status.level = DiagnosticStatus.OK
            status.message = "OK"

            if self._actual_rate - self._update_rate < -5:
                status.level = DiagnosticStatus.WARN
                status.message = "Actual update rate is lower than desired"

            diag_msg.status.append(status)

            for motor_state in self._current_state.motor_states:
                mid = motor_state.id

                status = DiagnosticStatus()

                status.name = f"Robotis Dynamixel Motor {mid} on port {self._port_namespace}"
                status.hardware_id = f"DXL-{motor_state.id}@{self._port_namespace}"
                
                status.values.append(KeyValue("Model Name", str(
                    self.motor_static_info[mid]["model"])))
                status.values.append(KeyValue("Firmware Version", str(
                    self.motor_static_info[mid]["firmware"])))
                status.values.append(KeyValue("Return Delay Time", str(
                    self.motor_static_info[mid]["delay"])))
                status.values.append(KeyValue("Minimum Voltage", str(
                    self.motor_static_info[mid]["min_voltage"])))
                status.values.append(KeyValue("Maximum Voltage", str(
                    self.motor_static_info[mid]["max_voltage"])))
                status.values.append(KeyValue("Minimum Position (CW)", str(
                    self.motor_static_info[mid]["min_angle"])))
                status.values.append(KeyValue("Maximum Position (CCW)", str(
                    self.motor_static_info[mid]["max_angle"])))

                status.values.append(KeyValue("Goal", str(motor_state.goal)))
                status.values.append(
                    KeyValue("Position", str(motor_state.position)))
                status.values.append(KeyValue("Error", str(motor_state.error)))
                status.values.append(
                    KeyValue("Velocity", str(motor_state.speed)))
                status.values.append(KeyValue("Load", str(motor_state.load)))
                status.values.append(
                    KeyValue("Voltage", str(motor_state.voltage)))
                status.values.append(
                    KeyValue("Temperature", str(motor_state.temperature)))
                status.values.append(
                    KeyValue("Moving", str(motor_state.moving)))

                if motor_state.temperature >= self.error_level_temp:
                    status.level = DiagnosticStatus.ERROR
                    status.message = "OVERHEATING"
                elif motor_state.temperature >= self.warn_level_temp:
                    status.level = DiagnosticStatus.WARN
                    status.message = "VERY HOT"
                else:
                    status.level = DiagnosticStatus.OK
                    status.message = "OK"

                diag_msg.status.append(status)

            self.diagnostics_pub.publish(diag_msg)
            rate.sleep()
