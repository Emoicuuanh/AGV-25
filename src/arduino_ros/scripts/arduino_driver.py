#!/usr/bin/env python
# -*- coding: utf-8 -*-

from logging import shutdown

from numpy import True_
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import os, time
from rospy.core import loginfo
import sys

python3 = True if sys.hexversion > 0x03000000 else False

if python3:
    import _thread as thread
    import numpy as np
else:
    import thread
import threading
import math
import json
from tf.transformations import euler_from_quaternion

from math import pi as PI, degrees, radians, sin, cos
import os
import time
import sys, traceback
from serial.serialutil import SerialException
from serial import Serial

import roslib
import rospkg

from geometry_msgs.msg import Quaternion, Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Empty, Int8, Int32, String
from tf.broadcaster import TransformBroadcaster

# from dashgo_driver.srv import *
# from SrvInt32.srv import *
from std_srvs.srv import Trigger, TriggerResponse

from sensor_msgs.msg import Range
from std_msgs.msg import Int16, Bool
from ros_arduino_msgs.msg import *
from agv_msgs.msg import EncoderDifferential, DiffDriverMotorSpeed, LedControl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_stamped_msgs.msg import (
    StringStamped,
    Int8Stamped,
    Float32Stamped,
    Int32Stamped,
    EmptyStamped,
    Int16MultiArrayStamped,
)
from std_stamped_msgs.srv import StringService, StringServiceResponse

common_folder = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_folder):
    common_folder = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_folder)
from common_function import (
    find_soft_port,
    find_soft_port_from_pid,
    get_yaw,
    print_info,
    print_debug,
    print_warn,
)

# refer MIR robot
# fmt: off
ODOM_POSE_COVARIANCE = [1e-4, 0, 0, 0, 0, 0,
                        0, 1e-4, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 0.01]
ODOM_POSE_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0,
                        0, 1e-3, 1e-9, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e-9]

ODOM_TWIST_COVARIANCE = [1e-6, 0, 0, 0, 0, 0,
                        0, 1e-6, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 0.01]
ODOM_TWIST_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0,
                        0, 1e-3, 1e-9, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e-9]
# fmt: on

# fmt: off
ANALOG_READ      = 'a'
GET_BAUDRATE     = 'b'
PIN_MODE         = 'c'
DIGITAL_READ     = 'd'
READ_ENCODERS    = 'e'
MOTOR_SPEEDS     = 'm'
CMD_VEL          = 'v'
PING_TEST        = 'p'
RESET_ENCODERS   = 'r'
MODULE_SETUP     = 's'
TOWER_LAMP       = 't'
READ_ULTRASONIC  = 'u'
DIGITAL_WRITE    = 'w'
ANALOG_WRITE     = 'x'
LOG_CONFIRM      = 'f'
INT_16_WRITE     = 'i'
LED_WRITE        = 'l'
LED_RGB          = 'n'
CARD_ID          = 'k'
LIFT             = 'g'
READ_PARAMETER   = 'z'
TURN_SIGNAL      = 'y'
CONVEYOR         = 'o'
STOPPER          = 'h'
VEL_CVR_VOL      = 'q' # Không biết tại sao lỗi, gửi xuống k phản hồi
# fmt: on

SERVO_MAX = 180
SERVO_MIN = 0

LOW = 0
HIGH = 1

INPUT = 0
OUTPUT = 1

LEFT = 0
RIGHT = 1

SENSOR_DEACTIVATE = 0
SENSOR_ACTIVATE = 1


class MessageType:
    ANALOG = 0
    DIGITAL = 1
    RANGE = 2
    FLOAT = 3
    INT = 4
    BOOL = 5


# Read parameter
class FloatParameter:
    def __init__(self, controller, name, rate, code):
        self.name = name
        self.controller = controller
        self.param_code = code
        self.param_rate = rate
        self.param_msg = Float32Stamped()
        self.param_pub = rospy.Publisher(
            "~float_param/" + self.name, Float32Stamped, queue_size=10
        )
        self.value = None
        # time rate accrding to rate of rospy
        self.t_delta = rospy.Duration(1.0 / self.param_rate)
        self.t_next = rospy.Time.now() + self.t_delta

    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
            try:
                # print_info('float parameter')
                value = self.controller.float_param_read(self.param_code)
                self.value = float(value)
                if self.value != None:
                    self.param_msg.stamp = rospy.Time.now()
                    self.param_msg.data = self.value
                    self.param_pub.publish(self.param_msg)
            except Exception as e:
                pass
                # rospy.logerr(
                #     "Polling float param value = {}. Error: {}".format(value, e)
                # )
            # time process for next publish
            self.t_next = now + self.t_delta


"""


   ####  ###### #    #  ####   ####  #####
  #      #      ##   # #      #    # #    #
   ####  #####  # #  #  ####  #    # #    #
       # #      #  # #      # #    # #####
  #    # #      #   ## #    # #    # #   #
   ####  ###### #    #  ####   ####  #    #


"""


class Sensor(object):
    def __init__(self, controller, name, frame_id, **kwargs):
        self.controller = controller
        self.name = name
        self.frame_id = frame_id

        self.pin = kwargs["pin"]
        self.rate = kwargs["rate"]
        self.direction = kwargs["direction"]
        self.logic_type = kwargs["logic_type"]
        if "value_to_vol_factor" in kwargs:
            self.value_to_vol_factor = kwargs["value_to_vol_factor"]

        self.blink_rate = 0
        if "blink_rate" in kwargs:
            self.blink_rate = kwargs["blink_rate"]
            if self.direction != "input":
                rospy.loginfo(
                    'Pin "{}" blink with rate = {}'.format(
                        self.name, self.blink_rate
                    )
                )
            if self.blink_rate > 0:
                self.rate = self.blink_rate

        if self.rate > 0:
            self.t_delta = rospy.Duration(1.0 / self.rate)
            self.t_next = rospy.Time.now() + self.t_delta
        self.value = None
        self.last_value = None

    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
            if self.direction == "input":
                try:
                    self.value = self.read_value()
                except:
                    return None
            else:
                try:
                    if self.blink_rate > 0:
                        self.ack = self.write_value()
                    else:
                        self.value = self.read_value()
                except:
                    return None

            # For range sensors, assign the value to the range message field
            if self.message_type == MessageType.RANGE:
                self.msg.range = self.value
            else:
                self.msg.value = self.value

            # Add a timestamp and publish the message
            self.msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.msg)

            self.t_next = now + self.t_delta
            self.last_value = self.value
            return self.value
        return self.last_value


class AnalogSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(AnalogSensor, self).__init__(*args, **kwargs)

        self.message_type = MessageType.ANALOG

        self.msg = Analog()
        self.msg.header.frame_id = self.frame_id

        self.pub = rospy.Publisher(
            "~sensor/" + self.name, Analog, queue_size=10
        )

        if self.direction == "output":
            self.controller.pin_mode(self.pin, OUTPUT)
        else:
            self.controller.pin_mode(self.pin, INPUT)

        self.value = LOW

    def read_value(self):
        return self.controller.analog_read(self.pin)

    def write_value(self, value):
        return self.controller.analog_write(self.pin, value)


class AnalogFloatSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(AnalogFloatSensor, self).__init__(*args, **kwargs)

        self.message_type = MessageType.ANALOG

        self.msg = AnalogFloat()
        self.msg.header.frame_id = self.frame_id

        self.pub = rospy.Publisher(
            "~sensor/" + self.name, AnalogFloat, queue_size=10
        )

        if self.direction == "output":
            self.controller.pin_mode(self.pin, OUTPUT)
        else:
            self.controller.pin_mode(self.pin, INPUT)

        self.value = LOW

    def read_value(self):
        return self.controller.analog_read(self.pin) * self.value_to_vol_factor

    def write_value(self, value):
        return self.controller.analog_write(self.pin, value)


class DigitalSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(DigitalSensor, self).__init__(*args, **kwargs)

        self.message_type = MessageType.BOOL

        self.msg = Digital()
        self.msg.header.frame_id = self.frame_id

        self.pub = rospy.Publisher(
            "~sensor/" + self.name, Digital, queue_size=10
        )

        if self.direction == "output":
            self.controller.pin_mode(self.pin, OUTPUT)
        else:
            self.controller.pin_mode(self.pin, INPUT)

        self.value = LOW

    def read_value(self):
        if self.logic_type == "high":
            return self.controller.digital_read(self.pin)
        elif self.logic_type == "low":
            if self.controller.digital_read(self.pin) == 0:
                return 1
            else:
                return 0

    def write_value(self):
        # Alternate HIGH/LOW when writing at a fixed rate
        if self.blink_rate > 0:
            self.value = not self.value
        return self.controller.digital_write(self.pin, self.value)


"""


    ##   #####  #####  #    # # #    #  ####
   #  #  #    # #    # #    # # ##   # #    #
  #    # #    # #    # #    # # # #  # #    #
  ###### #####  #    # #    # # #  # # #    #
  #    # #   #  #    # #    # # #   ## #    #
  #    # #    # #####   ####  # #    #  ####


"""


class Arduino:
    """Configuration Parameters"""

    N_ANALOG_PORTS = 6
    N_DIGITAL_PORTS = 12

    def __init__(self, port="/dev/ttyUSB0", baudrate=57600, timeout=0.5):
        self.PID_RATE = 30  # Do not change this!  It is a fixed property of the Arduino PID controller.
        self.PID_INTERVAL = 1000 / 30

        self.port_name = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.encoder_count = 0
        self.writeTimeout = timeout
        self.interCharTimeout = timeout / 30.0

        self.execute_success_time = 0
        self.execute_time = 0
        self.is_connect = False

        # Keep things thread safe
        self.mutex = thread.allocate_lock()

        # An array to cache analog sensor readings
        self.analog_sensor_cache = [None] * self.N_ANALOG_PORTS

        # An array to cache digital sensor readings
        self.digital_sensor_cache = [None] * self.N_DIGITAL_PORTS

        # ROS
        self.exe_ack_error_pub = rospy.Publisher(
            "~exe_ack_error", Int32Stamped, queue_size=10
        )
        self.exe_ack_err = Int32Stamped()
        self.arduino_error_pub = rospy.Publisher(
            "arduino_error", StringStamped, queue_size=10
        )

        # Loop: must be run with thread
        loopThread = threading.Thread(target=self.loop, args=())
        # loopThread.start()

    def loop(self):
        while not rospy.is_shutdown():
            if not type(self.port) is Serial:
                continue
            data_left = self.port.inWaiting()
            if data_left > 1:
                print_info(data_left)
                self.mutex.acquire()
                value = self.recv(self.timeout)
                if value != None:
                    rospy.loginfo("Log value: {}".format(value[1:]))
                self.mutex.release()

            time.sleep(0.1)

    def connect(self):
        # DEBUG:
        # if True:
        try:
            print_info(
                "Connecting to Arduino on port: {}...".format(self.port_name)
            )
            if self.port_name == None:
                print_info("Port is unavailable!")
                self.is_connect = False
                time.sleep(1)
            else:
                self.port = Serial(
                    port=self.port_name,
                    baudrate=self.baudrate,
                    timeout=self.timeout,
                    writeTimeout=self.writeTimeout,
                )
                # The next line is necessary to give the firmware time to wake up.
                time.sleep(1)
                test = self.get_baud()
                if test != self.baudrate:
                    time.sleep(1)
                    test = self.get_baud()
                    if test != self.baudrate:
                        raise SerialException
                print_info("Connected at: {}".format(self.baudrate))
                print_info("Arduino is ready.")
                self.is_connect = True

        except SerialException:
            print_info("Serial Exception:")
            print_info(sys.exc_info())
            print_info("Traceback follows:")
            traceback.print_exc(file=sys.stdout)
            print_info("Cannot connect to Arduino!")
            self.is_connect = False
            # os._exit(1)

    def open(self):
        """Open the serial port."""
        self.port.open()

    def close(self):
        """Close the serial port."""
        self.port.close()
        self.is_connect = False

    def send(self, cmd):
        """This command should not be used on its own: it is called by the execute commands
        below in a thread safe manner.
        """
        self.port.write(cmd + "\r")

    def recv(self, timeout=0.5):
        timeout = min(timeout, self.timeout)
        """ This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters from the Arduino
        """
        c = ""
        value = ""
        if python3:
            value = "".encode()
        attempts = 0
        while c != b"\r":
            # DEBUG:
            # if True:
            try:
                c = self.port.read(1)
                value += c
                attempts += 1
                if attempts * self.interCharTimeout > timeout:
                    return None
            except:
                rospy.logerr("Error when recv")
                return None
        if python3:
            value = value.decode().strip("\r")
        else:
            value = value.strip("\r")

        return value

    def recv_ack(self):
        """This command should not be used on its own: it is called by the execute commands
        below in a thread safe manner.
        """
        ack = self.recv(self.timeout)
        return ack == "OK"

    def recv_int(self):
        """This command should not be used on its own: it is called by the execute commands
        below in a thread safe manner.
        """
        value = self.recv(self.timeout)
        try:
            return int(value)
        except:
            return None

    def recv_array(self):
        """This command should not be used on its own: it is called by the execute commands
        below in a thread safe manner.
        """
        if python3:
            values = np.array(
                self.recv(self.timeout * self.N_ANALOG_PORTS).split()
            )
            # cv_value = np.array(values)
            list_value = []
            for i in values:
                list_value.append(int(i))
            # print_warn(list_value)
            return list_value
        else:
            try:
                values = self.recv(self.timeout * self.N_ANALOG_PORTS).split()
                return map(int, values)
            except:
                return []

    """


    ###### #    # ######  ####  #    # ##### ######
    #       #  #  #      #    # #    #   #   #
    #####    ##   #####  #      #    #   #   #####
    #        ##   #      #      #    #   #   #
    #       #  #  #      #    # #    #   #   #
    ###### #    # ######  ####   ####    #   ######


    """

    def make_command(self, str_cmd):
        """Generate command with sum check"""
        cmd_list = str_cmd.split("#")
        cmd_calr_sum = list(str_cmd.replace("#", ""))

        if len(cmd_calr_sum) == 0:
            return False

        sum_calr = 0
        for i in cmd_calr_sum:
            sum_calr += ord(i)
        sum_calr &= int("0x00FF", 16)

        cmd_list.insert(1, str(sum_calr))
        if python3:
            return "#".join(cmd_list).encode() + b"\r"
        else:
            return "#".join(cmd_list) + "\r"

    def execute(self, cmd, ret_string=False):
        """Thread safe execution of "cmd" on the Arduino returning a single integer value."""
        # Static variable define
        try:
            self.execute.__func__.last_print_time = (
                self.execute.__func__.last_print_time
            )
        except AttributeError:
            self.execute.__func__.last_print_time = rospy.get_time()
            self.execute.__func__.last_log_key = 0
            self.execute.__func__.last_log_time = rospy.get_time()

        self.mutex.acquire()
        # Reset input buffer
        try:
            self.port.flushInput()
        except:
            pass

        ntries = 1
        attempts = 0
        execute_error = False

        cmd = self.make_command(cmd)
        # print_debug(cmd)
        # DEBUG:
        # if True:
        try:
            # print_info('excute___*******************')
            # print_info(self.port)
            self.port.write(cmd)
            # print_info('write cmd ok____________________')
            value = self.recv(self.timeout)
            self.execute_time += 1
            # print_info(
            #     "excute complete ############################ {}".format(value)
            # )
            # if value == 'SUM_ERR':
            #     rospy.logwarn('SUM_ERR: {}'.format(cmd))
            execute_error = (
                value == ""
                or value == "Invalid Command"
                or value == None
                or value == "SUM_ERR"
            )
            if not execute_error and "$" in value:
                value_split = value.split("$")
                current_key = int(value_split[0])
                # if current_key - self.execute.__func__.last_log_key > 14000:
                #     rospy.logerr('miss arduino log')
                # if rospy.get_time() - self.execute.__func__.last_log_time > 14.0:
                #     rospy.logerr('miss ros log')
                rospy.logdebug(
                    value.replace("$", " ")
                    + " Diff Arduino: "
                    + str(
                        (current_key - self.execute.__func__.last_log_key)
                        / 1000.0
                    )
                    + " Diff ROS: "
                    + str(
                        rospy.get_time() - self.execute.__func__.last_log_time
                    )
                )
                self.execute.__func__.last_log_time = rospy.get_time()
                self.execute.__func__.last_log_key = current_key
                # TODO: Respone log value to Arduino

            while attempts < ntries and execute_error:
                # DEBUG:
                # if True:
                try:
                    # self.execute_time += 1
                    self.port.flushInput()
                    self.port.write(cmd)
                    value = self.recv(self.timeout)
                except:
                    print_info(
                        'Exception executing command "execute": {}'.format(cmd)
                    )
                attempts += 1
                # rospy.logwarn('Retry: {}, times: {}'.format(cmd, attempts))
        except:
            self.mutex.release()
            print_info('Exception executing command "execute": {}'.format(cmd))
            value = None
            self.close()
            print("close Arduino port")

        if not execute_error:
            self.execute_success_time += 1

        # if rospy.get_time() - self.execute.__func__.last_print_time > 5:
        #     rospy.logdebug(
        #         "execute success rate: {}%".format(
        #             (self.execute_success_time * 100) / self.execute_time
        #         )
        #     )
        #     self.execute.__func__.last_print_time = rospy.get_time()

        self.mutex.release()
        # print_info("excute return {}" .format(value))
        if not ret_string and value != None and value != "":
            return int(value)
        else:
            return value

    def execute_array(self, cmd):
        """Thread safe execution of "cmd" on the Arduino returning an array."""
        self.mutex.acquire()

        try:
            self.port.flushInput()
        except:
            pass

        ntries = 1
        attempts = 0

        cmd = self.make_command(cmd)
        # rospy.loginfo(cmd)
        # DEBUG:
        try:
            self.port.write(cmd)
            values = self.recv_array()
            if values == "SUM_ERR":
                rospy.logwarn("SUM_ERR: {}".format(cmd))
            while attempts < ntries and (
                values == ""
                or values == "Invalid Command"
                or values == []
                or values == None
                or values == "SUM_ERR"
            ):
                # DEBUG:
                try:
                    self.port.flushInput()
                    self.port.write(cmd)
                    values = self.recv_array()
                except:
                    print_info(
                        'Exception executing command "execute_array": ' + cmd
                    )
                attempts += 1
                # rospy.logwarn('Retry: {}, times: {}'.format(cmd, attempts))
        except:
            self.mutex.release()
            print_info(
                'Exception executing command "execute_array": {}'.format(cmd)
            )
            raise SerialException
            return []

        try:
            if python3:
                values = values
            else:
                values = map(int, values)
        except:
            values = []

        self.mutex.release()
        return values

    def execute_ack(self, cmd):
        """Thread safe execution of "cmd" on the Arduino returning True if response is ACK."""
        # Static variable define
        try:
            self.execute_ack.__func__.last_print_time = (
                self.execute_ack.__func__.last_print_time
            )
        except AttributeError:
            self.execute_ack.__func__.last_print_time = rospy.get_time()
            self.execute_ack.__func__.execute_time = 0
            self.execute_ack.__func__.execute_success_time = 0

        self.mutex.acquire()

        # TOCHECK: Sau khi re-connect Arduino, lệnh flushInput nếu để trong try-except
        # sẽ in ra lỗi mặc dù vẫn thực hiện được
        # Đặt các hàm led_control_cb, lift_cart_cb ... ở class ArduinoROS thì không bị lỗi
        self.port.flushInput()
        # try:
        #     self.port.flushInput()
        # except:
        #     pass

        ntries = 10
        attempts = 0
        execute_error = False

        cmd = self.make_command(cmd)
        # rospy.loginfo(cmd)
        # DEBUG:
        try:
            self.port.write(cmd)
            ack = self.recv(self.timeout)
            # print_info(ack)
            self.execute_ack.__func__.execute_time += 1
            # if ack == 'SUM_ERR':
            #     rospy.logwarn('SUM_ERR: {}'.format(cmd))
            execute_error = (
                ack != "OK"
                or ack == ""
                or ack == "Invalid Command"
                or ack == None
                or ack == "SUM_ERR"
            )
            while attempts < ntries and execute_error:
                # DEBUG:
                try:
                    self.port.flushInput()
                    self.port.write(cmd)
                    ack = self.recv(self.timeout)
                except:
                    print_info(
                        'Exception executing command "execute_ack": {}'.format(
                            cmd
                        )
                    )
                attempts += 1
                # rospy.logwarn('Retry: {}, times: {}'.format(cmd, attempts))
        except:
            self.mutex.release()
            print_info(
                'Exception executing command "execute_ack": {}'.format(cmd)
            )
            print_info(sys.exc_info())
            self.exe_ack_err.stamp = rospy.Time.now()
            self.exe_ack_err.data += 1
            self.exe_ack_error_pub.publish(self.exe_ack_err)
            return 0

        if not execute_error:
            self.execute_ack.__func__.execute_success_time += 1

        # if rospy.get_time() - self.execute_ack.__func__.last_print_time > 5:
        #     rospy.logdebug(
        #         "execute_ack success rate: {}%".format(
        #             (self.execute_ack.__func__.execute_success_time * 100)
        #             / self.execute_ack.__func__.execute_time
        #         )
        #     )
        #     self.execute_ack.__func__.last_print_time = rospy.get_time()

        self.mutex.release()
        return ack == "OK"

    def update_pid(self, Kp, Kd, Ki, Ko):
        """Set the PID parameters on the Arduino"""
        print_info("Updating PID parameters")
        cmd = "u " + str(Kp) + ":" + str(Kd) + ":" + str(Ki) + ":" + str(Ko)
        self.execute_ack(cmd)

    def get_baud(self):
        """Get the current baud rate on the serial port."""
        return int(self.execute("b"))

    def get_encoder_counts(self):
        values = self.execute_array("e")
        if len(values) != 2:
            # print_info(
            #     "Encoder count was not 2. Actual is: {}".format(len(values))
            # )
            raise SerialException
            return None
        else:
            return values

    def get_ultrasonic(self, count):
        values = self.execute_array("u#%d" % count)
        if len(values) != count:
            # print_info(
            #     "Utrasonic count was not {}. Actual is: {}".format(
            #         count, len(values)
            #     )
            # )
            raise SerialException
            return None
        else:
            return values

    def reset_encoders(self):
        """Reset the encoder counts to 0"""
        return self.execute_ack("r")

    def drive(self, right, left):
        """Speeds are given in encoder ticks per PID interval"""
        return self.execute_ack("m %d %d" % (right, left))

    def drive_m_per_s(self, right, left):
        """Set the motor speeds in meters per second."""
        left_revs_per_second = float(left) / (self.wheel_diameter * PI)
        right_revs_per_second = float(right) / (self.wheel_diameter * PI)

        left_ticks_per_loop = int(
            left_revs_per_second
            * self.encoder_resolution
            * self.PID_INTERVAL
            * self.gear_reduction
        )
        right_ticks_per_loop = int(
            right_revs_per_second
            * self.encoder_resolution
            * self.PID_INTERVAL
            * self.gear_reduction
        )

        self.drive(right_ticks_per_loop, left_ticks_per_loop)

    def stop(self):
        """Stop both motors."""
        self.drive(0, 0)

    def ping(self):
        values = self.execute_array("p")
        if len(values) != 5:
            print_info("ping count was not 5")
            raise SerialException
            return None
        else:
            return values

    def get_voltage(self):
        return self.execute("v")

    def get_emergency_button(self):
        return self.execute("j")

    def get_pidin(self):
        values = self.execute_array("i")
        if len(values) != 2:
            print_info("get_pidin count was not 2")
            raise SerialException
            return None
        else:
            return values

    def get_pidout(self):
        values = self.execute_array("f")
        if len(values) != 2:
            print_info("get_pidout count was not 2")
            raise SerialException
            return None
        else:
            return values

    def analog_read(self, pin):
        return self.execute("a#%d" % pin)

    def analog_write(self, pin, value):
        return self.execute_ack("x#%d#%d" % (pin, value))

    def digital_read(self, pin):
        return self.execute("d#%d" % pin)

    def digital_write(self, pin, value, retry=0):
        result = self.execute_ack("w#%d#%d" % (pin, value))
        # confirm = self.digital_read(pin)
        # if confirm != value:
        #     if retry >= 5:
        #         self.arduino_error_pub.publish(
        #             StringStamped(stamp=rospy.Time.now(), data="digital_write")
        #         )
        #         self.close()
        #         return 0
        #     rospy.logwarn("Retry digital_write: {}".format(retry + 1))
        #     self.digital_write(pin, value, retry=retry + 1)
        return result

    def pin_mode(self, pin, mode):
        return self.execute_ack("c#%d#%d" % (pin, mode))

    def led_write(self, value):
        return self.execute_ack("l#%d" % (value))

    def led_rgb_write(self, value):
        return self.execute_ack(
            "n#%s#%s#%s#%s#%s#%s"
            % (
                value.type,
                value.duration,
                value.blink_interval,
                value.r,
                value.g,
                value.b,
            )
        )

    def turn_signal_write(self, front_l, front_r, rear_l, rear_r):
        return self.execute_ack(
            "y#%s#%s#%s#%s"
            % (
                front_l,
                front_r,
                rear_l,
                rear_r,
            )
        )

    def conveyor_write(self, id, cmd):
        return self.execute_ack(
            "%s#%s#%s"
            % (
                CONVEYOR,
                id,
                cmd,
            )
        )

    def stopper_write(self, id, cmd):
        return self.execute_ack(
            "%s#%s#%s"
            % (
                STOPPER,
                id,
                cmd,
            )
        )

    def voltage_motor_write(self, vol_left, vol_right):
        # TOCHECK: Lỗi, không phản hồi
        return self.execute(
            "%s#%s#%s" % (VEL_CVR_VOL, round(vol_left, 2), round(vol_right, 2)),
            ret_string=True,
        )

    def towerlamp_set(self, red, green, yellow, buzzer, blink_interval):
        return self.execute_ack(
            "t#%s#%s#%s#%s#%s" % (red, green, yellow, buzzer, blink_interval)
        )

    def log_confirm(self, value):
        return self.execute("f#%s" % (value), ret_string=True)

    def cmd_vel_write(self, lin_vel, ang_vel):
        return self.execute(
            "v#%s#%s" % (round(lin_vel, 2), round(ang_vel, 2)), ret_string=True
        )

    def motor_speed_write(self, v_left, v_right):
        return self.execute(
            "m#%s#%s" % (round(v_left, 2), round(v_right, 2)), ret_string=True
        )

    def lift_cart(self, cmd):
        return self.execute_ack("g#%d" % (cmd))

    def float_param_read(self, code):
        try:
            return float(self.execute("z#%d" % (code), ret_string=True))
        except:
            return None


"""


  #####    ##    ####  ######     ####   ####  #    # ##### #####   ####  #      #      ###### #####
  #    #  #  #  #      #         #    # #    # ##   #   #   #    # #    # #      #      #      #    #
  #####  #    #  ####  #####     #      #    # # #  #   #   #    # #    # #      #      #####  #    #
  #    # ######      # #         #      #    # #  # #   #   #####  #    # #      #      #      #####
  #    # #    # #    # #         #    # #    # #   ##   #   #   #  #    # #      #      #      #   #
  #####  #    #  ####  ######     ####   ####  #    #   #   #    #  ####  ###### ###### ###### #    #


"""

""" Class to receive Twist commands and publish Odometry data """


class BaseController:
    def __init__(self, arduino, base_frame):
        self.arduino = arduino
        self.base_frame = base_frame
        self.rate = float(rospy.get_param("~base_controller_rate", 10))
        self.timeout = rospy.get_param("~base_controller_timeout", 1.0)
        self.use_odom = rospy.get_param("~use_odom", False)
        self.use_motor = rospy.get_param("~use_motor", True)
        self.use_dac_motor = rospy.get_param("~use_dac_motor", True)
        self.use_diff_speed = rospy.get_param("~use_diff_speed", False)
        self.use_dyp_ultrasonic = rospy.get_param("~use_dyp_ultrasonic", False)
        self.use_turn_signal = rospy.get_param("~use_turn_signal", False)
        self.use_conveyor = rospy.get_param("~use_conveyor", False)
        self.use_stopper = rospy.get_param("~use_stopper", False)

        self.footprint_frame = rospy.get_param(
            "~footprint_frame", "base_footprint"
        )
        self.use_arduino_encoder = rospy.get_param(
            "~use_arduino_encoder", False
        )
        self.use_outside_encoder = rospy.get_param(
            "~use_outside_encoder", False
        )
        self.sudo_password = str(rospy.get_param("~sudo_password", ""))

        self.use_emg_vel = rospy.get_param("~use_emg_vel", False)
        emg_name = rospy.get_param("~emg_name", "emg_button")
        self.emg_topic = rospy.get_name() + "/sensor/" + emg_name

        self.use_power_button = rospy.get_param("~use_power_button", False)
        power_button_name = rospy.get_param(
            "~power_button_name", "power_button"
        )
        self.power_button_topic = (
            rospy.get_name() + "/sensor/" + power_button_name
        )

        now = rospy.Time.now()
        self.then = now  # time for determining dx/dy when calculate odometry
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = rospy.get_time()
        self.t_delta_float = 1 / self.rate

        self.odom_rate_actual = 0
        self.odom_rate_time = rospy.get_time()
        self.left_moving = True
        self.right_moving = True
        self.left_err_cnt = 0
        self.right_err_cnt = 0
        self.left_raw_old = 0
        self.right_raw_old = 0

        # imu variable
        self.imu_data_status = 0
        self.last_th_imu = 0

        if self.use_odom and (
            self.use_arduino_encoder or self.use_outside_encoder
        ):
            self.init_variable_odom()
            rospy.Subscriber("/reset_odom", Empty, self.reset_odom_cb)
        if self.use_odom and not (
            self.use_arduino_encoder or self.use_outside_encoder
        ):
            rospy.logerr(
                "Use odom but do not use_arduino_encoder or use_outside_encoder. Please check config file!"
            )
        if self.use_arduino_encoder and self.use_outside_encoder:
            rospy.logwarn(
                "Read both of arduino_encoder and outside_encoder. Use outside_encoder for odometry calculate"
            )

        if self.use_turn_signal:
            self.last_signal_data = []
            rospy.Subscriber(
                "/arduino_driver/led_turn",
                Int16MultiArrayStamped,
                self.led_turn_cb,
            )

        if self.use_conveyor:
            self.last_signal_data = []
            rospy.Subscriber(
                "/arduino_driver/set_conveyor",
                StringStamped,
                self.set_conveyor_cb,
            )

        if self.use_turn_signal:
            self.last_signal_data = []
            rospy.Subscriber(
                "/arduino_driver/set_stopper",
                StringStamped,
                self.set_stopper_cb,
            )

        if self.use_arduino_encoder:
            self.arduino_encoder_msg = EncoderDifferential()
            self.arduino_encoder_pub = rospy.Publisher(
                "/arduino_encoder", EncoderDifferential, queue_size=5
            )

        if self.use_outside_encoder or True:
            rospy.Subscriber("/encoder", EncoderDifferential, self.encoder_cb)
            self.income_encoder_msg = EncoderDifferential()
            self.t_enc_next = rospy.Time.now()

        if self.use_emg_vel:
            rospy.Subscriber(self.emg_topic, Digital, self.emg_cb)
            self.reset_motor_pub = rospy.Publisher(
                "/reset_motor", Empty, queue_size=5
            )
            self.last_emg = SENSOR_DEACTIVATE

        if self.use_power_button:
            rospy.Subscriber(
                self.power_button_topic, Digital, self.power_button_cb
            )

        # self.ultrasonics_offset = rospy.get_param("~ultrasonics_offset", 0.0)
        if self.use_dyp_ultrasonic:
            self.ultrasonic_count = rospy.get_param("~ultrasonic_count", 0)
            # self.ultrasonic_name = rospy.get_param("~ultrasonic_name", "ultrasonic")
            self.ultrasonic_pub_cloud = rospy.Publisher(
                "/ultrasonic_cloudpoint", PointCloud2, queue_size=5
            )
            self.ultrasonic_maxval = 4.0

            self.sensors_offset = rospy.get_param("~sensors_offset", 0.0)
            self.ultrasonic_cloud = [
                [100.0, 0.105, 0.1],
                [100.0, -0.105, 0.1],
                [0.2, 100.0, 0.1],
                [0.2, -100.0, 0.1],
            ]

            self.ultrasonics_names = []
            self.ultrasonics_x = []
            self.ultrasonics_y = []
            self.ultrasonics_yaw = []
            i = 0
            dict_iter = None
            if python3:
                dict_iter = self.digital_output_dict.items()
            else:
                dict_iter = self.digital_output_dict.iteritems()
            for name, params in dict_iter:
                if "ultrasonic" in name:
                    topic_name = name
                    setattr(
                        self,
                        topic_name,
                        rospy.Publisher(
                            "~sensor/" + topic_name, Range, queue_size=5
                        ),
                    )
                    self.ultrasonic_cloud[i][0] = params[
                        "x"
                    ] + self.ultrasonic_maxval * math.cos(params["yaw"])
                    self.ultrasonic_cloud[i][1] = params[
                        "y"
                    ] + self.ultrasonic_maxval * math.sin(params["yaw"])
                    self.ultrasonic_cloud[i][2] = params["z"]

                    self.ultrasonics_names.append(name)
                    self.ultrasonics_x.append(params["x"])
                    self.ultrasonics_y.append(params["y"])
                    self.ultrasonics_yaw.append(params["yaw"])
                    i += 1

        rospy.Subscriber("/led_job", Int16, self.led_job_cb)
        rospy.Subscriber("/led_control", LedControl, self.led_control_cb)
        rospy.Subscriber("/lift_cart", Int8Stamped, self.lift_cart_cb)
        if self.use_arduino_encoder:
            rospy.Subscriber("/reset_encoder", Empty, self.reset_encoder_cb)

        if self.use_diff_speed:
            rospy.Subscriber(
                "/motor_speed", DiffDriverMotorSpeed, self.motor_speed_cb
            )

        if self.use_motor:
            self.motor_status_last_time = rospy.get_time()
            rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
            self.motor_status_pub = rospy.Publisher(
                "/motor_status", String, queue_size=5
            )

    def lift_cart_cb(self, msg):
        self.arduino.lift_cart(msg.data)

    def motor_speed_cb(self, msg):
        value = self.arduino.motor_speed_write(msg.Left, msg.Right)

    def led_job_cb(self, msg):
        self.led_state = msg.data
        ack = self.arduino.led_write(self.led_state)
        rospy.loginfo("Update led state: {}, {}".format(self.led_state, ack))

    def led_control_cb(self, msg):
        ack = self.arduino.led_rgb_write(msg)
        # rospy.loginfo('Update led rgb result is "{}" for cmd:\n{}'.format(ack, msg))

    def cmd_vel_cb(self, msg):
        if self.use_dac_motor:
            # TODO: Config direction
            left_vel = msg.linear.x + (msg.angular.z * self.wheel_track / 2)
            right_vel = msg.linear.x - (msg.angular.z * self.wheel_track / 2)

            # try:
            #     allowable_rate_left = abs(left_vel) / abs(self.left_vel_odom)
            #     allowable_rate_right = abs(right_vel) / abs(self.right_vel_odom)
            # except ZeroDivisionError:
            #     allowable_rate_left = 1
            #     allowable_rate_right = 1

            vol_left = left_vel * self.mps_to_dac
            vol_right = right_vel * self.mps_to_dac

            value = self.arduino.motor_speed_write(
                vol_left,
                vol_right,
            )
        else:
            value = self.arduino.cmd_vel_write(msg.linear.x, msg.angular.z)

    def encoder_cb(self, msg):
        t_delta = rospy.Duration(1.0 / 15.0)
        t_now = rospy.Time.now()
        if t_now > self.t_enc_next:
            if abs(msg.left - self.income_encoder_msg.left) < 2:
                self.left_moving = False
            else:
                self.left_moving = True

            if abs(msg.right - self.income_encoder_msg.right) < 2:
                self.right_moving = False
            else:
                self.right_moving = True

        rospy.logdebug(
            "msg: %i, %i, income_enc: %i, %i, moving: %i, %i"
            % (
                msg.left,
                msg.right,
                self.income_encoder_msg.left,
                self.income_encoder_msg.right,
                self.left_moving,
                self.right_moving,
            )
        )
        self.t_enc_next = t_now + t_delta
        self.income_encoder_msg = msg

    def led_turn_cb(self, msg):
        if len(msg.data) == 4:
            if msg.data != self.last_signal_data:
                ack = self.arduino.turn_signal_write(
                    # TODO: Set order from config file
                    msg.data[0],
                    msg.data[1],
                    msg.data[2],
                    msg.data[3],
                )
                self.last_signal_data = msg.data

    def set_conveyor_cb(self, msg):
        data_dict = json.loads(msg.data)
        id = data_dict["id"]
        cmd = data_dict["cmd"]
        self.arduino.conveyor_write(id, cmd)

    def set_stopper_cb(self, msg):
        data_dict = json.loads(msg.data)
        id = data_dict["id"]
        cmd = data_dict["cmd"]
        self.arduino.stopper_write(id, cmd)

    def emg_cb(self, msg):
        if msg.value == SENSOR_ACTIVATE and self.last_emg == SENSOR_DEACTIVATE:
            for i in range(5):
                self.reset_motor_pub.publish(Empty())
                rospy.sleep(0.1)
        self.last_emg = msg.value

    def power_button_cb(self, msg):
        if msg.value == SENSOR_ACTIVATE:
            rospy.loginfo("power button pressed")
            command = "poweroff"
            p = os.system("echo %s|sudo -S %s" % (self.sudo_password, command))

    def reset_odom_cb(self, msg):
        self.reset_encoder_pub.publish(Empty())
        if self.use_imu:
            self.reset_imu_pub.publish(EmptyStamped(stamp=rospy.Time.now()))
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.enc_left = 0
        self.enc_right = 0
        self.l_wheel_mult = 0
        self.r_wheel_mult = 0

    def reset_encoder_cb(self, msg):
        self.arduino.reset_encoders()
        self.arduino_encoder_msg.left = 0
        self.arduino_encoder_msg.right = 0
        self.left_err_cnt = 0
        self.right_err_cnt = 0

    def init_variable_odom(self):
        now = rospy.Time.now()

        self.wheel_diameter = rospy.get_param("~wheel_radius", 0.0) * 2
        self.wheel_track = rospy.get_param("~wheel_separation", 0.0)
        self.encoder_resolution = rospy.get_param("~encoder_resolution", 0.0)
        self.gear_reduction = rospy.get_param("~gear_reduction", 1.0)
        self.accel_limit = rospy.get_param("~accel_limit", 0.1)
        self.motors_reversed = rospy.get_param("~motors_reversed", False)
        self.gear_motor = rospy.get_param("~gear_motor", 40)  # Only for DAC vel
        self.rpm = rospy.get_param("~rpm", 2800)
        self.rps = (self.rpm / self.gear_motor) / 60  # rps
        self.wheel_perimeter = self.wheel_diameter * PI  # Chu vi
        self.vol_digital_max = rospy.get_param(
            "~vol_digital_max", 3276
        )  # 5V = 4095
        self.vel_max_mps = self.rps * self.wheel_perimeter  # m/s
        self.mps_to_dac = (
            self.vol_digital_max / self.vel_max_mps
        )  # Convert to DAC value

        self.ticks_per_meter = (
            self.encoder_resolution
            * self.gear_reduction
            / (self.wheel_diameter * PI)
        )

        # What is the maximum acceleration we will tolerate when changing wheel speeds?
        self.max_accel = self.accel_limit * self.ticks_per_meter / self.rate

        # Track how often we get a bad encoder count (if any)
        self.bad_encoder_count = 0

        self.use_imu = rospy.get_param("~use_imu", True)
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.encoder_min = rospy.get_param("encoder_min", -2 * 10**31)
        self.encoder_max = rospy.get_param("encoder_max", 2 * 10**31)
        self.encoder_low_wrap = rospy.get_param(
            "wheel_low_wrap",
            (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min,
        )
        self.encoder_high_wrap = rospy.get_param(
            "wheel_high_wrap",
            (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min,
        )
        self.l_wheel_mult = 0
        self.r_wheel_mult = 0

        if self.use_imu:
            self.imu_data = Imu()
            rospy.loginfo("IMU not connect")
            self.imu_connected = False
            rospy.Subscriber("/imu_data", Imu, self.imu_data_cb)
            self.reset_imu_pub = rospy.Publisher(
                "/reset_imu", EmptyStamped, queue_size=10
            )
            # TODO: Change reset_imu, reset_encoder to service
            rospy.sleep(0.5)
            self.reset_imu_pub.publish(EmptyStamped(stamp=rospy.Time.now()))
            rospy.loginfo("Reset IMU")

        # Internal data
        self.enc_left = None  # encoder readings
        self.enc_right = None
        self.x = 0  # position in xy plane
        self.y = 0
        self.th = 0  # rotation in radians
        self.v_left = 0
        self.v_right = 0
        self.v_des_left = 0  # cmd_vel setpoint
        self.v_des_right = 0
        self.last_cmd_vel = now

        # self.lVelPub = rospy.Publisher('Lvel', Int16, queue_size=5)
        # self.rVelPub = rospy.Publisher('Rvel', Int16, queue_size=5)

        # Clear any old odometry info
        self.arduino.reset_encoders()

        # Set up the odometry broadcaster
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=5)
        self.reset_encoder_pub = rospy.Publisher(
            "/reset_encoder", Empty, queue_size=10
        )
        self.odomBroadcaster = TransformBroadcaster()

        # Reset encoder and IMU
        self.reset_encoder_pub.publish(Empty())

    def imu_data_cb(self, msg):
        self.imu_data = msg

    def make_range_msg(self, range_data):
        if range_data != None:
            pccloud = PointCloud2()
            for i in range(self.ultrasonic_count):
                topic_name = self.ultrasonics_names[i]
                pub = getattr(self, topic_name)
                msg = Range()
                msg.header.frame_id = topic_name
                msg.header.stamp = rospy.Time.now()
                # DYP ultrasonic sensor
                msg.min_range = 0.25
                msg.max_range = 4.0
                msg.field_of_view = 0.7  # 40 Deg
                msg.radiation_type = 0  # ULTRASOUND=0
                msg.range = range_data[i] / 1000.0  # milimet to met
                if msg.range > msg.max_range or msg.range < msg.min_range:
                    msg.range = 10.0
                pub.publish(msg)
                self.ultrasonic_cloud[i][0] = self.ultrasonics_x[
                    i
                ] + msg.range * math.cos(self.ultrasonics_yaw[i])
                self.ultrasonic_cloud[i][1] = self.ultrasonics_y[
                    i
                ] + msg.range * math.sin(self.ultrasonics_yaw[i])
            pccloud.header.frame_id = "base_footprint"
            pccloud = pc2.create_cloud_xyz32(
                pccloud.header, self.ultrasonic_cloud
            )
            self.ultrasonic_pub_cloud.publish(pccloud)

    def odometry_calr(self):
        if self.use_arduino_encoder:
            try:
                # (
                #     self.arduino_encoder_msg.left,
                #     self.arduino_encoder_msg.right,
                # ) = self.arduino.get_encoder_counts()
                enc_left_raw, enc_right_raw = self.arduino.get_encoder_counts()
                if self.left_moving:
                    self.arduino_encoder_msg.left = (
                        enc_left_raw - self.left_err_cnt
                    )
                else:
                    self.left_err_cnt += enc_left_raw - self.left_raw_old
                self.left_raw_old = enc_left_raw

                if self.right_moving:
                    self.arduino_encoder_msg.right = (
                        enc_right_raw - self.right_err_cnt
                    )
                else:
                    self.right_err_cnt += enc_right_raw - self.right_raw_old
                self.right_raw_old = enc_right_raw
                rospy.logdebug(
                    "enc_raw: %i - %i | enc_err: %i - %i |enc_pub: %i - %i"
                    % (
                        enc_left_raw,
                        enc_right_raw,
                        self.left_err_cnt,
                        self.right_err_cnt,
                        self.arduino_encoder_msg.left,
                        self.arduino_encoder_msg.right,
                    )
                )

                self.arduino_encoder_msg.header.stamp = rospy.Time.now()
                self.arduino_encoder_pub.publish(self.arduino_encoder_msg)
            except Exception as e:
                self.bad_encoder_count += 1
                # rospy.logerr(
                #     "Encoder exception count: " + str(self.bad_encoder_count)
                # )
                # rospy.logerr(e)
                return

        if self.use_outside_encoder:
            left_enc = self.income_encoder_msg.left
            right_enc = self.income_encoder_msg.right
        else:
            left_enc = self.arduino_encoder_msg.left
            right_enc = self.arduino_encoder_msg.right

        now = rospy.Time.now()
        dt = now - self.then
        self.then = now
        dt = dt.to_sec()

        # Calculate odometry
        if self.enc_left == None:
            dright = 0
            dleft = 0
        else:
            if (
                left_enc < self.encoder_low_wrap
                and self.enc_left > self.encoder_high_wrap
            ):
                self.l_wheel_mult = self.l_wheel_mult + 1
            elif (
                left_enc > self.encoder_high_wrap
                and self.enc_left < self.encoder_low_wrap
            ):
                self.l_wheel_mult = self.l_wheel_mult - 1
            else:
                self.l_wheel_mult = 0
            if (
                right_enc < self.encoder_low_wrap
                and self.enc_right > self.encoder_high_wrap
            ):
                self.r_wheel_mult = self.r_wheel_mult + 1
            elif (
                right_enc > self.encoder_high_wrap
                and self.enc_right < self.encoder_low_wrap
            ):
                self.r_wheel_mult = self.r_wheel_mult - 1
            else:
                self.r_wheel_mult = 0
            # dright = (right_enc - self.enc_right) / self.ticks_per_meter
            # dleft = (left_enc - self.enc_left) / self.ticks_per_meter
            dleft = (
                1.0
                * (
                    left_enc
                    + self.l_wheel_mult * (self.encoder_max - self.encoder_min)
                    - self.enc_left
                )
                / self.ticks_per_meter
            )
            dright = (
                1.0
                * (
                    right_enc
                    + self.r_wheel_mult * (self.encoder_max - self.encoder_min)
                    - self.enc_right
                )
                / self.ticks_per_meter
            )

        self.enc_right = right_enc
        self.enc_left = left_enc

        dxy_ave = (dright + dleft) / 2.0
        dth = (dright - dleft) / self.wheel_track
        vxy = dxy_ave / dt
        vth = dth / dt

        dx = float()
        dy = float()
        if dxy_ave != 0:
            dx = cos(dth) * dxy_ave
            dy = -sin(dth) * dxy_ave

        if not (self.use_imu):
            if dth != 0:
                self.th += dth

        # declace imu variable ros
        if self.use_imu:
            if rospy.get_time() - self.imu_data.header.stamp.to_sec() < 0.2:
                # TODO: Add filter
                self.imu_connected = True
                # read Z_azimuth from imu
                raw_th_imu = euler_from_quaternion(
                    [
                        self.imu_data.orientation.x,
                        self.imu_data.orientation.y,
                        self.imu_data.orientation.z,
                        self.imu_data.orientation.w,
                    ]
                )[2]
                # read Z_azimuth rate from imu
                vth_imu = self.imu_data.angular_velocity.z
                ath_imu = self.imu_data.linear_acceleration.x
                # when robot not move
                if (abs(vth_imu) < 0.0004) and (abs(ath_imu) < 0.05):
                    dx = 0
                    dy = 0
                # set last of th_imu when imu connect first time
                if self.imu_data_status == 0:
                    self.imu_data_status = 1
                    # rospy.loginfo('IMU init')
                    self.last_th_imu = raw_th_imu
                # calculate delta theta
                if self.imu_data_status == 1:
                    dth_imu = raw_th_imu - self.last_th_imu
                    self.last_th_imu = raw_th_imu
                    # rospy.loginfo('IMU use')
                    # when robot move
                    if abs(dth_imu) > 0.0003:
                        self.th += dth_imu
                        # rospy.loginfo(self.th)

                vth = vth_imu
            else:
                self.imu_connected = False
                self.imu_data_status = 0
                # rospy.loginfo('IMU time error')
                # use dth of encoder wheel
                if dth != 0:
                    self.th += dth
        if dxy_ave != 0:
            self.x += cos(self.th) * dx - sin(self.th) * dy
            self.y += sin(self.th) * dx + cos(self.th) * dy

        # update data
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th / 2.0)
        quaternion.w = cos(self.th / 2.0)

        # Create the odometry transform frame broadcaster.
        if self.publish_tf:
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.footprint_frame,
                "odom",
            )

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = self.footprint_frame
        odom.header.stamp = now
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = vxy
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth

        odom.pose.covariance = ODOM_POSE_COVARIANCE
        odom.twist.covariance = ODOM_TWIST_COVARIANCE
        # todo sensor_state.distance == 0
        # if self.v_des_left == 0 and self.v_des_right == 0:
        #    odom.pose.covariance = ODOM_POSE_COVARIANCE2
        #    odom.twist.covariance = ODOM_TWIST_COVARIANCE2
        # else:
        #    odom.pose.covariance = ODOM_POSE_COVARIANCE
        #    odom.twist.covariance = ODOM_TWIST_COVARIANCE

        self.odomPub.publish(odom)

    def poll(self):
        now = rospy.get_time()
        if now > self.t_next:
            self.odom_rate_actual += 1
            dt = rospy.get_time() - self.odom_rate_time
            if dt >= 1.0:
                # rospy.logwarn(
                #     "odom rate: {}, dt: {}".format(self.odom_rate_actual, dt)
                # )
                self.odom_rate_time = rospy.get_time()
                # Adjust base_controller rate
                if self.odom_rate_actual < self.rate:
                    self.t_delta_float -= 0.005
                elif self.odom_rate_actual > self.rate:
                    self.t_delta_float += 0.005
                self.odom_rate_actual = 0
            if self.use_odom and (
                self.use_arduino_encoder or self.use_outside_encoder
            ):
                self.odometry_calr()
            if self.use_dyp_ultrasonic:
                self.make_range_msg(
                    self.arduino.get_ultrasonic(self.ultrasonic_count)
                )
                # try:
                #     self.make_range_msg(
                #         self.arduino.get_ultrasonic(self.ultrasonic_count)
                #     )
                # except Exception as e:
                #     rospy.logerr("Read Ultrasonic error: {}".format(e))
                #     pass

            self.t_next = now + (self.t_delta_float)

        if (
            self.use_motor
            and rospy.get_time() - self.motor_status_last_time >= 1.0
        ):
            self.motor_status_last_time = rospy.get_time()
            self.motor_status_pub.publish(
                String(json.dumps({"left": "NORMAL", "right": "NORMAL"}))
            )


"""


    ##   #####  #####  #    # # #    #  ####     #####   ####   ####
   #  #  #    # #    # #    # # ##   # #    #    #    # #    # #
  #    # #    # #    # #    # # # #  # #    #    #    # #    #  ####
  ###### #####  #    # #    # # #  # # #    #    #####  #    #      #
  #    # #   #  #    # #    # # #   ## #    #    #   #  #    # #    #
  #    # #    # #####   ####  # #    #  ####     #    #  ####   ####


"""


class ArduinoROS:
    def __init__(self):
        rospy.init_node("arduino_driver", log_level=rospy.INFO)

        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)

        self.port_pid = rospy.get_param("~port_pid", "/dev/ttyACM0")
        self.baud = int(rospy.get_param("~baud", 57600))
        self.timeout = rospy.get_param("~timeout", 0.5)
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.auto_find_from_pid = rospy.get_param("~auto_find_from_pid", False)
        rospy.loginfo("Set port pid: {}".format(self.port_pid))
        port_location = rospy.get_param("~port", "/dev/ttyUSB0")
        print("Set port: {}".format(port_location))
        self.port_name = find_soft_port(port_location)
        if self.auto_find_from_pid:
            self.port_name = find_soft_port_from_pid(self.port_pid)

        rospy.loginfo("Connecting port: {}".format(self.port_name))

        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 30))
        r = rospy.Rate(self.rate)
        self.rate_actual = 0
        self.main_rate_time = rospy.get_time()

        # use tower lamp
        self.use_tower_lamp = rospy.get_param("~use_tower_lamp", False)

        # Rate at which summary SensorState message is published. Individual sensors publish
        # at their own rates.
        self.sensorstate_rate = int(rospy.get_param("~sensorstate_rate", 10))

        self.use_base_controller = rospy.get_param(
            "~use_base_controller", False
        )

        # Set up the time for publishing the next SensorState message
        now = rospy.Time.now()
        self.t_delta_sensors = rospy.Duration(1.0 / self.sensorstate_rate)
        self.t_next_sensors = now + self.t_delta_sensors

        # Initialize a Twist message
        self.cmd_vel = Twist()

        # Reserve a thread lock
        self.mutex = thread.allocate_lock()
        sensors_msg_dict = {}
        std_io_msg = StringStamped()

        last_successed = rospy.get_time()
        count = int(0)
        self.connect_again = False
        # DEBUG:
        # if True:
        while True:
            try:
                self.controller = Arduino(self.port_name, self.baud, self.timeout)
                self.controller.connect()
                rospy.loginfo(
                    "Connected to Arduino on port "
                    + self.port_name
                    + " at "
                    + str(self.baud)
                    + " baud"
                )
                self.setup_params()
            except:
                rospy.loginfo("Connected to Arduino on port fail!")
            if self.controller.is_connect:
                break
            else:
                rospy.logwarn("Retry connect to arduino after 1s ...")
                rospy.sleep(1)

        # A cmd_vel publisher so we can stop the robot when shutting down
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
        self.standard_io_pub = rospy.Publisher(
            "standard_io", StringStamped, queue_size=5
        )
        rospy.Service(
            "~digital_write", StringService, self.digital_write_handle
        )
        rospy.Subscriber("digital_write", StringStamped, self.digital_write_cb)
        if self.use_tower_lamp:
            rospy.Subscriber(
                "/towerlamp_set", StringStamped, self.towerlamp_set_cb
            )
        # rospy.Subscriber('/led_control', LedControl, self.led_control_cb)
        # rospy.Subscriber('/lift_cart', Int8Stamped, self.lift_cart_cb)

        # Sensor status list
        # self.arduino_sensor_pub = rospy.Publisher(
        #     "arduino_sensor", ArduinoSensorList, queue_size=5
        # )

        # Start polling the sensors and base controller
        while not rospy.is_shutdown():
            # if self.connect_again:
            #     try:
            #         self.controller = Arduino(
            #             self.port_name, self.baud, self.timeout
            #         )
            #         self.controller.connect()
            #         rospy.loginfo(
            #             "Connected to Arduino on port "
            #             + self.port_name
            #             + " at "
            #             + str(self.baud)
            #             + " baud"
            #         )
            #         self.setup_params()
            #         self.connect_again = False
            #     except:
            #         rospy.loginfo("Connected to Arduino on port fail!")

            if self.controller.is_connect and (self.port_name != None):
                # print_info(self.port_name)
                sensors_msg_dict = {}
                for sensor in self.sensors_list:
                    if sensor.rate == 0:
                        continue
                    self.mutex.acquire()
                    value = sensor.poll()
                    if value != None:
                        last_successed = rospy.get_time()
                        sensors_msg_dict[sensor.name] = value
                    self.mutex.release()
                if sensors_msg_dict != {}:
                    std_io_msg.stamp = rospy.Time.now()
                    std_io_msg.data = json.dumps(sensors_msg_dict, indent=2)
                    self.standard_io_pub.publish(std_io_msg)

                for param in self.float_params_list:
                    self.mutex.acquire()
                    value = param.poll()
                    self.mutex.release()

                if self.use_base_controller:
                    self.mutex.acquire()
                    self.myBaseController.poll()
                    self.mutex.release()

            # if rospy.get_time() - last_successed > 2.0: # > 2 second to connect and read data
            #     rospy.logerr("Connection timeout")
            #     last_successed = rospy.get_time()
            #     # self.controller.connect()
            #     if self.controller.is_connect and (self.port_name != None):
            #         rospy.logwarn("Check port is open")
            #         if self.controller.port.is_open:
            #             rospy.logwarn("Try to close port")
            #             self.controller.port.close()
            #         self.controller.is_connect = False
            #     else:
            #         rospy.logwarn("Try to re-open port")
            #         self.port_name = find_soft_port_from_pid(self.port_pid)
            #         rospy.loginfo('Port find result : {}'.format(self.port_name))
            #         # Try the connection again
            #         if (self.port_name != None):
            #             self.connect_again = True

            # self.rate_actual += 1
            # dt = rospy.get_time() - self.main_rate_time
            # if dt >= 1.0:
            #     rospy.logerr('main rate: {}, dt: {}'.format(self.rate_actual, dt))
            #     self.main_rate_time = rospy.get_time()
            #     self.rate_actual = 0

            # del self.sensor_list_msg.sensors[:]
            # for sensor in self.sensors_list:
            #     mutex.acquire()
            #     each_sensor = ArduinoSensor()
            #     value = sensor.poll()
            #     if value != None:
            #         each_sensor.name = sensor.name
            #         each_sensor.value = str(value)
            #         self.sensor_list_msg.sensors.append(each_sensor)
            #     mutex.release()
            # if len(self.sensor_list_msg.sensors):
            #     print_info('---')
            #     print_info(self.sensor_list_msg)
            #     self.arduino_sensor_pub.publish(self.sensor_list_msg)

            # if not type(self.controller.port) is Serial:
            #     continue
            # data_left = self.controller.port.inWaiting()
            # if data_left > 1:
            #     mutex.acquire()
            #     value = self.controller.recv(self.timeout)
            #     if value != None:
            #         value = value[1:]
            #         if not '$' in value:
            #             rospy.logerr('Log garbage: {}'.format(value))
            #         else:
            #             value_split = value.split('$')
            #             logger_key = value_split[0].split('_')[1]
            #             if logger_key != self.last_key:
            #                 if rospy.get_time() - self.last_log_time < 1.0:
            #                     rospy.logerr('Double log: {}'.format(value.replace('$', ' ')))
            #                 else:
            #                     rospy.logwarn('Log value: {}'.format(value.replace('$', ' ')))
            #                 self.last_log_time = rospy.get_time()
            #                 self.last_key = logger_key

            #             self.controller.log_confirm(value_split[1])
            #     mutex.release()
            # print("loop ...{}".format(rospy.get_time()))
            r.sleep()

    def setup_params(self):
        # Initialize any sensors
        self.sensors_list = list()
        self.float_params_list = list()

        # For Arduino logger
        self.last_key = ""
        self.last_log_time = rospy.get_time()

        # Output list
        self.digital_output_dict = {}

        # Read sensors list
        sensor_params = rospy.get_param("~sensors", dict({}))
        sensor_dict_iter = None
        if python3:
            sensor_dict_iter = sensor_params.items()
        else:
            sensor_dict_iter = sensor_params.iteritems()
        for name, params in sensor_dict_iter:
            # Set the direction to input if not specified
            try:
                params["direction"]
            except:
                params["direction"] = "input"

            sensor = None
            if params["type"] == "Digital":
                sensor = DigitalSensor(
                    self.controller, name, self.base_frame, **params
                )
                if params["direction"] == "output":
                    self.digital_output_dict[name] = params
            elif params["type"] == "Analog":
                sensor = AnalogSensor(
                    self.controller, name, self.base_frame, **params
                )
            elif params["type"] == "AnalogFloat":
                sensor = AnalogFloatSensor(
                    self.controller, name, self.base_frame, **params
                )
            # elif params['type'] == 'DYPUltraSonic':
            #     sensor = DYPUltraSonic(self.controller, name, **params)
            if sensor != None:
                self.sensors_list.append(sensor)
            else:
                rospy.logerr("sensor {} = None".format(name))
            rospy.loginfo(name + " " + str(params))

        # Read float_params list
        float_params = rospy.get_param("~float_params", dict({}))
        float_params_iter = None
        if python3:
            float_params_iter = float_params.items()
        else:
            float_params_iter = float_params.iteritems()
        for param_name, param_config in float_params_iter:
            param = FloatParameter(
                self.controller,
                param_name,
                param_config["rate"],
                param_config["code"],
            )
            self.float_params_list.append(param)
            rospy.loginfo(param_name + " " + str(param_config))

        # Initialize the base controller if used
        if self.use_base_controller:
            self.myBaseController = BaseController(
                self.controller, self.base_frame
            )

    def digital_write_handle(self, req):
        rospy.loginfo("Digital write service: {}".format(req.request))
        self.mutex.acquire()
        try:
            data_dict = json.loads(req.request)
            rospy.loginfo("Digital write service: {}".format(json.dumps(data_dict, indent=2)))
            pin = data_dict["pin"] if "pin" in data_dict else ""
            try:
                pin = int(pin)
                rospy.loginfo("Pin without name: {}".format(pin))
            except:
                try:
                    dict_iter = None
                    if python3:
                        dict_iter = self.digital_output_dict.items()
                    else:
                        dict_iter = self.digital_output_dict.iteritems()
                    for name, params in dict_iter:
                        if name == pin:
                            pin = int(params["pin"])
                            rospy.loginfo("Matching pin: {}".format(pin))
                    if pin == "":
                        raise Exception(
                            'The digital output pin "{}" is not defined'.format(
                                pin
                            )
                        )
                except Exception as e:
                    raise e
            value = bool(int(data_dict["value"]))
            hold_duration = (
                int(data_dict["hold_duration"])
                if "hold_duration" in data_dict
                else 0
            )

            rospy.loginfo("Digital write: {}: {}".format(pin, value))
            if self.controller.digital_write(pin, value):
                self.mutex.release()
                return StringServiceResponse("OK")

            # TODO: return service respond after release_hold
            if hold_duration > 0:
                release_hold = threading.Thread(
                    target=self.release_digital_hold,
                    args=(pin, not value, hold_duration),
                )
                release_hold.start()

        except Exception as e:
            rospy.logerr("Digital write service: {}".format(e))
        self.mutex.release()
        return StringServiceResponse("")

    # Nếu đặt 2 hàm này ở ngoài BaseController thì không bị lỗi flushInput sau khi re-connect Arduino
    # def led_control_cb(self, msg):
    #     ack = self.controller.led_rgb_write(msg)
    #     # rospy.loginfo('Update led rgb result is "{}" for cmd:\n{}'.format(ack, msg))

    # def lift_cart_cb(self, msg):
    #     self.controller.lift_cart(msg.data)

    def towerlamp_set_cb(self, msg):
        # rospy.loginfo("Tower lamp set: {}".format(msg.data))
        try:
            # if True:
            try:
                # if True:
                # Json string
                data = json.loads(msg.data)
                red = data["red"] if "red" in data else 0
                green = data["green"] if "green" in data else 0
                yellow = data["yellow"] if "yellow" in data else 0
                buzzer = data["buzzer"] if "buzzer" in data else 0
                blink_interval = (
                    data["blink_interval"] if "blink_interval" in data else 0
                )
                if self.controller.is_connect:
                    self.controller.towerlamp_set(
                        red, green, yellow, buzzer, blink_interval
                    )
            except Exception as e:
                # String split by ','
                try:
                    data = msg.data.split(",")
                    red = data[0]
                    green = data[1]
                    yellow = data[2]
                    buzzer = data[3]
                    blink_interval = data[4]
                    if self.controller.is_connect:
                        self.controller.towerlamp_set(
                            red, green, yellow, buzzer, blink_interval
                        )
                except:
                    raise e
        except Exception as e:
            rospy.logerr("towerlamp_set_cb: {}".format(e))

    def digital_write_cb(self, msg):
        """
        pin_name,value (0 or 1),release_hold (0: set and hold, greater than 0: milisecond to revert)
        Eg: "auto_charging_en_pin,1,0"
        """
        rospy.loginfo("Digital write: {}".format(msg.data))
        self.mutex.acquire()
        try:
            data = msg.data.split(",")
            rospy.loginfo("Digital write topic: {}".format(data))
            try:
                pin = int(data[0])
                rospy.loginfo("Pin without name: {}".format(pin))
            except:
                try:
                    pin = -1
                    dict_iter = None
                    if python3:
                        dict_iter = self.digital_output_dict.items()
                    else:
                        dict_iter = self.digital_output_dict.iteritems()
                    for name, params in dict_iter:
                        if name == data[0]:
                            pin = int(params["pin"])
                            rospy.loginfo("Matching pin: {}".format(pin))
                    if pin == -1:
                        raise Exception(
                            'The digital output pin "{}" is not defined'.format(
                                data[0]
                            )
                        )
                except Exception as e:
                    raise e
            value = bool(int(data[1]))
            hold_duration = int(data[2])

            self.controller.digital_write(pin, value)
            rospy.loginfo("Digital write: {}: {}".format(pin, value))
            if hold_duration > 0:
                release_hold = threading.Thread(
                    target=self.release_digital_hold,
                    args=(pin, not value, hold_duration),
                )
                release_hold.start()
        except Exception as e:
            rospy.logerr("Digital write error: {}".format(e))
        self.mutex.release()

    def release_digital_hold(self, pin, value, sleep_time):
        rospy.sleep(float(sleep_time) / 1000.0)
        self.mutex.acquire()
        self.controller.digital_write(pin, value)
        rospy.loginfo("Digital pin release: {}: {}".format(pin, value))
        self.mutex.release()

    def shutdown(self):
        # Stop the robot
        try:
            rospy.loginfo("Stopping the robot...")
            self.cmd_vel_pub.Publish(Twist())
            self.controller.port.close()
            rospy.sleep(2)
        except:
            pass
        rospy.loginfo("Shutting down Arduino Node...")


if __name__ == "__main__":
    myArduino = ArduinoROS()
