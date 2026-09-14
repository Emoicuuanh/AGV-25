#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import subprocess
import sys

python3 = True if sys.hexversion > 0x03000000 else False
import rospy
import rospkg
from std_msgs.msg import String, Int8
from std_stamped_msgs.msg import Float32Stamped, Int8Stamped
import time
import serial
import libscrc
import struct
from enum import Enum
from agv_msgs.msg import FollowLineSensor, DiffDriverMotorSpeed
from serial_port import get_serial_ports
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu

common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)

common_folder = os.path.join(
    rospkg.RosPack().get_path("followline_define"), "scripts"
)
if not os.path.isdir(common_folder):
    common_folder = os.path.join(
        rospkg.RosPack().get_path("followline_define"), "release"
    )
sys.path.insert(0, common_folder)

from common_function import find_soft_port
from followline_constant import *


class ReceiveStep(Enum):
    SEND = 0
    RECEIVE = 1


SENSOR_TYPE = "LENET"
FOLLOW_LINE_PORT = rospy.get_param(
    "~port",
    "2-8.2",
)

print("FollowLine sensor type: {}".format(SENSOR_TYPE))
# FOLLOW_LINE_PORT = LocalSetting.select().where(LocalSetting.Name == 'Follow_Line_Port')[0].Value
print("Setting followline port: {}".format(FOLLOW_LINE_PORT))

if SENSOR_TYPE == "CCF":
    FF_REQUEST_ID_1 = b"\x01\x03\x00\x00\x00\x02\xC4\x0B"
    FF_REQUEST_ID_2 = b"\x02\x03\x00\x00\x00\x02\xC4\x38"
    FF_DATA_LOW_BYTE = 5
    FF_DATA_HIGH_BYTE = 6
    MODBUS_ADD = 0  # 0x00
    FRONT_MODBUS_ID = 1
    REAR_MODBUD_ID = 2
    BAUD_RATE = 57600
    FF_DATA_LEN = 9
elif SENSOR_TYPE == "LENET":
    FF_REQUEST_ID_1 = b"\x01\x03\x00\x28\x00\x01\x04\x02"
    FF_REQUEST_ID_2 = b"\x02\x03\x00\x28\x00\x01\x04\x31"
    FF_DATA_LOW_BYTE = 3
    FF_DATA_HIGH_BYTE = 4
    MODBUS_ADD = 40  # 0x28
    FRONT_MODBUS_ID = 1
    REAR_MODBUD_ID = 2
    BAUD_RATE = 115200
    FF_DATA_LEN = 7

step = ReceiveStep.SEND
prev_step = None
serial_port = None
iDirection = FORWARD
rtu_master = None
enable_two_sensor = False
reset_followline = False
last_followline_sensor_receive = 0
last_lift_control_receive = 0

def revertString(value):
    return value[len(value) :: -1]


def Sensor_Receiver():
    # if iDirection == FORWARD:
    #     serial_port.write(FF_REQUEST_ID_1)
    # else:
    #     serial_port.write(FF_REQUEST_ID_2)
    # data_receive = []
    # data_left = serial_port.inWaiting()  # Check for remaining byte
    # for i in range(data_left):
    #     data_receive.append(serial_port.read())
    # if len(data_receive) != FF_DATA_LEN:
    #     return

    # crc_high, crc_low = calc_crc16_modbus(data_receive[: FF_DATA_HIGH_BYTE + 1])
    # if crc_high == int.from_bytes(data_receive[-1], "big") and int.from_bytes(
    #     data_receive[-2], "big"
    # ):
    #     data_high = data_receive[FF_DATA_LOW_BYTE]
    #     data_low = data_receive[FF_DATA_HIGH_BYTE]
    #     data_high = data_high.hex()
    #     data_low = data_low.hex()
    #     # rospy.logerr(data_high)
    #     make_followline(data_high, data_low)
    if not python3:
        if iDirection == FORWARD:
            serial_port.write(FF_REQUEST_ID_1)
        else:
            serial_port.write(FF_REQUEST_ID_2)
        data_receive = []
        data_left = serial_port.inWaiting()  # Check for remaining byte
        for i in range(data_left):
            data_receive.append(serial_port.read())
        # print(data_receive)
        if len(data_receive) != FF_DATA_LEN:
            return

        crc_high, crc_low = calc_crc16_modbus(
            data_receive[: FF_DATA_HIGH_BYTE + 1]
        )
        if crc_high == hex_str_to_int8(
            data_receive[-1]
        ) and crc_low == hex_str_to_int8(data_receive[-2]):
            data_high = data_receive[FF_DATA_LOW_BYTE]
            data_low = data_receive[FF_DATA_HIGH_BYTE]
            make_followline(data_high, data_low)


def int8_to_hex_str(int_value):
    # 16 -> '\x10'
    return str(hex(int_value))[-2:]


def hex_str_to_int8(hex_value):
    # '\10' -> 16
    hex_str = hex_value.encode("hex")
    int_value = int(hex_str, 16)
    return int_value


def calc_crc16_modbus(array_in):
    # crc16 = libscrc.modbus(b'1234')                       # Calculate ASCII of modbus
    # crc16 = libscrc.modbus(b'\x01\x03\x00\x00\x00\x02')   # Calculate HEX of modbus
    # crc16 = libscrc.modbus(bytearray( [ 0x01, 0x02 ] ))
    array_out = b""
    for i in range(len(array_in)):
        array_out = array_out + array_in[i]
    crc16 = libscrc.modbus(array_out)
    s = struct.pack(">H", crc16)
    first, second = struct.unpack(">BB", s)
    # print(int8_to_hex_str(first), int8_to_hex_str(second))
    return first, second


@static_vars(prev_txt="")
def make_followline(data_high, data_low):
    # a = format(int(data_high, 16), "040b")
    # b = format(int(data_low, 16), "040b")
    # # Order of output data is reversed with label paste in sensor CCF
    # txt = a[-8:] + b[-8:]
    # # Order ouput same order pasted in sensor CCF
    # txt = revertString(txt)

    # # if txt != make_followline.prev_txt:
    # #     print(txt)
    # # make_followline.prev_txt = txt

    # followline = FollowLineSensor()
    # bit_revert_list = ""
    # if SENSOR_TYPE == "CCF":
    #     bit_revert_list = list(map(lambda c2: c2, txt))
    #     followline.data = [0 if i == "1" else 1 for i in bit_revert_list]
    #     # TOCHECK: revert array in agv_tape_jp_1. Có thể do cảm biến bị lỗi or lắp ngược, LENET k bị
    #     if iDirection == BACKWARD:
    #         followline.data = followline.data[::-1]
    # elif SENSOR_TYPE == "LENET":
    #     bit_revert_list = list(map(lambda c2: c2, txt))
    #     followline.data = [1 if i == "1" else 0 for i in bit_revert_list]
    # followline.header.stamp = rospy.Time.now()
    # followline_pub.publish(followline)
    a = format(int(data_high.encode("hex"), 16), "040b")
    b = format(int(data_low.encode("hex"), 16), "040b")
    # Order of output data is reversed with label paste in sensor CCF
    txt = a[-8:] + b[-8:]
    # Order ouput same order pasted in sensor CCF
    txt = revertString(txt)

    # if txt != make_followline.prev_txt:
    #     print(txt)
    # make_followline.prev_txt = txt

    followline = FollowLineSensor()
    bit_revert_list = ""
    if SENSOR_TYPE == "CCF":
        bit_revert_list = list(map(lambda c2: c2, txt))
        followline.data = [0 if i == "1" else 1 for i in bit_revert_list]
        # TOCHECK: revert array in agv_tape_jp_1. Có thể do cảm biến bị lỗi or lắp ngược, LENET k bị
        if iDirection == BACKWARD:
            followline.data = followline.data[::-1]
    elif SENSOR_TYPE == "LENET":
        bit_revert_list = list(map(lambda c2: c2, txt))
        followline.data = [1 if i == "1" else 0 for i in bit_revert_list]
    followline.header.stamp = rospy.Time.now()
    followline_pub.publish(followline)


"""


   ####    ##   #      #      #####    ##    ####  #    #
  #    #  #  #  #      #      #    #  #  #  #    # #   #
  #      #    # #      #      #####  #    # #      ####
  #      ###### #      #      #    # ###### #      #  #
  #    # #    # #      #      #    # #    # #    # #   #
   ####  #    # ###### ###### #####  #    #  ####  #    #


"""


def lift_control_cb(msg):
    global last_lift_control_receive
    last_lift_control_receive = rospy.get_time()

def followline_cb(msg):
    global last_followline_sensor_receive
    last_followline_sensor_receive = rospy.get_time()


def motor_speed_cb(msg):
    global iDirection
    iDirection = msg.Direction


def enable_two_followline_cb(msg):
    global enable_two_sensor
    if msg.data == 1:
        enable_two_sensor = True
    else:
        enable_two_sensor = False


def reset_followline_cb(msg):
    global reset_followline
    reset_followline = msg.data


followline_pub = rospy.Publisher(
    "/followline_sensor", FollowLineSensor, queue_size=10
)
followline_value_pub = rospy.Publisher(
    "/followline_value", Float32Stamped, queue_size=10
)
followline_pub_2 = rospy.Publisher(
    "/followline_sensor_2", FollowLineSensor, queue_size=10
)
followline_value_pub_2 = rospy.Publisher(
    "/followline_value_2", Float32Stamped, queue_size=10
)
dashboard_logger_publisher = rospy.Publisher(
    "/dashboard_logger", String, queue_size=10
)


def main():
    global serial_port, rtu_master, reset_followline, last_followline_sensor_receive

    # CheckDbFile('followline_sensor')
    (options, args) = parse_opts()
    use_modbus_tk = options.use_modbus_tk

    rospy.init_node("followline_sensor")
    rospy.logwarn("init node : {}".format(rospy.get_name()))
    rospy.Subscriber("/motor_speed", DiffDriverMotorSpeed, motor_speed_cb)
    rospy.Subscriber("/enable_two_followline", Int8, enable_two_followline_cb)
    rospy.Subscriber("/reset_followline_sensor", Int8, reset_followline_cb)
    rospy.Subscriber("/followline_sensor", FollowLineSensor, followline_cb)
    rospy.Subscriber(
            "/lift_cart", Int8Stamped, lift_control_cb
        )
    port_name = rospy.get_param(
        "~port",
        "2-8.2",
    )
    port_name = find_soft_port(port_name)
    rospy.loginfo("Port name: {}".format(port_name))
    rospy.loginfo("use_modbus_tk: {}".format(use_modbus_tk))
    enable_use_two_followline_sensor = rospy.get_param(
        "~enable_use_two_followline_sensor",
        "2-8.2",
    )

    if use_modbus_tk:
        rtu_master = modbus_rtu.RtuMaster(
            serial.Serial(
                port=port_name,
                baudrate=BAUD_RATE,
                bytesize=8,
                parity="N",
                stopbits=1,
                xonxoff=0,
            )
        )
        rtu_master.set_timeout(5.0)
        rtu_master.set_verbose(True)
    else:
        serial_port = serial.Serial(
            port=port_name,
            baudrate=BAUD_RATE,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )
        print("Serial port: " + str(serial_port.isOpen()))

    PULLING_FREQ = 30
    NOP_TIME = 0.0001
    rate = rospy.Rate(PULLING_FREQ)
    t = rospy.get_time()
    diff_time = 1.0 / PULLING_FREQ
    success_1 = False
    success_2 = False

    while not rospy.is_shutdown():
        if rospy.get_time() - t < diff_time:
            # Must be sleep to prevent high CPU load
            rospy.sleep(NOP_TIME)
            continue
        else:
            # rospy.loginfo("{}".format(round(1.0/(rospy.get_time() - t - NOP_TIME), 2)))
            t = rospy.get_time()
        if reset_followline:
            reset_followline = False
            rospy.logwarn("Reinit modbus by reset topic")
            rtu_master = modbus_rtu.RtuMaster(
                serial.Serial(
                    port=port_name,
                    baudrate=BAUD_RATE,
                    bytesize=8,
                    parity="N",
                    stopbits=1,
                    xonxoff=0,
                )
            )
            rtu_master.set_timeout(5.0)
            rtu_master.set_verbose(True)
        if use_modbus_tk:
            if iDirection == FORWARD:
                modbus_id = FRONT_MODBUS_ID
                modbus_id_2 = REAR_MODBUD_ID
            else:
                modbus_id = REAR_MODBUD_ID
                modbus_id_2 = FRONT_MODBUS_ID
            if not enable_two_sensor or not enable_use_two_followline_sensor:
                success_1 = False
                success_2 = True
            else:
                if success_1 and success_2:
                    success_1 = False
                    success_2 = True
            if rospy.get_time() - last_lift_control_receive < 0.5:
                success_1 = True
                success_2 = True
            if not success_1:
                try:
                    (data_high,) = rtu_master.execute(
                        modbus_id, cst.READ_HOLDING_REGISTERS, MODBUS_ADD, 1
                    )
                    txt = str(bin(data_high)[2:]).zfill(16)
                    txt = revertString(txt)
                    # rospy.loginfo("Data: {}".format(txt))
                    bit_revert_list = list(map(lambda c2: c2, txt))
                    msg = FollowLineSensor()
                    msg.data = [1 if i == "1" else 0 for i in bit_revert_list]
                    sensor_reverse = msg.data[::-1]
                    # print(sensor_reverse)
                    # mylist[::-1]
                    msg.header.stamp = rospy.Time.now()
                    followline_pub.publish(msg)
                    # Convert to int
                    active_count = 0
                    total = 0
                    for i in range(16):
                        total = total + msg.data[i] * i
                        active_count += msg.data[i]
                    if active_count > 0:
                        value = float(total) / float(active_count)
                    else:
                        value = 0.0
                    followline_value_pub.publish(
                        Float32Stamped(stamp=rospy.Time.now(), data=value)
                    )
                    success_1 = True
                    success_2 = False
                except Exception as e:
                    e = str(e)
                    rospy.logerr(e)
                    rospy.loginfo(type(e))
                    if "Response" in e:
                        rospy.loginfo("error normal")
                    if (
                        ("Response" in e)
                        and (
                            rospy.get_time() - last_followline_sensor_receive
                            > 1
                        )
                    ) or not ("Response" in e):
                        last_followline_sensor_receive = rospy.get_time()
                        try:
                            # s = serial.Serial(port=port_name)
                            # s.close()
                            rtu_master.close()
                            rospy.logwarn("closed serial port")
                        except (OSError, serial.SerialException):
                            rospy.logerr("error when close serial port")
                            rospy.loginfo("try to close serial ...")
                            rospy.sleep(0.5)
                        while True:
                            try:
                                rospy.sleep(0.5)
                                rospy.logwarn("reinit modbus")
                                rtu_master = modbus_rtu.RtuMaster(
                                    serial.Serial(
                                        port=port_name,
                                        baudrate=BAUD_RATE,
                                        bytesize=8,
                                        parity="N",
                                        stopbits=1,
                                        xonxoff=0,
                                    )
                                )
                                rtu_master.set_timeout(5.0)
                                rtu_master.set_verbose(True)
                                break
                            except:
                                rospy.logerr("Reinit modbus error")
                    success_1 = False
            elif not success_2:
                try:
                    (data_high,) = rtu_master.execute(
                        modbus_id_2, cst.READ_HOLDING_REGISTERS, MODBUS_ADD, 1
                    )
                    txt = str(bin(data_high)[2:]).zfill(16)
                    txt = revertString(txt)
                    # rospy.loginfo("Data: {}".format(txt))
                    bit_revert_list = list(map(lambda c2: c2, txt))
                    msg = FollowLineSensor()
                    msg.data = [1 if i == "1" else 0 for i in bit_revert_list]
                    sensor_reverse = msg.data[::-1]
                    msg.data = msg.data[::-1]
                    # print(sensor_reverse)
                    # mylist[::-1]
                    msg.header.stamp = rospy.Time.now()
                    followline_pub_2.publish(msg)
                    # Convert to int
                    active_count = 0
                    total = 0
                    for i in range(16):
                        total = total + msg.data[i] * i
                        active_count += msg.data[i]
                    if active_count > 0:
                        value = float(total) / float(active_count)
                    else:
                        value = 0.0
                    followline_value_pub_2.publish(
                        Float32Stamped(stamp=rospy.Time.now(), data=value)
                    )
                    success_1 = False
                    success_2 = True
                except Exception as e:
                    e = str(e)
                    rospy.logerr(e)
                    rospy.loginfo(type(e))
                    if "Response" in e:
                        rospy.loginfo("error normal")
                    if (
                        ("Response" in e)
                        and (
                            rospy.get_time() - last_followline_sensor_receive
                            > 1
                        )
                    ) or not ("Response" in e):
                        last_followline_sensor_receive = rospy.get_time()
                        try:
                            # s = serial.Serial(port=port_name)
                            # s.close()
                            rtu_master.close()
                            rospy.logwarn("closed serial port")
                        except (OSError, serial.SerialException):
                            rospy.logerr("error when close serial port")
                            rospy.loginfo("try to close serial ...")
                            rospy.sleep(0.5)
                        while True:
                            try:
                                rospy.sleep(0.5)
                                rospy.logwarn("reinit modbus")
                                rtu_master = modbus_rtu.RtuMaster(
                                    serial.Serial(
                                        port=port_name,
                                        baudrate=BAUD_RATE,
                                        bytesize=8,
                                        parity="N",
                                        stopbits=1,
                                        xonxoff=0,
                                    )
                                )
                                rtu_master.set_timeout(5.0)
                                rtu_master.set_verbose(True)
                                break
                            except:
                                rospy.logerr("Reinit modbus error")
                    success_2 = False
        else:
            Sensor_Receiver()
        # rate.sleep()


def parse_opts():
    from optparse import OptionParser

    parser = OptionParser()
    parser.add_option(
        "-m",
        "--use_modbus_tk",
        action="store_true",
        dest="use_modbus_tk",
        default=False,
        help='type "-m" if use_modbus_tk',
    )

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)


if __name__ == "__main__":
    main()
