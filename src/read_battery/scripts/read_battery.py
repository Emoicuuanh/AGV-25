#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from psutil import sensors_battery
import rospy
import serial
import os
import rospkg
import sys
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Header, String

common_folder = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_folder):
    common_folder = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_folder)
from common_function import find_soft_port


class ReadBattery:
    def __init__(self):
        rospy.init_node("read_battery")
        rospy.loginfo("Init node read_battery")
        self.init_varialble()
        self.init_ros()
        self.init_serial()
        self.poll()

    def init_varialble(self):
        self.msg = BatteryState()
        self.port_battery = rospy.get_param("~port_battery", "1-1")
        self.port_battery = find_soft_port(self.port_battery)
        self.baud = rospy.get_param("~baud", 9600)
        self.cmd_battery_info = b"\xDD\xA5\x03\x00\xFF\xFD\x77"
        self.cmd_cell_info = b"\xDD\xA5\x04\x00\xFF\xFC\x77"
        self.vol = 0
        self.current = 0
        self.capacity_remain = 0
        self.percentage = 0
        self.temperature = 0
        self.baterry_state = None
        self.rate = rospy.Rate(20)

    def init_serial(self):
        try:
            self.serial_port = serial.Serial(
                port=self.port_battery,
                baudrate=self.baud,
                parity="N",
                stopbits=1,
                bytesize=8,
                timeout=0.1,
            )
            if self.serial_port.isOpen():
                rospy.loginfo(
                    "Connected to port: {}".format(self.serial_port.portstr)
                )
        except:
            rospy.logerr("Serial connect fail")

    def init_ros(self):
        self.battery_status = rospy.Publisher(
            "battery_status", BatteryState, queue_size=1
        )
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.serial_port.close()
        print("ros shutdown in {}".format(rospy.get_name()))

    def check_sum(self, s):
        sum = int()
        for c in s:
            sum += c
        sum = -(sum % 65536)
        # print((sum & 0xFFFF).to_bytes(2, byteorder="big"))
        return (sum & 0xFFFF).to_bytes(2, byteorder="big")

    def hexstr_to_hex_byte(self, hex_string):
        return bytes.fromhex(hex_string)

    def read_battery(self):
        # BATTERY INFO:
        # VOL: (BYTE 4-5)
        # CURRENT: (BYTE 6-7)
        # CAPACITY REMAIN: (BYTE 8-9)
        # PERCENTAGE: (BYTE 23)
        # MODE: (BYTE 24)
        # input_data = self.serial_port.read_until("\r")
        input_data = self.serial_port.read(34)
        if len(input_data) != 34:
            return
        else:
            if self.check_sum(input_data[2:31]) != input_data[-3:-1]:
                rospy.logwarn(input_data)
                rospy.logwarn(
                    "check_sum  {} != {}".format(
                        self.check_sum(input_data[2:31]), input_data[-3:1]
                    )
                )
                return
        if (
            int.from_bytes(
                input_data[6:8],
                "big",
            )
            == 0
        ):
            print("fail")
            return
        self.current = int.from_bytes(
            input_data[6:8],
            "big",
        )
        mode = "{:016b}".format(self.current, 16)
        if mode[0] == "0":
            self.baterry_state = self.msg.POWER_SUPPLY_STATUS_CHARGING
            self.current = self.current / 100
        else:
            self.current = (65536 - self.current) / 100
            self.baterry_state = self.msg.POWER_SUPPLY_STATUS_DISCHARGING
        self.vol = int.from_bytes(input_data[4:6], "big")
        self.vol = self.vol / 100
        self.capacity_remain = int.from_bytes(input_data[8:10], "big")
        self.capacity_remain = self.capacity_remain / 100
        self.percentage = int.from_bytes(input_data[23:24], "big")
        temp_1 = (int.from_bytes(input_data[27:29], "big") - 2731) / 10
        temp_2 = (int.from_bytes(input_data[29:31], "big") - 2731) / 10
        self.temperature = (temp_1 + temp_2) / 2

        # print("vol: {} (V)".format(self.vol))
        # print("current: {} (A)".format(self.current))
        # print("capacity_remain: {} (Ah)".format(self.capacity_remain))
        # print("percentage: {} (%)".format(self.percentage))
        # print("status: {}".format(self.baterry_state))
        # print("temperature: {} (C)".format(self.temperature))
        self.msg.capacity = self.capacity_remain
        self.msg.design_capacity = 60
        self.msg.percentage = self.percentage
        self.msg.temperature = self.temperature
        self.msg.current = self.current
        self.msg.voltage = self.vol
        self.msg.power_supply_status = self.baterry_state
        self.msg.header.stamp = rospy.Time.now()

        self.msg.power_supply_technology = self.msg.POWER_SUPPLY_TECHNOLOGY_LIPO

    def poll(self):
        self.pre_time = rospy.get_time()
        while not rospy.is_shutdown():
            self.serial_port.write(self.cmd_battery_info)
            self.read_battery()
            self.battery_status.publish(self.msg)
            self.rate.sleep()


def main():
    Battery = ReadBattery()


if __name__ == "__main__":
    main()
