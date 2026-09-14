#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry  # log odometry
from geometry_msgs.msg import Twist, Pose  # log cmd_vel
import datetime
import rospkg
from nav_msgs.msg import Path
from std_msgs.msg import Float64
import sys, os
from shutil import copyfile, copy
from sensor_msgs.msg import BatteryState
from datetime import *
import json
import re
from std_stamped_msgs.msg import (
    StringStamped,
    Int8Stamped,
    StringFeedback,
    StringResult,
    StringAction,
    StringActionGoal,
    EmptyStamped,
    Float32Stamped,
)

pkg_path = rospkg.RosPack().get_path("read_battery")
results_path = os.path.join(pkg_path, "results")


class LogTest:
    def __init__(self):
        rospy.init_node("logging")
        self.remove_old_log()
        timenow = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        rospy.loginfo("Init node logging. Log to {}_*.csv".format(timenow))
        # fileName = os.path.join(results_path, str(timenow) + ".csv")
        self.cmdFileName = os.path.join(results_path, timenow + "_battery_log.csv")
        # self.lastestLogFileName = os.path.join(
        #     results_path, "lastest_battery_log.csv"
        # )
        self.lastestLogFile = open(self.cmdFileName, "w+")
        self.vol = None
        self.current = None
        self.capacity_remain = None
        self.percentage = None
        self.temperature = None
        self.baterry_state = None
        self.start_log = True
        self.stop_log = False
        self.timeout = 2
        self.lastCmd = datetime.now()
        self.lastWrite = datetime.now()
        self.first_time = True
        self.emg_button = None

        rospy.Subscriber(
            "/arduino_driver/float_param/battery_voltage",
            Float32Stamped,
            self.voltageCB,
        )
        rospy.Subscriber(
            "/arduino_driver/float_param/battery_ampe", Float32Stamped, self.ampeCB
        )
        rospy.Subscriber(
            "/arduino_driver/float_param/battery_percent",
            Float32Stamped,
            self.percentCB,
        )
        rospy.Subscriber("/standard_io", StringStamped, self.standard_io_cb)

        self.lastestLogFile.write(
            "time_start: {}\ntime,vol,current,percent\n".format(
                datetime.now().strftime("%Y-%m-%d-%H:%M:%S.%f")
            )
        )
        while True:
            if (
                self.vol is not None
                and self.current is not None
                and self.percentage is not None
            ):
                break
        while not rospy.is_shutdown():
            rospy.sleep(3)  # rate 10Hz
            if self.start_log and not self.stop_log:
                rospy.logwarn_once("write data")
                self.lastestLogFile.write(
                    "{},{},{},{}\n".format(
                        datetime.now().strftime("%Y-%m-%d-%H:%M:%S.%f"),
                        round(self.vol, 2),
                        round(self.current, 2),
                        round(self.percentage, 2),
                    )
                )
            if (datetime.now() - self.lastCmd).seconds > 10:
                self.stop_log = True

            if self.stop_log:
                self.lastestLogFile.write(
                    "time end: {}".format(
                        datetime.now().strftime("%Y-%m-%d-%H:%M:%S.%f")
                    )
                )
                self.lastestLogFile.close()
                rospy.loginfo("Close log file!")
                # copy(self.cmdFileName, self.lastestLogFileName)
                # os.system(pkg_path + "/scripts/plot_battery.py")
                rospy.signal_shutdown("shut down node")

    def voltageCB(self, msg):
        self.lastCmd = datetime.now()
        self.vol = msg.data
        # self.current = msg.current
        # self.capacity_remain = msg.capacity
        # self.percentage = msg.percentage
        # self.temperature = msg.temperature
        # self.baterry_state = msg.power_supply_status

    def ampeCB(self, msg):
        self.current = msg.data

    def percentCB(self, msg):
        self.percentage = msg.data

    def standard_io_cb(self, msg):
        self.std_io_status = json.loads(msg.data)
        if "emg_button" in self.std_io_status:
            self.emg_button = self.std_io_status["emg_button"]

    def remove_old_log(self):
        current_date_time = datetime.now()
        file_list = os.listdir(results_path)
        for filename in file_list:
            date_time_pattern = r"(\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2})"
            match = re.search(date_time_pattern, filename)

            if match:
                date_time_str = match.group(1)
                # Chuyển đổi chuỗi thành đối tượng datetime
                file_date_time = datetime.strptime(date_time_str, "%Y-%m-%d-%H-%M-%S")
                one_month_ago = current_date_time - timedelta(days=30)
                if file_date_time < one_month_ago:
                    os.remove(results_path + "/" + filename)
                    rospy.loginfo("remove " + filename + " log baterry")
            else:
                rospy.loginfo("No date and time found in : " + filename)

if __name__ == "__main__":
    log = LogTest()
