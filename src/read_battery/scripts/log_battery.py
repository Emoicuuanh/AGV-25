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

pkg_path = rospkg.RosPack().get_path("agv_tape_config")
results_path = os.path.join(pkg_path, "results")


class LogTest:
    def __init__(self):
        rospy.init_node("logging")
        timenow = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        rospy.loginfo("Init node logging. Log to {}_*.txt".format(timenow))
        # fileName = os.path.join(results_path, str(timenow) + ".txt")
        self.cmdFileName = os.path.join(
            results_path, timenow + "_battery_log.txt"
        )
        self.lastestLogFileName = os.path.join(
            results_path, "lastest_battery_log.txt"
        )
        self.lastestLogFile = open(self.lastestLogFileName, "w+")
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

        rospy.Subscriber("/battery_status", BatteryState, self.batteryCB)

        self.lastestLogFile.write(
            "time_start: {}\ntime,vol,current,capacity_remain,percentage,temperature,baterry_state\n".format(
                datetime.now().strftime("%Y-%m-%d-%H:%M:%S.%f")
            )
        )
        while True:
            if (
                self.vol is not None
                and self.current is not None
                and self.capacity_remain is not None
                and self.percentage is not None
                and self.temperature is not None
                and self.baterry_state is not None
            ):
                break
        while not rospy.is_shutdown():
            rospy.sleep(0.1)  # rate 10Hz
            if self.start_log and not self.stop_log:
                rospy.logwarn("write data")
                self.lastestLogFile.write(
                    "{},{},{},{},{},{},{}\n".format(
                        datetime.now().strftime("%Y-%m-%d-%H:%M:%S.%f"),
                        round(self.vol, 2),
                        round(self.current, 2),
                        round(self.capacity_remain, 2),
                        int(self.percentage),
                        round(self.temperature, 2),
                        int(self.baterry_state),
                    )
                )
            if (datetime.now() - self.lastCmd).seconds > 5:
                self.stop_log = True

            if self.stop_log:
                self.lastestLogFile.write(
                    "time end: {}".format(
                        datetime.now().strftime("%Y-%m-%d-%H:%M:%S.%f")
                    )
                )
                self.lastestLogFile.close()
                rospy.loginfo("Close log file!")
                copy(self.lastestLogFileName, self.cmdFileName)
                # os.system(pkg_path + "/scripts/plot_battery.py")
                rospy.signal_shutdown("shut down node")

    def batteryCB(self, msg):
        self.lastCmd = datetime.now()
        self.vol = msg.voltage
        self.current = msg.current
        self.capacity_remain = msg.capacity
        self.percentage = msg.percentage
        self.temperature = msg.temperature
        self.baterry_state = msg.power_supply_status


if __name__ == "__main__":
    log = LogTest()
