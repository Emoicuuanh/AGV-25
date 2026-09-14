#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry  # log odometry
from geometry_msgs.msg import Twist, Pose  # log cmd_vel
import datetime
import rospkg
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Float32, Empty
import sys, os
from shutil import copyfile, copy
from sensor_msgs.msg import BatteryState
from datetime import *
from agv_msgs.msg import *

pkg_path = rospkg.RosPack().get_path("agv_tape_config")
results_path = os.path.join(pkg_path, "results")


class LogTest:
    def __init__(self):
        rospy.init_node("logging_data")
        timenow = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        rospy.loginfo("Init node logging. Log to {}_*.csv".format(timenow))
        self.cmdFileName = os.path.join(results_path, timenow + "_data_log.csv")
        self.lastestLogFileName = os.path.join(
            results_path, "lastest_data_log.csv"
        )
        self.lastestLogFile = open(self.lastestLogFileName, "w+")
        self.vel_fb_motor_left = None
        self.vel_fb_motor_right = None
        self.vel_fb_odom = None
        self.vel_control = None
        self.odom_x = None
        self.odom_y = None
        self.vol = None
        self.current = None
        self.capacity_remain = None
        self.percentage = None
        self.temperature = None
        self.baterry_state = None
        self.start_log = True
        self.stop_log = False
        self.time_write = 0.1
        self.write = False
        self.timeout = 2
        self.id = 0
        self.lastCmd = datetime.now()
        self.lastWrite = datetime.now()
        self.get_new_card = False
        self.first_time = True
        self.reset_odom = rospy.Publisher("/reset_odom", Empty, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odomCB)
        rospy.Subscriber("/vel_left_feedback", Float32, self.velLeftCB)
        rospy.Subscriber("/vel_right_feedback", Float32, self.velRightCB)
        rospy.Subscriber("/final_cmd_vel_mux/output", Twist, self.cmd_vel_cb)
        rospy.Subscriber("/battery_status", BatteryState, self.batteryCB)
        # rospy.Subscriber("/card_id", CardID, self.carIdCB)
        self.lastestLogFile.write(
            "time start: {}\ntime,vel_control,vel_feedback_motor_left,vel_feedback_motor_right,vel_feedback_odom,odom_x,odom_y,vol,current,capacity_remain,percentage,temperature,baterry_state\n".format(
                datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
            )
        )
        while True:
            if (
                self.vel_fb_motor_left is not None
                and self.vel_fb_motor_right is not None
                and self.vel_fb_odom is not None
                and self.vel_control is not None
                and self.odom_x is not None
                and self.odom_y is not None
                and self.vol is not None
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
                # if (
                #     self.vel_fb_motor_left > 0.001
                #     and self.vel_fb_motor_right > 0.001
                # ):
                rospy.logwarn("write data")
                self.lastestLogFile.write(
                    "{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
                        datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"),
                        round(self.vel_control, 2),
                        round(self.vel_fb_motor_left, 2),
                        round(self.vel_fb_motor_right, 2),
                        round(self.vel_fb_odom, 2),
                        round(self.odom_x, 2),
                        round(self.odom_y, 2),
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
                        datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                    )
                )
                self.lastestLogFile.close()
                rospy.loginfo("Close log file!")
                copy(self.lastestLogFileName, self.cmdFileName)
                # os.system(pkg_path + "/scripts/plot_battery.py")
                rospy.signal_shutdown("shut down node")

    def velRightCB(self, msg):
        self.vel_fb_motor_right = msg.data

    def velLeftCB(self, msg):
        self.vel_fb_motor_left = msg.data

    def odomCB(self, msg):
        self.lastCmd = datetime.now()
        self.vel_fb_odom = msg.twist.twist.linear.x
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    def cmd_vel_cb(self, msg):
        self.vel_control = msg.linear.x

    def batteryCB(self, msg):
        # self.lastCmd = datetime.now()
        self.vol = msg.voltage
        self.current = msg.current
        self.capacity_remain = msg.capacity
        self.percentage = msg.percentage
        self.temperature = msg.temperature
        self.baterry_state = msg.power_supply_status

    # def carIdCB(self, msg):

    #     self.id += 1
    #     self.get_new_card = True


if __name__ == "__main__":
    log = LogTest()
