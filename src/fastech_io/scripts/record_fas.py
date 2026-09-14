#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
import json
import sys
import rospy
import rospkg
import yaml
import time
import re
import csv
import datetime
import pandas as pd
from agv_msgs.msg import *
from shutil import copyfile, copy
from std_msgs.msg import Float64, String, Int16, Bool
from std_stamped_msgs.msg import StringStamped, Float32Stamped, EmptyStamped


from std_msgs.msg import Float32MultiArray, Int16MultiArray
from fastech_io.msg import io
from fastech_io.srv import GetIO, SetValueOutput

from std_stamped_msgs.msg import (
    StringStamped,
    StringAction,
    StringFeedback,
    StringResult,
    StringGoal,
    EmptyStamped,
)


class LOGGING_:
    def __init__(self, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        rospy.init_node("LOGGING_and_test_EZ")
        rospy.loginfo("Init node " + rospy.get_name())

        self.get_io_rate = rospy.get_param("~get_io_rate", "fastech_output")
        self.topic_record = rospy.get_param("~name_topic", "fastech_output")
        self.frequency_device = rospy.get_param("~frequency", "fastech_output")

        self.autolog_header = [
            "TIME_PROBLEM",
            "REAL_FREQUENCY",
        ]
        self.data_log = [None] * len(self.autolog_header)

        self.total_ = 0

        self.Start_ = True
        self.Stop_ = False
        self.Pause_ = False

        rospy.Subscriber(
            self.topic_record,
            Int16MultiArray,
            self.topic_record_,
        )
        rospy.Subscriber(
            "/control_logging",
            String,
            self.control_log_cb,
        )
        self.time_device_pre = rospy.get_time()
        self.time_day = datetime.datetime.now().strftime("%Y-%m-%d")
        self.end_time = datetime.datetime.now().strftime("%H:%M")
        self.path = self.config_path + "/{}.csv".format(self.time_day)
        # self.copy_fileName = self.results_path + "/{}.csv".format(
        #     self.time_day + "_" + self.end_time
        # )
        self.time_start = datetime.datetime.now().strftime(
            "%Y-%m-%d %H:%M:%S.%f"
        )

        self.time_end = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")

        self.down_time = datetime.datetime.now().strftime(
            "%Y-%m-%d %H:%M:%S.%f"
        )

        self.up_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        self.path = self.config_path + "/{}.csv".format(self.time_day)

        self.loop()

    def init_variable(self, *args, **kwargs):
        self.config_path = kwargs["config_path"]
        self.config_ez_io = kwargs["config_path"] + "/ethernet_io_1.yaml"
        # self.load_default_params(path=kwargs["planner_setting"])

    def control_log_cb(self, msg):

        _logging = str.upper(msg.data)
        if _logging == "STOP":
            self.Stop_ = True
        elif _logging == "PAUSE":
            self.Pause_ = True
        elif _logging == "START":
            self.Start_ = False

    def update_recordFile(self):

        self.df = pd.read_csv(self.path)
        for self.index, row in self.df.iterrows():
            self.total_ = int(self.index)
        self.N_count_number = self.total_ - 1

    def record_data(self):
        try:
            with open(self.path, "a") as self.record_file:
                self.now = datetime.datetime.now().strftime(
                    "%Y-%m-%d %H:%M:%S.%f"
                )
                self.data_log[0] = self.now
                writer = csv.writer(self.record_file)
                writer.writerow(self.data_log)
        except FileNotFoundError:
            pass

        if self.Stop_:
            rospy.loginfo(" STOP logging file ")
            self.record_file.close()

    def calc_diff(self, time_start, time_end):

        # self.first_charging = datetime.datetime.now().strftime(
        #     "%Y-%m-%d %H:%M:%S.%f"
        # )
        d_start = datetime.datetime.strptime(time_start, "%Y-%m-%d %H:%M:%S.%f")
        d_end = datetime.datetime.strptime(time_end, "%Y-%m-%d %H:%M:%S.%f")
        return d_end - d_start

    def log_signal(self, msg):
        self.data_log[1] = 1 / msg
        self.record_data()

    def topic_record_(self, msg):
        self.time_device_pre = rospy.get_time()

    def loop(self):

        rate = rospy.Rate(self.get_io_rate)
        if self.Start_ and not self.Pause_:
            with open(self.path, "a") as file:
                writer = csv.writer(file)
                writer.writerow(self.autolog_header)
            with open(self.path, "r") as csvfile:
                csvreader = csv.reader(csvfile)
                for row in csvreader:
                    self.update_recordFile()
                    if not row[0]:
                        rospy.INFO(
                            "log file day {} empty".format(self.time_day)
                        )

        while not rospy.is_shutdown():

            if self.Stop_:
                # copy(self.path, self.copy_fileName)
                rospy.signal_shutdown("shut down logging")

            if rospy.get_time() - self.time_device_pre >= (
                1.0 / (self.frequency_device)
            ):
                self.log_signal(rospy.get_time() - self.time_device_pre)
            self.time_day = datetime.datetime.now().strftime("%Y-%m-%d")
            self.path = self.config_path + "/{}.csv".format(self.time_day)
            self.time_end = datetime.datetime.now().strftime("%H:%M")

            self.time_now = datetime.datetime.now().strftime(
                "%Y-%m-%d %H:%M:%S.%f"
            )
            rate.sleep()

        self.time_end = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        total_time = self.calc_diff(self.time_start, self.time_end)
        rospy.logwarn("total time record {}".format(total_time))


def parse_opts():
    from optparse import OptionParser

    parser = OptionParser()
    parser.add_option(
        "-d",
        "--ros_debug",
        action="store_true",
        dest="log_debug",
        default=False,
        help="log_level=rospy.DEBUG",
    )
    parser.add_option(
        "-p",
        "--config_path",
        dest="config_path",
        default=os.path.join(
            rospkg.RosPack().get_path("fastech_io"),
            "record",
        ),
    )

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)


def main():
    (options, args) = parse_opts()
    log_level = None
    if options.log_debug:
        log_level = rospy.DEBUG
    LOGGING_(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
