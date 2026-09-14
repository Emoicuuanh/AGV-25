#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
from re import A
import os
import sys
import rospy
import actionlib
import numpy as np
import pandas as pd
import json
import rospy
import rospkg
from safety_msgs.msg import SafetyStatus
from std_msgs.msg import Time, Int16, Int32
from std_stamped_msgs.msg import StringStamped
from vl53l5cx.msg import Vl53l5cxRanges, Safety_Esp, vl53l5cx_safety
from safety_msgs.msg import SafetyStatus
import message_filters
import numpy as np
import getch


class Save_default_lidarvl53l5:
    def __init__(self, *args, **kwargs):
        self.init_variable(*args, **kwargs)

        rospy.init_node("default_value_sensor", anonymous=True)
        rospy.loginfo("Init node " + rospy.get_name())
        # Subscriber
        rospy.Subscriber("/vl35l5cx_r1", Vl53l5cxRanges, self.range_senser_1_cb)
        rospy.Subscriber("/vl35l5cx_r2", Vl53l5cxRanges, self.range_senser_2_cb)
        rospy.Subscriber("/vl35l5cx_r3", Vl53l5cxRanges, self.range_senser_3_cb)
        rospy.Subscriber("/vl35l5cx_r4", Vl53l5cxRanges, self.range_senser_4_cb)

        self.centimetros_to_metros = [1000] * 64
        self.error = [30] * 64
        self.ranger_sensor1 = None
        self.ranger_sensor2 = None
        self.ranger_sensor3 = None
        self.ranger_sensor4 = None
        self.data_sensor_1 = None
        self.data_sensor_2 = None
        self.data_sensor_3 = None
        self.data_sensor_4 = None
        self.shutdow_ros = False
        # self.update_json()
        # Publisher
        self.sensor_1_calib = rospy.Publisher(
            "/status_vl53l5_sensor1", StringStamped, queue_size=5
        )
        self.sensor_2_calib = rospy.Publisher(
            "/status_vl53l5_sensor2", StringStamped, queue_size=5
        )
        self.sensor_3_calib = rospy.Publisher(
            "/status_vl53l5_sensor3", StringStamped, queue_size=5
        )
        self.sensor_4_calib = rospy.Publisher(
            "/status_vl53l5_sensor4", StringStamped, queue_size=5
        )
        self.loop()

    def init_variable(self, *args, **kwargs):
        self.config_path = kwargs["config_path"] + "/default_range.json"
        self.min_path = kwargs["config_path"] + "/min_noise.json"
        self.minimum_path = kwargs["config_path"] + "/minimum.json"

    def range_senser_1_cb(self, msg):
        self.ranger_sensor1 = msg.range
        self.analysis_array_vl53l5(self.ranger_sensor1, n_sensor="senser_1")

    def range_senser_2_cb(self, msg):
        self.ranger_sensor2 = msg.range
        self.analysis_array_vl53l5(self.ranger_sensor2, n_sensor="senser_2")

    def range_senser_3_cb(self, msg):
        self.ranger_sensor3 = msg.range
        # self.analysis_array_vl53l5(self.ranger_sensor3, n_sensor="senser_3")

    def range_senser_4_cb(self, msg):
        self.ranger_sensor4 = msg.range
        self.analysis_array_vl53l5(self.ranger_sensor4, n_sensor="senser_4")

    def save_default(self, data, n_sensor="n_sensor"):

        # for i in data[:10]:
        #     print(i)
        rospy.loginfo(" SAVE {}".format(n_sensor))

        if n_sensor == "sensor 1":
            print(data[:10])
            self.data_sensor_1 = data
        elif n_sensor == "sensor 2":
            self.data_sensor_2 = data
        elif n_sensor == "sensor 3":
            self.data_sensor_3 = data
        elif n_sensor == "sensor 4":
            self.data_sensor_4 = data

        dictionary = {
            "lidarvl53l5_sensor1": self.data_sensor_1,
            "lidarvl53l5_sensor2": self.data_sensor_2,
            "lidarvl53l5_sensor3": self.data_sensor_3,
            "lidarvl53l5_sensor4": self.data_sensor_4,
        }
        json_object = json.dumps(dictionary, indent=4)
        with open(self.config_path, "w") as outfile:
            outfile.write(json_object)

    def analysis_array_vl53l5(self, array, n_sensor="n_sensor"):

        count_min = 0
        min_value = 0
        max_value = 0
        lenght = 8
        min_angle = array[:8]
        for i in array:
            if i < min_value:
                min_value = i

        for i in min_angle:
            count_min += i
        angle_min = count_min / lenght
        # print(angle_min)

        msg_dict = {
            "n_sensor": n_sensor,
            "angle_min": angle_min,
            "angle_max": min_angle,
            "mid_array": 15,
            "max_array": 400,
            "lenght ": lenght,
            "count_min": count_min,
            "OK or NG ": "OK",
        }
        monitor_vl53l5 = StringStamped()
        monitor_vl53l5.stamp = rospy.Time.now()
        monitor_vl53l5.data = json.dumps(msg_dict, indent=2)

        if n_sensor == "senser_1":
            self.sensor_1_calib.publish(monitor_vl53l5)
        elif n_sensor == "senser_2":
            self.sensor_2_calib.publish(monitor_vl53l5)
        elif n_sensor == "senser_3":
            self.sensor_3_calib.publish(monitor_vl53l5)
        elif n_sensor == "senser_4":
            self.sensor_4_calib.publish(monitor_vl53l5)
        else:
            pass

    def input_keyboard(self):

        keyboard_value = ord(getch.getch())
        if keyboard_value == 49:
            pass
            # self.save_default(self.ranger_sensor1, n_sensor="sensor 1")
        elif keyboard_value == 50:
            self.save_default(self.ranger_sensor2, n_sensor="sensor 2")
        elif keyboard_value == 51:
            self.save_default(self.ranger_sensor3, n_sensor="sensor 3")
        elif keyboard_value == 52:
            self.save_default(self.ranger_sensor4, n_sensor="sensor 4")
        elif keyboard_value == 53:
            self.update_json()
            print("open json")
        return keyboard_value

    def update_json(self):

        """Open Json file."""
        with open(self.min_path, "r") as min_default:
            param_min_safety = json.load(min_default)
        self.min_default_safety = pd.DataFrame(param_min_safety)


        for sensor, param in self.min_default_safety.items():
            if sensor == "lidarvl53l5_sensor1":
                min_data1 = np.array(param)
            if sensor == "lidarvl53l5_sensor2":
                min_data2 = np.array(param)
            if sensor == "lidarvl53l5_sensor3":
                min_data3 = np.array(param)
            if sensor == "lidarvl53l5_sensor4":
                min_data4 = np.array(param)

        with open(self.minimum_path, "r") as minimum_value_default:
            param_minimum_safety = json.load(minimum_value_default)
        self.minimum_default_safety = pd.DataFrame(param_minimum_safety)

        for sensor, param in self.minimum_default_safety.items():
            if sensor == "lidarvl53l5_sensor1":
                minimum_data1 = np.array(param)
            if sensor == "lidarvl53l5_sensor2":
                minimum_data2 = np.array(param)
            if sensor == "lidarvl53l5_sensor3":
                minimum_data3 = np.array(param)
            if sensor == "lidarvl53l5_sensor4":
                minimum_data4 = np.array(param)

        ss_1 = list(min_data1 + minimum_data1 )
        ss_2 = list(min_data2 + minimum_data2 )
        ss_3 = list(min_data3)
        ss_4 = list(min_data4)

        default_dictionary = {
            "lidarvl53l5_sensor1": ss_1,
            "lidarvl53l5_sensor2": ss_2,
            "lidarvl53l5_sensor3": ss_3,
            "lidarvl53l5_sensor4": ss_4,
        }

        json_object = json.dumps(default_dictionary, indent=4)
        with open(self.min_path, "w") as outfile:
            outfile.write(json_object)

        print("update success")

    def loop(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.input_keyboard()
            if self.shutdow_ros:
                rospy.on_shutdown(self.loop)
            rate.sleep()

    def shutdownhook(self):
        print(self.input_keyboard())
        self.shutdow_ros = False


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
            rospkg.RosPack().get_path("vl53l5cx"),
            "cfg",
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
    Save_default_lidarvl53l5(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
