#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
from re import A
import os
import sys
import math
import json
import rospy
import rospkg
import numpy as np
import pandas as pd
from safety_msgs.msg import SafetyStatus
from std_msgs.msg import Time, Int16, Int32, String
from std_stamped_msgs.msg import StringStamped
from vl53l5cx.msg import Vl53l5cxRanges, Safety_Esp, vl53l5cx_safety
from safety_msgs.msg import SafetyStatus
import message_filters
import numpy as np
import getch


class calculate_noise:
    def __init__(self, *args, **kwargs):
        self.init_variable(*args, **kwargs)

        rospy.init_node("config_paramer_lidarvl3l5cx", anonymous=True)
        rospy.loginfo("Init node " + rospy.get_name())
        self.lenth_range = [0] * 64
        self.sensor_1 = [0] * 64
        self.sensor_2 = [0] * 64
        self.sensor_3 = [0] * 64
        self.sensor_4 = [0] * 64
        # Subscriber
        rospy.Subscriber("/vl35l5cx_r1", Vl53l5cxRanges, self.range_senser_1_cb)
        rospy.Subscriber("/vl35l5cx_r2", Vl53l5cxRanges, self.range_senser_2_cb)
        rospy.Subscriber("/vl35l5cx_r3", Vl53l5cxRanges, self.range_senser_3_cb)
        rospy.Subscriber("/vl35l5cx_r4", Vl53l5cxRanges, self.range_senser_4_cb)
        self.loop()

    def init_variable(self, *args, **kwargs):
        default_path = kwargs["config_path"]
        self.config_path = default_path + "/" + "default_range.json"
        self.after_noise = default_path + "/" + "noise_removal.json"
        self.max_noise = default_path + "/" + "max_noise.json"
        self.min_noise = default_path + "/" + "min_noise.json"

    def default_parameter(self):
        pass

        # self.open_json()
        # self.identification_noise()

    def range_senser_1_cb(self, msg):
        self.sensor_1 = np.array(msg.range)
        # lenght_data = len(self.sensor_1)
        # for i in range(0 , lenght_data):
        #     print(self.sensor_1[i])

    def range_senser_2_cb(self, msg):
        self.sensor_2 = np.array(msg.range)

    def range_senser_3_cb(self, msg):
        self.sensor_3 = np.array(msg.range)

    def range_senser_4_cb(self, msg):
        self.sensor_4 = msg.range

    def identification_noise(self):
        self.open_json()
        data_sensor_1 = []
        data_sensor_2 = []
        data_sensor_3 = []
        data_sensor_4 = []
        noise_sensor = []
        current_data_sensor1 = self.sensor_1 + self.default_sensor_1
        current_data_sensor2 = self.sensor_2 + self.default_sensor_2
        current_data_sensor3 = self.sensor_3 + self.default_sensor_3
        current_data_sensor4 = self.sensor_4 + self.default_sensor_4
        array_noise = []

        test_current_data1 = []
        test_current_data2 = []
        test_current_data3 = []
        test_current_data4 = []

        lenght = len(self.lenth_range)

        for i in current_data_sensor1:
            data_sensor_1.append(i / 2)

        for i in current_data_sensor2:
            data_sensor_2.append(i / 2)

        for i in current_data_sensor3:
            data_sensor_3.append(i / 2)

        for i in current_data_sensor4:
            data_sensor_4.append(i / 2)

        for i in range(0, lenght):
            if self.default_sensor_1[i] - self.sensor_1[i] < 0:
                pass
                # print(i , self.default_sensor_1[i] - self.sensor_1[i] )
            test_current_data1.append(
                self.default_sensor_1[i] - self.sensor_1[i]
            )

        for i in range(0, lenght):
            if self.default_sensor_2[i] - self.sensor_2[i] < 0:
                pass
                # print(i , self.default_sensor_1[i] - self.sensor_1[i] )
            test_current_data2.append(
                self.default_sensor_2[i] - self.sensor_2[i]
            )

        for i in range(0, lenght):
            if self.default_sensor_3[i] - self.sensor_3[i] < 0:
                pass
                # print(i , self.default_sensor_1[i] - self.sensor_1[i] )
            test_current_data3.append(
                self.default_sensor_3[i] - self.sensor_3[i]
            )

        for i in range(0, lenght):
            if self.default_sensor_4[i] - self.sensor_4[i] < 0:
                pass
                # print(i , self.default_sensor_1[i] - self.sensor_1[i] )
            test_current_data4.append(
                self.default_sensor_4[i] - self.sensor_4[i]
            )

        ########################################################################

        remove_noise_sensor1 = list(
            np.array(test_current_data1) + np.array(data_sensor_1)
        )
        remove_noise_sensor2 = list(
            np.array(test_current_data2) + np.array(data_sensor_2)
        )
        remove_noise_sensor3 = list(
            np.array(test_current_data3) + np.array(data_sensor_3)
        )
        remove_noise_sensor4 = list(
            np.array(test_current_data4) + np.array(data_sensor_4)
        )
        ########################################################################

        max_sensor1 = []
        max_sensor2 = []
        max_sensor3 = []
        max_sensor4 = []
        for i in range(0, lenght):
            if self.max_noise_data_1[i] > self.sensor_1[i]:
                max_sensor1.append(self.max_noise_data_1[i])
            else:
                max_sensor1.append(self.sensor_1[i])

        for i in range(0, lenght):
            if self.max_noise_data_2[i] > self.sensor_2[i]:
                max_sensor2.append(self.max_noise_data_2[i])
            else:
                max_sensor2.append(self.sensor_2[i])

        for i in range(0, lenght):
            if self.max_noise_data_3[i] > self.sensor_3[i]:
                max_sensor3.append(self.max_noise_data_3[i])
            else:
                max_sensor3.append(self.sensor_3[i])

        for i in range(0, lenght):
            if self.max_noise_data_4[i] > self.sensor_4[i]:
                max_sensor4.append(self.max_noise_data_4[i])
            else:
                max_sensor4.append(self.sensor_4[i])

        self.save_data_range(
            max_sensor1,
            max_sensor2,
            max_sensor3,
            max_sensor4,
            self.max_noise,
        )
        ########################################################################

        # print(self.min_noise_data_1)
        min_sensor1 = []
        min_sensor2 = []
        min_sensor3 = []
        min_sensor4 = []

        for i in range(0, lenght):
            if (
                self.min_noise_data_1[i] != 0
                and self.sensor_1[i] != 0
                and self.sensor_1[60] > 100
            ):
                if self.min_noise_data_1[i] < self.sensor_1[i]:
                    # pass
                    min_sensor1.append(self.min_noise_data_1[i])
                else:
                    min_sensor1.append(self.sensor_1[i])

        for i in range(0, lenght):
            if (
                self.min_noise_data_2[i] != 0
                and self.sensor_2[i] != 0
                and self.sensor_2[60] > 100
            ):
                if self.min_noise_data_2[i] < self.sensor_2[i]:
                    # pass
                    min_sensor2.append(self.min_noise_data_2[i])
                else:
                    min_sensor2.append(self.sensor_2[i])

        for i in range(0, lenght):
            if (
                self.min_noise_data_3[i] != 0
                and self.sensor_3[i] != 0
                and self.sensor_3[60] > 100
            ):
                if self.min_noise_data_3[i] < self.sensor_3[i]:
                    # pass
                    min_sensor3.append(self.min_noise_data_3[i])
                else:
                    min_sensor3.append(self.sensor_3[i])

        for i in range(0, lenght):
            if (
                self.min_noise_data_4[i] != 0
                and self.sensor_4[i] != 0
                and self.sensor_4[60] > 100
            ):
                if self.min_noise_data_4[i] > self.sensor_4[i]:
                    # pass
                    min_sensor4.append(self.sensor_4[i])
                else:
                    min_sensor4.append(self.min_noise_data_4[i])
                    # min_sensor4.append(self.sensor_4[i])
        print("min_sensor1", min_sensor1)
        if (
            len(min_sensor1) == 64
            and len(min_sensor2) == 64
            and len(min_sensor3) == 64
            and len(min_sensor4) == 64
        ):
            self.save_data_range(
                min_sensor1,
                min_sensor2,
                min_sensor3,
                min_sensor4,
                self.min_noise,
            )

        ########################################################################

        self.save_data_range(
            data_sensor_1,
            data_sensor_2,
            data_sensor_3,
            data_sensor_4,
            self.config_path,
        )
        self.save_data_range(
            remove_noise_sensor1,
            remove_noise_sensor2,
            remove_noise_sensor3,
            remove_noise_sensor4,
            self.after_noise,
        )

    def save_data_range(
        self,
        ss_1,
        ss_2,
        ss_3,
        ss_4,
        n_sensor="sensor 1",
    ):

        default_dictionary = {
            "lidarvl53l5_sensor1": ss_1,
            "lidarvl53l5_sensor2": ss_2,
            "lidarvl53l5_sensor3": ss_3,
            "lidarvl53l5_sensor4": ss_4,
        }
        # print("this funtion")
        json_object = json.dumps(default_dictionary, indent=4)
        with open(n_sensor, "w") as outfile:
            outfile.write(json_object)

    def input_keyboard(self):

        keyboard_value = ord(getch.getch())
        if keyboard_value == 49:
            print("hello")
        #     self.save_default(self.ranger_sensor1, n_sensor="sensor 1")
        # elif keyboard_value == 50:
        #     self.save_default(self.ranger_sensor2, n_sensor="sensor 2")
        # elif keyboard_value == 51:
        #     self.save_default(self.ranger_sensor3, n_sensor="sensor 3")
        # elif keyboard_value == 52:
        #     self.save_default(self.ranger_sensor4, n_sensor="sensor 4")
        # return keyboard_value

    def open_json(self):
        """Open Json file."""

        with open(self.config_path, "r") as f:
            data = json.load(f)
        default_range = pd.DataFrame(data)
        for sensor, param in default_range.items():
            if sensor == "lidarvl53l5_sensor1":
                self.default_sensor_1 = np.array(param)
            if sensor == "lidarvl53l5_sensor2":
                self.default_sensor_2 = np.array(param)
            if sensor == "lidarvl53l5_sensor3":
                self.default_sensor_3 = np.array(param)
            if sensor == "lidarvl53l5_sensor4":
                self.default_sensor_4 = np.array(param)

        with open(self.max_noise, "r") as mx:
            _max_data = json.load(mx)
        max_noise_data = pd.DataFrame(_max_data)
        for sensor, param in max_noise_data.items():
            if sensor == "lidarvl53l5_sensor1":
                self.max_noise_data_1 = np.array(param)
            if sensor == "lidarvl53l5_sensor2":
                self.max_noise_data_2 = np.array(param)
            if sensor == "lidarvl53l5_sensor3":
                self.max_noise_data_3 = np.array(param)
            if sensor == "lidarvl53l5_sensor4":
                self.max_noise_data_4 = np.array(param)

        with open(self.min_noise, "r") as minx:
            _min_data = json.load(minx)
        min_noise_data = pd.DataFrame(_min_data)
        for sensor, param in min_noise_data.items():
            if sensor == "lidarvl53l5_sensor1":
                self.min_noise_data_1 = np.array(param)
            if sensor == "lidarvl53l5_sensor2":
                self.min_noise_data_2 = np.array(param)
            if sensor == "lidarvl53l5_sensor3":
                self.min_noise_data_3 = np.array(param)
            if sensor == "lidarvl53l5_sensor4":
                self.min_noise_data_4 = np.array(param)

    def loop(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.identification_noise()
            # self.input_keyboard()
            rate.sleep()


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
            # "default_range.json",
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
    calculate_noise(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
