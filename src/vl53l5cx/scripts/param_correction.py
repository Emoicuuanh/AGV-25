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


class param_correction:
    def __init__(self, *args, **kwargs):
        self.init_variable(*args, **kwargs)

        rospy.init_node("config_paramer_lidarvl3l5cx", anonymous=True)
        rospy.loginfo("Init node " + rospy.get_name())
        self.default_parameter()
        # Subscriber
        rospy.Subscriber("/vl35l5cx_r1", Vl53l5cxRanges, self.range_senser_1_cb)
        rospy.Subscriber("/vl35l5cx_r2", Vl53l5cxRanges, self.range_senser_2_cb)
        rospy.Subscriber("/vl35l5cx_r3", Vl53l5cxRanges, self.range_senser_3_cb)
        rospy.Subscriber("/vl35l5cx_r4", Vl53l5cxRanges, self.range_senser_4_cb)
        rospy.Subscriber("/task_jsonConfig", String, self.parse_jsons_cb)
        self.loop()

    def init_variable(self, *args, **kwargs):
        self.config_path = kwargs["config_path"]

    def default_parameter(self):
        self.lenth_range = [0] * 64
        self.df = None
        self.x_pos = [0] * 64
        self.y_pos = [0] * 64
        self.z_pos = [0] * 64
        self.sin0_yaw = [0] * 64
        self.cos0_yaw = [0] * 64
        self.sin0_pitch = [0] * 64
        self.cos0_pitch = [0] * 64
        self.yaw_transform = [] * 64
        self.pitch_transform = [] * 64
        self.centimetros_to_metros = [1000] * 64
        self.position_sensor_2 = None
        self.position_sensor_3 = None
        self._h = 0.25
        self.open_json()

    def range_senser_1_cb(self, msg):
        vl53l5cxranger = np.array(msg.range)
        # self.convert_dist_coords(vl53l5cxranger)
        # self.angle_sensor(vl53l5cxranger, frame_id="lidar 1")

    def range_senser_2_cb(self, msg):
        vl53l5cxranger = np.array(msg.range)
        # self.convert_dist_coords(vl53l5cxranger)
        # self.angle_sensor(vl53l5cxranger, frame_id="lidar 2")

    def range_senser_3_cb(self, msg):
        vl53l5cxranger = np.array(msg.range)
        self.convert_dist_coords(vl53l5cxranger)
        # self.angle_sensor(vl53l5cxranger, frame_id="lidar 3")

    def range_senser_4_cb(self, msg):
        range_data = msg.range

    def parse_jsons_cb(self, msg):
        # sensor_data = {"sensor_0", "sensor_1", "sensor_2", "sensor_3"}
        paramater = json.loads(msg.data)
        for sensor, params in paramater.items():
            np.array(params["ranges"])
            if sensor == "sensor_2":
                # for i in np.array(params["ranges"]):
                #     print(i[0])
                self.position_sensor_2 = np.array(params["ranges"])
            if sensor == "sensor_3":
                self.position_sensor_3 = np.array(params["ranges"])
                # print(self.position_sensor_3)

    def convert_dist_coords(self, current_value):
        """Compute SinCos Tables."""
        # References
        # https://community.st.com/s/question/0D53W000015XpcBSAS/vl53l5cx-multizone-sensor-get-xyz-of-points-relative-to-origin

        # TODO not use Z
        self.default_parameter()
        self.len_pcl = len(self.lenth_range)
        range_data = current_value / self.centimetros_to_metros
        for z in range(0, self.len_pcl):
            self.sin0_pitch[z] = math.sin(math.radians(self.pitch_transform[z]))
            self.cos0_pitch[z] = math.cos(math.radians(self.pitch_transform[z]))
            self.sin0_yaw[z] = math.sin(math.radians(self.yaw_transform[z]))
            self.cos0_yaw[z] = math.cos(math.radians(self.yaw_transform[z]))

        """Converts data axis Y and Z from axis X."""
        hyp = range_data / self.sin0_pitch
        self.x_pos = self.cos0_yaw * hyp * self.cos0_pitch
        self.y_pos = self.sin0_yaw * hyp * self.cos0_pitch
        self.z_pos = range_data

        array_2d_square = np.square(range_data) - np.square(self.x_pos)
        array_sqrt = np.sqrt(array_2d_square)
        print(array_sqrt)

        return array_sqrt

    def open_json(self):
        """Open Json file."""
        with open(self.config_path, "r") as f:
            data = json.load(f)
        self.df = pd.DataFrame(data)

        self.pitch_transform = np.array(
            self.df.params.VL53L5_Zone_Pitch8x8,
            dtype=np.float32,
        )
        self.yaw_transform = np.array(
            self.df.params.VL53L5_Zone_Yaw8x8,
            dtype=np.float32,
        )

    def loop(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
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
            "vl53l5cx_config.json",
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
    param_correction(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
