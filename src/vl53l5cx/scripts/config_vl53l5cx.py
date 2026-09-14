#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
from re import A
import os
import sys
import math
import json
import rospy
import rospkg
import actionlib
import numpy as np
import pandas as pd
from pymongo import MongoClient, errors
from std_msgs.msg import Time, Int8, Int16, Int32, String
from vl53l5cx.msg import Vl53l5cxRanges, Safety_Esp, vl53l5cx_safety
from safety_msgs.msg import SafetyStatus

# from common_funtion import *
# from estimation_data import *
import numpy as np
from collections import defaultdict

from std_stamped_msgs.msg import (
    StringStamped,
    StringAction,
    StringFeedback,
    StringResult,
    StringGoal,
    EmptyStamped,
)

from agv_msgs.msg import (
    FollowLineSensor,
    DiffDriverMotorSpeed,
    EncoderDifferential,
)

common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)
agv_mongodb_dir = os.path.join(
    rospkg.RosPack().get_path("agv_mongodb"), "scripts"
)
if not os.path.isdir(agv_mongodb_dir):
    agv_mongodb_dir = os.path.join(
        rospkg.RosPack().get_path("agv_mongodb"), "release"
    )
sys.path.insert(0, agv_mongodb_dir)

from mongodb import mongodb

from common_function import (
    EnumString,
    print_debug,
)


class vl53l5cx_config:
    def __init__(self, *args, **kwargs):
        rospy.init_node("lidarvl3l5cx_config", anonymous=True)
        rospy.loginfo("Init node " + rospy.get_name())
        self.init_variable(*args, **kwargs)

        self.db = mongodb("mongodb://coffee:coffee@localhost:27017")
        try:
            self.db = mongodb("mongodb://coffee:coffee@localhost:27017")
        except errors.ServerSelectionTimeoutError as err:
            print("pymongo ERROR:", err)
        self.set_safety_job_client = actionlib.SimpleActionClient(
            "set_safety_job", StringAction
        )
        self.set_footprint_client = actionlib.SimpleActionClient(
            "set_footprint", StringAction
        )
        self.safety_sensor1_timeout = rospy.get_time()
        self.safety_sensor2_timeout = rospy.get_time()
        self.safety_sensor3_timeout = rospy.get_time()
        self.safety_sensor4_timeout = rospy.get_time()
        self.safety_timeout = rospy.get_time()
        self.update_exclude_width = rospy.get_time()
        self.disable_safety = False
        self.disable_esp_safety = False
        self.centimetros_to_metros = [1000] * 64
        self.sensor_1 = [0] * 64
        self.sensor_2 = [0] * 64
        self.sensor_3 = [0] * 64
        self.sensor_4 = [0] * 64

        self.sin0_pitch = [0] * 64
        self.cos0_pitch = [0] * 64
        self.sin0_yaw = [0] * 64
        # self.minimum_sensor1 = []
        # self.minimum_sensor2 = []
        # self.minimum_sensor3 = []
        # self.minimum_sensor4 = []
        self.exclude_array_ss1 = []
        self.exclude_array_ss2 = []
        self.length_data_sensor = [0] * 64
        self.minimum_threshold = 260
        self.width_region_safety = []
        self.len_pcl = len(self.length_data_sensor)

        self.direct = False
        self.is_safety = False
        self.safety_dir = False

        # subscribe
        rospy.Subscriber(
            "/vl35l5cx_r1", Vl53l5cxRanges, self.sensor1_vl53l5cx_cb
        )
        rospy.Subscriber(
            "/vl35l5cx_r2", Vl53l5cxRanges, self.sensor2_vl53l5cx_cb
        )
        rospy.Subscriber(
            "/vl35l5cx_r3", Vl53l5cxRanges, self.sensor3_vl53l5cx_cb
        )
        rospy.Subscriber(
            "/vl35l5cx_r4", Vl53l5cxRanges, self.sensor4_vl53l5cx_cb
        )

        rospy.Subscriber("/safety_disable", Int8, self.disable_safety_cb)
        rospy.Subscriber(
            "/esp_safety_control", Int8, self.disable_esp_safety_cb
        )

        rospy.Subscriber(
            "/motor_speed", DiffDriverMotorSpeed, self.motor_dir_cb
        )
        rospy.Subscriber("/set_safety_job", String, self.set_safety_cb)
        # Publish
        self.safety_status_pub = rospy.Publisher(
            "/safety_status", SafetyStatus, queue_size=10
        )
        self.set_dir_esp = rospy.Publisher(
            "/lidarvl53l5vc_dir", Int8, queue_size=10
        )
        self.pub_json = rospy.Publisher(
            "/safety_jsonConfig", String, queue_size=10, latch=True
        )
        self.set_safety(field_foodprint="amr_run_alone", field_safety="FORWARD")
        # self.calculate_tolerance()
        # self.update_exclude_width()
        self.loop()

    def init_variable(self, *args, **kwargs):
        self.normal_param_safety = (
            kwargs["config_path"] + "/set_safety_goal.json"
        )
        # self.default_range_value = kwargs["config_path"] + "/default_range.json"
        self.default_range_value = kwargs["config_path"] + "/min_noise.json"
        self.max_noise = kwargs["config_path"] + "/fake_data.json"
        self.min_noise = kwargs["config_path"] + "/min_noise.json"
        self.update_min_noise = kwargs["config_path"] + "/minimum.json"
        self.sensor_config = kwargs["config_path"] + "/vl53l5cx_config.json"

    def safety_towards(self, dir):

        # if dir != self.direct:
        #     rospy.logerr(" SET SAFETY WRONG DIRECTION")
        if not dir:
            self.set_safety(
                field_foodprint="amr_run_alone", field_safety="FORWARD"
            )
            self.safety_dir = False
        else:
            self.set_safety(
                field_foodprint="amr_run_alone", field_safety="BACKWARD"
            )
            self.safety_dir = True

    def pub_dir_esp(self):

        pub_dir = Int8()
        # print((self.direct))
        if self.direct:
            pub_dir.data = 1
        else:
            pub_dir.data = 0
        self.set_dir_esp.publish(pub_dir)

    def set_safety(
        self, field_foodprint="field_foodprint", field_safety="field_safety"
    ):
        self.open_json()
        self.safety_job = {}
        self.footprint = {}
        self.job = {}
        self.lidarvl53l5cx_min_range = rospy.get_param(
            "~ultrasonic_min_range", 0.25
        )
        self.lidarvl53l5cx_offset = rospy.get_param("~ultrasonic_offset", 0.5)
        self.job["safety"] = self.default_safety.params.safety
        self.elements_param = [0.0] * len(self.job["safety"])
        self.arr = np.array(self.elements_param)
        current_safety_job = self.set_safety_job(field_safety)
        # set_footprint = json.loads(SetAreaSafety.set_footprint(self, field_foodprint))
        if current_safety_job is None:
            rospy.logerr("Safety not exist")
            self.safety_job["safety_job"] = self.job["safety"]
        else:
            safetyjob = json.loads(current_safety_job)
            for senser, params in safetyjob.items():
                self.safety_job["safety_job"] = params
        # print(self.safety_job["safety_job"])
        self.estimation_position_from_base_link()

    def estimation_position_from_base_link(self):

        self.lidarvl53l5cx_sensors = rospy.get_param(
            "~lidarvl53l5cx_sensors", dict({})
        )
        area_fields = []
        min_area_fields = []
        count_time_i = 0
        width_safety_max = []
        width_safety_min = []
        for sensor, params in self.lidarvl53l5cx_sensors.items():
            # print(params)
            position_sensor = params["position_from_base_link"]
            params["ranges"] = self.arr.reshape(1, 3)
            for i in range(len(self.safety_job["safety_job"])):

                if count_time_i < (len(self.safety_job["safety_job"])):
                    safety_max_position = self.get_max_position(
                        self.safety_job["safety_job"][i]["data"],
                        position_sensor[2],
                    )
                    safety_min_position = self.get_min_position(
                        self.safety_job["safety_job"][i]["data"],
                        position_sensor[2],
                    )
                    min_area_fields.append(safety_min_position)
                    area_fields.append(safety_max_position)
                    count_time_i += i

            # print("min_area_fields ", min_area_fields)
            # print("position_sensor", position_sensor)

            if position_sensor[0] > 0:
                params["ranges"] = self.rounding_pos(
                    area_fields, position_sensor
                )
                # print(" params",  params["ranges"])
                # print(" params",  params["ranges"][0][0])
            else:
                params["ranges"] = self.rounding_pos(
                    min_area_fields, position_sensor
                )
            # print(params["ranges"] )
            params["ranges"] = params["ranges"].tolist()
            for i in range(0, 3):
                width_safety_min.append(area_fields[i][1])
        # print( width_safety_min)
        self.pub_json.publish(json.dumps(self.lidarvl53l5cx_sensors))

        # print_debug(
        #     "Lidarvl53l5cx_config :",
        #     json.dumps(self.lidarvl53l5cx_sensors, indent=4),
        # )
        self.distant_are_direction_X(self.lidarvl53l5cx_sensors)

    def distant_are_direction_X(self, lidarvl53l5cx_config):

        length_region_safety = []
        width_region_safety = []
        for sensor, param in lidarvl53l5cx_config.items():
            length = 0
            width = 1
            # print(len(param["ranges"]))
            for i in range(length, len(param["ranges"])):
                length_region_safety.append(param["ranges"][i][length])
                width_region_safety.append(param["ranges"][i][width])
            # print("width_region_safety", width_region_safety)
            # for i in range(width, len(param["ranges"])):
            #     length_region_safety.append(param["ranges"][i][width])
            # print(param["ranges"][i])
            # for i in self.default_range.lidarvl53l5_sensor1:
            # print(i)
        # print("width_region_safety", width_region_safety)

        self.width_region_safety = width_region_safety
        self.distanct_lidarvl53l5(length_region_safety, width_region_safety)

    def calculate_tolerance(self):

        Z_pos = 350
        self.tolerance = rospy.get_param("~tolerance", 3)
        lenght_ = len(self.length_data_sensor)
        ratio_between_sides_z = self.tolerance / Z_pos
        distance_ss_1 = []
        distance_ss_2 = []
        distance_ss_3 = []
        distance_ss_4 = []
        for sensor, param in self.max_default_range.items():

            if sensor == "lidarvl53l5_sensor1":
                distance_ss_1 = param
            if sensor == "lidarvl53l5_sensor2":
                distance_ss_2 = param
            if sensor == "lidarvl53l5_sensor3":
                distance_ss_3 = param
            if sensor == "lidarvl53l5_sensor4":
                distance_ss_4 = param

        lengthiness_1 = []
        lengthiness_2 = []
        lengthiness_3 = []
        lengthiness_4 = []
        for i in range(0, lenght_):
            lengthiness_1.append(
                np.sqrt(abs(np.square(distance_ss_1[i]) - np.square(Z_pos)))
            )
            lengthiness_2.append(
                np.sqrt(abs(np.square(distance_ss_2[i]) - np.square(Z_pos)))
            )
            lengthiness_3.append(
                np.sqrt(abs(np.square(distance_ss_3[i]) - np.square(Z_pos)))
            )
            lengthiness_4.append(
                np.sqrt(abs(np.square(distance_ss_4[i]) - np.square(Z_pos)))
            )

        m_1 = []
        for i in range(0, lenght_):
            m_1.append(lengthiness_1[i] * self.tolerance / Z_pos)
        # print(lengthiness_1[0])
        # print(lengthiness_1[8])
        # print(lengthiness_1[16])
        # print(lengthiness_1[32])
        # print("current" , lengthiness_1)
        # print("y1" , convert_dist_coords(distance_ss_1))
        # print("y2" , convert_dist_coords(distance_ss_2))

    def to_exclude_width(self, data_exclude, dir_safety):

        exclude_sensor = []
        lenth_safety = 64
        value_exclude = data_exclude * self.centimetros_to_metros
        # print("width_region_safety[0]", self.width_region_safety[0])
        for i in range(0, lenth_safety):
            if dir_safety:
                if (
                    self.width_region_safety[0]
                    < self.convert_dist_coords(value_exclude)[i]
                ):
                    exclude_sensor.append(i)
            else:
                if self.width_region_safety[0] > 0:
                    if (
                        self.width_region_safety[0]
                        < self.convert_dist_coords(value_exclude)[i]
                    ):
                        exclude_sensor.append(i)
                else:
                    if (
                        self.width_region_safety[0]
                        > self.convert_dist_coords(value_exclude)[i]
                    ):
                        exclude_sensor.append(i)
        self.update_exclude_width = rospy.get_time()
        return exclude_sensor

    def distanct_lidarvl53l5(self, length_region_safety, width_region_safety):

        # print(self.default_range[sensor])
        # print(length_region_safety)
        self.tolerance = rospy.get_param("~tolerance", 0.03)
        for sensor, param in self.default_range.items():
            # print("befor", self.default_range[sensor])
            self.default_range[sensor] = (
                self.default_range[sensor] / 1000
            ) - self.tolerance
            # sai so cho phep

        are_safety_1_1 = []
        are_safety_1_2 = []
        are_safety_1_3 = []

        are_safety_2_1 = []
        are_safety_2_2 = []
        are_safety_2_3 = []

        are_safety_3_1 = []
        are_safety_3_2 = []
        are_safety_3_3 = []

        are_safety_4_1 = []
        are_safety_4_2 = []
        are_safety_4_3 = []

        # width_data_1 = []
        # for sensor, param in self.max_default_range.items():
        #     if sensor == "lidarvl53l5_sensor1":
        #         # print("convert_dist_coords(param)[i]",convert_dist_coords(param))
        #         for i in range(0, 64):
        #             # print("convert_dist_coords(param)[i]",convert_dist_coords(param))
        #             if length_region_safety[0] < convert_dist_coords(param)[i]:
        #                 width_data_1.append(i)

        # print("width_data_1",width_data_1)
        for sensor, param in self.default_range.items():
            if sensor == "lidarvl53l5_sensor1":
                for i in param:
                    # print("i cu" ,i)
                    if i < (length_region_safety[0]):
                        are_safety_1_1.append(i)
                    else:
                        are_safety_1_1.append(-10)

                    if i < (length_region_safety[1]):
                        are_safety_1_2.append(i)
                    else:
                        are_safety_1_2.append(-10)

                    if i < (length_region_safety[2]):
                        are_safety_1_3.append(i)
                    else:
                        are_safety_1_3.append(-10)

            if sensor == "lidarvl53l5_sensor2":
                for i in param:
                    if i < (length_region_safety[3]):
                        are_safety_2_1.append(i)
                    else:
                        are_safety_2_1.append(-10)

                    if i < (length_region_safety[4]):
                        are_safety_2_2.append(i)
                    else:
                        are_safety_2_2.append(-10)

                    if i < (length_region_safety[5]):
                        are_safety_2_3.append(i)
                    else:
                        are_safety_2_3.append(-10)

            if sensor == "lidarvl53l5_sensor3":
                for i in param:
                    if i < (length_region_safety[6]):
                        are_safety_3_1.append(i)
                    else:
                        are_safety_3_1.append(length_region_safety[6])

                    if i < (length_region_safety[7]):
                        are_safety_3_2.append(i)
                    else:
                        are_safety_3_2.append(length_region_safety[7])

                    if i < (length_region_safety[8]):
                        are_safety_3_3.append(i)
                    else:
                        are_safety_3_3.append(length_region_safety[8])

            if sensor == "lidarvl53l5_sensor4":
                for i in param:
                    if i < (length_region_safety[9]):
                        are_safety_4_1.append(i)
                    else:
                        are_safety_4_1.append(length_region_safety[9])

                    if i > (length_region_safety[10]):
                        are_safety_4_2.append(i)
                    else:
                        are_safety_4_2.append(length_region_safety[10])

                    if i > (length_region_safety[11]):
                        are_safety_4_3.append(i)
                    else:
                        are_safety_4_3.append(length_region_safety[11])

        self.are_safety_after_change = {
            "lidarvl53l5_sensor1": {
                "region_safety": [
                    {
                        "data": are_safety_1_1,
                        "index": 0,
                    },
                    {
                        "data": are_safety_1_2,
                        "index": 1,
                    },
                    {
                        "data": are_safety_1_3,
                        "index": 2,
                    },
                ]
            },
            "lidarvl53l5_sensor2": {
                "region_safety": [
                    {
                        "data": are_safety_2_1,
                        "index": 0,
                    },
                    {
                        "data": are_safety_2_2,
                        "index": 1,
                    },
                    {
                        "data": are_safety_2_3,
                        "index": 2,
                    },
                ]
            },
            "lidarvl53l5_sensor3": {
                "region_safety": [
                    {
                        "data": are_safety_3_1,
                        "index": 0,
                    },
                    {
                        "data": are_safety_3_2,
                        "index": 1,
                    },
                    {
                        "data": are_safety_3_3,
                        "index": 2,
                    },
                ]
            },
            "lidarvl53l5_sensor4": {
                "region_safety": [
                    {
                        "data": are_safety_4_1,
                        "index": 0,
                    },
                    {
                        "data": are_safety_4_2,
                        "index": 1,
                    },
                    {
                        "data": are_safety_4_3,
                        "index": 2,
                    },
                ]
            },
        }
        # print("self.are_safety_after_change" , self.are_safety_after_change)
        # self.set_lidar_safety_zone(self.are_safety_after_change)

    def set_lidar_safety_zone(self):

        self.time_out = rospy.get_param("~time_out", 0.1)
        safety_status = SafetyStatus()
        safety_status.fields = [1] * 3
        area_sensor1 = False
        area_sensor2 = False
        area_sensor3 = False
        area_sensor4 = False

        self.exclude_array_ss1 = self.to_exclude_width(
            np.array(self.sensor_1), True
        )
        self.exclude_array_ss2 = self.to_exclude_width(
            np.array(self.sensor_2), False
        )
        # print("data exclude 1 ", self.exclude_array_ss1)
        # print("data exclude 2", self.exclude_array_ss2)
        if not self.direct:

            for sensor, param in self.are_safety_after_change.items():
                # print(param)
                if sensor == "lidarvl53l5_sensor1":
                    for i in self.exclude_array_ss1:
                        param["region_safety"][0]["data"][i] = -1
                    safety_1 = self.sensor_1 - np.array(
                        param["region_safety"][0]["data"]
                    )
                    if self.check_current_safety(safety_1, "sensor_1"):
                        if (
                            rospy.get_time() - self.safety_sensor1_timeout
                            > self.time_out
                        ):
                            area_sensor1 = True
                    print("self.sensor_1", self.sensor_1)
                    print("ss 1 ", safety_1)
                if sensor == "lidarvl53l5_sensor2":
                    for i in self.exclude_array_ss2:
                        param["region_safety"][0]["data"][i] = -1
                    safety_2 = self.sensor_2 - np.array(
                        param["region_safety"][0]["data"]
                    )

                    if self.check_current_safety(safety_2, "sensor_2"):
                        if (
                            rospy.get_time() - self.safety_sensor2_timeout
                            > self.time_out
                        ):
                            area_sensor2 = True
                    # print("ss 2", safety_2)
            if area_sensor1 or area_sensor2:
                self.is_safety = True
            else:
                self.is_safety = False

            self.update_safety(
                safety_1,
                safety_2,
            )
        else:

            for sensor, param in self.are_safety_after_change.items():
                if sensor == "lidarvl53l5_sensor3":
                    safety_3 = self.sensor_3 - np.array(
                        param["region_safety"][0]["data"]
                    )
                    if self.check_current_safety(safety_3, "sensor_3"):
                        if (
                            rospy.get_time() - self.safety_sensor3_timeout
                            > self.time_out
                        ):
                            area_sensor3 = True
                    # print("3", safety_3)
                if sensor == "lidarvl53l5_sensor4":
                    safety_4 = self.sensor_4 - np.array(
                        param["region_safety"][0]["data"]
                    )
                    if self.check_current_safety(safety_4, "sensor_4"):
                        if (
                            rospy.get_time() - self.safety_sensor4_timeout
                            > self.time_out
                        ):
                            area_sensor4 = True

            if area_sensor3 or area_sensor4:
                self.is_safety = True
            else:
                self.is_safety = False

        print("area_sensor1 ", area_sensor1)
        print("area_sensor2", area_sensor2)
        print("area_sensor3", area_sensor3)
        print("area_sensor4", area_sensor4)
        # print(" self.is_safety", self.is_safety)
        # print("sssssss", self.disable_esp_safety)

    def stop_immediately(self):

        is_safety = False
        immediately_1 = np.array(self.sensor_1) * self.centimetros_to_metros
        immediately_2 = np.array(self.sensor_2) * self.centimetros_to_metros
        immediately_3 = np.array(self.sensor_3) * self.centimetros_to_metros
        immediately_4 = np.array(self.sensor_4) * self.centimetros_to_metros

        lenght_ = len(self.length_data_sensor)
        for i in range(0, lenght_):
            if immediately_1[i] != 0 and immediately_2[i] != 0:
                if (
                    immediately_1[i] < self.minimum_threshold
                    or immediately_2[i] < self.minimum_threshold
                ):
                    is_safety = True

                else:
                    is_safety = False

        return is_safety

    def check_current_safety(self, currentset, name_sensor="sensor"):

        _safety = False
        safety_set = []

        for i in currentset:
            safety_set.append(i)

        lenth_safety = len(safety_set)

        for i in range(0, lenth_safety):
            if safety_set[i] < 0:
                _safety = True
                # print("end funtion")
                return _safety

        if not _safety:
            if name_sensor == "sensor_1":
                self.safety_sensor1_timeout = rospy.get_time()
            if name_sensor == "sensor_2":
                self.safety_sensor2_timeout = rospy.get_time()
            if name_sensor == "sensor_3":
                self.safety_sensor3_timeout = rospy.get_time()
            if name_sensor == "sensor_4":
                self.safety_sensor4_timeout = rospy.get_time()

        return _safety

    def safety_or_not(self):

        safety_status = SafetyStatus()
        safety_status.fields = [1] * 3
        if self.stop_immediately():
            print("stop_immediately ")
            for i in range(0, 3):
                safety_status.fields[i] = 1
            self.safety_status_pub.publish(safety_status)

        if self.disable_esp_safety == True or self.disable_safety == True:
            for i in range(0, 3):
                safety_status.fields[i] = 0
            self.safety_status_pub.publish(safety_status)
        else:
            if self.is_safety:
                for i in range(0, 3):
                    safety_status.fields[i] = 1
                self.safety_status_pub.publish(safety_status)
            else:
                for i in range(0, 3):
                    safety_status.fields[i] = 0
                self.safety_status_pub.publish(safety_status)

    def rounding_pos(self, fields, position):

        config_position = []
        for i in position:
            if i < 0:
                i = abs(i)
            config_position.append(i)
        return np.array(fields) - np.array(config_position)

    def get_max_position(self, data_array, z_pos):
        _x_max = 0.0
        _y_max = 0.0
        _z_max = z_pos

        for i in range(0, 3):
            for j in data_array:
                if i == 0:
                    if j[i] > _x_max:
                        _x_max = j[i]
                if i == 1:
                    if j[i] > _y_max:
                        _y_max = j[i]

        xyz_max_pos = [_x_max, _y_max, _z_max]
        return xyz_max_pos

    def get_min_position(self, data_array, z_pos):
        _x_max = 0.0
        _y_max = 0.0
        _z_max = z_pos

        for i in range(0, 3):
            for j in data_array:
                if i == 0:
                    if j[i] < _x_max:
                        _x_max = j[i]
                if i == 1:
                    if j[i] < _y_max:
                        _y_max = j[i]

        xyz_min_pos = [_x_max, _y_max, _z_max]
        abs_xyz = [abs(_x_max), abs(_y_max), abs(_z_max)]
        return abs_xyz

    def set_footprint(self, footprint):
        footprint = self.db.getFootprint(footprint)
        if footprint != None:
            footprint = json.dumps(footprint, indent=2, sort_keys=True)
            footprint = json.loads(footprint)
            footprint = json.dumps({"footprint": footprint})
            goal = StringGoal()
            try:
                # print("set_footprint: {}".format(footprint))
                goal.data = json.dumps({"params": json.loads(footprint)})
                self.set_footprint_client.send_goal(goal)
                # rospy.logwarn("Set footprint success")
            except Exception as e:
                rospy.logerr("set_footprint: {}".format(e))
        return footprint

    def set_safety_job(self, name_safety):
        name_safety = self.db.getSafety(name_safety)
        if name_safety != None:
            name_safety = json.dumps(name_safety, indent=2, sort_keys=True)
            name_safety = json.loads(name_safety)
            name_safety = json.dumps({"safety": name_safety})
            goal = StringGoal()
            try:
                print("set_safety_job: {}".format(name_safety))
                goal.data = json.dumps({"params": json.loads(name_safety)})
                self.set_safety_job_client.send_goal(goal)
                rospy.logwarn("Set safety job success")
            except Exception as e:
                rospy.logerr("set_safety_job: {}".format(e))
        return name_safety

    def update_safety(self, minimum_number_1, minimum_number_2):

        parameters_minimum_1 = minimum_number_1 * self.centimetros_to_metros
        parameters_minimum_2 = minimum_number_2 * self.centimetros_to_metros
        minimum_1 = []
        minimum_2 = []
        minimum_3 = []
        minimum_4 = []

        # print("ss1" , parameters_minimum_1)
        # print("ss2" , parameters_minimum_2)

        for sensor, param in self.update_minumum_safety.items():
            if sensor == "lidarvl53l5_sensor1":
                minimum_data_1 = np.array(param)
            if sensor == "lidarvl53l5_sensor2":
                minimum_data_2 = np.array(param)
            if sensor == "lidarvl53l5_sensor3":
                minimum_data_3 = np.array(param)
            if sensor == "lidarvl53l5_sensor4":
                minimum_data_4 = np.array(param)

        __lenght = len(self.length_data_sensor)
        for i in range(0, __lenght):
            if parameters_minimum_1[i] != 0:
                if (
                    parameters_minimum_1[i] < minimum_data_1[i]
                    and parameters_minimum_1[i] > -150
                ):
                    minimum_1.append(parameters_minimum_1[i])
                else:
                    minimum_1.append(minimum_data_1[i])

                if (
                    parameters_minimum_2[i] < minimum_data_2[i]
                    and parameters_minimum_2[i] > -150
                ):
                    minimum_2.append(parameters_minimum_2[i])
                else:
                    minimum_2.append(minimum_data_2[i])

        # print("minimum_1", minimum_1)
        # print("minimum_2", minimum_2)
        if len(minimum_2) == 64 and len(minimum_1) == 64:
            self.load_parameter_again(
                minimum_1, minimum_2, minimum_data_3, minimum_data_4
            )

    def load_parameter_again(self, minimum_1, minimum_2, minimum_3, minimum_4):

        ss_1 = list(minimum_1)
        ss_2 = list(minimum_2)
        ss_3 = list(minimum_3)
        ss_4 = list(minimum_4)

        default_dictionary = {
            "lidarvl53l5_sensor1": ss_1,
            "lidarvl53l5_sensor2": ss_2,
            "lidarvl53l5_sensor3": ss_3,
            "lidarvl53l5_sensor4": ss_4,
        }

        json_object = json.dumps(default_dictionary, indent=4)
        with open(self.update_min_noise, "w") as outfile:
            outfile.write(json_object)

    def convert_dist_coords(self, current_value):

        self.pitch_transform = np.array(
            self.config_vl53l5cx.params.VL53L5_Zone_Pitch8x8,
            dtype=np.float32,
        )
        self.yaw_transform = np.array(
            self.config_vl53l5cx.params.VL53L5_Zone_Yaw8x8,
            dtype=np.float32,
        )

        range_data = np.array(current_value) / np.array(
            self.centimetros_to_metros
        )
        for z in range(0, self.len_pcl):
            self.sin0_pitch[z] = math.sin(math.radians(self.pitch_transform[z]))
            self.cos0_pitch[z] = math.cos(math.radians(self.pitch_transform[z]))
            self.sin0_yaw[z] = math.sin(math.radians(self.yaw_transform[z]))

        """Converts data axis Y and Z from axis X."""
        hyp = np.array(range_data) / np.array(self.sin0_pitch)
        y_pos = self.sin0_yaw * hyp * self.cos0_pitch
        return y_pos

    # Callback
    def sensor1_vl53l5cx_cb(self, msg):
        self.sensor_1 = np.array(msg.range) / self.centimetros_to_metros

    def sensor2_vl53l5cx_cb(self, msg):
        self.sensor_2 = np.array(msg.range) / self.centimetros_to_metros

    def sensor3_vl53l5cx_cb(self, msg):
        self.sensor_3 = np.array(msg.range) / self.centimetros_to_metros

    def sensor4_vl53l5cx_cb(self, msg):
        self.sensor_4 = np.array(msg.range) / self.centimetros_to_metros

    def disable_esp_safety_cb(self, msg):

        if msg.data == 1:
            self.disable_esp_safety = False
        if msg.data == 0:
            self.disable_esp_safety = True

    def disable_safety_cb(self, msg):
        if msg.data == 1:
            self.disable_safety = True
        else:
            self.disable_safety = False

    def set_safety_cb(self, msg):
        pass

    def motor_dir_cb(self, msg):

        if msg.Direction == self.direct:
            pass
        else:
            # print(self.direct)
            self.direct = msg.Direction
            self.safety_towards(self.direct)

    def open_json(self):

        """Open Json file."""
        with open(self.normal_param_safety, "r") as param_safety:
            set_safety_data = json.load(param_safety)
        self.default_safety = pd.DataFrame(set_safety_data)

        with open(self.default_range_value, "r") as value_default:
            value_default_Lidarvl53l5cx = json.load(value_default)
        self.default_range = pd.DataFrame(value_default_Lidarvl53l5cx)

        with open(self.max_noise, "r") as max_ranger:
            max_value_default_Lidarvl53l5cx = json.load(max_ranger)
        self.max_default_range = pd.DataFrame(max_value_default_Lidarvl53l5cx)

        with open(self.min_noise, "r") as min_ranger:
            min_value_default_Lidarvl53l5cx = json.load(min_ranger)
        self.min_default_range = pd.DataFrame(min_value_default_Lidarvl53l5cx)

        with open(self.update_min_noise, "r") as update_min_ranger:
            _min_noise = json.load(update_min_ranger)
        self.update_minumum_safety = pd.DataFrame(_min_noise)

        with open(self.sensor_config, "r") as config_vl53l5cx:
            _config_ssvl53l5cx = json.load(config_vl53l5cx)
        self.config_vl53l5cx = pd.DataFrame(_config_ssvl53l5cx)

    def loop(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.open_json()
            self.safety_or_not()
            self.set_lidar_safety_zone()
            self.pub_dir_esp()
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
            # "set_safety_goal.json",
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
    vl53l5cx_config(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
