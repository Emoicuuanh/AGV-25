#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
import os
import sys
import rospy
import rospkg
import json
import yaml
import socket
import time
import numpy as np
import math
import traceback
import math
import re
from ast import literal_eval
from std_stamped_msgs.msg import (
    StringAction,
    StringStamped,
    StringResult,
    StringFeedback,
    StringGoal,
    Int8Stamped,
    EmptyStamped,
    Float32Stamped,
)
from std_msgs.msg import Int64, Int16, Int8, String, Empty, Bool
from geometry_msgs.msg import PoseStamped
from agv_msgs.msg import DataMatrixStamped

from math import *
from enum import Enum

import tf2_ros
from tf_conversions import transformations
from geometry_msgs.msg import TransformStamped

common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)

# from common_function import (
#     EnumString,
# )

"""
Format data in data matrix
# D1x000y000;a;b;c.c1;fams;a1;b1;
len = 48 A4x0003y0002;14;-12;359.400390;5259656;-41;39;
len = 8 ['\x02A4x0003y0002', '14', '-13', '359.243225', '5284227', '-41', '39', '\x03']
D1: name code
x0000: coor x of point (m)
y0000: coor y of point (m)
a: coor x of data matrix with camera
b: coor y of data matrix with camera
c: angle of data matrix with camera from 0 – 360 Grad
c1: 6 digits after decimal point of data matrix with camera
fams : frame UTC unit equal ms
a1: (absolute x ) distance between central point of code and camera's central point along X
b1: (absolute y ) distance between central point of code and camera's central point along Y
"""


class FrameType_datamatrix(Enum):
    LABLE = 0
    X_LABLE = 0
    Y_LABLE = 1
    X_POSSITION = 1
    Y_POSSITION = 2
    ANGLE_POSSITION = 3
    FAMS = 4
    X_ABSOLUTE = 5
    Y_ABSOLUTE = 6
    NONE_ = 7
    LENGHT = 8

class Gls621_driver_connect:
    def __init__(self, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        rospy.init_node("Gls621_driver_connect", anonymous=True)
        rospy.loginfo("Init node " + rospy.get_name())

        # Param
        self.host = rospy.get_param("~ip_address", "192.168.20.23")
        self.port = rospy.get_param("~port_gls621", 2111)
        self.get_io_rate = rospy.get_param("~get_io_rate", 35)
        self.data_type = rospy.get_param("~type_data", "Data_matrix")
        self.mechanical_angle = rospy.get_param("~mechanical_angle", 90)
        self.label_frame = rospy.get_param("~label_frame", "map")
        self.camera_frame = rospy.get_param("~camera_frame", "camera") 
        self.posestamped_topic = rospy.get_param("~posestamped_topic", "pose_from_qrcode")
        self.cmd_vel_timeout = rospy.get_param("~cmd_vel_timeout", 0.3)

        self._sick_gls = StringStamped()
        self._gls_621 = DataMatrixStamped()

        self.pre_time_device = 0
        self.time_device = 0
        self.time_disconnect = True
        self.timeout_ethernet = 0.1
        self.ethernet_protocal = None
        self.frequency_device = self.get_io_rate + 5
        self.disconnect_ = False
        self.current_time = rospy.get_time()
        self.frequency_pre = rospy.get_time()
        
        self.curr_label_frame_id = ""

        # Publisher
        self.pub_qrcode = rospy.Publisher(
            "/frame_gls621", StringStamped, queue_size=10
        )
        self.tmp_tf_posestamped = PoseStamped()

        self.pub_posestamped = rospy.Publisher(
            self.posestamped_topic, PoseStamped, queue_size=10
        )
        self.decode_data_qr = rospy.Publisher(
            "/data_gls621", DataMatrixStamped, queue_size=10
        )
        
        self.label_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.camera_to_label_broadcaster = tf2_ros.TransformBroadcaster()

        if self.connect_ethernet():
            self.loop()

    def init_variable(self, *args, **kwargs):
        self.config_path = kwargs["config_path"]

    def connect_ethernet(self):
        try:
            self.ethernet_protocal = socket.socket(
                socket.AF_INET, socket.SOCK_STREAM
            )
            self.ethernet_protocal.settimeout(self.timeout_ethernet)
            self.server_address = (self.host, self.port)
            self.ethernet_protocal.connect(self.server_address)
            # Ethernet_protocal.bind(server_address)
            self.disconnect_ = False
            return True

        except:
            rospy.loginfo(
                "[qr_gls621]: Init gls621 driver failure: %s",
                traceback.format_exc(),
            )
            self.disconnect_ = True
            # sys.exit()

    def disconnect_ethernet(self):
        try:
            if (
                self.ethernet_protocal is not None
                or self.ethernet_protocal.connect_ex(self.server_address)
            ):
                self.ethernet_protocal.close()
                return True
            else:
                print("Don't have connect to disconnect")
                return False
        except OSError as e:
            print("error disconnect Protocal:", str(e))
            return False

    def check_disconnect(self):
        # check real time
        if (rospy.get_time() - self.current_time > 0.01) or (
            rospy.get_time() - self.frequency_pre
            >= (1.0 / self.frequency_device)
        ):
            self.frequency_pre = rospy.get_time()
            # self.time_disconnect = False
            return False
        else:
            # self.time_disconnect = True
            return True

    def find_lable(self, msg):
        xy_uper = msg.upper()
        xy_convert = xy_uper[xy_uper.find("X") :]
        # y = xy_uper[xy_uper.find("Y") :]
        # xy_convert = [x + y]
        xy = re.findall(
            r"\d+",
            xy_convert,
        )
        return xy

    def processing_frame(self):
        protocal_ = self.ethernet_protocal.recv(4096)
        self._sick_gls.stamp = rospy.Time.now()
        self._sick_gls.data = protocal_.decode()
        self.pub_qrcode.publish(self._sick_gls)

        decryption_dataMatrix = protocal_.decode().split(";")

        try:
            if len(decryption_dataMatrix) == FrameType_datamatrix.LENGHT.value:
                self._gls_621.header.frame_id = self.data_type
                self._gls_621.header.stamp = rospy.Time.now()
                self.current_time = rospy.get_time()

                xy = self.find_lable(decryption_dataMatrix[FrameType_datamatrix.LABLE.value])

                label_x = int(xy[FrameType_datamatrix.X_LABLE.value])
                label_y = int(xy[FrameType_datamatrix.Y_LABLE.value])
                absolute_x = int(decryption_dataMatrix[FrameType_datamatrix.X_ABSOLUTE.value])
                absolute_y = int(decryption_dataMatrix[FrameType_datamatrix.Y_ABSOLUTE.value])
                pose_x = int(decryption_dataMatrix[FrameType_datamatrix.X_POSSITION.value])
                pose_y = int(decryption_dataMatrix[FrameType_datamatrix.Y_POSSITION.value])
                theta = float(decryption_dataMatrix[FrameType_datamatrix.ANGLE_POSSITION.value])
                
                self._gls_621.lable.x = label_x
                self._gls_621.lable.y = label_y
                self._gls_621.possition.x = absolute_x
                self._gls_621.possition.y = absolute_y
                self._gls_621.absolute.x = pose_x
                self._gls_621.absolute.y = pose_y
                self._gls_621.possition.angle = self.normalize_angle_degrees_to_radians(theta)

                # Publish raw data_matrix_msgs
                self.decode_data_qr.publish(self._gls_621)
                

        except Exception as e:
            traceback.print_exc()
            # rospy.logerr("An error occurred: {}".format(str(e)))
            self.disconnect_ethernet()

    def normalize_angle_degrees_to_radians(self, angle_degrees):
        angle_radians = math.radians(angle_degrees)
        # Normalize to the range -pi to pi
        normalized_angle = math.atan2(math.sin(angle_radians), math.cos(angle_radians))
        return normalized_angle
    
    def loop(self):
        rate = rospy.Rate(self.get_io_rate)
        while not rospy.is_shutdown():
            if self.disconnect_ or self.check_disconnect():
                self.connect_ethernet()
                # rospy.logerr("Enthernet disconnect please review the portal")
            else:
                try:
                    # protocal_ = self.ethernet_protocal.recv(4096)
                    self.processing_frame()
                except Exception as e:
                    pass
                #     rospy.logerr("An error occurred: {}".format(str(e)))
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
            rospkg.RosPack().get_path("amr_config"),
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
    Gls621_driver_connect(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
