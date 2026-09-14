#! /usr/bin/env python
# -*- coding: utf-8 -*-
from pyclbr import Function
import yaml
import os
import sys
import rospy
import rospkg
import numpy as np
from math import *
from agv_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
import copy
import json
import dynamic_reconfigure.client
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import (
    Twist,
    Pose,
    PoseStamped,
    PoseWithCovarianceStamped,
)
from std_msgs.msg import Bool, Int16, String, Int16MultiArray
from math import pi, sqrt
from tf.listener import TransformListener, Transformer
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros

common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)
from common_function import make_transform_stamped, angle_two_pose
from std_stamped_msgs.msg import (
    StringAction,
    StringStamped,
    StringResult,
    StringFeedback,
    StringGoal,
    Int8Stamped,
    EmptyStamped,
    Float32Stamped,
    Int16MultiArrayStamped,
)
from cognex_qr_code.srv import *
from agv_msgs.msg import ArduinoIO, DigitalSensor, FollowLineSensor, LedControl
from agv_msgs.msg import (
    FollowLineSensor,
    DiffDriverMotorSpeed,
    EncoderDifferential,
)
from common_function import (
    EnumString,
    lockup_pose,
    dict_to_obj,
    merge_two_dicts,
    print_debug,
    print_info,
    print_warn,
    print_error,
    obj_to_dict,
    angle_robot_vs_robot_to_goal,
    distance_two_pose,
    YamlDumper,
)

SENSOR_DEACTIVATE = 0
SENSOR_ACTIVATE = 1


class MainState(EnumString):
    NONE = -1
    DONE = 0
    CHECK_FOLLOWLINE = 1
    CHECK_CARD_ID = 2
    CHECK_MOTOR = 3
    CHECK_QR_CODE = 4


class TEST_IO_AGV(object):
    def __init__(self, name, *args, **kwargs):
        # """
        # ..######..##.....##.########...######..########..####.########..########.########.
        # .##....##.##.....##.##.....##.##....##.##.....##..##..##.....##.##.......##.....##
        # .##.......##.....##.##.....##.##.......##.....##..##..##.....##.##.......##.....##
        # ..######..##.....##.########..##.......########...##..########..######...########.
        # .......##.##.....##.##.....##.##.......##...##....##..##.....##.##.......##...##..
        # .##....##.##.....##.##.....##.##....##.##....##...##..##.....##.##.......##....##.
        # ..######...#######..########...######..##.....##.####.########..########.##.....##
        # """
        rospy.Subscriber("/card_id", String, self.card_id_cb)
        rospy.Subscriber("/standard_io", StringStamped, self.standard_io_cb)
        rospy.Subscriber(
            "/followline_sensor", FollowLineSensor, self.followline_cb
        )
        rospy.Subscriber(
            "/motor_encoder", EncoderDifferential, self.motor_encoder_cb
        )

        # """
        # .########..##.....##.########..##.......####..######..##.....##.########.########.
        # .##.....##.##.....##.##.....##.##........##..##....##.##.....##.##.......##.....##
        # .##.....##.##.....##.##.....##.##........##..##.......##.....##.##.......##.....##
        # .########..##.....##.########..##........##...######..#########.######...########.
        # .##........##.....##.##.....##.##........##........##.##.....##.##.......##...##..
        # .##........##.....##.##.....##.##........##..##....##.##.....##.##.......##....##.
        # .##.........#######..########..########.####..######..##.....##.########.##.....##
        # """
        self.pub_led_turn = rospy.Publisher(
            "/arduino_driver/led_turn", Int16MultiArrayStamped, queue_size=10
        )
        self.fastech_control_pub = rospy.Publisher(
            "/fastech_control_multiarray", Int16MultiArray, queue_size=10
        )
        self.pub_led_status = rospy.Publisher(
            "/led_status", StringStamped, queue_size=5, latch=True
        )

        # """
        # ..######..########.########..##.....##.####..######..########
        # .##....##.##.......##.....##.##.....##..##..##....##.##......
        # .##.......##.......##.....##.##.....##..##..##.......##......
        # ..######..######...########..##.....##..##..##.......######..
        # .......##.##.......##...##....##...##...##..##.......##......
        # .##....##.##.......##....##....##.##....##..##....##.##......
        # ..######..########.##.....##....###....####..######..########
        # """
        self.get_qr_code = rospy.ServiceProxy("ReadQrCode", QrCode)

        self.init_variable(*args, **kwargs)
        self.loop()

    def yaml_load(self, filepath):
        try:
            with open(filepath, "r") as read_file:
                data = yaml.load(read_file, Loader=yaml.FullLoader)
            return data
        except Exception as e:
            rospy.logerr("Load yaml file error: {}".format(e))

    def init_variable(self, *args, **kwargs):
        self.rate = rospy.Rate(10)
        config_file_led = kwargs["config_file_led"]
        rospy.loginfo("config_file_led: %s" % config_file_led)
        led_effect_config = self.yaml_load(config_file_led)
        self.led_effect_list = list(led_effect_config["led_effect"])
        self.led_turn = [0, 0, 0, 0]
        self.liftup_finish = False
        self.liftdown_finish = False
        self.emg_status = False
        self.start_1 = False
        self.start_2 = False
        self.stop_1 = False
        self.stop_2 = False
        self.detect_vrack = False
        self.bumper = False
        self.pre_liftup_finish = False
        self.pre_liftdown_finish = False
        self.pre_emg_status = False
        self.pre_start_1 = False
        self.pre_start_2 = False
        self.pre_stop_1 = False
        self.pre_stop_2 = False
        self.pre_detect_vrack = False
        self.pre_bumper = False
        self.led_number = 0
        self.led_turn_msg = Int16MultiArrayStamped()
        self.led_msg = StringStamped()
        self.optical_msg = Int16MultiArray()
        self.followline = ""
        self.card_id = ""
        self.motor = ""
        self.qr_code = ""

    def shutdown(self):
        rospy.loginfo("Shuting down")

    """
    .########.##.....##.##....##..######..########.####..#######..##....##
    .##.......##.....##.###...##.##....##....##.....##..##.....##.###...##
    .##.......##.....##.####..##.##..........##.....##..##.....##.####..##
    .######...##.....##.##.##.##.##..........##.....##..##.....##.##.##.##
    .##.......##.....##.##..####.##..........##.....##..##.....##.##..####
    .##.......##.....##.##...###.##....##....##.....##..##.....##.##...###
    .##........#######..##....##..######.....##....####..#######..##....##
    """

    def loop(self):
        _state = MainState.CHECK_FOLLOWLINE
        _prev_state = MainState.NONE
        while not rospy.is_shutdown():
            if _prev_state != _state:
                rospy.loginfo(
                    "MODULE CHECK STATE: {} -> {}".format(
                        _prev_state.toString(), _state.toString()
                    )
                )
                _prev_state = _state
            if _state == MainState.CHECK_FOLLOWLINE:
                if self.followline != "":
                    rospy.logwarn("FOLLOWLINE OK")
                    _state = MainState.CHECK_MOTOR
            elif _state == MainState.CHECK_MOTOR:
                if self.motor != "":
                    rospy.logwarn("MOTOR OK")
                    _state = MainState.CHECK_CARD_ID
            elif _state == MainState.CHECK_CARD_ID:
                if self.card_id != "":
                    rospy.logwarn("RFID OK")
                    _state = MainState.CHECK_QR_CODE
            elif _state == MainState.CHECK_QR_CODE:
                resp = self.get_qr_code(2)
                self.qr_code = resp.Res
                if self.qr_code != "":
                    rospy.logwarn("QR CODE OK")
                    _state = MainState.DONE
            self.led_turn_control()
            self.led_test()
            # self.optical_sensor()
            self.rate.sleep()

    def led_turn_control(self):
        if self.start_1:
            self.led_turn_msg.data = [0, 1, 0, 0]
        elif self.stop_1:
            self.led_turn_msg.data = [1, 0, 0, 0]
        elif self.start_2:
            self.led_turn_msg.data = [0, 0, 0, 1]
        elif self.stop_2:
            self.led_turn_msg.data = [0, 0, 1, 0]
        elif self.detect_vrack:
            self.led_turn_msg.data = [1, 1, 0, 0]
        elif not self.emg_status:
            self.led_turn_msg.data = [1, 1, 1, 1]
        elif not self.bumper:
            self.led_turn_msg.data = [0, 0, 1, 1]
        else:
            self.led_turn_msg.data = [0, 0, 0, 0]
        self.led_turn_msg.stamp = rospy.Time.now()
        self.pub_led_turn.publish(self.led_turn_msg)

    def led_test(self):
        if (
            self.start_1 == SENSOR_ACTIVATE
            and self.pre_start_1 == SENSOR_DEACTIVATE
            or self.start_2 == SENSOR_ACTIVATE
            and self.pre_start_2 == SENSOR_DEACTIVATE
        ):
            self.led_number += 1
        if (
            self.stop_1 == SENSOR_ACTIVATE
            and self.pre_stop_1 == SENSOR_DEACTIVATE
            or self.stop_2 == SENSOR_ACTIVATE
            and self.pre_stop_2 == SENSOR_DEACTIVATE
        ):
            self.led_number -= 1
        if self.led_number < 0:
            self.led_number = len(self.led_effect_list) - 1
        elif self.led_number > len(self.led_effect_list) - 1:
            self.led_number = 0
        self.pre_start_1 = self.start_1
        self.pre_start_2 = self.start_2
        self.pre_stop_1 = self.stop_1
        self.pre_stop_2 = self.stop_1
        self.led_msg.stamp = rospy.Time.now()
        self.led_msg.data = self.led_effect_list[self.led_number]
        self.pub_led_status.publish(self.led_msg)

    def optical_sensor(self):
        for i in range(8):
            list = [0, 0, 0, 0, 0, 0, 0, 0]
            list[i] = 1
            self.optical_msg.data = list
            self.fastech_control_pub.publish(self.optical_msg)
            rospy.sleep(1)

    """
     ######     ###    ##       ##       ########     ###     ######  ##    ##
    ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
    ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
    ##       ##     ## ##       ##       ########  ##     ## ##       #####
    ##       ######### ##       ##       ##     ## ######### ##       ##  ##
    ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
     ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
    """

    def standard_io_cb(self, msg):
        data = json.loads(msg.data)
        self.liftup_finish = data["lift_max_sensor"]
        self.liftdown_finish = data["lift_min_sensor"]
        self.emg_status = data["emg_button"]
        self.start_1 = data["start_1_button"]
        self.start_2 = data["start_2_button"]
        self.stop_1 = data["stop_1_button"]
        self.stop_2 = data["stop_2_button"]
        self.detect_vrack = data["detect_vrack"]
        self.bumper = data["bumper"]

    def motor_encoder_cb(self, msg):
        self.motor = msg.left

    def followline_cb(self, msg):
        FollowLineCounter = 0
        for i in range(16):
            FollowLineCounter += msg.data[i]
        if FollowLineCounter != 0:
            self.followline = msg.data

    def card_id_cb(self, msg):
        self.card_id = msg.data


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
        "-c",
        "--config_file_led",
        dest="config_file_led",
        default=os.path.join(
            rospkg.RosPack().get_path("amr_config"), "cfg", "led_effect.yaml"
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
    rospy.init_node("test_io_agv", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    TEST_IO_AGV(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
