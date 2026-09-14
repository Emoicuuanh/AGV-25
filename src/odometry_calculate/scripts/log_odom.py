#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
import json
from math import pi
from std_stamped_msgs.msg import Int8Stamped, StringStamped, EmptyStamped, UInt32Stamped
from agv_msgs.msg import EncoderDifferential

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
)

HINT = 0


class OdomLog:
    def __init__(self):
        self.init_varialble()
        self.init_ros()
        self.poll()

    def init_ros(self):
        self.mode_status_pub = rospy.Publisher(
            "~module_status", StringStamped, queue_size=10
        )
        self.record_odom_pub = rospy.Publisher(
            "log_distance_move", UInt32Stamped, queue_size=10
        )
        rospy.Subscriber(
            "/motor_encoder", EncoderDifferential, self.motor_encoder_cb
        )

    def init_varialble(self):
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.075)
        self.encoder_resolution = rospy.get_param("~encoder_resolution", 1024)
        TICK_PER_ROUND = float(self.encoder_resolution)
        self.tick_per_meter = (
            1.0 / (self.wheel_radius * 2 * pi) * TICK_PER_ROUND
        )

        # Mongodb
        db_address = rospy.get_param("/mongodb_address")
        self.db = mongodb(db_address)
        self.odom_data = json.loads(self.db.loadOdom(["left", "right"]))
        rospy.loginfo(self.odom_data)

        # Others
        self.first_cb = True
        self.origin_left_tick = 0
        self.origin_right_tick = 0
        self.last_left_tick = 0
        self.last_right_tick = 0
        self.last_left_round = 0
        self.last_right_round = 0
        self.left_real_distance = 0
        self.right_real_distance = 0
        self.odom_count_msg = UInt32Stamped()
        self.total_odom = self.odom_data["left"]

    def motor_encoder_cb(self, msg):
        if HINT:
            msg = EncoderDifferential()
        if msg.left == 0 and msg.right == 0:
            self.first_cb = True
        if self.first_cb:
            self.first_cb = False
            self.last_left_tick = msg.left
            self.last_right_tick = msg.right

        diff_left_tick = abs(msg.left - self.last_left_tick)
        diff_right_tick = abs(msg.right - self.last_right_tick)
        left_distance = diff_left_tick / self.tick_per_meter
        right_distance = diff_right_tick / self.tick_per_meter
        # Distance from AGV start, not yet + value in DB
        self.left_real_distance += left_distance
        self.right_real_distance += right_distance

        # Round up to metter
        round_left_distance = round(self.left_real_distance)
        round_right_distance = round(self.right_real_distance)
        if (
            round_left_distance != self.last_left_round
            or round_right_distance != self.last_right_round
        ):
            self.db.saveOdom(
                ["left", "right"],
                [
                    self.odom_data["left"] + round_left_distance,
                    self.odom_data["right"] + round_right_distance,
                ],
            )
            # rospy.loginfo(
            #     "left: {}, right: {}".format(
            #         self.odom_data["left"] + round_left_distance,
            #         self.odom_data["right"] + round_right_distance,
            #     )
            # )
            self.total_odom = self.odom_data["left"] + round_left_distance
            self.last_left_round = round_left_distance
            self.last_right_round = round_right_distance

        self.last_left_tick = msg.left
        self.last_right_tick = msg.right

    def poll(self):
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.mode_status_pub.publish(StringStamped(stamp=rospy.Time.now()))
            self.odom_count_msg.stamp = rospy.Time.now()
            self.odom_count_msg.data = int(self.total_odom)
            self.record_odom_pub.publish(self.odom_count_msg)
            r.sleep()


def main():
    rospy.init_node("log_odom")
    rospy.loginfo("Init node: " + rospy.get_name())
    OdomLog()


if __name__ == "__main__":
    main()
