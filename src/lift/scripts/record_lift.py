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


class LiftLog:
    def __init__(self):
        self.init_varialble()
        self.init_ros()
        self.poll()

    def init_ros(self):
        self.mode_status_pub = rospy.Publisher(
            "~module_status", StringStamped, queue_size=10
        )
        rospy.Subscriber("/standard_io", StringStamped, self.arduino_cb)
        self.record_lift_pub = rospy.Publisher(
            "log_lift", UInt32Stamped, queue_size=10
        )
    def init_varialble(self):
        # Mongodb
        db_address = rospy.get_param("/mongodb_address")
        self.db = mongodb(db_address)
        try:
            self.lift_data = json.loads(self.db.loadLift("cycle"))
        except:
            self.lift_data = 0
        rospy.loginfo("total lift last: {}".format(self.lift_data))
        self.lift_data = json.loads(self.db.loadLift("cycle"))
        self.lift_count_msg = UInt32Stamped()

        # Others
        self.first_cb = True
        self.pre_lift_up = None
        self.pre_lift_down = None
        self.lift_up = None
        self.lift_down = None
        self.counter = 0
        self.pre_counter = 0
        self.total_lift = self.lift_data["cycle"]

    def arduino_cb(self, msg):
        data = json.loads(msg.data)
        if self.first_cb:
            self.first_cb = False
            if "lift_max_sensor" in data:
                self.pre_lift_up = data["lift_max_sensor"]
            if "lift_min_sensor" in data:
                self.pre_lift_down = data["lift_min_sensor"]
        else:
            if "lift_max_sensor" in data:
                self.lift_up = data["lift_max_sensor"]
            if "lift_min_sensor" in data:
                self.lift_down = data["lift_min_sensor"]
            if self.pre_lift_down != self.lift_down:
                self.counter += 1
                self.pre_lift_down = self.lift_down
            elif self.pre_lift_up != self.lift_up:
                self.counter += 1
                self.pre_lift_up = self.lift_up

        if (self.counter // 4) != self.pre_counter:
            self.db.saveLift(
                "cycle",
                self.total_lift + self.counter // 4,
            )
            rospy.loginfo(
                "lift total: {}".format(
                    self.total_lift + self.counter // 4,
                )
            )
            self.pre_counter = self.counter // 4
            # self.total_lift = self.total_lift + self.counter // 4

    def poll(self):
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.mode_status_pub.publish(StringStamped(stamp=rospy.Time.now()))
            self.lift_count_msg.stamp = rospy.Time.now()
            # print(self.total_lift)
            self.lift_count_msg.data = int(self.total_lift + self.counter // 4)
            self.record_lift_pub.publish(self.lift_count_msg)
            r.sleep()


def main():
    rospy.init_node("log_lift")
    rospy.loginfo("Init node: " + rospy.get_name())
    LiftLog()


if __name__ == "__main__":
    main()
