#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
from std_stamped_msgs.msg import Int8Stamped, StringStamped, EmptyStamped
from std_msgs.msg import Int64, Int16, Int8, String, Empty

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, sin, cos, atan


class JoystickHandle:
    def __init__(self):
        self.init_ros()
        self.init_varialble()
        self.poll()

    def init_ros(self):
        rospy.init_node("joystick_handle")
        rospy.loginfo("Init node: " + rospy.get_name())
        self.pub_lift_cmd = rospy.Publisher(
            "/lift_cart", Int8Stamped, queue_size=10
        )
        self.safety_status_disable = rospy.Publisher(
            "/safety_disable", Int8, queue_size=10
        )
        rospy.Subscriber("/joy", Joy, self.joy_cb)

    def init_varialble(self):
        self.lift_msg = Int8Stamped()
        self.lift_up = 1
        self.lift_down = 2
        self.lift_stop = 3
        self.button_lift_up = 3  # Y
        self.button_lift_down = 1  # A
        self.button_disable_safety = 2  # B

        # self.button_reset = 0  # X
        # self.button_run = 9  # START
        # self.button_pause = 8  # BACK

    def joy_cb(self, joy):
        self.buttons_joy = joy.buttons
        if (
            self.buttons_joy[self.button_lift_up]
            and not self.buttons_joy[self.button_lift_down]
        ):
            self.lift_msg.stamp = rospy.Time.now()
            self.lift_msg.data = self.lift_up
            self.pub_lift_cmd.publish(self.lift_msg)
        elif (
            self.buttons_joy[self.button_lift_down]
            and not self.buttons_joy[self.button_lift_up]
        ):
            self.lift_msg.stamp = rospy.Time.now()
            self.lift_msg.data = self.lift_down
            self.pub_lift_cmd.publish(self.lift_msg)
        # else:
        #     self.lift_msg.stamp = rospy.Time.now()
        #     self.lift_msg.data = self.lift_stop
        #     self.pub_lift_cmd.publish(self.lift_msg)
        if self.buttons_joy[self.button_disable_safety]:
            self.safety_status_disable.publish(Int8(1))
        else:
            self.safety_status_disable.publish(Int8(0))

    def poll(self):
        rospy.spin()


def main():
    JoystickHandle()


if __name__ == "__main__":
    main()
