#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
from geometry_msgs.msg import Twist
import getch


class my_class:
    def __init__(self):
        self.ctrl_c = False
        self.rate = rospy.Rate(10)  # 10hz
        rospy.on_shutdown(self.shutdownhook)

    def input(self):
        keyboard_value = ord(getch.getch())
        print(keyboard_value)
        if keyboard_value == "49":
            self.publish_once_in_cmd_vel()

    def publish_once_in_cmd_vel(self):
        rate = rospy.Rate(30)
        while not self.ctrl_c:
            rate.sleep()

    def shutdownhook(self):
        self.ctrl_c = True


if __name__ == "__main__":
    rospy.init_node("class_test", anonymous=True)
