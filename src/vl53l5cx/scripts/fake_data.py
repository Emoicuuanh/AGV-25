#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
from re import A
import os
import json
import sys
import rospy
import rospkg
import serial
import time
import re
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import numpy as np
import pandas as pd
import math
from std_msgs.msg import Int64, Int16, Int8, String, Empty
from vl53l5cx.msg import Vl53l5cxRanges, Safety_Esp, vl53l5cx_safety
from safety_msgs.msg import SafetyStatus
from agv_msgs.msg import *

common_folder = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_folder):
    common_folder = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_folder)
from common_function import find_soft_port

from common_function import (
    MIN_FLOAT,
    EnumString,
)


class Vl53l5cxState(EnumString):
    NONE = -1
    FORWARD = 0
    BACKWARD = 1
    ALL = 2


class AutoPub:
    def __init__(self, *args, **kwargs):
        self.init_variable(*args, **kwargs)

        rospy.init_node("Publish_FakeData", anonymous=True)
        rospy.loginfo("Init node " + rospy.get_name())

        self.vl53l5cx_fake_senser1 = [0] * 64
        self.vl53l5cx_fake_senser2 = [0] * 64
        self.vl53l5cx_fake_senser3 = [0] * 64
        self.vl53l5cx_fake_senser4 = [0] * 64
        self._distance1 = [1000] * 64
        self._distance2 = [2000] * 64
        self._distance3 = [3000] * 64
        self.vl53dir = 2

        self.safety_status = SafetyStatus()
        self.safety_status.fields = [1] * 3
        # Publisher
        self.pub_senser1 = rospy.Publisher(
            "/vl35l5cx_r1", Vl53l5cxRanges, queue_size=5
        )
        self.pub_senser2 = rospy.Publisher(
            "/vl35l5cx_r2", Vl53l5cxRanges, queue_size=5
        )
        self.pub_senser3 = rospy.Publisher(
            "/vl35l5cx_r3", Vl53l5cxRanges, queue_size=5
        )
        self.pub_senser4 = rospy.Publisher(
            "/vl35l5cx_r4", Vl53l5cxRanges, queue_size=5
        )
        self.safety_status_pub = rospy.Publisher(
            "/safety_status", SafetyStatus, queue_size=10
        )
        # sub
        rospy.Subscriber("/vldir", Int8, self.vl53l5cx_dir_cb)

        self.open_json()
        self.loop()

    def init_variable(self, *args, **kwargs):
        self.config_path = kwargs["config_path"]

    def pub_fake_data(self):
        Lidarvl53l5cx_1_msg = Vl53l5cxRanges()
        Lidarvl53l5cx_2_msg = Vl53l5cxRanges()
        Lidarvl53l5cx_3_msg = Vl53l5cxRanges()
        Lidarvl53l5cx_4_msg = Vl53l5cxRanges()

        Lidarvl53l5cx_1_msg.range = [1720 ,1685 ,1639, 1662 ,1705 ,1684, 1738, 1742, 1617 ,1625, 1648, 1637, 1659 ,1642, 1677, 1711, 1625, 1600, 1608, 1618, 1632, 1665 ,1649 ,1662, 1589, 1601, 1609 ,1618, 1616, 1610 ,1648 ,1654 ,1548 ,1573 ,1576, 1581 ,1621, 1598 ,1614, 1643 ,1550, 1544 ,1545 ,1576 ,1585 ,1539, 1585, 1609 ,1512 ,1538 ,1543 ,1560 ,1549 ,1556 ,1566 ,1549 ,1513 ,1506 ,1511 ,1519, 1533 ,1533 ,1553 ,1562]
        Lidarvl53l5cx_2_msg.range = [1720 ,1635 ,1661 ,1661, 1719 ,1696 ,1680 ,1576, 1645, 1625 ,1637 ,1645, 1662 ,1666 ,1668, 1714 ,1606 ,1616 ,1593 ,1605 ,1617 ,1670, 1662 ,1688 ,1583 ,1579 ,1614 ,1622, 1616 ,1613 ,1642 ,1619 ,1561, 1567, 1570 ,1607 ,1593 ,1588 ,1615 ,1627 ,1526 ,1528 ,1561 ,1572 ,1580 ,1575 ,1576 ,1582 ,1528 ,1515 ,1544 ,1540 ,1552 ,1547 ,1590 ,1595 ,1522 ,1513 ,1525 ,1540 ,1528 ,1532 ,1570 ,1544]
        Lidarvl53l5cx_3_msg.range = self.vl53l5cx_fake_senser3
        Lidarvl53l5cx_4_msg.range = self.vl53l5cx_fake_senser4
        # # print(Lidarvl53l5cx_2_msg.range)
        # # print(list(self.vl53l5cx_fake_senser4))
        # if self.vl53dir == Vl53l5cxState.FORWARD.value:
        #     self.pub_senser1.publish(Lidarvl53l5cx_1_msg)
        #     self.pub_senser2.publish(Lidarvl53l5cx_2_msg)
        # if self.vl53dir == Vl53l5cxState.BACKWARD.value:
        #     self.pub_senser3.publish(Lidarvl53l5cx_3_msg)
        #     self.pub_senser4.publish(Lidarvl53l5cx_4_msg)
        # if self.vl53dir == Vl53l5cxState.ALL.value:
        #     self.pub_senser1.publish(Lidarvl53l5cx_1_msg)
        #     self.pub_senser2.publish(Lidarvl53l5cx_2_msg)
        #     self.pub_senser3.publish(Lidarvl53l5cx_3_msg)
        #     self.pub_senser4.publish(Lidarvl53l5cx_4_msg)

        self.safety_status.fields[0] = 0
        self.safety_status.fields[1] = 0
        self.safety_status.fields[2] = 0
        self.safety_status_pub.publish(self.safety_status)

    def vl53l5cx_dir_cb(self, msg):
        self.vl53dir = msg.data

    def open_json(self):
        """Open Json file."""
        with open(self.config_path, "r") as f:
            data = json.load(f)
        self.df = pd.DataFrame(data)

        # self.vl53l5cx_fake_senser1 = self.df.VL53L5_FakeData
        self.vl53l5cx_fake_senser1 = self._distance2
        self.vl53l5cx_fake_senser2 = self._distance2
        self.vl53l5cx_fake_senser3 = self._distance1
        self.vl53l5cx_fake_senser4 = self._distance1

    def loop(self):
        # rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.pub_fake_data()
            rospy.sleep(0.01)


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
            "fake_data.json",
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
    AutoPub(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
