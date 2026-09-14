#!/usr/bin/env python

import roslib
import numpy
import rospy
import sys
import serial
from std_msgs.msg import (
    String,
    Int32,
    Int16MultiArray,
    MultiArrayLayout,
    MultiArrayDimension,
)
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

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


def main(args):
    pub = rospy.Publisher(
        "/fastech_control_multiarray",
        Int16MultiArray,
        queue_size=10,
    )
    rospy.init_node("ca_serial")
    frequency = 25
    array_input = Int16MultiArray()
    i = 0
    len = 8
    while not rospy.is_shutdown():
        array_input.data = [0] * len
        array_input.data[i] = 1
        i += 1
        if i > len - 1:
            i = 0
        pub.publish(array_input)
        rospy.sleep(1 / frequency)


if __name__ == "__main__":
    import sys, getopt

    main(sys.argv)
