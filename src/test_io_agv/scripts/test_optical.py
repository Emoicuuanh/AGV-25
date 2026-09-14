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
        "/fastech_control_multiarray", Int16MultiArray, queue_size=10
    )
    rospy.init_node("ca_serial")
    r = rospy.Rate(0.2)
    array_input = Int16MultiArray()
    while not rospy.is_shutdown():
        for i in range(8):
            list = [0, 0, 0, 0, 0, 0, 0, 0]
            list[i] = 1
            array_input.data = list
            pub.publish(array_input)
            rospy.sleep(1)
        r.sleep()


if __name__ == "__main__":
    import sys, getopt

    main(sys.argv)
