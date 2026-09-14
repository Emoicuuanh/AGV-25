#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
from scan_safety.srv import SafetyEnableRequest, SafetyEnableResponse


if __name__ == "__main__":

    # Service
    safety_enable = rospy.ServiceProxy('enable_safety', SafetyEnableRequest)
    rospy.loginfo("Waiting safety service active...")
    rospy.wait_for_service('enable_safety')
    set_result = safety_enable(False)
    rospy.loginfo("Safety disable result: {}".format(str(set_result.success)))