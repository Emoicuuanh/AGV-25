#!/usr/bin/env python
import sys
import rospy
from safety_msgs.msg import SafetyStatus
from std_msgs.msg import Time, Int16, Int32
from std_stamped_msgs.msg import StringStamped
from vl53l5cx.msg import Vl53l5cxRanges, Safety_Esp, vl53l5cx_safety
from safety_msgs.msg import SafetyStatus
import message_filters
import numpy as np

class Vl53l5cx():
    def __init__(self):
        _log_level = rospy.INFO
        if 'debug' in sys.argv:
            _log_level = rospy.DEBUG

        rospy.init_node("vl53l5cx_node", log_level=_log_level)
        rospy.loginfo("Init node " + rospy.get_name())
        self.left_sub = message_filters.Subscriber('/vl35l5cx_ranges_left', Vl53l5cxRanges)
        self.right_sub= message_filters.Subscriber('/vl35l5cx_ranges_right', Vl53l5cxRanges) 
        self.pub_result = rospy.Publisher('/vl53l5_status_node', vl53l5cx_safety, queue_size=10)
        self.safety_status_pub = rospy.Publisher('/safety_status', SafetyStatus, queue_size=10)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.left_sub, self.right_sub], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.callback)

        self.vl53l53cx_status = vl53l5cx_safety()
        self.safety_status = SafetyStatus()

        self.safety_status.fields = [1] * 3
        self.vl53l53cx_status.status_left = False
        self.vl53l53cx_status.status_right = False
        self.loop()
    
    def callback(self, vl35l5cx_ranges_left, vl35l5cx_ranges_right):
        self.time_out = rospy.get_time()
        # eliminate 0.0 on range
        self.range_left = list(filter(lambda x: x != 0.0, vl35l5cx_ranges_left.range))
        self.range_right = list(filter(lambda x: x != 0.0, vl35l5cx_ranges_right.range))
        # check on sensor
        self.status_sensor_left = vl35l5cx_ranges_left.status
        self.status_sensor_right = vl35l5cx_ranges_right.status

        if not self.status_sensor_left or not self.status_sensor_right:
            if rospy.get_time() - self.time_out > 2 :
                self.vl53l53cx_status.status_right = False
                self.vl53l53cx_status.status_left = False
            else :
                for i in self.range_left[0:38]:
                    if i < 520: 
                        self.vl53l53cx_status.status_left = False
                    else:
                        self.vl53l53cx_status.status_left = True 
                for i in self.range_right[0:30]:
                    if i < 620:
                        self.vl53l53cx_status.status_right = False 
                    else:
                        self.vl53l53cx_status.status_right = True
        
        self.vl53l53cx_status.header.stamp = rospy.Time.now()
        self.pub_result.publish(self.vl53l53cx_status)

    def PubSafetystatus (self):
        # print("right {}".format(self.vl53l53cx_status.status_right))
        # print("left {}".format(self.vl53l53cx_status.status_left))
        if self.vl53l53cx_status.status_left == False or self.vl53l53cx_status.status_right  == False:
            self.safety_status.fields[0] = 1
            self.safety_status.fields[1] = 1
            self.safety_status.fields[2] = 1
        elif self.vl53l53cx_status.status_left == True or self.vl53l53cx_status.status_right  == True:
            self.safety_status.fields[0] = 0
            self.safety_status.fields[1] = 0
            self.safety_status.fields[2] = 0 
        self.safety_status_pub.publish(self.safety_status)

    def loop(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.PubSafetystatus ()
            rate.sleep()

if __name__ == "__main__":
    vl53l5cx_node = Vl53l5cx()