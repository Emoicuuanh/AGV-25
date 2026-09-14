#! /usr/bin/env python

from std_stamped_msgs.msg import StringAction, StringGoal
import json
import sys, os
import rospy
import rospkg
import actionlib

path = os.path.join(rospkg.RosPack().get_path("charging_process"), "json_template")

try:
    mission_file = os.path.join(path, sys.argv[1])
except:
    rospy.logwarn("Did not input mission file. Using default")
    mission_file = os.path.join(path, "charging.json")

rospy.loginfo("Action file: %s"%mission_file)

def feedback(fb):
    rospy.loginfo("Feedback: %s"%fb)

rospy.init_node("charging_client", log_level=rospy.DEBUG)

charging_client = actionlib.SimpleActionClient("auto_charging", StringAction)

charging_client.wait_for_server()

mgoal = StringGoal()

with open(mission_file) as j:
    data = json.load(j)

rospy.loginfo("Send mission: \n%s"%(json.dumps(data, indent=2)))

mgoal.data = json.dumps(data, sort_keys=True)

charging_client.send_goal(mgoal, feedback_cb=feedback)
charging_client.wait_for_result()