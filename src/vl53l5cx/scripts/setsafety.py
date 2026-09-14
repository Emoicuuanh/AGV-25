#!/usr/bin/env python
from pymongo import MongoClient, errors
import actionlib
import rospy
import json
import sys
import os
from safety_msgs.msg import SafetyStatus
from agv_msgs.msg import *
import rospkg
from std_stamped_msgs.msg import (
    StringStamped,
    StringAction,
    StringFeedback,
    StringResult,
    StringGoal,
    EmptyStamped,
)

agv_mongodb_dir = os.path.join(rospkg.RosPack().get_path("agv_mongodb"), "scripts")
if not os.path.isdir(agv_mongodb_dir):
    agv_mongodb_dir = os.path.join(rospkg.RosPack().get_path("agv_mongodb"), "release")
sys.path.insert(0, agv_mongodb_dir)

from mongodb import mongodb


class SetAreaSafety:

    idirection = None

    def __init__(self, db):
        self.db = db
        self.set_safety_job_client = actionlib.SimpleActionClient(
            "set_safety_job", StringAction
        )
        self.set_footprint_client = actionlib.SimpleActionClient(
            "set_footprint", StringAction
        )
        self.set_footprint("amr_run_alone")
        self.poll()

    def init_ros(self):
        rospy.Subscriber(
            "/followline_control", FollowLineControl, self.followline_control_cb
        )

    def followline_control_cb(self, msg):
        self.idirection = msg.Direction

    def set_footprint(self, footprint):
        footprint = self.db.getFootprint(footprint)
        if footprint != None:
            footprint = json.dumps(footprint, indent=2, sort_keys=True)
            footprint = json.loads(footprint)
            footprint = json.dumps({"footprint": footprint})
            goal = StringGoal()
            try:
                # print("set_footprint: {}".format(footprint))
                goal.data = json.dumps({"params": json.loads(footprint)})
                self.set_footprint_client.send_goal(goal)
                # rospy.logwarn("Set footprint success")
            except Exception as e:
                rospy.logerr("set_footprint: {}".format(e))
        return footprint

    def set_safety_job(self, name_safety):
        name_safety = self.db.getSafety(name_safety)
        if name_safety != None:
            name_safety = json.dumps(name_safety, indent=2, sort_keys=True)
            name_safety = json.loads(name_safety)
            name_safety = json.dumps({"safety": name_safety})
            goal = StringGoal()
            try:
                # print("set_safety_job: {}".format(name_safety))
                goal.data = json.dumps({"params": json.loads(name_safety)})
                self.set_safety_job_client.send_goal(goal)
                rospy.logwarn("Set safety job success")
            except Exception as e:
                rospy.logerr("set_safety_job: {}".format(e))
        return name_safety

    def poll(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            t = rospy.get_time()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("User_safety", anonymous=True)
    rospy.loginfo("Init node :{}".format(rospy.get_name()))

    db = mongodb("mongodb://coffee:coffee@localhost:27017")
    try:
        db = mongodb("mongodb://coffee:coffee@localhost:27017")
    except errors.ServerSelectionTimeoutError as err:
        print("pymongo ERROR:", err)

    Set = SetAreaSafety(db)
