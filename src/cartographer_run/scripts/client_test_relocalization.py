#!/usr/bin/env python
import rospy
import rospkg
import os
import sys
import actionlib
import numpy as np
import copy
import json

from std_stamped_msgs.msg import StringAction, StringStamped, StringResult, StringFeedback, StringGoal, Int8Stamped, EmptyStamped
from geometry_msgs.msg import Pose

common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)

from common_function import (
    EnumString,
    lockup_pose,
    obj_to_dict,
    offset_pose_x,
    pose_stamped_array_to_pose_array,
    distance_two_pose,
    get_yaw,
    delta_angle,
    dict_to_obj,
    merge_two_dicts,
    print_debug,
    print_warn,
    print_error,
    print_info,
    MIN_FLOAT,
    distance_to_line_perpendicular_vs_goal,
    angle_robot_vs_robot_to_goal,
    yaw_to_quaternion,
    pose_dict_template,
    angle_vector_two_point,
)

class Test:
    def __init__(self):
        # Initialize action client
        self.action_client_relocaltion = actionlib.SimpleActionClient('/relocaization_cartographer_server', StringAction)

        # Prepare action goal
        self.action_goal = StringGoal()
        pose_init = Pose()

        goal = {}
        goal["params"] = {}

        # Set initial pose
        pose_init.position.x = 0
        pose_init.position.y = 0
        pose_init.position.z = 0
        pose_init.orientation.x = 0
        pose_init.orientation.y = 0
        pose_init.orientation.z = 0
        pose_init.orientation.w = 1

        goal["params"]["init_pose"] = obj_to_dict(
            pose_init,
            copy.deepcopy(pose_dict_template),
        )

        self.action_goal.data = json.dumps(goal)

        # Wait for action server and send goal
        self.action_client_relocaltion.wait_for_server()
        rospy.loginfo("Connected to action server")
        self.action_client_relocaltion.send_goal(self.action_goal)

        # Wait for result and print it
        self.wait_and_print_result()

    def wait_and_print_result(self):
        """
        Wait for the action result and print the outcome.
        """
        # Wait for the action to complete with a timeout (e.g., 30 seconds)
        finished = self.action_client_relocaltion.wait_for_result(rospy.Duration(120))

        if finished:
            result = self.action_client_relocaltion.get_result()
            state = self.action_client_relocaltion.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Action succeeded with status: %s", result.status)
            else:
                rospy.logwarn("Action failed with state: %s, status: %s", state, result.status)
        else:
            rospy.logerr("Action did not complete within the timeout period")
            self.action_client_relocaltion.cancel_goal()

if __name__ == '__main__':
    rospy.init_node('test', anonymous=True)
    test = Test()
    rospy.spin()