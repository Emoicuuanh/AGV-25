#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def move_base_client():
    # Initialize the ROS node
    rospy.init_node('move_base_client')

    # Create an action client for the MoveBase action
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Wait for the action server to start
    client.wait_for_server()

    # Create a MoveBaseGoal object
    goal = MoveBaseGoal()

    # Fill in the goal with the desired values
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = 1.0
    goal.target_pose.pose.position.y = 2.0
    goal.target_pose.pose.orientation.w = 1.0

    # Send the goal to the action server
    client.send_goal(goal)

    # Wait for the result with a timeout (e.g., 30 seconds)
    client.wait_for_result(rospy.Duration(30.0))

    # Check if the action was successful
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Hooray, the robot reached the goal!")
    else:
        rospy.logwarn("The robot failed to reach the goal.")

if __name__ == '__main__':
    try:
        move_base_client()
    except rospy.ROSInterruptException:
        pass
