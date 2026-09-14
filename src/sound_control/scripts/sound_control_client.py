#! /usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import sys

import rospy
# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the sound_control action, including the
# goal message and the result message.
import sound_control.msg


class SoundControlClient():
    def __init__(self):

        client = actionlib.SimpleActionClient('sound_control_server', sound_control.msg.SoundControlAction)
        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()
        # Creates a goal to send to the action server.
        goal = sound_control.msg.SoundControlGoal()
        goal.SoundName = "hello"
        goal.SoundLoop = False

        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()
        # Prints out the result of executing the action
        result = client.get_result()  # A sound_controlResult
        print("Play sound complete! {}".format(result))

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('sound_control_client')
        client = SoundControlClient()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
