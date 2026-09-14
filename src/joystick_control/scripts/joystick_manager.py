#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
import json
from math import pi
from safety_msgs.msg import SafetyStatus
from std_stamped_msgs.msg import Int8Stamped, StringStamped, EmptyStamped
from sensor_msgs.msg import Joy
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
    Twist,
    PoseStamped,
    PoseWithCovarianceStamped,
)
common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)

agv_mongodb_dir = os.path.join(
    rospkg.RosPack().get_path("agv_mongodb"), "scripts"
)
if not os.path.isdir(agv_mongodb_dir):
    agv_mongodb_dir = os.path.join(
        rospkg.RosPack().get_path("agv_mongodb"), "release"
    )
sys.path.insert(0, agv_mongodb_dir)

from mongodb import mongodb
from module_manager import ModuleServer, ModuleStatus
from common_function import (
    EnumString,
)

class MainState(EnumString):
    NONE = -1
    STOP_BY_SAFETY = 2
    SAFETY_OFF = 3
    NORMAL = 4

class joystickControl:
    def __init__(self):
        self.init_varialble()
        self.init_ros()
        self.poll()

    def init_ros(self):
        self.mode_status_pub = rospy.Publisher(
            "~module_status", StringStamped, queue_size=10
        )
        self.safety_job_pub = rospy.Publisher(
            "/safety_job_joystick_name", StringStamped, queue_size=5
        )
        self.footprint_job_pub = rospy.Publisher(
            "/safety_footprint_name", StringStamped, queue_size=5
        )
        self.pub_vel = rospy.Publisher(
            self.vel_topic_output, Twist, queue_size=5
        )
        self.pub_joy = rospy.Publisher(self.joy_remap, Joy, queue_size=10)
        rospy.Subscriber(self.joy_origin, Joy, self.joy_cb)
        rospy.Subscriber("/standard_io", StringStamped, self.standard_io_cb)
        rospy.Subscriber("/safety_job_name", StringStamped, self.set_safety_cb)
        rospy.Subscriber(self.vel_topic_input, Twist, self.vel_cb)
        rospy.Subscriber('/safety_status', SafetyStatus, self.safety_status_cb)
        # ModuleServer
        self._asm = ModuleServer(rospy.get_name())

    def init_varialble(self):
        # Others
        self.button_manual_status = False
        self.charge_manual_status = False
        self.current_safety_job_name_auto_mode = ""
        self.current_safety_job_name_manual_mode = ""
        self.pre_safety_job_name = ""
        self.vel_x = 0
        self.vel_theta = 0
        self.footprint_job = rospy.get_param("~footprint_job", "amr_run_alone")
        self.safety_job_forward = rospy.get_param("~safety_job_forward", "joystick_forward")
        self.safety_job_backward = rospy.get_param("~safety_job_backward", "joystick_backward")
        self.safety_job_rotation = rospy.get_param("~safety_job_rotation", "joystick_rotation")
        self.invert_direct_joystick = rospy.get_param("~invert_direct_joystick", True)
        self.joy_remap = rospy.get_param("~joy_remap", "joy")
        self.joy_origin = rospy.get_param("~joy_origin", "joy_origin")
        self.vel_topic_input = rospy.get_param("~vel_topic_input", "teleop_joy_cmd_vel")
        self.vel_topic_output = rospy.get_param("~vel_topic_output", "teleop_joy_cmd_vel_safety")
        # safety
        self.last_safety_stop = rospy.get_time()
        self.safety_fields = []
        self.safety_field_idx = 0
        self.is_safety_stop = False
        self.is_safety = False
        self.disable_safety = False
        # vel limit
        self.max_vel_move_straight = [0, 0.1, 0.2, 0.4]
        self.max_vel_move_rotation = [0, 0.1, 0.2, 0.4]
        self.max_field_restrict_vel = min(len(self.max_vel_move_straight), len(self.max_vel_move_rotation))
        # state module
        self.state = MainState.NORMAL
        self.prev_state = MainState.NONE

        self.last_time_pub = rospy.get_time()


    """
     ######     ###    ##       ##       ########     ###     ######  ##    ##
    ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
    ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
    ##       ##     ## ##       ##       ########  ##     ## ##       #####
    ##       ######### ##       ##       ##     ## ######### ##       ##  ##
    ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
     ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
    """

    def joy_cb(self, msg):
        joy_data = msg
        try:
            new_axis = []
            new_buttons = []
            if joy_data.buttons[2]:
                self.disable_safety = True
            else:
                self.disable_safety = False
            if (not self.charge_manual_status and not self.button_manual_status) or joy_data.buttons[0]:
                self.pub_joy.publish(joy_data)
            else:
                for i in range(len(joy_data.axes)):
                    new_axis.append(0.0)
                for i in range(len(joy_data.buttons)):
                    new_buttons.append(0)
                joy_data.axes = new_axis
                joy_data.buttons = new_buttons
                self.pub_joy.publish(joy_data)

        except rospy.ServiceException as e:
            pass

    def safety_status_cb(self, msg):
        self.safety_fields = list(msg.fields)  # [1, 2, 3, ...]
        self.is_safety_stop = False
        self.is_safety = False
        if len(self.safety_fields) and self.safety_fields[0] == 1:
            self.is_safety_stop = True
            self.last_safety_stop = rospy.get_time()
        else:
            self.is_safety_stop = False
        # print_warn("Update safety fields: {}".format(self.safety_fields))
        if len(self.safety_fields):
            for field_idx in range(len(self.safety_fields)):
                if self.safety_fields[field_idx] == 1:
                    self.is_safety = True
                    self.safety_field_idx = field_idx
                    break

    def set_safety_cb(self, msg):
        self.current_safety_job_name_auto_mode = msg.data

    def standard_io_cb(self, msg):
        data = json.loads(msg.data)
        if "auto_manual_sw" in data: self.button_manual_status = data["auto_manual_sw"]
        if "manual_charging_detect" in data: self.charge_manual_status = data["manual_charging_detect"]

    def vel_cb(self, msg):
        _state = MainState.NORMAL
        if self.invert_direct_joystick:
            msg.linear.x = -msg.linear.x
        self.vel_remap = msg
        self.vel_x = msg.linear.x
        self.vel_theta = msg.angular.z
        if not self.disable_safety:
            if self.is_safety and self.safety_field_idx <= self.max_field_restrict_vel:
                if self.vel_x > 0:
                    self.vel_remap.linear.x = min(self.vel_x, self.max_vel_move_straight[self.safety_field_idx])
                elif self.vel_x < 0:
                    self.vel_remap.linear.x = max(self.vel_x, -self.max_vel_move_straight[self.safety_field_idx])
                if self.vel_theta > 0:
                    self.vel_remap.angular.z = min(self.vel_theta, self.max_vel_move_rotation[self.safety_field_idx])
                elif self.vel_x < 0:
                    self.vel_remap.angular.z = max(self.vel_theta, -self.max_vel_move_rotation[self.safety_field_idx])
                if self.is_safety_stop or (rospy.get_time() - self.last_safety_stop) < 1:
                    _state = MainState.STOP_BY_SAFETY
                    self.pub_vel.publish(Twist())
                else:
                    self.pub_vel.publish(self.vel_remap)
            else:
                self.pub_vel.publish(self.vel_remap)
        else:
            self.pub_vel.publish(self.vel_remap)
            _state = MainState.SAFETY_OFF
        self.state = _state

    """
    ######## ##     ## ##    ##  ######  ######## ####  #######  ##    ##
    ##       ##     ## ###   ## ##    ##    ##     ##  ##     ## ###   ##
    ##       ##     ## ####  ## ##          ##     ##  ##     ## ####  ##
    ######   ##     ## ## ## ## ##          ##     ##  ##     ## ## ## ##
    ##       ##     ## ##  #### ##          ##     ##  ##     ## ##  ####
    ##       ##     ## ##   ### ##    ##    ##     ##  ##     ## ##   ###
    ##        #######  ##    ##  ######     ##    ####  #######  ##    ##
    """

    def manager_safety(self):
        if self.button_manual_status:
            if self.current_safety_job_name_auto_mode != self.pre_safety_job_name:
                self.pre_safety_job_name = self.current_safety_job_name_auto_mode
                msg = StringStamped()
                msg.stamp = rospy.Time.now()
                msg.data = self.current_safety_job_name_auto_mode
                self.safety_job_pub.publish(msg)
        else:
            if rospy.get_time() - self.last_time_pub >= 2:
                self.last_time_pub = rospy.get_time()
                footprint_msg = StringStamped()
                footprint_msg.stamp = rospy.Time.now()
                footprint_msg.data = self.footprint_job
                self.footprint_job_pub.publish(footprint_msg)
            if self.vel_x > 0:
                self.pre_safety_job_name = self.safety_job_forward
                msg = StringStamped()
                msg.stamp = rospy.Time.now()
                msg.data = self.safety_job_forward
                self.safety_job_pub.publish(msg)
            elif self.vel_x < 0:
                self.pre_safety_job_name = self.safety_job_backward
                msg = StringStamped()
                msg.stamp = rospy.Time.now()
                msg.data = self.safety_job_backward
                self.safety_job_pub.publish(msg)
            else:
                if self.vel_theta != 0:
                    self.pre_safety_job_name = self.safety_job_rotation
                    msg = StringStamped()
                    msg.stamp = rospy.Time.now()
                    msg.data = self.safety_job_rotation
                    self.safety_job_pub.publish(msg)


    """
    ##        #######   #######  ########
    ##       ##     ## ##     ## ##     ##
    ##       ##     ## ##     ## ##     ##
    ##       ##     ## ##     ## ########
    ##       ##     ## ##     ## ##
    ##       ##     ## ##     ## ##
    ########  #######   #######  ##
    """

    def poll(self):
        r = rospy.Rate(10.0)
        status_msg = StringStamped()
        status_msg_dict = {
            "state": MainState.NORMAL.toString(),
        }
        while not rospy.is_shutdown():
            self.manager_safety()
            if self.prev_state != self.state:
                rospy.loginfo(
                    "Loop state: {} -> {}".format(
                        self.prev_state.toString(), self.state.toString()
                    )
                )
                self.prev_state = self.state
            status_msg_dict = {
                "state": self.state.toString(),
            }
            status_msg.data = json.dumps(status_msg_dict, indent=2)
            status_msg.stamp = rospy.Time.now()
            self._asm.module_status_pub.publish(status_msg)
            self.state = MainState.NORMAL
            r.sleep()


def main():
    rospy.init_node("joystick_manager")
    rospy.loginfo("Init node: " + rospy.get_name())
    joystickControl()


if __name__ == "__main__":
    main()
