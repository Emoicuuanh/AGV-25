#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os, signal
import subprocess
import sys
import rospy
import rospkg
import copy
import json
import yaml
from std_stamped_msgs.msg import (
    StringStamped,
    StringAction,
    StringFeedback,
    StringResult,
    StringGoal,
    EmptyStamped,
)

common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)

from common_function import (
    MIN_FLOAT,
    EnumString,
    lockup_pose,
    offset_pose_x,
    offset_pose_yaw,
    distance_two_pose,
    delta_angle,
    dict_to_obj,
    merge_two_dicts,
    print_debug,
    print_info,
    print_warn,
    print_error,
    obj_to_dict,
    offset_pose_xy_theta,
    angle_two_pose,
    pose_dict_template,
)


class MainState(EnumString):
    NONE = -1
    NORMAL = 0
    CONNECT = 1
    DISCONNECT = 2
    RESTARTING = 3


class HmiManager(object):
    def __init__(self, name, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        self.load_config(kwargs["config_file"])
        # Publisher
        # Subscriber
        rospy.Subscriber("/hmi_status", StringStamped, self.hmi_status_cb)
        self.loop()

    def init_variable(self, *args, **kwargs):
        self.last_data_received = rospy.get_time()
        self.data_time_out = 10.0  # second
        self.launch_hmi = None
        self.hmi_received = False

    def load_config(self, file_path):
        try:
            with open(file_path) as file:
                self.config_file = yaml.load(file, Loader=yaml.Loader)
                return True
        except Exception as e:
            rospy.logerr("Error loading: {}".format(e))
            return False

    def kill_proccess_by_name(self, name_filter, name_node):
        try:

            # iterating through each instance of the process
            for line in os.popen("ps ax | grep " + name_filter + " | grep -v grep"):
                rospy.logwarn(line)
                if name_node in line:
                    fields = line.split()

                    # extracting Process ID from the output
                    pid = fields[0]

                    # terminating process
                    os.kill(int(pid), signal.SIGKILL)
            print("Process Successfully terminated")

        except:
            print("Error Encountered while running script")

    def kill_hmi(self):
        rospy.logwarn_throttle(10, "Killing hmi")
        self.kill_proccess_by_name("AppRun", "AppRun")
        if self.launch_hmi != None:
            self.launch_hmi.kill()
            self.hmi_received = False
            self.launch_hmi = None

    def run_hmi(self):
        rospy.logwarn_throttle(10, "Starting hmi")
        self.hmi_received = False
        self.launch_hmi = subprocess.Popen(
            ["roslaunch", self.config_file["ros_pkg"], self.config_file["launch_file"]]
        )

    def shutdown(self):
        rospy.loginfo("Shuting down")

    """
     ######     ###    ##       ##       ########     ###     ######  ##    ##
    ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
    ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
    ##       ##     ## ##       ##       ########  ##     ## ##       #####
    ##       ######### ##       ##       ##     ## ######### ##       ##  ##
    ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
     ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
    """

    def hmi_status_cb(self, msg):
        self.hmi_received = True
        self.last_data_received = rospy.get_time()

    """
    ##        #######   #######  ########
    ##       ##     ## ##     ## ##     ##
    ##       ##     ## ##     ## ##     ##
    ##       ##     ## ##     ## ########
    ##       ##     ## ##     ## ##
    ##       ##     ## ##     ## ##
    ########  #######   #######  ##
    """

    def loop(self):
        _state = MainState.NONE
        _prev_state = MainState.NONE
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if _state != _prev_state:
                print_debug(
                    "State: {} -> {}".format(_prev_state.toString(), _state.toString())
                )
                _prev_state = _state
            if (rospy.get_time() - self.last_data_received) > self.data_time_out:
                _state = MainState.DISCONNECT
                print("disconnect")
            else:
                if self.hmi_received:
                    _state = MainState.CONNECT
            if _state == MainState.DISCONNECT:
                self.kill_hmi()
                self.run_hmi()
                _state = MainState.RESTARTING
                self.restart_hmi = rospy.get_time()
            if _state == MainState.RESTARTING:
                self.last_data_received = rospy.get_time()
                if (rospy.get_time() - self.restart_hmi) > 10.0:
                    _state = MainState.NONE
            # print(_state)
            # print(rospy.get_time() - self.last_data_received)
            rate.sleep()


def parse_opts():
    from optparse import OptionParser

    parser = OptionParser()
    parser.add_option(
        "-d",
        "--ros_debug",
        action="store_true",
        dest="log_debug",
        default=False,
        help="log_level=rospy.DEBUG",
    )
    parser.add_option(
        "-c",
        "--config_file",
        dest="config_file",
        default=os.path.join(
            rospkg.RosPack().get_path("hmi_for_agv_release"), "cfg", "hmi_config.yaml"
        ),
    )

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)


def main():
    (options, args) = parse_opts()
    log_level = None
    if options.log_debug:
        log_level = rospy.DEBUG
    rospy.init_node("hmi_manager", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    HmiManager(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
