#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
python3 = True if sys.hexversion > 0x03000000 else False
if python3:
    pass
else:
    pass
import json
import yaml
import threading
import argparse
import rospy
import rospkg
import tf
import copy
import actionlib
import time
from nav_msgs.msg import OccupancyGrid
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import  Pose, PoseStamped, PoseArray
from std_msgs.msg import Int8, String
from std_stamped_msgs.msg import StringStamped, StringFeedback, StringResult, StringAction
from actionlib_msgs.msg import GoalStatus, GoalID, GoalStatusArray
from agv_msgs.msg import LedControl

common_func_dir = os.path.join(rospkg.RosPack().get_path('agv_common_library'), 'scripts')
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(rospkg.RosPack().get_path('agv_common_library'), 'release')
sys.path.insert(0, common_func_dir)

from common_function import (
    EnumString,
    lockup_pose,
    offset_pose_x,
    distance_two_pose,
    delta_angle,
    dict_to_obj,
    merge_two_dicts,
    print_debug,
    print_warn,
    print_error,
    print_info,
    get_line_info
)

class Led(object):
    def __init__(self, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        rospy.on_shutdown(self.shutdown)
        # Publisher
        self.led_control_pub = rospy.Publisher("/led_control", LedControl, queue_size=5, latch=True)
        self.led_control_alive = rospy.Publisher("/led_control_alive", StringStamped, queue_size=5)
        # Subscriber
        rospy.Subscriber("/led_status", StringStamped, self.led_status_cb)
        # Initial
        # self.load_config(kwargs["config_file"])
        # Loop
        self.loop()

    """
    #### ##    ## #### ######## ####    ###    ##
     ##  ###   ##  ##     ##     ##    ## ##   ##
     ##  ####  ##  ##     ##     ##   ##   ##  ##
     ##  ## ## ##  ##     ##     ##  ##     ## ##
     ##  ##  ####  ##     ##     ##  ######### ##
     ##  ##   ###  ##     ##     ##  ##     ## ##
    #### ##    ## ####    ##    #### ##     ## ########
    """

    def init_variable(self, *args, **kwargs):
        self.simulation = kwargs["simulation"]
        print_debug("simulation: {}".format(self.simulation))
        self.simulation_str = "true" if self.simulation else "false"
        # TF
        self.led_effect_file = kwargs["config_file"]
        #
        self.led_status = ""
        #
        self.alway_check_led_effect_config = True
        self.load_file_only_one = True
        self.first_update = True
        self.first_load = True
        self.last_led_effect_config = None
        self.led_control_msg = LedControl()



    def shutdown(self):
        # Kill all node
        print("Shutdown!")

    """
    ######## ##     ## ##    ##  ######  ######## ####  #######  ##    ##
    ##       ##     ## ###   ## ##    ##    ##     ##  ##     ## ###   ##
    ##       ##     ## ####  ## ##          ##     ##  ##     ## ####  ##
    ######   ##     ## ## ## ## ##          ##     ##  ##     ## ## ## ##
    ##       ##     ## ##  #### ##          ##     ##  ##     ## ##  ####
    ##       ##     ## ##   ### ##    ##    ##     ##  ##     ## ##   ###
    ##        #######  ##    ##  ######     ##    ####  #######  ##    ##
    """

    def load_led_status(self):
        try:
            if not self.load_file_only_one or self.first_load:
                self.first_load = False
                with open(self.led_effect_file) as file:
                    self.led_effect_config = yaml.load(file)['led_effect']
                    self.led_effect_iter = None
                    if python3:
                        self.led_effect_iter = self.led_effect_config.items()
                    else:
                        self.led_effect_iter = self.led_effect_config.iteritems()
            for effect, params in self.led_effect_iter:
                if effect == self.led_status:
                    led_type = params['type']
                    #
                    if 'duration' in params:
                        led_duration = params['duration']
                    else:
                        led_duration = 0
                    #
                    if 'blink_interval' in params:
                        blink_interval = params['blink_interval']
                    else:
                        blink_interval = 0
                    #
                    if 'r' in params:
                        led_r = params['r']
                    else:
                        led_r = 0
                    #
                    if 'g' in params:
                        led_g = params['g']
                    else:
                        led_g = 0
                    #
                    if 'b' in params:
                        led_b = params['b']
                    else:
                        led_b = 0
                    self.led_control_msg.type = int(led_type)
                    self.led_control_msg.duration = int(led_duration)
                    self.led_control_msg.blink_interval = int(blink_interval)
                    self.led_control_msg.r = int(led_r)
                    self.led_control_msg.g = int(led_g)
                    self.led_control_msg.b = int(led_b)
                    if self.last_led_effect_config != self.led_effect_config or self.first_update:
                        self.first_update = False
                        self.led_control_msg.stamp = rospy.Time.now()
                        self.led_control_pub.publish(self.led_control_msg)
                        rospy.loginfo('led_effect.yaml change: {}'.format(self.led_status))
                    break
            self.last_led_effect_config = self.led_effect_config
        except Exception as e:
            rospy.logerr('Update led error: {}'.format(e))

    """
     ######     ###    ##       ##       ########     ###     ######  ##    ##
    ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
    ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
    ##       ##     ## ##       ##       ########  ##     ## ##       #####
    ##       ######### ##       ##       ##     ## ######### ##       ##  ##
    ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
     ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
    """

    def led_status_cb(self, msg):
        self.led_status = msg.data
        print_info("Led status update: {}".format(msg.data))


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
        last_status = ""
        last_led_alive = rospy.get_time()
        last_publish = rospy.get_time()
        r = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            # Update LED
            if self.alway_check_led_effect_config:
                self.load_led_status()
                if self.led_status != last_status or rospy.get_time() - last_publish >= 1.0:
                    last_publish = rospy.get_time()
                    self.led_control_msg.stamp = rospy.Time.now()
                    self.led_control_pub.publish(self.led_control_msg)
            last_status = self.led_status

            if rospy.get_time() - last_led_alive >= 1.0:
                last_led_alive = rospy.get_time()
                self.led_control_alive.publish(StringStamped(stamp=rospy.Time.now()))
            r.sleep()

def parse_opts():
    from optparse import OptionParser
    parser = OptionParser()
    parser.add_option("-s", "--simulation",
                    action="store_true", dest="simulation", default=False, help="type \"-s\" if simulation")
    parser.add_option("-d", "--ros_debug",
                    action="store_true", dest="log_debug", default=False, help="log_level=rospy.DEBUG")
    parser.add_option("-c", "--config_file", dest="config_file",
                    default=os.path.join(rospkg.RosPack().get_path('led_control'), 'cfg', 'led_effect.yaml'))

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)

def main():
    (options, args) = parse_opts()
    log_level = None
    if options.log_debug:
        log_level = rospy.DEBUG
    rospy.init_node('led_control', log_level=log_level)
    rospy.loginfo('Init node ' + rospy.get_name())
    Led(**vars(options))

if __name__ == '__main__':
    main()