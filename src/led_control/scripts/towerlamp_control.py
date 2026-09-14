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

class TowerLamp(object):
    def __init__(self, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        rospy.on_shutdown(self.shutdown)
        # Publisher
        self.towerlamp_set_pub = rospy.Publisher("/towerlamp_set", StringStamped, queue_size=5, latch=True)
        self.towerlamp_control_alive_pub = rospy.Publisher("/towerlamp_control_alive", StringStamped, queue_size=5)
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
        self.towerlamp_effect_file = kwargs["config_file"]
        #
        self.led_status = ""
        #
        self.alway_check_towerlamp_effect_config = True
        self.first_update = True
        self.last_towerlamp_effect = None
        self.towerlamp_msg = StringStamped()

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

    def func(self, effect, params, towerlamp_effect_config):
        if effect == self.led_status:
            if 'blink_interval' in params:
                blink_interval = params['blink_interval']
            else:
                blink_interval = 0
            #
            if 'r' in params:
                towerlamp_r = params['r']
            else:
                towerlamp_r = 0
            #
            if 'g' in params:
                towerlamp_g = params['g']
            else:
                towerlamp_g = 0
            #
            if 'y' in params:
                towerlamp_y = params['y']
            else:
                towerlamp_y = 0
            #
            if 'buzzer' in params:
                towerlamp_buzzer = params['buzzer']
            else:
                towerlamp_buzzer = 0
            msg_dict = {"name": self.led_status, "red": towerlamp_r, "green": towerlamp_g, "yellow": towerlamp_y, "buzzer": towerlamp_buzzer, "blink_interval": blink_interval}

            self.towerlamp_msg.data = json.dumps(msg_dict)
            if self.last_towerlamp_effect != towerlamp_effect_config or self.first_update:
                self.first_update = False
                self.towerlamp_msg.stamp = rospy.Time.now()
                self.towerlamp_set_pub.publish(self.towerlamp_msg)
                rospy.loginfo('towerlamp_effect.yaml change: {}'.format(self.led_status))
            return True
        return False
        
    def load_towerlamp_status(self):
        # try:
        if True:
            with open(self.towerlamp_effect_file) as file:
                towerlamp_effect_config = yaml.load(file)['towerlamp_effect']
                if not python3:
                    for effect, params in towerlamp_effect_config.iteritems():
                        if self.func(effect, params, towerlamp_effect_config): break
                else:
                    for effect, params in towerlamp_effect_config.items():
                        if self.func(effect, params, towerlamp_effect_config): break
                    
            self.last_towerlamp_effect = towerlamp_effect_config
        # except Exception as e:
        #     rospy.logerr('Update tower lamp error: {}'.format(e))

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
        last_towerlamp_alive = rospy.get_time()
        last_publish = rospy.get_time()
        r = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            # Update LED
            if self.alway_check_towerlamp_effect_config:
                self.load_towerlamp_status()
                if self.led_status != last_status: # or rospy.get_time() - last_publish >= 1.0:
                    last_publish = rospy.get_time()
                    self.towerlamp_msg.stamp = rospy.Time.now()
                    self.towerlamp_set_pub.publish(self.towerlamp_msg)
            last_status = self.led_status

            if rospy.get_time() - last_towerlamp_alive >= 1.0:
                last_towerlamp_alive = rospy.get_time()
                self.towerlamp_control_alive_pub.publish(StringStamped(stamp=rospy.Time.now()))
            r.sleep()

def parse_opts():
    from optparse import OptionParser
    parser = OptionParser()
    parser.add_option("-s", "--simulation",
                    action="store_true", dest="simulation", default=False, help="type \"-s\" if simulation")
    parser.add_option("-d", "--ros_debug",
                    action="store_true", dest="log_debug", default=False, help="log_level=rospy.DEBUG")
    parser.add_option("-c", "--config_file", dest="config_file",
                    default=os.path.join(rospkg.RosPack().get_path('led_control'), 'cfg', 'towerlamp_effect.yaml'))

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)

def main():
    (options, args) = parse_opts()
    log_level = None
    if options.log_debug:
        log_level = rospy.DEBUG
    rospy.init_node('towerlamp_control', log_level=log_level)
    rospy.loginfo('Init node ' + rospy.get_name())
    TowerLamp(**vars(options))

if __name__ == '__main__':
    main()