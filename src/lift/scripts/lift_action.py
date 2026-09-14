#! /usr/bin/env python
# -*- coding: utf-8 -*-

from logging import debug
from bson.json_util import dumps
import os
import sys
import rospy
import rospkg
import copy
import actionlib
import json
from actionlib_msgs.msg import GoalStatus
from std_stamped_msgs.msg import (
    StringAction,
    StringStamped,
    StringResult,
    StringFeedback,
    StringGoal,
    Int8Stamped,
    EmptyStamped,
)
from std_stamped_msgs.srv import StringService, StringServiceResponse
from std_msgs.msg import Int8, Header, Int16, String
from agv_msgs.msg import ArduinoIO

common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)

from module_manager import ModuleServer, ModuleClient, ModuleStatus
from common_function import (
    EnumString,
    lockup_pose,
    dict_to_obj,
    merge_two_dicts,
    print_debug,
    print_info,
    print_warn,
    print_error,
    obj_to_dict,
)

LIFT_UP = 1
LIFT_DOWN = 2


class MainState(EnumString):
    NONE = -1
    INIT = 0
    LIFT_UP = 1
    LIFT_DOWN = 2
    PAUSED = 3
    ERROR = 4
    DONE = 5


class ActionServerTemplate(object):
    _feedback = StringFeedback()
    _result = StringResult()

    def __init__(self, name, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        # Action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            StringAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()
        # Action client

        # Publisher
        self.pub_lift_cmd = rospy.Publisher(
            "/lift_cart", Int8Stamped, queue_size=10
        )
        # Subscriber
        rospy.Subscriber("/standard_io", StringStamped, self.arduino_cb)
        # ModuleServer
        self._asm = ModuleServer(name)
        # Event
        rospy.on_shutdown(self.shutdown)
        # Loop
        self.loop()

    def init_variable(self, *args, **kwargs):
        self.config_path = kwargs["config_path"]

    def shutdown(self):
        rospy.loginfo("[{}] Shutting down...".format(self._action_name))

    def send_feedback(self, action, msg):
        self._feedback.data = msg
        action.publish_feedback(self._feedback)

    """
     ######     ###    ##       ##       ########     ###     ######  ##    ##
    ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
    ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
    ##       ##     ## ##       ##       ########  ##     ## ##       #####
    ##       ######### ##       ##       ##     ## ######### ##       ##  ##
    ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
     ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
    """

    def arduino_cb(self, msg):
        data = json.loads(msg.data)
        if "lift_max_sensor" in data:
            self.liftup_finish = data["lift_max_sensor"]
        if "lift_min_sensor" in data:
            self.liftdown_finish = data["lift_min_sensor"]

    """
    ######## ##     ## ########  ######  ##     ## ######## ########
    ##        ##   ##  ##       ##    ## ##     ##    ##    ##
    ##         ## ##   ##       ##       ##     ##    ##    ##
    ######      ###    ######   ##       ##     ##    ##    ######
    ##         ## ##   ##       ##       ##     ##    ##    ##
    ##        ##   ##  ##       ##    ## ##     ##    ##    ##
    ######## ##     ## ########  ######   #######     ##    ########
    """

    def execute_cb(self, goal):
        success = True
        rospy.loginfo("%s action started" % (self._action_name))
        data = json.loads(goal.data)
        pick_or_place = data["params"]["liftup_or_liftdown"]
        r = rospy.Rate(10)
        _state = MainState.INIT
        _prev_state = MainState.NONE
        _prev_state_temp = MainState.NONE
        feedback_msg = ""
        _state_when_pause = MainState.NONE
        _state_when_error = MainState.NONE
        _state_bf_error = MainState.NONE
        self._asm.reset_flag()
        self._asm.action_running = True
        self.liftup_finish = False
        self.liftdown_finish = False
        self.lift_msg = Int8Stamped()
        while not rospy.is_shutdown():
            if self._as.is_preempt_requested() or self._asm.reset_action_req:
                rospy.loginfo("%s: Preempted" % self._action_name)
                self._as.set_preempted()
                success = False
                self.send_feedback(
                    self._as, GoalStatus.to_string(GoalStatus.PREEMPTED)
                )
                break

            if self._asm.module_status != ModuleStatus.ERROR:
                self._asm.error_code = ""
            if _state != MainState.PAUSED and self._asm.error_code == "":
                self._asm.module_status = ModuleStatus.RUNNING

            if _prev_state != _state:
                rospy.loginfo(
                    "Action state: {} -> {}".format(
                        _prev_state.toString(), _state.toString()
                    )
                )
                _prev_state = _state
                feedback_msg = _state.toString()
                self._asm.module_state = _state.toString()
            self.send_feedback(self._as, feedback_msg)

            # State: INIT
            if _state == MainState.INIT:
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    _state_when_pause = _state
                    _state = MainState.PAUSED
                    continue
                if pick_or_place:
                    _state = MainState.LIFT_UP
                else:
                    _state = MainState.LIFT_DOWN
            # State: LIFT_UP
            elif _state == MainState.LIFT_UP:
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    _state_when_pause = _state
                    _state = MainState.PAUSED
                    continue
                else:
                    self.lift_msg.stamp = rospy.Time.now()
                    self.lift_msg.data = LIFT_UP
                    self.pub_lift_cmd.publish(self.lift_msg)
                if False:  # ERROR
                    _state_bf_error = MainState.LIFT_UP
                    _state_when_error = _state
                    _state = MainState.ERROR
                    continue
                if self.liftup_finish:
                    _state = MainState.DONE
            # State: LIFT_DOWN
            elif _state == MainState.LIFT_DOWN:
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    _state_when_pause = _state
                    _state = MainState.PAUSED
                    continue
                else:
                    self.lift_msg.stamp = rospy.Time.now()
                    self.lift_msg.data = LIFT_DOWN
                    self.pub_lift_cmd.publish(self.lift_msg)
                if False:  # ERROR
                    _state_bf_error = MainState.STATE_1
                    _state_when_error = _state
                    _state = MainState.ERROR
                    continue
                if self.liftdown_finish:
                    _state = MainState.DONE
            # State: PAUSED
            elif _state == MainState.PAUSED:
                self._asm.module_status = ModuleStatus.PAUSED
                if self._asm.resume_req:
                    self._asm.reset_flag()
                    _state = _state_when_pause
            # State: ERROR
            elif _state == MainState.ERROR:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = rospy.get_name() + ": {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = _state_when_error
            # State: DONE
            elif _state == MainState.DONE:
                success = True
                break
            r.sleep()

        self._asm.action_running = False

        if success:
            rospy.loginfo("%s: Succeeded" % self._action_name)
            self._as.set_succeeded(self._result)

    """
    ######## ##     ## ##    ##  ######  ######## ####  #######  ##    ##
    ##       ##     ## ###   ## ##    ##    ##     ##  ##     ## ###   ##
    ##       ##     ## ####  ## ##          ##     ##  ##     ## ####  ##
    ######   ##     ## ## ## ## ##          ##     ##  ##     ## ## ## ##
    ##       ##     ## ##  #### ##          ##     ##  ##     ## ##  ####
    ##       ##     ## ##   ### ##    ##    ##     ##  ##     ## ##   ###
    ##        #######  ##    ##  ######     ##    ####  #######  ##    ##
    """

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
        r = rospy.Rate(1)
        status_msg = StringStamped()
        while not rospy.is_shutdown():
            if not self._asm.action_running:
                self._asm.module_status = ModuleStatus.WAITING
                self._asm.module_state = ModuleStatus.WAITING.toString()
                self._asm.error_code = ""
            status_msg.stamp = rospy.Time.now()
            status_msg.data = json.dumps(
                {
                    "status": self._asm.module_status.toString(),
                    "state": self._asm.module_state,
                    "error_code": self._asm.error_code,
                }
            )
            self._asm.module_status_pub.publish(status_msg)
            r.sleep()


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
        "--config_path",
        dest="config_path",
        default=os.path.join(rospkg.RosPack().get_path("follow_line"), "cfg"),
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
    rospy.init_node("lifting_server", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    ActionServerTemplate(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
