#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys

python3 = True if sys.hexversion > 0x03000000 else False
import rospy
import json
import threading
from std_stamped_msgs.msg import StringStamped, EmptyStamped
from common_function import EnumString
from nav_msgs.msg import Odometry

class ModuleStatus(EnumString):
    WAITING = 0
    RUNNING = 1
    PAUSED = 2
    ERROR = 3


class ModuleServer:
    def __init__(self, name):
        # Ros
        self.module_status_pub = rospy.Publisher(
            name + "/module_status", StringStamped, queue_size=5
        )
        rospy.Subscriber(
            name + "/run_pause_req", StringStamped, self.run_pause_req_cb
        )
        rospy.Subscriber(
            name + "/reset_error", EmptyStamped, self.reset_error_cb
        )
        rospy.Subscriber(
            "/request_start_mission",
            StringStamped,
            self.request_start_mission_cb,
        )
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        # Variable
        self.name = name
        self.error_code = ""
        self.reset_action_req = False
        self.reset_error_request = False
        self.resume_req = False
        self.pause_req = False
        self.resume_req_by_server = False
        self.pause_req_by_server = False
        self.action_running = False
        self.module_state = ""
        self.module_status = ModuleStatus.WAITING
        self.param_dict = {}

    def odom_cb(self, msg):
        self.vel_x = msg.twist.twist.linear.x

    def request_start_mission_cb(self, msg):
        if msg.data == "STOP":
            self.reset_action_req = True
            rospy.loginfo("STOP MISSION")

    def run_pause_req_cb(self, msg):
        if msg.data == "RUN":
            rospy.loginfo("Resume request")
            self.resume_req = True
            self.pause_req = False
            self.resume_req_by_server = True
            self.pause_req_by_server = False
        elif msg.data == "PAUSE":
            rospy.loginfo("Pause request")
            self.resume_req = False
            self.pause_req = True
        elif msg.data == "PAUSE_BY_SERVER":
            rospy.loginfo("Pause request")
            self.resume_req = False
            self.pause_req = True
            self.resume_req_by_server = False
            self.pause_req_by_server = True

    def reset_error_cb(self, msg):
        self.reset_error_request = True

    def reset_flag(self):
        self.pause_req = False
        self.resume_req = False
        self.resume_req_by_server = False
        self.pause_req_by_server = False
        self.reset_action_req = False
        self.reset_error_request = False
        self.error_code = ""

    def send_feedback(self, action, msg):
        self._feedback.data = msg
        action.publish_feedback(self._feedback)


class ModuleClient:
    def __init__(self, name, param_dict={}):
        self.param_dict = param_dict
        rospy.loginfo("Init ModuleClient for: {}".format(name))
        # Ros
        if not "status_topic" in param_dict:
            rospy.Subscriber(
                name + "/module_status", StringStamped, self.module_status_cb
            )
        else:
            [msg_ns, msg_type] = param_dict["status_msg"].split("/", 1)
            cmd = "from {}.msg import {}".format(msg_ns, msg_type)
            exec(cmd)
            cmd = 'rospy.Subscriber("{}", {}, self.module_status_cb)'.format(
                param_dict["status_topic"], msg_type
            )
            exec(cmd)

        # Variable
        self.name = name
        self.display_name = param_dict["display_name"]
        self.parse_json = (
            param_dict["parse_json"] if "parse_json" in param_dict else True
        )
        self.handle_when_sim = (
            param_dict["handle_when_sim"]
            if "handle_when_sim" in param_dict
            else True
        )
        self.timeout = param_dict["timeout"] if "timeout" in param_dict else 2.0
        self.pause_if_error = (
            param_dict["pause_if_error"]
            if "pause_if_error" in param_dict
            else True
        )
        self.error_code_number_default = (
            param_dict["error_code_number_default"]
            if "error_code_number_default" in param_dict
            else -1
        )
        self.reset_when_mission_error = True
        self.display_error = (
            param_dict["display_error"]
            if "display_error" in param_dict
            else True
        )
        self.error_list = (
            param_dict["error_list"] if "error_list" in param_dict else []
        )
        self.state_list = (
            param_dict["state_list"] if "state_list" in param_dict else []
        )
        self.check_script = (
            param_dict["check_script"] if "check_script" in param_dict else ""
        )

        self.last_module_status = rospy.get_time()
        self.module_alive = False
        self.module_status_msg = None
        self.map_save_thread = threading.Thread(target=self.loop, args=())
        self.map_save_thread.start()
        self.module_status_dict = {}
        self.special_matching_error = ""
        self.special_matching_state = ""
        self.special_error_led = ""
        self.special_state_led = ""
        self.special_state_sound = ""
        self.special_error_sound = ""
        self.error_code = ""
        self.error_code_number = -1  # not define

    def module_status_cb(self, msg):
        self.module_status_msg = msg
        self.last_module_status = rospy.get_time()
        if not self.parse_json:
            return
        self.special_matching_error = ""
        self.special_matching_state = ""
        self.special_error_led = ""
        self.special_state_led = ""
        self.special_state_sound = ""
        self.special_error_sound = ""
        self.error_code_number = -1
        try:
            self.module_status_dict = json.loads(msg.data)
            # Normal error
            if "error_code" in self.module_status_dict:
                self.error_code = self.module_status_dict["error_code"]
            # Special error
            if python3:
                if "error_code" in self.module_status_dict and len(
                    self.error_list
                ):
                    for i in self.error_list:
                        if (
                            list(i.keys())[0]
                            in self.module_status_dict["error_code"]
                        ):
                            if "detail_status" in list(i.values())[0]:
                                self.special_matching_error = list(i.values())[0][
                                    "detail_status"
                                ]
                            if "reset_when_mission_error" in list(i.values())[0]:
                                self.reset_when_mission_error = list(i.values())[0][
                                    "reset_when_mission_error"
                                ]
                            else:
                                self.reset_when_mission_error = True
                            if "error_code_number" in list(i.values())[0]:
                                self.error_code_number = list(i.values())[0][
                                    "error_code_number"
                                ]
                            if "led_status" in list(i.values())[0]:
                                self.special_error_led = list(i.values())[0][
                                    "led_status"
                                ]
                            if "sound_status" in list(i.values())[0]:
                                self.special_error_sound = list(i.values())[0][
                                    "sound_status"
                                ]
                            return
                if "state" in self.module_status_dict and len(self.state_list):
                    for i in self.state_list:
                        if (
                            list(i.keys())[0]
                            in self.module_status_dict["state"]
                        ):
                            self.special_matching_state = list(i.values())[0][
                                "detail_status"
                            ]
                            if "led_status" in list(i.values())[0]:
                                self.special_state_led = list(i.values())[0][
                                    "led_status"
                                ]
                            if "sound_status" in list(i.values())[0]:
                                self.special_state_sound = list(i.values())[0][
                                    "sound_status"
                                ]
                            return
            else:
                if "error_code" in self.module_status_dict and len(
                    self.error_list
                ):
                    for i in self.error_list:
                        if i.keys()[0] in self.module_status_dict["error_code"]:
                            self.special_matching_error = i.values()[0][
                                "detail_status"
                            ]
                            if "led_status" in i.values()[0]:
                                self.special_error_led = i.values()[0][
                                    "led_status"
                                ]
                            if "sound_status" in i.values()[0]:
                                self.special_error_sound = i.values()[0][
                                    "sound_status"
                                ]
                            return
                if "state" in self.module_status_dict and len(self.state_list):
                    for i in self.state_list:
                        if i.keys()[0] in self.module_status_dict["state"]:
                            self.special_matching_state = i.values()[0][
                                "detail_status"
                            ]
                            if "led_status" in i.values()[0]:
                                self.special_state_led = i.values()[0][
                                    "led_status"
                                ]
                            if "sound_status" in i.values()[0]:
                                self.special_state_sound = i.values()[0][
                                    "sound_status"
                                ]
                            return
        except Exception as e:
            rospy.logerr("module_status_cb: {}".format(e))

    def loop(self):
        r = rospy.Rate(2)
        self.vel_x = 0
        self.start_time_check_disconnect = rospy.get_time()
        while not rospy.is_shutdown():
            if "status_topic" in self.param_dict:
                if self.param_dict["status_topic"] == "/followline_sensor":
                    if self.vel_x == 0:
                        self.start_time_check_disconnect = rospy.get_time()
                        self.last_module_status = rospy.get_time()
                    if (rospy.get_time() - self.start_time_check_disconnect <= 0.5):
                        self.last_module_status = rospy.get_time()
            if self.check_script == "":
                if rospy.get_time() - self.last_module_status > self.timeout:
                    self.module_alive = False
                else:
                    self.module_alive = True
            else:
                # Ex: msg.ranges[0] == -1
                if self.module_status_msg != None:
                    cmd = "self.module_alive = {}".format(self.check_script)
                    exec(cmd)
                if rospy.get_time() - self.last_module_status > self.timeout:
                    self.module_alive = False
            r.sleep()
