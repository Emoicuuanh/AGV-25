#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
from pickle import NONE
import sys
import rospy
import rospkg
import tf
import copy
import actionlib
import json
import yaml
import dynamic_reconfigure.client
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import (
    Twist,
    Pose,
    PoseStamped,
    PoseWithCovarianceStamped,
)
from nav_msgs.msg import Odometry
from std_stamped_msgs.msg import Int8Stamped, EmptyStamped, Float32Stamped
from math import pi
from actionlib_msgs.msg import GoalStatus
from tf.listener import TransformListener, Transformer
from tf.transformations import euler_from_quaternion
from std_stamped_msgs.msg import (
    StringStamped,
    StringAction,
    StringFeedback,
    StringResult,
    StringGoal,
)
from std_stamped_msgs.srv import StringService, StringServiceResponse

common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)

from module_manager import ModuleServer, ModuleStatus
from common_function import EnumString, YamlDumper, print_debug


class MainState(EnumString):
    NONE = -1
    INIT = 0
    ACTIVATE = 1
    AUTO_CHARGING = 2
    MANUAL_CHARGING = 3
    FULL = 4
    DONE = 5
    NON_CHARGE = 6
    CONTACT_ERROR = 7
    IO_ERROR = 8


class LoopState(EnumString):
    NONE = -1
    INIT = 0
    AUTO_CHARGING = 2
    MANUAL_CHARGING = 3
    NON_CHARGE = 4
    FULL = 5
    IO_ERROR = 6


SENSOR_DEACTIVATE = 0
SENSOR_ACTIVATE = 1
SWITCH_MANUAL_MODE = 0
SWITCH_AUTO_MODE = 1
OUTPUT_OFF = 0
OUTPUT_ON = 1


class ChargingAction(object):
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
        if self.load_config(kwargs["config_file"]):
            self._as.start()
        else:
            return
        rospy.on_shutdown(self.shutdown)
        # Publisher
        self.digital_write_pub = rospy.Publisher(
            "/digital_write", StringStamped, queue_size=10
        )
        # Subscriber
        rospy.Subscriber("/arduino_driver/float_param/battery_percent",
            Float32Stamped, self.battery_cb)
        rospy.Subscriber(
            "/cancel_charging", EmptyStamped, self.cancel_charging_cb
        )
        rospy.Subscriber("/standard_io", StringStamped, self.standard_io_cb)
        rospy.Subscriber(
            "/arduino_driver/float_param/battery_ampe",
            Float32Stamped,
            self.charging_current_cb,
        )
        # ModuleServer
        self._asm = ModuleServer(name)
        # Service
        self.active_charging_srv = rospy.ServiceProxy(
            "/arduino_driver/digital_write", StringService
        )
        # Loop
        self.loop()

    def init_variable(self, *args, **kwargs):
        self.battery_percentage = 0.0
        self.charging_full_time = 10
        self.percent_to_charge_again = 80
        self.max_percent_to_charge = 90
        self.cancel_charging_request = False
        self.action_running = False
        self.is_charging_on = False
        self.auto_charging_detected = SENSOR_DEACTIVATE
        self.manual_charging_detected = SENSOR_DEACTIVATE
        self.auto_charging_en_pin = SENSOR_DEACTIVATE
        self.auto_manual_sw = SENSOR_ACTIVATE
        self.last_std_io_msg = rospy.get_time()
        self.arduino_connect = False
        self.charging_current_msg = Float32Stamped()
        self.receive_battery_percent = False

    def load_config(
        self,
        path=os.path.join(
            rospkg.RosPack().get_path("charging_process"),
            "cfg",
            "charging_process.yaml",
        ),
    ):
        try:
            with open(path) as file:
                self.charging_config = yaml.load(file, Loader=yaml.Loader)
                self.charging_full_time = self.charging_config[
                    "charging_full_time"
                ]
                self.percent_to_charge_again = self.charging_config["percent_to_charge_again"]
                self.max_percent_to_charge = self.charging_config["max_percent_to_charge"]
                print(
                    yaml.dump(
                        self.charging_config,
                        Dumper=YamlDumper,
                        default_flow_style=False,
                    )
                )
                return True
        except Exception as e:
            rospy.logerr("Error loading: {}".format(e))
            return False

    def shutdown(self):
        self.moving_control_client.cancel_all_goals()

    def send_feedback(self, action, msg):
        self._feedback.data = msg
        action.publish_feedback(self._feedback)

    def set_charging_en(self, value):
        # TODO: Check /standart_io to confirm
        try:
            if self.arduino_connect:
                self.arduino_connect = False
                for i in range(3):
                    # Publish to set charge pin
                    # rospy.loginfo("arduino is connect")
                    self.digital_write_pub.publish(
                        StringStamped(
                            stamp=rospy.Time.now(),
                            data="auto_charging_en_pin,{},0".format(value),
                        )
                    )
                    rospy.sleep(0.1)
                    # print("auto charing pin: {}".format(self.auto_charging_en_pin))
                    if self.auto_charging_en_pin == value:
                        rospy.loginfo("Write to pin success")
                        return True
                    else:
                        # rospy.logwarn("")
                        # NOTE: Nếu 100% thì dưới Arduino đã tự ngắt sạc, không thể set auto_charging_en_pin
                        pass
                rospy.logwarn("Write to pin error")
            return False

        except Exception as e:
            rospy.logerr("Set charging error: {}".format(e))
            return False

    """
     ######     ###    ##       ##       ########     ###     ######  ##    ##
    ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
    ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
    ##       ##     ## ##       ##       ########  ##     ## ##       #####
    ##       ######### ##       ##       ##     ## ######### ##       ##  ##
    ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
     ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
    """

    def battery_cb(self, msg):
        self.battery_percentage = msg.data
        if self.battery_percentage >= 100.0:
            self.battery_percentage = 100.0
        self.receive_battery_percent = True

    def cancel_charging_cb(self, msg):
        self.cancel_charging_request = True
        self.set_charging_en(OUTPUT_OFF)
        rospy.loginfo("Cancel charging")

    def standard_io_cb(self, msg):
        try:
            self.std_io_status = json.loads(msg.data)
            if "auto_charging_detect" in self.std_io_status:
                self.auto_charging_detected = self.std_io_status[
                    "auto_charging_detect"
                ]
            if "manual_charging_detect" in self.std_io_status:
                self.manual_charging_detected = self.std_io_status[
                    "manual_charging_detect"
                ]
            if "auto_manual_sw" in self.std_io_status:
                self.auto_manual_sw = self.std_io_status["auto_manual_sw"]
            if "auto_charging_en_pin" in self.std_io_status:
                self.auto_charging_en_pin = self.std_io_status[
                    "auto_charging_en_pin"
                ]
            # self.is_charging_on = self.auto_charging_detected or self.manual_charging_detected
        except Exception as e:
            rospy.logerr("standard_io error: {}".format(e))
        self.last_std_io_msg = rospy.get_time()
        self.arduino_connect = True

    def charging_current_cb(self, msg):
        self.charging_current_msg = msg
        if self.charging_current_msg.data > 0.0:
            self.is_charging_on = True
        else:
            self.is_charging_on = False

    """
       ###     ######  ######## ####  #######  ##    ##    ######## ##     ## ########
      ## ##   ##    ##    ##     ##  ##     ## ###   ##    ##        ##   ##  ##
     ##   ##  ##          ##     ##  ##     ## ####  ##    ##         ## ##   ##
    ##     ## ##          ##     ##  ##     ## ## ## ##    ######      ###    ######
    ######### ##          ##     ##  ##     ## ##  ####    ##         ## ##   ##
    ##     ## ##    ##    ##     ##  ##     ## ##   ###    ##        ##   ##  ##
    ##     ##  ######     ##    ####  #######  ##    ##    ######## ##     ## ########
    """

    def execute_cb(self, goal):
        try:
            goal_dict = json.loads(goal.data)
            rospy.loginfo(
                "Auto charging goal:\n{}".format(
                    json.dumps(goal_dict, indent=2)
                )
            )
            minimum_percentage = goal_dict["params"][
                "minimum_percentage"
            ]  # Percentage
            minimum_time = goal_dict["params"]["minimum_time"]  # Minutes
            rospy.loginfo("minimum percentage: {}".format(minimum_percentage))
            rospy.loginfo("minimum time: {}".format(minimum_time))
        except Exception as e:
            self._as.set_aborted(text="Auto charging goal syntax error")
            rospy.logerr("Auto charging syntax goal error: {}".format(e))
            return

        _state = MainState.INIT
        _prev_state = MainState.NONE
        _first_state_changed = False
        charging_time = 0.0
        charging_cnt = 0
        full_charging_time = 0.0
        begin_full_time = rospy.get_time()
        begin_action_time = rospy.get_time()
        begin_error_time = rospy.get_time()
        start_error = False
        self.cancel_charging_request = False
        self._asm.reset_flag()
        self.action_running = True
        status_msg = StringStamped()
        status_msg_dict = {}

        success = False
        rate = 2.0
        r = rospy.Rate(rate)
        while not rospy.is_shutdown():
            if _state != _prev_state:
                rospy.loginfo(
                    "Action state: {} -> {}".format(
                        _prev_state.toString(), _state.toString()
                    )
                )
                _prev_state = _state
                feedback_msg = _state.toString()
                _first_state_changed = True
            self.send_feedback(self._as, feedback_msg)

            status_msg_dict = {
                "state": _state.toString(),
                "charging_time": charging_time,
                "full_charging_time": full_charging_time,
                "execute": "ACTION",
                "error_code": self._asm.error_code,
            }
            status_msg.data = json.dumps(status_msg_dict, indent=2)
            status_msg.stamp = rospy.Time.now()
            self._asm.module_status_pub.publish(status_msg)

            if self._as.is_preempt_requested() or self._asm.reset_action_req:
                rospy.loginfo("%s: Preempted" % self._action_name)
                self._as.set_preempted()
                success = False
                self.send_feedback(
                    self._as, GoalStatus.to_string(GoalStatus.PREEMPTED)
                )
                break

            if (
                _state == MainState.AUTO_CHARGING
                or _state == MainState.MANUAL_CHARGING
            ):
                charging_cnt += 1
                charging_time = (charging_cnt * 1 / rate) / 60.0  # Minutes

            if self.cancel_charging_request:
                self.cancel_charging_request = False
                success = True
                break

            if self._asm.module_status != ModuleStatus.ERROR:
                self._asm.error_code = ""

            # Transaction
            if _state == MainState.INIT:
                if rospy.get_time() - begin_action_time >= 3.0:
                    _state = MainState.ACTIVATE
            elif _state == MainState.ACTIVATE:
                if self.set_charging_en(OUTPUT_ON):
                    _state = MainState.AUTO_CHARGING
                else:
                    _state = MainState.IO_ERROR
            elif _state == MainState.AUTO_CHARGING:
                if (
                    charging_time > minimum_time
                    and self.battery_percentage >= minimum_percentage
                    or self.battery_percentage == 100
                ):
                    _state = MainState.DONE
                if self.auto_charging_detected == SENSOR_DEACTIVATE:
                    # begin_error_time = rospy.get_time()
                    _state = MainState.CONTACT_ERROR
                if self.manual_charging_detected == SENSOR_ACTIVATE:
                    _state = MainState.MANUAL_CHARGING
            elif _state == MainState.MANUAL_CHARGING:
                if self.manual_charging_detected == SENSOR_DEACTIVATE:
                    _state = MainState.AUTO_CHARGING
            elif _state == MainState.DONE:
                success = True
                break
            elif _state == MainState.CONTACT_ERROR:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/charging_process: {}".format(
                    _state.toString()
                )
                if _first_state_changed:
                    _first_state_changed = False
                    if not self.set_charging_en(OUTPUT_OFF):
                        _state = MainState.IO_ERROR
                if self.auto_charging_detected == SENSOR_ACTIVATE:
                    begin_action_time = rospy.get_time()
                    _state = MainState.INIT
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    begin_action_time = rospy.get_time()
                    _state = MainState.INIT
            elif _state == MainState.IO_ERROR:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/charging_process: {}".format(
                    _state.toString()
                )
                # if rospy.get_time() - self.last_std_io_msg < 1.0:
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = MainState.INIT

            if rospy.get_time() - self.last_std_io_msg > 1.0:
                self.arduino_connect = False
                _state = MainState.IO_ERROR
            r.sleep()

        self.action_running = False
        # Do not turn off charging when action finish

        if success:
            rospy.loginfo("%s: Succeeded" % self._action_name)
            self._as.set_succeeded(self._result)

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
        status_msg_dict = {}
        begin_full_time = rospy.get_time()
        begin_contact_charger = rospy.get_time()
        begin_auto_charger_jack_connect = rospy.get_time()
        set_auto_charging_en_pin = False
        disconnect_charger = False
        r = rospy.Rate(2.0)
        status_msg = StringStamped()
        status_msg_dict = {
            "state": LoopState.NON_CHARGE.toString(),
            "charging_time": 0.0,
            "full_charging_time": 0.0,
            "execute": "LOOP",
            "error_code": "",
        }
        _state = LoopState.INIT
        _prev_state = LoopState.NONE
        prev_auto_man = SWITCH_AUTO_MODE
        prev_auto_charging_detected = self.auto_charging_detected
        full_charing_time = 0

        while not rospy.is_shutdown():
            # Nếu sạc không tiếp xúc thì Relay sạc cũng không ON cho dù IO điều khiển đang ON vì nguồn COM-IN cho Relay lấy từ sạc
            # NON_CHARGE, FULL, AUTO_CHARGING

            if self.action_running:
                _state = LoopState.INIT
                r.sleep()
                continue

            if _prev_state != _state:
                rospy.loginfo(
                    "Loop state: {} -> {}".format(
                        _prev_state.toString(), _state.toString()
                    )
                )
                _prev_state = _state

            if rospy.get_time() - self.last_std_io_msg > 1.0:
                self.arduino_connect = False
                _state = LoopState.IO_ERROR

            # Transaction
            # State: INIT
            if _state == LoopState.INIT:
                if not self.is_charging_on:
                    _state = LoopState.NON_CHARGE
                elif self.check_full_charging() and self.receive_battery_percent:
                    begin_full_time = rospy.get_time()
                    disconnect_charger = False
                    _state = LoopState.FULL
                elif self.check_manual_charging():
                    _state = LoopState.MANUAL_CHARGING
                elif self.check_auto_charging():
                    _state = LoopState.AUTO_CHARGING
            # State: NON_CHARGE
            elif _state == LoopState.NON_CHARGE:
                if self.is_charging_on:
                    if self.check_manual_charging():
                        _state = LoopState.MANUAL_CHARGING
                    elif self.check_auto_charging():
                        _state = LoopState.AUTO_CHARGING
                if (
                    self.auto_charging_en_pin == SENSOR_ACTIVATE
                    and self.auto_charging_detected == SENSOR_DEACTIVATE
                ):
                    self.set_charging_en(OUTPUT_ON)
                
                if self.battery_percentage > self.percent_to_charge_again:
                    begin_auto_charger_jack_connect = rospy.get_time()
                # State: MANUAL_CHARGING
            elif _state == LoopState.MANUAL_CHARGING:
                if self.check_auto_charging():
                    _state = LoopState.AUTO_CHARGING
                elif not self.is_charging_on:
                    _state = LoopState.NON_CHARGE
            # State: AUTO_CHARGING
            elif _state == LoopState.AUTO_CHARGING:
                if self.check_full_charging() and self.receive_battery_percent:
                    begin_full_time = rospy.get_time()
                    disconnect_charger = False
                    _state = LoopState.FULL
                elif self.check_manual_charging():
                    _state = LoopState.MANUAL_CHARGING
                elif not self.is_charging_on:
                    _state = LoopState.NON_CHARGE
            # State: FULL
            elif _state == LoopState.FULL:
                full_charing_time = (rospy.get_time() - begin_full_time) / 60.0
                status_msg_dict["full_charging_time"] = full_charing_time
                if full_charing_time >= self.charging_full_time:
                    begin_auto_charger_jack_connect = rospy.get_time()
                    if not disconnect_charger:
                        disconnect_charger = True
                        self.set_charging_en(OUTPUT_ON)
                if not self.is_charging_on:
                    _state = LoopState.NON_CHARGE
                if self.check_manual_charging():
                    _state = LoopState.MANUAL_CHARGING
               
            # State: IO_ERROR
            elif _state == LoopState.IO_ERROR:
                if rospy.get_time() - self.last_std_io_msg < 1.0:
                    _state = LoopState.INIT

            status_msg_dict["state"] = _state.toString()
            status_msg.data = json.dumps(status_msg_dict, indent=2)
            status_msg.stamp = rospy.Time.now()
            self._asm.module_status_pub.publish(status_msg)

            # Tự động enable sạc ở MANUAL mode
            if (
                self.auto_manual_sw == SWITCH_AUTO_MODE
                and self.auto_charging_detected == SENSOR_ACTIVATE
            ) and self.auto_charging_en_pin == SENSOR_DEACTIVATE:
                if (rospy.get_time() - begin_auto_charger_jack_connect > 2.0) and  not self.check_full_charging() and self.receive_battery_percent and self.check_in_charging_range():
                    if self.set_charging_en(OUTPUT_ON):
                        begin_auto_charger_jack_connect = rospy.get_time()
                        rospy.logwarn("Auto activate AUTO_CHARGING")

            prev_auto_man = self.auto_manual_sw
            prev_auto_charging_detected = self.auto_charging_detected
            r.sleep()
        
    def check_manual_charging(self):
        if (
            self.manual_charging_detected == SENSOR_ACTIVATE
            or self.auto_charging_detected == SENSOR_ACTIVATE
            and self.auto_manual_sw == SWITCH_MANUAL_MODE
        ):
            if self.is_charging_on:
                return True
        return False

    def check_auto_charging(self):
        if (
            self.auto_charging_detected == SENSOR_ACTIVATE
            and self.auto_manual_sw == SWITCH_AUTO_MODE
            and self.manual_charging_detected == SENSOR_DEACTIVATE
        ):
            if self.is_charging_on:
                return True
        return False

    def check_full_charging(self):
        if (
            self.battery_percentage >= self.max_percent_to_charge
            and self.manual_charging_detected == SENSOR_DEACTIVATE
            and self.auto_charging_detected == SENSOR_ACTIVATE
            and self.auto_manual_sw == SWITCH_AUTO_MODE
        ):
            return True
        return False

    def check_in_charging_range(self):
        if (
            self.battery_percentage < self.percent_to_charge_again
            and self.manual_charging_detected == SENSOR_DEACTIVATE
            and self.auto_charging_detected == SENSOR_ACTIVATE
            and self.auto_manual_sw == SWITCH_AUTO_MODE
        ):
            return True
        return False
    
    def check_low_voltage(self):
        return False


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
            rospkg.RosPack().get_path("charging_process"),
            "cfg",
            "charging_process.yaml",
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
    rospy.init_node("charging_process", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    ChargingAction(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
