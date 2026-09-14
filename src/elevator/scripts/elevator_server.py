#! /usr/bin/env python
# -*- coding: utf-8 -*-
import threading
from datetime import datetime
import agf_mc_protocol

plc = agf_mc_protocol

from time import sleep
from logging import debug
from bson.json_util import dumps
import os
import sys
import rospy
import rospkg
import tf
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import copy
import actionlib
import time
import dynamic_reconfigure.client
from dynamic_reconfigure.server import Server
import requests
import yaml
from geometry_msgs.msg import (
    Twist,
    Pose,
    PoseStamped,
    Quaternion,
    PoseWithCovarianceStamped,
)
from nav_msgs.msg import Odometry
import json
from std_msgs.msg import Bool, Int16, Int8, String, Float32
from math import sqrt, pow, pi, sin, cos, atan2, degrees
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
from cognex_qr_code.srv import *

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
from mongodb import mongodb, LogLevel, MissionStatus
from enum import Enum
from module_manager import ModuleServer, ModuleClient, ModuleStatus
from common_function import (
    EnumString,
    lockup_pose,
    offset_pose_x,
    offset_pose_yaw,
    SENSOR_DEACTIVATE,
    SENSOR_ACTIVATE,
    delta_angle,
    dict_to_obj,
    merge_two_dicts,
    print_debug,
    print_info,
    print_warn,
    print_error,
    obj_to_dict,
    offset_pose_xy_theta,
    distance_two_pose,
    YamlDumper,
    distance_two_points,
)

from os.path import expanduser
from agv_msgs.msg import ErrorRobotToPath

HOME = expanduser("~")


class MainState(EnumString):
    NONE = -1
    SEND_DOCKING_HUB = 0
    DOCKING_TO_HUB = 1
    CHECK_CART = 2
    LIFT_MAX = 3
    LIFT_MIN = 4
    DONE = 8
    MOVING_ERROR = 10
    PAUSED = 12
    WAITING = 13
    SEND_GOTO_WAITING = 14
    GOING_TO_WAITING = 15
    SEND_GOTO_OUT_OF_HUB = 23
    GOING_TO_OUT_OF_HUB = 24
    MOVING_DISCONNECTED = 28
    INIT = 29
    LIFT_POSITION_WRONG = 30
    NO_CART = 31
    OPTICAL_SENSOR_ERROR = 32
    EMG_AGV = 33
    LECH_TAM = 34
    ALIGNMENT_SENSOR = 35
    LIFT_MIN_END = 36
    LIFT_MIN_FIRST = 37
    LIFT_MAX_FIRST = 38
    UPDATE_CART_ERROR = 44
    UNABLE_PLACE_CART = 47
    WRONG_CART = 48
    PAUSED_BY_ELEVATOR = 49
    COLLISION_POSSIBLE = 50
    EMG_ELEVATOR = 60
    TIMEOUT_WHEN_WAIT_ELEVATOR_ALLOW_MOVE = 70
    NETWORK_ERROR = 71
    WAIT_RESET_IO = 72


class RunType(Enum):
    NONE = -1
    GO_NOMAL = 0
    GO_DOCKING = 1
    GO_OUT_DOCKING = 2
    STOP_ACCURACY = 3
    STOP_BY_CROSS_LINE = 4


class MainStatePlace(EnumString):
    NONE = -1
    CHECK_ELEVATOR_POSIBLE = 200
    REQUEST_ENTER_ELEVATOR = 201
    CHECK_ENTER_POSSIBLE = 202
    CHECK_AGV_PLACE_COMPLETE = 203
    DONE_CARRY_IN = 204
    CARRY_IN_POSSIBLE = 205


class MainStatePick(EnumString):
    NONE = -1
    CHECK_ELEVATOR_POSIBLE = 100
    REQUEST_ENTER_ELEVATOR = 101
    CHECK_ENTER_POSSIBLE = 102
    CHECK_AGV_PICK_COMPLETE = 103
    DONE_CARRY_OUT = 104
    PAUSE = 105


class ElevatorName(EnumString):
    ELEVATOR_A = 0
    ELEVATOR_B = 1
    ELEVATOR_C = 2
    ELEVATOR_D = 3


PICK = 1
PLACE = 0
ON = 1
OFF = 0
LIFT_UP = 1
LIFT_DOWN = 2
FAKE_QR_CODE = False
FORWARD = 1
BACKWARD = 0

# INPUT PLC
carry_in_request = [1030, 1031, 1032]
carry_in_completed = [1040, 1041, 1042]
carry_out_request = [1050, 1051, 1052]
carry_out_completion = [1060, 1061, 1062]
destination_ST_indication = [100, 102, 104]
carry_in_ID = [120, 121, 122]
request_door_open = [1090, 1091, 1092]
id_floor_destination = [1001, 1002, 1003]

# OUTPUT PLC
carry_in_instruction_possible = [1020, 1021, 1022]
carry_in_possible = [1050, 1051, 1052]
loading_and_unloading = [1060, 1061, 1062]
carrying_out_possible = [1070, 1071, 1072]
completion_ACK = [1080, 1081, 1082]
carry_out_ID = [30, 31, 32]
autorator_enter_stop = 1005
emg_elevator = 1001
# fmt: off
x_value_address = [
    1000,
    1001,
    1002,
    1003,
    1004,
    1030,
    1031,
    1032,
    1040,
    1041,
    1042,
    1050,
    1051,
    1052,
    1060,
    1061,
    1062,
    1070
]
y_value_address = [
    1004,
    1005,
    1006,
    1020,
    1021,
    1022,
    1050,
    1051,
    1052,
    1060,
    1061,
    1062,
    1070,
    1071,
    1072,
    1080,
    1081,
    1082,
]
# fmt: on
w_value_address_input = [100, 101, 102, 103, 104, 105, 120, 121, 122]
w_value_address_output = [15, 30, 31, 32]

pre_x_value = []
pre_y_value = []
pre_w_value_input = []
pre_w_value_output = []


class ElevatorAction(object):
    _feedback = StringFeedback()
    _result = StringResult()

    def __init__(self, name, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        if not self.load_config():
            return
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
        self.moving_control_client = actionlib.SimpleActionClient(
            "/moving_control", StringAction
        )
        self.moving_control_client.wait_for_server()
        rospy.on_shutdown(self.shutdown)
        # Publisher
        self.disable_check_error_qr_code_pub = rospy.Publisher(
            "/disable_check_error_qr_code", Int8Stamped, queue_size=5
        )
        self.status_io_pub = rospy.Publisher(
            "/status_elevator_io", StringStamped, queue_size=10
        )
        self.safety_job_pub = rospy.Publisher(
            "/safety_job_name", StringStamped, queue_size=5
        )
        self.moving_control_run_pause_pub = rospy.Publisher(
            "/moving_control/run_pause_req", StringStamped, queue_size=5
        )
        self.moving_control_reset_error_pub = rospy.Publisher(
            "/moving_control/reset_error", EmptyStamped, queue_size=5
        )
        self.pub_lift_cmd = rospy.Publisher(
            "/lift_cart", Int8Stamped, queue_size=10
        )
        self.pub_vel = rospy.Publisher(
            "/retry_docking_cmd_vel", Twist, queue_size=5
        )
        self.pub_continue_run = rospy.Publisher(
            "/request_run_stop", StringStamped, queue_size=10
        )
        # Subscriber
        rospy.Subscriber(
            "/error_robot_to_path",
            ErrorRobotToPath,
            self.error_robot_to_path_cb,
        )
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/standard_io", StringStamped, self.standard_io_cb)
        rospy.Subscriber(
            "/moving_control/result",
            StringResult,
            self.moving_control_result_cb,
        )
        rospy.Subscriber(
            "/moving_control/module_status",
            StringStamped,
            self.moving_control_module_status_cb,
        )
        rospy.Subscriber("/robot_status", StringStamped, self.robot_status_cb)
        # Service client
        self.get_qr_code = rospy.ServiceProxy("ReadQrCode", QrCode)
        # dynamic reconfig client
        self.client_reconfig_movebase = dynamic_reconfigure.client.Client(
            "/move_base/NeoLocalPlanner",
            timeout=300,
            config_callback=self.dynamic_callback,
        )
        # Service client
        self.get_qr_code = rospy.ServiceProxy("ReadQrCode", QrCode)

        # Thread monitor data plc
        self.reset_value_all()
        self.start_thread = False
        self.print_first = True
        self.thread_publish = threading.Thread(
            name="read_data_elevator", target=self.read_data_elevator
        )
        self.thread_publish.daemon = True
        self.thread_publish.start()

        # ModuleServer
        self._asm = ModuleServer(name)
        self.init_server()
        # Loop
        self.loop()

    def init_variable(self, *args, **kwargs):
        self.config_path = kwargs["config_path"]
        self.robot_config_file = kwargs["robot_config_file"]
        self.server_config_file = kwargs["robot_define"]
        self.use_tf2 = False
        self.tf_listener = tf.TransformListener()
        #
        self.last_moving_control_fb = rospy.get_time()
        self.moving_control_result = -1
        #
        self.moving_control_error_code = ""
        # Database
        db_address = rospy.get_param("/mongodb_address")
        print_debug(db_address)
        self.db = mongodb(db_address)
        self.emg_status = True
        self.liftup_finish = False
        self.detect_vrack = False
        self.liftdown_finish = False
        self.plc_address = rospy.get_param("~plc_address", "192.168.20.65")
        self.port = rospy.get_param("~plc_port", 2002)

        self.lift_msg = Int8Stamped()
        self.disable_qr_code_msg = Int8Stamped()
        self.std_io_msg = StringStamped()
        self.last_time_get_lift_up = rospy.get_time()
        self.last_time_get_lift_down = rospy.get_time()

        self.vel_move_base = 0.0

        self.elevator_name = None
        elevator_name_str = rospy.get_param("~elevator_name", "ELEVATOR_B")
        if elevator_name_str == "ELEVATOR_B":
            self.elevator_name = ElevatorName.ELEVATOR_B
        elif elevator_name_str == "ELEVATOR_A":
            self.elevator_name = ElevatorName.ELEVATOR_A
        # Add more elevator names here if necessary

        self.resetTimeoutError = False
        self.is_plc_connect_fail = False

        self.data_run = StringStamped()
        self.data_run.data = "RUN"
        self.mode_robot = ""

    def shutdown(self):
        # self.auto_docking_client.cancel_all_goals()
        if plc.check_plc_connected():
            plc.plc_close()
        self.dynamic_reconfig_movebase(self.vel_move_base, True)
        self.moving_control_client.cancel_all_goals()

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

    def robot_status_cb(self, msg):
        robot_status = json.loads(msg.data)
        if "status" in robot_status:
            self.status_robot = robot_status["status"]
        if "mode" in robot_status:
            self.mode_robot = robot_status["mode"]

    def error_robot_to_path_cb(self, msg):
        self.error_position = msg.error_position
        self.error_angle = msg.error_angle

    def odom_cb(self, msg):
        self.pose_odom2robot = msg.pose.pose

    def standard_io_cb(self, msg):
        data = json.loads(msg.data)
        if "lift_max_sensor" in data:
            if data["lift_max_sensor"] and (
                rospy.get_time() - self.last_time_get_lift_up >= 2
            ):
                self.liftup_finish = True
            if not data["lift_max_sensor"]:
                self.last_time_get_lift_up = rospy.get_time()
                self.liftup_finish = False
        if "lift_min_sensor" in data:
            if data["lift_min_sensor"] and (
                rospy.get_time() - self.last_time_get_lift_down >= 2
            ):
                self.liftdown_finish = True
            if not data["lift_min_sensor"]:
                self.last_time_get_lift_down = rospy.get_time()
                self.liftdown_finish = False
        if "emg_button" in data:
            self.emg_status = data["emg_button"]
        if "start_1_button" in data:
            self.start_1 = data["start_1_button"]
        if "start_2_button" in data:
            self.start_2 = data["start_2_button"]
        if "stop_1_button" in data:
            self.stop_1 = data["stop_1_button"]
        if "stop_2_button" in data:
            self.stop_2 = data["stop_2_button"]
        # if "detect_vrack" in data: self.detect_vrack = data["detect_vrack"]
        self.detect_vrack = False

    def dynamic_callback(config, level):
        # rospy.loginfo("Suceeed change vel of robot")
        pass

    def moving_control_fb(self, msg):
        self.last_moving_control_fb = rospy.get_time()

    def moving_control_result_cb(self, msg):
        self.moving_control_result = msg.status.status
        # rospy.logerr(self.moving_control_result)

    def moving_control_module_status_cb(self, msg):
        try:
            self.moving_control_error_code = json.loads(msg.data)["error_code"]
        except Exception as e:
            rospy.logerr("moving_control_module_status_cb: {}".format(e))

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
        while True:
            if self.get_odom():
                break
        # self.trans[0], self.trans[1]
        use_server = True
        hub_type = "hub"
        try:
            # Hub config
            hub_cfg_file = os.path.join(self.config_path, hub_type + ".json")
            with open(hub_cfg_file) as j:
                hub_dict = json.load(j)
                dist_check_go_in = hub_dict["dist_check_go_in"]
                dist_check_go_out = hub_dict["dist_check_go_out"]
                max_error_position_in_hub = hub_dict[
                    "max_error_position_in_hub"
                ]
                max_error_angle_in_hub = hub_dict["max_error_angle_in_hub"]
                max_error_position_out_hub = hub_dict[
                    "max_error_position_out_hub"
                ]
                max_error_angle_out_hub = hub_dict["max_error_angle_out_hub"]
                safety_job_docking_forward = hub_dict[
                    "safety_job_docking_forward"
                ]
                safety_job_docking_backward = hub_dict[
                    "safety_job_docking_backward"
                ]
                safety_job_undocking_forward = hub_dict[
                    "safety_job_undocking_forward"
                ]
                safety_job_undocking_backward = hub_dict[
                    "safety_job_undocking_backward"
                ]
                footprint_dock = hub_dict["footprint_dock"]
                foorprint_undock = hub_dict["foorprint_undock"]
                enable_check_error_when_docking = hub_dict[
                    "enable_check_error_when_docking"
                ]
                distance_turn_off_safety_when_docking = hub_dict[
                    "distance_turn_off_safety_when_docking"
                ]
                vel_docking_hub = hub_dict["max_vel_docking"]
                if "safety_job_rotation" in hub_dict:
                    safety_job_rotation = hub_dict["safety_job_rotation"]
                else:
                    safety_job_rotation = "ROTATION"
            # Waiting path config
            waiting_path_cfg = os.path.join(
                self.config_path, "waiting_path.json"
            )
            with open(waiting_path_cfg) as j:
                waiting_path_dict = json.load(j)
                return_pose_dict = waiting_path_dict["waypoints"][0]["position"]
            # Docking path config
            docking_path_cfg = os.path.join(
                self.config_path, "docking_path.json"
            )
            with open(docking_path_cfg) as j:
                docking_path_dict = json.load(j)
            # UnDocking path config
            undocking_path_cfg = os.path.join(
                self.config_path, "undocking_path.json"
            )
            with open(undocking_path_cfg) as j:
                undocking_path_dict = json.load(j)
        except Exception as e:
            rospy.logerr("Read config file error: {}".format(e))
            self._as.set_aborted("Read config file error")
            return

        # Load data from action
        data_dict = json.loads(goal.data)
        source_floor = int(data_dict["params"]["source_floor"])
        destination_floor = int(data_dict["params"]["destination_floor"])
        rospy.logwarn(data_dict)
        direction = BACKWARD
        self.enable_safety = True
        try:
            hub_pose_x = data_dict["params"]["position"]["x"]
            hub_pose_y = data_dict["params"]["position"]["y"]
            waiting_pose_x = data_dict["params"]["waiting_position"]["x"]
            waiting_pose_y = data_dict["params"]["waiting_position"]["y"]
            self.type = "ELEVATOR"
            self.name = data_dict["params"]["name"]
            self.cell = 0  # data_dict["params"]["cell"]
            self.cart = data_dict["params"]["cart"]
            self.lot = data_dict["params"]["lot"]
            if "properties" in data_dict["params"]:
                if "Safety" in data_dict["params"]["properties"]:
                    if data_dict["params"]["properties"]["Safety"] == "Disable":
                        self.enable_safety = False
                if "Invert" in data_dict["params"]["properties"]:
                    direction = data_dict["params"]["properties"]["invert"]
        except:
            hub_pose_x = data_dict["params"]["position"]["position"]["x"]
            hub_pose_y = data_dict["params"]["position"]["position"]["y"]
            waiting_pose_x = self.trans[0]
            waiting_pose_y = self.trans[1]
            self.type = "ELEVATOR"
            self.name = "AGV 01"
            self.cell = 0
            self.cart = "VRACK"
            self.lot = "1"
            use_server = False
        if use_server:
            FAKE_QR_CODE = False
        else:
            FAKE_QR_CODE = True
        if direction == FORWARD:
            cur_orient = atan2(
                hub_pose_y - waiting_pose_y, hub_pose_x - waiting_pose_x
            )
        else:
            cur_orient = atan2(
                waiting_pose_y - hub_pose_y, waiting_pose_x - hub_pose_x
            )

        # Calculate waiting goal
        waiting_goal = StringGoal()
        waiting_pose = self.calculate_pose_offset(
            0,
            waiting_pose_x,
            waiting_pose_y,
            cur_orient,
        )
        waiting_path_dict["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(waiting_pose, return_pose_dict)
        )
        waiting_goal.data = json.dumps(waiting_path_dict, indent=2)
        rospy.logwarn(
            "Waiting goal position:\n{}".format(
                json.dumps(waiting_path_dict, indent=2)
            )
        )

        # Calculate docking goal
        docking_goal = StringGoal()
        docking_pose = self.calculate_pose_offset(
            0,
            hub_pose_x,
            hub_pose_y,
            cur_orient,
        )
        docking_path_dict["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(waiting_pose, return_pose_dict)
        )
        docking_path_dict["waypoints"][1]["position"] = copy.deepcopy(
            obj_to_dict(docking_pose, return_pose_dict)
        )
        docking_goal.data = json.dumps(docking_path_dict, indent=2)
        rospy.logwarn(
            "Docking goal position:\n{}".format(
                (json.dumps(docking_path_dict, indent=2))
            )
        )

        # Calculate undocking goal
        undocking_goal = StringGoal()
        undocking_path_dict["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(docking_pose, return_pose_dict)
        )

        undocking_path_dict["waypoints"][1]["position"] = copy.deepcopy(
            obj_to_dict(waiting_pose, return_pose_dict)
        )
        if "param_test" not in undocking_path_dict:
            undocking_path_dict["param_test"] = {}
        undocking_path_dict["param_test"][
            "need_to_wait_receive_new_path"
        ] = True
        undocking_goal.data = json.dumps(undocking_path_dict, indent=2)
        rospy.logwarn(
            "UnDocking goal position:\n{}".format(
                (json.dumps(undocking_path_dict, indent=2))
            )
        )

        pick_or_place = data_dict["params"]["pick_or_place"]
        if pick_or_place:
            goal_type = PICK
            _state = MainState.INIT
        else:
            goal_type = PLACE
            _state = MainState.WAIT_RESET_IO

        r = rospy.Rate(15)
        success = False
        _prev_state = MainState.NONE
        feedback_msg = ""
        _state_when_pause = MainState.NONE
        _state_when_error = MainState.NONE
        _state_when_network_timeout = MainState.NONE
        _state_when_emg_agv = MainState.NONE
        _state_bf_error = MainState.NONE
        self._asm.reset_flag()
        self._asm.action_running = True
        self.pose_map2robot = None
        self.safety_job_name = None
        self.get_first_time_error = True
        self.retry_get_center_tape = 0
        self.pose_waitting = None
        self.step = 0
        self.cmd_vel_msg = Twist()
        distance_to_hub = 0

        self.reset_value_all()

        is_pause_by_elevator = False
        is_preemted = False
        is_send_goal_out = False

        on_bit = True
        t1 = datetime.now()

        self.start_thread = False
        self.print_first = False

        time_send_reset_elevator = rospy.get_time()
        timeout_waiting_elevator_respond = rospy.get_time()
        begin_check_timeout = True
        last_reset_action = False
        couter_connect_error = 0
        first_check_timout = True
        begin_time_ok = rospy.get_time()
        self.is_plc_connect_fail = True
        if plc.plc_connect(self.plc_address, self.port):
            rospy.sleep(1)
            rospy.logwarn("connect plc success first time")
        else:
            rospy.logwarn("connect plc false first time")
        first_go_to_waiting = True
        is_emg_elevator = False
        last_time_pub_safety_job = rospy.get_time()
        self.pre_safety_job_name = None
        self.reset_success_first_time = False
        self.reset_success_when_cancle = False

        while not rospy.is_shutdown():

            if plc.plc_connect_fail:
                # plc.plc_connect_fail = False
                couter_connect_error += 1
                self.is_plc_connect_fail = True
            else:
                begin_time_ok = rospy.get_time()
                couter_connect_error = 0
                self.is_plc_connect_fail = False
            try:
                if not plc.check_plc_connected() or plc.plc_connect_fail:
                    if first_check_timout:
                        if (
                            _state == MainState.GOING_TO_OUT_OF_HUB
                            or _state == MainState.DOCKING_TO_HUB
                        ):
                            self.moving_control_run_pause_pub.publish(
                                StringStamped(
                                    stamp=rospy.Time.now(), data="PAUSE"
                                )
                            )
                        _state_when_network_timeout = _state
                        first_check_timout = False
                        rospy.sleep(0.5)
                    _state = MainState.NETWORK_ERROR
            except Exception as e:
                rospy.logerr(e)

            # self.read_value_x()
            # self.read_value_y()
            # self.read_value_w_input()
            # self.read_value_w_output()
            # self.pub_io()
            # if (datetime.now() - t1).seconds >= 1:
            #     if on_bit:
            #         plc.write_x(1000, [1])
            #         t1 = datetime.now()
            #         on_bit = False
            #     else:
            #         plc.write_x(1000, [0])
            #         t1 = datetime.now()
            #         on_bit = True
            if _state != MainState.NETWORK_ERROR:
                plc.write_x(1001, [1])
            if not self.get_odom():
                continue
            distance_to_hub = distance_two_points(
                self.pose_map2robot.position.x,
                self.pose_map2robot.position.y,
                hub_pose_x,
                hub_pose_y,
            )
            if (
                self._as.is_preempt_requested()
                or self._asm.reset_action_req
                or is_preemted
            ):
                self._asm.reset_flag()
                if is_send_goal_out:
                    rospy.logerr(
                        "reset_action_req : {}".format(
                            self._asm.reset_action_req
                        )
                    )
                    rospy.loginfo("%s: Preempted" % self._action_name)
                    # self.auto_docking_client.cancel_all_goals()
                    self.moving_control_client.cancel_all_goals()
                    self.dynamic_reconfig_movebase(self.vel_move_base, True)
                    self.start_thread = False
                    if last_reset_action:
                        self._as.set_preempted()
                        success = False
                        self.send_feedback(
                            self._as, GoalStatus.to_string(GoalStatus.PREEMPTED)
                        )
                        break
                    else:
                        last_reset_action = True
                        _state = MainState.DONE
                else:
                    if not is_preemted:
                        rospy.logerr(
                            "reset_action_req : {}".format(
                                self._asm.reset_action_req
                            )
                        )
                        rospy.loginfo("%s: Preempted" % self._action_name)
                        success = False
                        # self.auto_docking_client.cancel_all_goals()
                        self.moving_control_client.cancel_all_goals()
                        self.start_thread = False
                        time_send_reset_elevator = rospy.get_time()
                        self.reset()
                    if _state != MainState.NETWORK_ERROR:
                        plc.write_x(1070, [1])
                        if plc.read_x(1070, 1)[0] != 1:
                            rospy.logerr(
                                "Checking for x[1070] = 1 to write to PLC"
                            )
                        if plc.read_y(1008, 1)[0] == 1:
                            rospy.sleep(0.5)
                            time_send_reset_elevator = rospy.get_time()
                            while True:
                                plc.write_x(1070, [0])
                                if (
                                    plc.read_x(1070, 1)[0] == 0
                                    and not plc.plc_connect_fail
                                ) or (
                                    rospy.get_time() - time_send_reset_elevator
                                    > 5
                                ):
                                    rospy.logwarn("CANCLE ACTION AND RESET PLC")
                                    rospy.logwarn("Close connect to plc")
                                    self._as.set_preempted()
                                    self.send_feedback(
                                        self._as,
                                        GoalStatus.to_string(
                                            GoalStatus.PREEMPTED
                                        ),
                                    )
                                    break
                                else:
                                    rospy.logerr(
                                        "wait for write x(1070, 1)[0] = 0 to plc"
                                    )
                            break
                        else:
                            rospy.logerr("wait for read_y(1008, 1)[0] == 1")
                            if rospy.get_time() - time_send_reset_elevator > 10:
                                rospy.logwarn(
                                    "CANNOT RESET PLC BECAUSE TIMEOUT"
                                )
                                rospy.logwarn("Close connect to plc")
                                self._as.set_preempted()
                                self.send_feedback(
                                    self._as,
                                    GoalStatus.to_string(GoalStatus.PREEMPTED),
                                )
                                break
                    else:
                        rospy.logwarn("CANNOT RESET PLC BECAUSE TIMEOUT")
                        rospy.logwarn("Close connect to plc")
                        self._as.set_preempted()
                        self.send_feedback(
                            self._as, GoalStatus.to_string(GoalStatus.PREEMPTED)
                        )
                        break
                    is_preemted = True
            if self._asm.module_status != ModuleStatus.ERROR:
                self._asm.error_code = ""
            if _state != MainState.PAUSED and self._asm.error_code == "":
                self._asm.module_status = ModuleStatus.RUNNING
            if (
                _state != MainState.PAUSED
                and _state != MainState.NETWORK_ERROR
                and self._asm.error_code == ""
                and not is_preemted
            ):
                if (
                    not plc.read_y(emg_elevator, 1)[0]
                    and not plc.plc_connect_fail
                    and not is_pause_by_elevator
                    and not is_emg_elevator
                ):
                    rospy.sleep(0.2)
                    if (
                        not plc.read_y(autorator_enter_stop, 1)[0]
                        and not plc.plc_connect_fail
                    ):
                        self.moving_control_run_pause_pub.publish(
                            StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                        )
                        rospy.logwarn("EMG ELEVATOR")
                        _state_when_error = _state
                        _state = MainState.EMG_ELEVATOR
                        is_emg_elevator = True
            if _prev_state != _state:
                begin_check_timeout = True
                # Record log
                # self.db.recordLog(
                #     "Action state: {} -> {}".format(
                #         _prev_state.toString(), _state.toString()
                #     ),
                #     rospy.get_name(),
                #     LogLevel.INFO.toString(),
                # )
                rospy.loginfo(
                    "Action state: {} -> {}".format(
                        _prev_state.toString(), _state.toString()
                    )
                )
                _prev_state = _state
                feedback_msg = _state.toString()
                self._asm.module_state = _state.toString()
            self.send_feedback(self._as, feedback_msg)

            # """
            # .####.##....##.####.########
            # ..##..###...##..##.....##...
            # ..##..####..##..##.....##...
            # ..##..##.##.##..##.....##...
            # ..##..##..####..##.....##...
            # ..##..##...###..##.....##...
            # .####.##....##.####....##...
            # """
            # State:
            if _state == MainState.INIT:
                self.vel_move_base = rospy.get_param(
                    "/move_base/NeoLocalPlanner/max_vel_x"
                )
                self.dynamic_reconfig_movebase(vel_docking_hub, False)
                _state = MainState.SEND_GOTO_WAITING
                if self._asm.pause_req:
                    # self.moving_control_client.cancel_all_goals()
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            elif _state == MainState.WAIT_RESET_IO:
                if not self.reset_success_first_time:
                    if self.reset():
                        self.reset_success_first_time = True
                        rospy.logwarn("reset all value x and w input success")
                        _state = MainState.INIT
                    else:
                        rospy.logwarn("reset all value x and w input false")
                else:
                    _state = MainState.INIT

                if self._asm.pause_req:
                    # self.moving_control_client.cancel_all_goals()
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # State: SEND_GOTO_WAITING
            elif _state == MainState.SEND_GOTO_WAITING:
                self.moving_control_client.send_goal(
                    waiting_goal,
                    feedback_cb=self.moving_control_fb,
                )
                self.moving_control_result = -1
                self.last_moving_control_fb = rospy.get_time()
                _state = MainState.GOING_TO_WAITING
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # """
            # ..######....#######..........##......##....###....####.########.####.##....##..######..
            # .##....##..##.....##.........##..##..##...##.##....##.....##.....##..###...##.##....##.
            # .##........##.....##.........##..##..##..##...##...##.....##.....##..####..##.##.......
            # .##...####.##.....##.........##..##..##.##.....##..##.....##.....##..##.##.##.##...####
            # .##....##..##.....##.........##..##..##.#########..##.....##.....##..##..####.##....##.
            # .##....##..##.....##.........##..##..##.##.....##..##.....##.....##..##...###.##....##.
            # ..######....#######..#######..###..###..##.....##.####....##....####.##....##..######..
            # """
            # State: GOING_TO_WAITING
            elif _state == MainState.GOING_TO_WAITING:
                if self.enable_safety and first_go_to_waiting:
                    self.safety_job_name = safety_job_rotation
                else:
                    self.safety_job_name = ""
                if self.moving_control_result == GoalStatus.SUCCEEDED:
                    if goal_type == PICK:
                        _state = MainState.LIFT_MIN_FIRST
                    else:
                        _state = MainState.LIFT_MAX_FIRST
                elif (
                    self.moving_control_result != GoalStatus.SUCCEEDED
                    and self.moving_control_result != GoalStatus.ACTIVE
                    and self.moving_control_result != -1
                ) or self.moving_control_error_code != "":
                    rospy.logerr(
                        "Go to waiting fail: {}".format(
                            GoalStatus.to_string(self.moving_control_result)
                        )
                    )
                    _state_bf_error = MainState.SEND_GOTO_WAITING
                    _state_when_error = _state
                    _state = MainState.MOVING_ERROR
                if rospy.get_time() - self.last_moving_control_fb >= 5.0:
                    rospy.logerr("/moving control disconnected!")
                    self.send_feedback(
                        self._as, GoalStatus.to_string(GoalStatus.ABORTED)
                    )
                    _state_bf_error = MainState.SEND_GOTO_WAITING
                    _state_when_error = _state
                    _state = MainState.MOVING_DISCONNECTED
                if is_pause_by_elevator:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_error = _state
                    _state = MainState.PAUSED_BY_ELEVATOR
                else:
                    if self._asm.pause_req:
                        self._asm.reset_flag()
                        self.moving_control_run_pause_pub.publish(
                            StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                        )
                        _state_when_pause = _state
                        _state = MainState.PAUSED

            # State: SEND_DOCKING_HUB
            elif _state == MainState.SEND_DOCKING_HUB:
                first_go_to_waiting = False
                check_go_in = True
                self.moving_control_client.send_goal(
                    docking_goal,
                    feedback_cb=self.moving_control_fb,
                )
                self.moving_control_result = -1
                self.last_moving_control_fb = rospy.get_time()
                _state = MainState.DOCKING_TO_HUB
                # else:
                #     _state = MainState.OPTICAL_SENSOR_ERROR
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # """
            # .########...#######...######..##....##.####.##....##..######..
            # .##.....##.##.....##.##....##.##...##...##..###...##.##....##.
            # .##.....##.##.....##.##.......##..##....##..####..##.##.......
            # .##.....##.##.....##.##.......#####.....##..##.##.##.##...####
            # .##.....##.##.....##.##.......##..##....##..##..####.##....##.
            # .##.....##.##.....##.##....##.##...##...##..##...###.##....##.
            # .########...#######...######..##....##.####.##....##..######..
            # """
            # State: DOCKING_TO_HUB
            elif _state == MainState.DOCKING_TO_HUB:
                if plc.read_y(autorator_enter_stop, 1)[0]:
                    self.disable_qr_code_msg.stamp = rospy.Time.now()
                    self.disable_qr_code_msg.data = 1
                    self.disable_check_error_qr_code_pub.publish(
                        self.disable_qr_code_msg
                    )
                    self.moving_control_client.cancel_all_goals()
                    rospy.sleep(2)
                    _state = MainState.SEND_GOTO_WAITING
                    is_pause_by_elevator = True
                    continue
                if direction == FORWARD:
                    if self.enable_safety:
                        self.safety_job_name = safety_job_docking_forward
                    else:
                        self.safety_job_name = ""
                else:
                    if self.enable_safety:
                        self.safety_job_name = safety_job_docking_backward
                    else:
                        self.safety_job_name = ""
                if distance_to_hub < distance_turn_off_safety_when_docking:
                    self.safety_job_name = ""
                if enable_check_error_when_docking:
                    if (
                        distance_to_hub > dist_check_go_in
                        and distance_to_hub < dist_check_go_in + 0.2
                    ):
                        if self.detect_vrack and goal_type == PLACE:
                            _state_bf_error = MainState.SEND_DOCKING_HUB
                            _state_when_error = _state
                            _state = MainState.UNABLE_PLACE_CART
                            self.moving_control_client.cancel_all_goals()
                            continue
                        disable_auto_get_center_tape = False

                        if (
                            abs(self.error_position)
                            >= max_error_position_out_hub
                        ):
                            if self.get_first_time_error:
                                self.get_first_time_error = False
                                first_time_error = rospy.get_time()
                            if (
                                rospy.get_time() - first_time_error > 0.5
                            ) or True:
                                self.get_first_time_error = True
                                _state_bf_error = MainState.SEND_DOCKING_HUB
                                _state_when_error = _state
                                _state = MainState.ALIGNMENT_SENSOR
                                self.step = 0
                                self._asm.reset_flag()
                                self.moving_control_run_pause_pub.publish(
                                    StringStamped(
                                        stamp=rospy.Time.now(),
                                        data="PAUSE",
                                    )
                                )
                                continue
                        else:
                            self.get_first_time_error = True
                            if abs(self.error_angle) >= max_error_angle_out_hub:
                                _state_bf_error = MainState.SEND_DOCKING_HUB
                                _state_when_error = _state
                                _state = MainState.ALIGNMENT_SENSOR
                                self.step = 0
                                self._asm.reset_flag()
                                self.moving_control_run_pause_pub.publish(
                                    StringStamped(
                                        stamp=rospy.Time.now(),
                                        data="PAUSE",
                                    )
                                )
                                continue
                    elif distance_to_hub <= dist_check_go_in:
                        disable_auto_get_center_tape = True
                        self.get_first_time_error = True
                        if (
                            abs(self.error_angle) >= max_error_angle_in_hub
                            or abs(self.error_position)
                            >= max_error_position_in_hub
                        ):
                            _state_bf_error = MainState.SEND_DOCKING_HUB
                            _state_when_error = _state
                            _state = MainState.COLLISION_POSSIBLE
                            self.moving_control_client.cancel_all_goals()
                            # self.moving_control_run_pause_pub.publish(
                            #     StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                            # )
                            continue
                if self.moving_control_result == GoalStatus.SUCCEEDED:
                    _state = MainState.CHECK_CART
                    continue
                elif (
                    self.moving_control_result != GoalStatus.SUCCEEDED
                    and self.moving_control_result != GoalStatus.ACTIVE
                    and self.moving_control_result != -1
                ) or self.moving_control_error_code != "":
                    rospy.logerr(
                        "Go to waiting fail: {}".format(
                            GoalStatus.to_string(self.moving_control_result)
                        )
                    )
                    _state_bf_error = MainState.SEND_GOTO_WAITING
                    _state_when_error = _state
                    _state = MainState.MOVING_ERROR
                    continue
                if rospy.get_time() - self.last_moving_control_fb >= 5.0:
                    rospy.logerr("/moving control disconnected!")
                    self.send_feedback(
                        self._as, GoalStatus.to_string(GoalStatus.ABORTED)
                    )
                    _state_bf_error = MainState.SEND_DOCKING_HUB
                    _state_when_error = _state
                    _state = MainState.MOVING_DISCONNECTED
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED
                    continue

            # """
            # ..######..##.....##.########..######..##....##..........######.....###....########..########
            # .##....##.##.....##.##.......##....##.##...##..........##....##...##.##...##.....##....##...
            # .##.......##.....##.##.......##.......##..##...........##........##...##..##.....##....##...
            # .##.......#########.######...##.......#####............##.......##.....##.########.....##...
            # .##.......##.....##.##.......##.......##..##...........##.......#########.##...##......##...
            # .##....##.##.....##.##.......##....##.##...##..........##....##.##.....##.##....##.....##...
            # ..######..##.....##.########..######..##....##.#######..######..##.....##.##.....##....##...
            # """
            # State: CHECK_CART
            elif _state == MainState.CHECK_CART:
                if goal_type == PICK:
                    _state = MainState.LIFT_MAX
                else:
                    _state = MainState.LIFT_MIN
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # """
            # .##.......####.########.########.........##.....##....###....##.....##
            # .##........##..##..........##............###...###...##.##....##...##.
            # .##........##..##..........##............####.####..##...##....##.##..
            # .##........##..######......##............##.###.##.##.....##....###...
            # .##........##..##..........##............##.....##.#########...##.##..
            # .##........##..##..........##............##.....##.##.....##..##...##.
            # .########.####.##..........##....#######.##.....##.##.....##.##.....##
            # """
            # State: LIFT_MAX
            elif _state == MainState.LIFT_MAX:
                if self.liftup_finish:
                    if not FAKE_QR_CODE and self.server_config != None:
                        if self.upDateCart(
                            self.type, self.name, self.cell, "", ""
                        ) and self.upDateCart(
                            "AGV", self.data, 0, self.cart, self.lot
                        ):
                            rospy.sleep(1)
                            _state = MainState.SEND_GOTO_OUT_OF_HUB
                        else:
                            # _state = MainState.UPDATE_CART_ERROR
                            rospy.logwarn("UPDATE_CART_ERROR --> RETRY")
                    else:
                        rospy.sleep(1)
                        _state = MainState.SEND_GOTO_OUT_OF_HUB
                    if self.lot == "":
                        self.db.saveStatusCartData(
                            "status_cart", "have_empty_cart"
                        )
                    else:
                        self.db.saveStatusCartData(
                            "status_cart", "have_full_cart"
                        )
                else:
                    self.lift_msg.stamp = rospy.Time.now()
                    self.lift_msg.data = LIFT_UP
                    self.pub_lift_cmd.publish(self.lift_msg)
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # """
            # .##.......####.########.########.........##.....##.####.##....##
            # .##........##..##..........##............###...###..##..###...##
            # .##........##..##..........##............####.####..##..####..##
            # .##........##..######......##............##.###.##..##..##.##.##
            # .##........##..##..........##............##.....##..##..##..####
            # .##........##..##..........##............##.....##..##..##...###
            # .########.####.##..........##....#######.##.....##.####.##....##
            # """
            # State: LIFT_MIN
            elif _state == MainState.LIFT_MIN:
                if self.liftdown_finish:
                    if not FAKE_QR_CODE and self.server_config != None:
                        if self.upDateCart(
                            self.type, self.name, self.cell, self.cart, self.lot
                        ) and self.upDateCart("AGV", self.data, 0, "", ""):
                            rospy.sleep(1)
                            _state = MainState.SEND_GOTO_OUT_OF_HUB
                        else:
                            # _state = MainState.UPDATE_CART_ERROR
                            rospy.logwarn("UPDATE_CART_ERROR --> RETRY")
                    else:
                        rospy.sleep(1)
                        _state = MainState.SEND_GOTO_OUT_OF_HUB
                    self.db.saveStatusCartData("status_cart", "no_cart")
                else:
                    self.lift_msg.stamp = rospy.Time.now()
                    self.lift_msg.data = LIFT_DOWN
                    self.pub_lift_cmd.publish(self.lift_msg)
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # State: LIFT_MAX
            elif _state == MainState.LIFT_MAX_FIRST:
                if self.liftup_finish:
                    _state = MainStatePlace.REQUEST_ENTER_ELEVATOR
                else:
                    self.lift_msg.stamp = rospy.Time.now()
                    self.lift_msg.data = LIFT_UP
                    self.pub_lift_cmd.publish(self.lift_msg)
                if self._asm.pause_req:
                    # self.moving_control_client.cancel_all_goals()
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # State: LIFT_MIN_FIRST
            elif _state == MainState.LIFT_MIN_FIRST:
                if self.liftdown_finish:
                    _state = MainStatePick.REQUEST_ENTER_ELEVATOR
                else:
                    self.lift_msg.stamp = rospy.Time.now()
                    self.lift_msg.data = LIFT_DOWN
                    self.pub_lift_cmd.publish(self.lift_msg)
                if self._asm.pause_req:
                    # self.moving_control_client.cancel_all_goals()
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # State: LIFT_MIN_END
            elif _state == MainState.LIFT_MIN_END:
                if self.liftdown_finish:
                    _state = MainState.DONE
                else:
                    self.lift_msg.stamp = rospy.Time.now()
                    self.lift_msg.data = LIFT_DOWN
                    self.pub_lift_cmd.publish(self.lift_msg)
                if self._asm.pause_req:
                    # self.moving_control_client.cancel_all_goals()
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # State: SEND_GOTO_OUT_OF_HUB
            elif _state == MainState.SEND_GOTO_OUT_OF_HUB:
                is_send_goal_out = True
                check_go_in = False
                self.moving_control_client.send_goal(
                    undocking_goal, feedback_cb=self.moving_control_fb
                )
                self.moving_control_result = -1
                self.last_moving_control_fb = rospy.get_time()
                _state = MainState.GOING_TO_OUT_OF_HUB
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # ..######....#######...........#######..##.....##.########.........##.....##....###....########.########.##.....##....###....##....##
            # .##....##..##.....##.........##.....##.##.....##....##............###...###...##.##......##....##.......##.....##...##.##...###...##
            # .##........##.....##.........##.....##.##.....##....##............####.####..##...##.....##....##.......##.....##..##...##..####..##
            # .##...####.##.....##.........##.....##.##.....##....##............##.###.##.##.....##....##....######...#########.##.....##.##.##.##
            # .##....##..##.....##.........##.....##.##.....##....##............##.....##.#########....##....##.......##.....##.#########.##..####
            # .##....##..##.....##.........##.....##.##.....##....##............##.....##.##.....##....##....##.......##.....##.##.....##.##...###
            # ..######....#######..#######..#######...#######.....##....#######.##.....##.##.....##....##....########.##.....##.##.....##.##....##
            # State: GOING_TO_OUT_OF_HUB
            elif _state == MainState.GOING_TO_OUT_OF_HUB:
                if direction == FORWARD:
                    if self.enable_safety:
                        self.safety_job_name = safety_job_undocking_backward
                    else:
                        self.safety_job_name = ""
                else:
                    if self.enable_safety:
                        self.safety_job_name = safety_job_undocking_forward
                    else:
                        self.safety_job_name = ""

                if enable_check_error_when_docking:
                    if abs(distance_to_hub) < dist_check_go_out:
                        disable_auto_get_center_tape = True
                        self.get_first_time_error = True
                        if (
                            abs(self.error_angle) >= max_error_angle_in_hub
                            or abs(self.error_position)
                            >= max_error_position_in_hub
                        ):
                            _state_bf_error = MainState.SEND_DOCKING_HUB
                            _state_when_error = _state
                            _state = MainState.COLLISION_POSSIBLE
                            self.moving_control_client.cancel_all_goals()
                            continue

                if self.moving_control_result == GoalStatus.SUCCEEDED:
                    # _state = MainState.LIFT_MIN_END
                    _state = MainState.DONE
                elif (
                    self.moving_control_result != GoalStatus.SUCCEEDED
                    and self.moving_control_result != GoalStatus.ACTIVE
                    and self.moving_control_result != -1
                ) or self.moving_control_error_code != "":
                    rospy.logerr(
                        "Go to waiting fail: {}".format(
                            GoalStatus.to_string(self.moving_control_result)
                        )
                    )
                    _state_bf_error = MainState.SEND_GOTO_OUT_OF_HUB
                    _state_when_error = _state
                    _state = MainState.MOVING_ERROR
                if rospy.get_time() - self.last_moving_control_fb >= 5.0:
                    rospy.logerr("/moving control disconnected!")
                    self.send_feedback(
                        self._as, GoalStatus.to_string(GoalStatus.ABORTED)
                    )
                    _state_bf_error = MainState.SEND_GOTO_OUT_OF_HUB
                    _state_when_error = _state
                    _state = MainState.MOVING_DISCONNECTED
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED
            # .##.....##..#######..##.....##.####.##....##..######...........########.########..########...#######..########.
            # .###...###.##.....##.##.....##..##..###...##.##....##..........##.......##.....##.##.....##.##.....##.##.....##
            # .####.####.##.....##.##.....##..##..####..##.##................##.......##.....##.##.....##.##.....##.##.....##
            # .##.###.##.##.....##.##.....##..##..##.##.##.##...####.........######...########..########..##.....##.########.
            # .##.....##.##.....##..##...##...##..##..####.##....##..........##.......##...##...##...##...##.....##.##...##..
            # .##.....##.##.....##...##.##....##..##...###.##....##..........##.......##....##..##....##..##.....##.##....##.
            # .##.....##..#######.....###....####.##....##..######...#######.########.##.....##.##.....##..#######..##.....##

            elif _state == MainState.UNABLE_PLACE_CART:
                # rospy.logwarn(self.error_2_sensor)
                # rospy.logerr(self.error_angle_sensor)
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = (
                    "/matehan_server: {}".format(_state.toString())
                    + self.moving_control_error_code
                )
                self.cmd_vel_msg.angular.z = 0
                self.cmd_vel_msg.linear.x = 0
                self.pub_vel.publish(self.cmd_vel_msg)
                if self._asm.reset_error_request:
                    self.disable_qr_code_msg.stamp = rospy.Time.now()
                    self.disable_qr_code_msg.data = 1
                    self.disable_check_error_qr_code_pub.publish(
                        self.disable_qr_code_msg
                    )
                    self._asm.reset_flag()
                    # self.moving_control_reset_error_pub.publish(
                    #     EmptyStamped(stamp=rospy.Time.now())
                    # )
                    # _state = _state_when_error
                    _state = MainState.SEND_GOTO_WAITING
                    self.moving_control_error_code = ""

            # State: MOVING_ERROR
            elif _state == MainState.MOVING_ERROR:
                self.moving_control_result = -1
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = (
                    "/elevator_server: {}".format(_state.toString())
                    + self.moving_control_error_code
                )
                self.cmd_vel_msg.angular.z = 0
                self.cmd_vel_msg.linear.x = 0
                self.pub_vel.publish(self.cmd_vel_msg)
                if self._asm.reset_error_request:
                    self.disable_qr_code_msg.stamp = rospy.Time.now()
                    self.disable_qr_code_msg.data = 1
                    self.disable_check_error_qr_code_pub.publish(
                        self.disable_qr_code_msg
                    )
                    self.moving_control_client.cancel_all_goals()
                    rospy.sleep(0.1)
                    rospy.logwarn(
                        "Reset error --> state: {}".format(
                            _state_bf_error.toString()
                        )
                    )
                    self._asm.reset_flag()
                    # self.moving_control_reset_error_pub.publish(
                    #     EmptyStamped(stamp=rospy.Time.now())
                    # )
                    # _state = _state_bf_error
                    if _state_when_error == MainState.GOING_TO_OUT_OF_HUB:
                        _state = MainState.SEND_GOTO_OUT_OF_HUB
                    else:
                        _state = MainState.SEND_GOTO_WAITING
                    self.moving_control_error_code = ""
            # State: COLLISION_POSSIBLE
            elif _state == MainState.COLLISION_POSSIBLE:
                self.moving_control_result = -1
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = (
                    "/elevator_server: {}".format(_state.toString())
                    + self.moving_control_error_code
                )
                self.cmd_vel_msg.angular.z = 0
                self.cmd_vel_msg.linear.x = 0
                self.pub_vel.publish(self.cmd_vel_msg)
                if self._asm.reset_error_request:
                    self.disable_qr_code_msg.stamp = rospy.Time.now()
                    self.disable_qr_code_msg.data = 1
                    self.disable_check_error_qr_code_pub.publish(
                        self.disable_qr_code_msg
                    )
                    rospy.logwarn(
                        "Reset error --> state: {}".format(
                            _state_bf_error.toString()
                        )
                    )
                    self._asm.reset_flag()
                    # self.moving_control_reset_error_pub.publish(
                    #     EmptyStamped(stamp=rospy.Time.now())
                    # )
                    # _state = _state_bf_error
                    if _state_when_error == MainState.GOING_TO_OUT_OF_HUB:
                        _state = MainState.SEND_GOTO_OUT_OF_HUB
                    else:
                        _state = MainState.SEND_GOTO_WAITING
                    self.moving_control_error_code = ""

                # """
            # ....###....##.......####..######...##....##.##.....##.########.##....##.########..........######..########.##....##..######...#######..########.
            # ...##.##...##........##..##....##..###...##.###...###.##.......###...##....##............##....##.##.......###...##.##....##.##.....##.##.....##
            # ..##...##..##........##..##........####..##.####.####.##.......####..##....##............##.......##.......####..##.##.......##.....##.##.....##
            # .##.....##.##........##..##...####.##.##.##.##.###.##.######...##.##.##....##.............######..######...##.##.##..######..##.....##.########.
            # .#########.##........##..##....##..##..####.##.....##.##.......##..####....##..................##.##.......##..####.......##.##.....##.##...##..
            # .##.....##.##........##..##....##..##...###.##.....##.##.......##...###....##............##....##.##.......##...###.##....##.##.....##.##....##.
            # .##.....##.########.####..######...##....##.##.....##.########.##....##....##....#######..######..########.##....##..######...#######..##.....##
            # """
            elif _state == MainState.ALIGNMENT_SENSOR:
                rospy.logwarn(self.step)
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    _state_when_pause = _state
                    _state = MainState.PAUSED
                if self.step == 0:
                    self.step = 1
                elif self.step == 1:
                    if self.error_angle > 0:
                        self.cmd_vel_msg.angular.z = 0.06
                        self.cmd_vel_msg.linear.x = 0
                    else:
                        self.cmd_vel_msg.angular.z = -0.06
                        self.cmd_vel_msg.linear.x = 0
                    self.step = 2
                elif self.step == 2:
                    if abs(self.error_angle) <= 1:
                        self.step = 3
                        if abs(self.error_position) >= 0.02:
                            # _state_bf_error = MainState.SEND_DOCKING_HUB
                            # _state_when_error = _state
                            if disable_auto_get_center_tape:
                                self._asm.reset_flag()
                                self.moving_control_run_pause_pub.publish(
                                    StringStamped(
                                        stamp=rospy.Time.now(), data="RUN"
                                    )
                                )
                                self.moving_control_client.cancel_all_goals()
                                _state = MainState.COLLISION_POSSIBLE
                                if check_go_in:
                                    _state_when_error = MainState.DOCKING_TO_HUB
                                else:
                                    _state_when_error = (
                                        MainState.GOING_TO_OUT_OF_HUB
                                    )
                            rospy.sleep(0.5)
                        else:
                            _state = _state_when_error
                            self._asm.reset_flag()
                            self.moving_control_run_pause_pub.publish(
                                StringStamped(
                                    stamp=rospy.Time.now(), data="RUN"
                                )
                            )
                    else:
                        self.pub_vel.publish(self.cmd_vel_msg)

                elif self.step == 3:
                    cur_pose = self.pose_map2robot
                    diff_sensor_tape = self.error_position
                    if diff_sensor_tape > 0:
                        self.cmd_vel_msg.angular.z = -0.06
                        self.cmd_vel_msg.linear.x = 0
                    else:
                        self.cmd_vel_msg.angular.z = 0.06
                        self.cmd_vel_msg.linear.x = 0
                    self.step = 4

                elif self.step == 4:
                    diff_angle = self.diff_angle(self.pose_map2robot, cur_pose)
                    if abs(diff_angle) >= 15:
                        distance_move = diff_sensor_tape / sin(pi / 12)
                        rospy.logerr("distance : {}".format(distance_move))
                        self.step = 5
                    else:
                        self.pub_vel.publish(self.cmd_vel_msg)
                elif self.step == 5:
                    distance = distance_two_pose(self.pose_map2robot, cur_pose)
                    if abs(distance) >= abs(distance_move) - 0.01:
                        cur_pose = self.pose_map2robot
                        self.cmd_vel_msg.linear.x = 0
                        if diff_sensor_tape > 0:
                            self.cmd_vel_msg.angular.z = 0.06
                        else:
                            self.cmd_vel_msg.angular.z = -0.06
                        self.step = 6
                    else:
                        if direction == BACKWARD:
                            self.cmd_vel_msg.linear.x = 0.06
                        else:
                            self.cmd_vel_msg.linear.x = -0.06
                        self.cmd_vel_msg.angular.z = 0
                        self.pub_vel.publish(self.cmd_vel_msg)
                elif self.step == 6:
                    diff_angle = self.diff_angle(self.pose_map2robot, cur_pose)
                    if abs(diff_angle) >= 15:
                        self.step = 0
                        rospy.sleep(0.5)
                    else:
                        self.pub_vel.publish(self.cmd_vel_msg)

            # State: MOVING_DISCONNECTED
            elif _state == MainState.MOVING_DISCONNECTED:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/elevator_server: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = _state_bf_error
                    self.moving_control_error_code = ""

            # State: LIFT_POSITION_WRONG
            elif _state == MainState.LIFT_POSITION_WRONG:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/elevator_server: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = _state_when_error

            # State: NO_CART
            elif _state == MainState.NO_CART:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/elevator_server: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = _state_when_error

            # State: NO_CART
            elif _state == MainState.WRONG_CART:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/elevator_server: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = _state_when_error

            # State: NO_CART
            elif _state == MainState.NETWORK_ERROR:
                self._asm.module_status = ModuleStatus.ERROR
                error_code_convert = ""
                if "timed out" in str(plc.error_code_connect):
                    error_code_convert = "ELEVATOR_DISCONNECT"
                elif "No route to host" in str(plc.error_code_connect):
                    error_code_convert = "ELEVATOR_DISCONNECT"
                elif "Connection refused" in str(plc.error_code_connect):
                    error_code_convert = "ELEVATOR_REFUSED_CONNECTION"
                else:
                    error_code_convert = plc.error_code_connect

                self._asm.error_code = "/elevator_server: {}".format(
                    error_code_convert
                )
                detail_error = "/elevator_server: ERROR_CONNECT: {}, ERROR_READ_WRITE: {}".format(
                    plc.error_code_connect, plc.error_code_when_write_read
                )
                rospy.logerr(detail_error)
                try:
                    if plc.check_plc_connected():
                        rospy.logwarn("call close connect to plc")
                        if plc.plc_close():
                            rospy.logwarn("Closed connect to plc")
                            rospy.sleep(5)
                            rospy.logerr(
                                "Connect to plc error. Retry connect after 5 s ..."
                            )
                            plc.plc_connect(self.plc_address, self.port)
                            rospy.sleep(1)
                        else:
                            rospy.sleep(2)
                    else:
                        rospy.logerr(
                            "Connect to plc error. Retry connect after 5 s ..."
                        )
                        plc.plc_connect(self.plc_address, self.port)
                        rospy.sleep(1)
                except Exception as e:
                    rospy.logerr(e)
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                if not plc.plc_connect_fail and self.mode_robot == "AUTO":
                    self._asm.reset_flag()
                    first_check_timout = True
                    _state = _state_when_network_timeout
                    if (
                        _state_when_network_timeout
                        == MainState.GOING_TO_OUT_OF_HUB
                        or _state_when_network_timeout
                        == MainState.DOCKING_TO_HUB
                    ):
                        self.moving_control_run_pause_pub.publish(
                            StringStamped(stamp=rospy.Time.now(), data="RUN")
                        )
                        rospy.sleep(0.1)
                        self.pub_continue_run.publish(self.data_run)

            # State: PAUSED
            elif _state == MainState.PAUSED:
                self._asm.module_status = ModuleStatus.PAUSED
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                if self._asm.resume_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="RUN")
                    )
                    _state = _state_when_pause

            elif _state == MainState.PAUSED_BY_ELEVATOR:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/elevator_server: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    is_pause_by_elevator = False
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="RUN")
                    )

                    _state = _state_when_error

            elif _state == MainState.EMG_ELEVATOR:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/elevator_server: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                if plc.read_y(emg_elevator, 1)[0] and self.mode_robot == "AUTO":
                    # if self._asm.resume_req or self.start_1 or self.start_2:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="RUN")
                    )
                    rospy.sleep(0.1)
                    self.pub_continue_run.publish(self.data_run)

                    _state = _state_when_error
                    is_emg_elevator = False

            elif _state == MainState.TIMEOUT_WHEN_WAIT_ELEVATOR_ALLOW_MOVE:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/elevator_server: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = _state_when_error
                    self.resetTimeoutError = True

            # State: EMG_AGV
            elif _state == MainState.EMG_AGV:
                if self.emg_status:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="RUN")
                    )
                    _state = _state_when_emg_agv

            # State: DONE
            elif _state == MainState.DONE:
                self.dynamic_reconfig_movebase(self.vel_move_base, True)
                if goal_type == PLACE:
                    # [1040, 1041, 1042]
                    plc.write_x(carry_in_completed[source_floor - 1], [1])
                    if (
                        plc.read_x(carry_in_completed[source_floor - 1], 1)[0]
                        != 1
                    ):
                        rospy.logerr(
                            "Checking for carry_in_completed x: {} to write to PLC".format(
                                carry_in_completed[source_floor - 1]
                            )
                        )
                    else:
                        if not plc.plc_connect_fail:
                            if (
                                plc.read_w(15, 1)[0] and plc.read_y(1004, 1)[0]
                            ) or self.resetTimeoutError:
                                self.resetTimeoutError = False
                                t = datetime.now()
                                _state = MainStatePlace.DONE_CARRY_IN
                            else:
                                rospy.logerr(
                                    "wait for read_w(15, 1) and plc.read_y(1004, 1)"
                                )
                                if begin_check_timeout:
                                    timeout_waiting_elevator_respond = (
                                        rospy.get_time()
                                    )
                                    begin_check_timeout = False
                                else:
                                    if (
                                        rospy.get_time()
                                        - timeout_waiting_elevator_respond
                                        > 30
                                    ):
                                        _state_when_error = _state
                                        _state = (
                                            MainState.TIMEOUT_WHEN_WAIT_ELEVATOR_ALLOW_MOVE
                                        )
                elif goal_type == PICK:
                    # [1060, 1061, 1062]
                    plc.write_x(
                        carry_out_completion[destination_floor - 1], [1]
                    )
                    if (
                        plc.read_x(
                            carry_out_completion[destination_floor - 1], 1
                        )[0]
                        != 1
                    ):
                        rospy.logerr(
                            "Checking for carry_out_completion x: {} to write to PLC".format(
                                carry_out_completion[destination_floor - 1]
                            )
                        )
                    else:
                        # [1080, 1081, 1082]
                        if not plc.plc_connect_fail:
                            if (
                                plc.read_y(
                                    completion_ACK[destination_floor - 1], 1
                                )[0]
                                or self.resetTimeoutError
                            ):
                                self.resetTimeoutError = False
                                t = datetime.now()
                                _state = MainStatePick.DONE_CARRY_OUT
                            else:
                                rospy.logerr(
                                    "wait for read_y: {}".format(
                                        completion_ACK[destination_floor - 1]
                                    )
                                )
                                if begin_check_timeout:
                                    timeout_waiting_elevator_respond = (
                                        rospy.get_time()
                                    )
                                    begin_check_timeout = False
                                else:
                                    if (
                                        rospy.get_time()
                                        - timeout_waiting_elevator_respond
                                        > 30
                                    ):
                                        _state_when_error = _state
                                        _state = (
                                            MainState.TIMEOUT_WHEN_WAIT_ELEVATOR_ALLOW_MOVE
                                        )

            elif _state == MainStatePlace.DONE_CARRY_IN:
                # [1040, 1041, 1042]
                plc.write_x(carry_in_completed[source_floor - 1], [0])
                rospy.sleep(0.5)
                # [100, 102, 104]
                plc.write_w(destination_ST_indication[source_floor - 1], [0])
                plc.write_w(
                    destination_ST_indication[source_floor - 1] + 1, [0]
                )
                # [120, 121, 122]
                plc.write_w(carry_in_ID[source_floor - 1], [0])
                # [1030, 1031, 1032]
                plc.write_x(carry_in_request[source_floor - 1], [0])
                # plc.write_x(request_door_open[source_floor - 1], [0])
                if (
                    plc.read_x(carry_in_completed[source_floor - 1], 1)[0] == 0
                    and plc.read_w(
                        destination_ST_indication[source_floor - 1], 1
                    )[0]
                    == 0
                    and plc.read_w(
                        destination_ST_indication[source_floor - 1] + 1, 1
                    )[0]
                    == 0
                    and plc.read_w(carry_in_ID[source_floor - 1], 1)[0] == 0
                    and plc.read_x(carry_in_request[source_floor - 1], 1)[0]
                    == 0
                ):
                    if not plc.plc_connect_fail:
                        success = True
                        break
            elif _state == MainStatePick.DONE_CARRY_OUT:
                # [1050, 1051, 1052]
                plc.write_x(carry_out_request[destination_floor - 1], [0])
                # [1060, 1061, 1062]
                plc.write_x(carry_out_completion[destination_floor - 1], [0])
                if (
                    plc.read_x(carry_out_request[destination_floor - 1], 1)[0]
                    == 0
                    and plc.read_x(
                        carry_out_completion[destination_floor - 1], 1
                    )[0]
                    == 0
                ):
                    if not plc.plc_connect_fail:
                        success = True
                        break
            # """
            # .########.##.......##.....##....###....########..#######..########.
            # .##.......##.......##.....##...##.##......##....##.....##.##.....##
            # .##.......##.......##.....##..##...##.....##....##.....##.##.....##
            # .######...##.......##.....##.##.....##....##....##.....##.########.
            # .##.......##........##...##..#########....##....##.....##.##...##..
            # .##.......##.........##.##...##.....##....##....##.....##.##....##.
            # .########.########....###....##.....##....##.....#######..##.....##
            # """
            elif _state == MainStatePlace.REQUEST_ENTER_ELEVATOR:
                # X [1020, 1021, 1022]
                write_st_indication_ok = False
                if plc.read_y(
                    carry_in_instruction_possible[source_floor - 1], 1
                )[0]:
                    # rospy.logerr("debug 1")
                    if destination_floor == 1:
                        if self.elevator_name == ElevatorName.ELEVATOR_B:
                            floor_id = 12337
                        elif self.elevator_name == ElevatorName.ELEVATOR_A:
                            floor_id = 12592
                        if (
                            plc.read_w(
                                destination_ST_indication[source_floor - 1], 1
                            )[0]
                            != floor_id  # 12592
                        ):
                            plc.write_w(
                                destination_ST_indication[source_floor - 1],
                                [floor_id],  # 12592
                            )
                        elif (
                            plc.read_w(
                                destination_ST_indication[source_floor - 1] + 1,
                                1,
                            )[0]
                            != 12592
                        ):
                            plc.write_w(
                                destination_ST_indication[source_floor - 1] + 1,
                                [12592],
                            )
                        else:
                            write_st_indication_ok = True
                    elif destination_floor == 2:
                        if (
                            plc.read_w(
                                destination_ST_indication[source_floor - 1], 1
                            )[0]
                            != 12337
                        ):
                            plc.write_w(
                                destination_ST_indication[source_floor - 1],
                                [12337],
                            )
                        elif (
                            plc.read_w(
                                destination_ST_indication[source_floor - 1] + 1,
                                1,
                            )[0]
                            != 12848
                        ):
                            plc.write_w(
                                destination_ST_indication[source_floor - 1] + 1,
                                [12848],
                            )
                        else:
                            write_st_indication_ok = True
                    elif destination_floor == 3:
                        if self.elevator_name == ElevatorName.ELEVATOR_B:
                            floor_id = 12337
                        elif self.elevator_name == ElevatorName.ELEVATOR_A:
                            floor_id = 12592

                        if (
                            plc.read_w(
                                destination_ST_indication[source_floor - 1], 1
                            )[0]
                            != floor_id  # 12592 use for elevator A
                        ):
                            plc.write_w(
                                destination_ST_indication[source_floor - 1],
                                [floor_id],  # 12592 use for elevator A
                            )
                        elif (
                            plc.read_w(
                                destination_ST_indication[source_floor - 1] + 1,
                                1,
                            )[0]
                            != 13104
                        ):
                            plc.write_w(
                                destination_ST_indication[source_floor - 1] + 1,
                                [13104],
                            )
                        else:
                            write_st_indication_ok = True
                    else:
                        rospy.logerr("destination floor is not set in config")
                    # [120, 121, 122]
                    if (
                        not plc.read_w(carry_in_ID[source_floor - 1], 1)[0]
                        or plc.plc_connect_fail
                    ):
                        plc.write_w(carry_in_ID[source_floor - 1], [1])
                    else:
                        disable_write_X1004 = False
                        # plc.write_x(request_door_open[source_floor - 1], [1])
                        if plc.read_y(1006, 1)[0]:
                            disable_write_X1004 = True
                        if disable_write_X1004:
                            plc.write_x(1004, [0])
                        else:
                            plc.write_x(1004, [1])
                        if write_st_indication_ok:
                            _state = MainStatePlace.CARRY_IN_POSSIBLE
                else:
                    rospy.logerr(
                        "wait for plc.read_y: {}".format(
                            carry_in_instruction_possible[source_floor - 1]
                        )
                    )
                # else:
                #     if begin_check_timeout:
                #         timeout_waiting_elevator_respond = rospy.get_time()
                #         begin_check_timeout = False
                #     else:
                #         if rospy.get_time() - timeout_waiting_elevator_respond > 30:
                #             _state_when_error = _state
                #             _state = MainState.TIMEOUT_WHEN_WAIT_ELEVATOR_ALLOW_MOVE
                if self._asm.pause_req:
                    # self.moving_control_client.cancel_all_goals()
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED
            elif _state == MainStatePlace.CARRY_IN_POSSIBLE:
                # [1050, 1051, 1052]
                if plc.read_y(1006, 1)[0]:
                    disable_write_X1004 = True
                else:
                    rospy.logerr("wait for plc.read_y: 1006")
                if disable_write_X1004:
                    plc.write_x(1004, [0])
                    # [1030, 1031, 1032]
                    plc.write_x(carry_in_request[source_floor - 1], [1])

                else:
                    plc.write_x(1004, [1])
                if plc.read_y(carry_in_possible[source_floor - 1], 1)[0]:
                    _state = MainState.SEND_DOCKING_HUB
                else:
                    rospy.logerr(
                        "wait for plc.read_y: {}".format(
                            carry_in_possible[source_floor - 1]
                        )
                    )
                # else:
                #     if begin_check_timeout:
                #         timeout_waiting_elevator_respond = rospy.get_time()
                #         begin_check_timeout = False
                #     else:
                #         if rospy.get_time() - timeout_waiting_elevator_respond > 30:
                #             _state_when_error = _state
                #             _state = MainState.TIMEOUT_WHEN_WAIT_ELEVATOR_ALLOW_MOVE
                if self._asm.pause_req:
                    # self.moving_control_client.cancel_all_goals()
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED
            elif _state == MainStatePick.REQUEST_ENTER_ELEVATOR:
                # [30, 31, 32] and [1060, 1061, 1062]
                if (
                    plc.read_w(carry_out_ID[destination_floor - 1], 1)[0] == 1
                    and plc.read_y(
                        loading_and_unloading[destination_floor - 1], 1
                    )[0]
                ):
                    # [1050, 1051, 1052]
                    plc.write_x(carry_out_request[destination_floor - 1], [1])
                    if (
                        plc.read_x(carry_out_request[destination_floor - 1], 1)[
                            0
                        ]
                        != 1
                    ):
                        rospy.logerr(
                            "Checking for carry_out_request x: {} to write to PLC".format(
                                carry_out_request[destination_floor - 1]
                            )
                        )
                    # [1070, 1071, 1072]
                    if plc.read_y(
                        carrying_out_possible[destination_floor - 1], 1
                    )[0]:
                        _state = MainState.SEND_DOCKING_HUB
                    else:
                        rospy.logerr(
                            "wait for plc.read_y: {}".format(
                                carrying_out_possible[destination_floor - 1]
                            )
                        )
                else:
                    rospy.logerr(
                        "Wait for plc.read_w: {} and plc.read_y: {}".format(
                            carry_out_ID[destination_floor - 1],
                            loading_and_unloading[destination_floor - 1],
                        )
                    )
                # else:
                #     if begin_check_timeout:
                #         timeout_waiting_elevator_respond = rospy.get_time()
                #         begin_check_timeout = False
                #     else:
                #         if rospy.get_time() - timeout_waiting_elevator_respond > 30:
                #             _state_when_error = _state
                #             _state = MainState.TIMEOUT_WHEN_WAIT_ELEVATOR_ALLOW_MOVE
                if self._asm.pause_req:
                    # self.moving_control_client.cancel_all_goals()
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED
            if self.safety_job_name is not None:
                if (
                    self.safety_job_name != self.pre_safety_job_name
                    or _prev_state != _state
                    or rospy.get_time() - last_time_pub_safety_job > 1
                ):
                    self.pre_safety_job_name = self.safety_job_name
                    self.pre_safety_state = _state
                    last_time_pub_safety_job = rospy.get_time()
                    msg = StringStamped()
                    msg.stamp = rospy.Time.now()
                    msg.data = self.safety_job_name
                    self.safety_job_pub.publish(msg)

            r.sleep()
        rospy.logwarn("Close connect to plc")
        if plc.plc_close():
            rospy.logwarn("Closed connect to plc when finish")
        else:
            rospy.logwarn("Close connect to plc when finish false")
        self._asm.action_running = False
        if success:
            self.start_thread = False
            rospy.loginfo("%s: Succeeded" % self._action_name)
            self._as.set_succeeded(self._result)
        self.disable_qr_code_msg.stamp = rospy.Time.now()
        self.disable_qr_code_msg.data = 0
        self.disable_check_error_qr_code_pub.publish(self.disable_qr_code_msg)

    """
    ######## ##     ## ##    ##  ######  ######## ####  #######  ##    ##
    ##       ##     ## ###   ## ##    ##    ##     ##  ##     ## ###   ##
    ##       ##     ## ####  ## ##          ##     ##  ##     ## ####  ##
    ######   ##     ## ## ## ## ##          ##     ##  ##     ## ## ## ##
    ##       ##     ## ##  #### ##          ##     ##  ##     ## ##  ####
    ##       ##     ## ##   ### ##    ##    ##     ##  ##     ## ##   ###
    ##        #######  ##    ##  ######     ##    ####  #######  ##    ##
    """

    def dynamic_reconfig_movebase(self, vel_x, publish_safety):
        new_config = {
            "max_vel_x": vel_x,
            "max_vel_trans": vel_x,
            "publish_safety": publish_safety,
        }
        for i in range(3):
            self.client_reconfig_movebase.update_configuration(new_config)
            rospy.sleep(0.1)

    def read_data_elevator(self):
        while True:
            # try:
            if 1:
                if self.print_first:
                    rospy.loginfo("START THREAD MONITOR IO PLC")
                    self.print_first = False
                if self.start_thread:
                    self.read_value_x()
                    self.read_value_y()
                    self.read_value_w_input()
                    self.read_value_w_output()
                    self.pub_io()
            # except Exception as e:
            #     rospy.logerr(e)
            sleep(0.1)

    def load_config(self):
        try:
            # Server config
            if os.path.exists(self.server_config_file):
                with open(self.server_config_file) as file:
                    self.server_config = yaml.load(file, Loader=yaml.Loader)
                    print_info("Server config file:")
                    print(
                        yaml.dump(
                            self.server_config,
                            Dumper=YamlDumper,
                            default_flow_style=False,
                        )
                    )
                    if "server_address" not in self.server_config:
                        self.server_config = None
        except Exception as e:
            rospy.logerr("load_config: {}".format(e))
            return False
        return True

    def init_server(self):
        if self.server_config == None:
            rospy.loginfo_throttle(30, "Server was not configured!")
            return
        self.api_url = self.server_config["server_address"]
        self.data = self.server_config["agv_name"]  # {"agv":"AGV 01"}
        self.header = {
            "X-Parse-Application-Id": "APPLICATION_ID",
            "X-Parse-Master-Key": "YOUR_MASTER_KEY",
            "Content-Type": "application/json",
        }

    def upDateCart(self, type, name, cell, cart_no, lot_no):
        data = {
            "function": "UPDATE_CART",
            "type": type,
            "name": name,
            "cell": cell,
            "cart": cart_no,
            "lot": lot_no,
        }
        try:
            response = requests.post(
                self.api_url + "functions/agvapi",
                data=json.dumps(data),
                headers=self.header,
                timeout=1,
            )  # TOCHECK
            _temp = json.loads(response.text)
            print_debug(_temp)
            if _temp["result"] == "OK":
                return True
            else:
                return False
        except requests.exceptions.RequestException as e:
            rospy.logerr_throttle(10.0, e)
            return False

    def get_angle_elevator(self, xA, yA, xB, yB, orient_robot):
        orient = atan2(yB - yA, xB - xA)
        ret = orient_robot - orient
        ret = (ret + pi) % (2 * pi) - pi
        ret_deg = degrees(ret)
        if ret_deg <= 135 and ret_deg >= 45:
            orient_hub = orient_robot + pi / 2
        elif ret_deg <= 45 and ret_deg >= -45:
            orient_hub = orient_robot + pi
        elif ret_deg <= -45 and ret_deg >= -135:
            orient_hub = orient_robot - pi / 2
        elif ret_deg <= -135 or ret_deg >= 135:
            orient_hub = orient_robot
        orient_hub = (orient_hub + pi) % (2 * pi) - pi
        return orient_hub

    def reset(self):
        rospy.logwarn("Start reset io")
        if plc.plc_connect_fail:
            rospy.logwarn("Reset io fail")
            return False
        else:
            counter_check = 0
            while True:
                plc.plc_connect_fail = False
                for i in x_value_address:
                    plc.write_x(i, [0])
                    rospy.sleep(0.01)
                for i in w_value_address_input:
                    plc.write_w(i, [0])
                    rospy.sleep(0.01)
                x_value = self.read_value_x()
                w_input_value = self.read_value_w_input()
                total_x = len(x_value_address)
                total_w_input = len(w_value_address_input)
                if (
                    w_input_value == [0] * total_w_input
                    and x_value == [0] * total_x
                    and not plc.plc_connect_fail
                ):
                    rospy.logwarn("Reset io success")
                    return True
                else:
                    counter_check += 1
                if counter_check > 3:
                    rospy.logwarn("Reset io fail")
                    return False
                rospy.sleep(1)

    def read_value_x(self):
        global pre_x_value
        index = 0
        for i in x_value_address:
            value = plc.read_x(i, 1)[0]
            rospy.sleep(0.01)
            if pre_x_value[index] != value:
                rospy.logwarn("Write data  X{} = {} to PLC".format(i, value))
                pre_x_value[index] = value
            index += 1
        return pre_x_value

    def read_value_y(self):
        global pre_y_value
        index = 0
        for i in y_value_address:
            value = plc.read_y(i, 1)[0]
            rospy.sleep(0.01)
            if pre_y_value[index] != value:
                rospy.logwarn("Read data Y{} = {} from PLC".format(i, value))
                pre_y_value[index] = value
            index += 1
        return pre_y_value

    def read_value_w_input(self):
        global pre_w_value_input
        index = 0
        for i in w_value_address_input:
            value = plc.read_w(i, 1)[0]
            rospy.sleep(0.01)
            if pre_w_value_input[index] != value:
                rospy.logwarn("Write data W{} = {} to PLC".format(i, value))
                pre_w_value_input[index] = value
            index += 1
        return pre_w_value_input

    def read_value_w_output(self):
        global pre_w_value_output
        index = 0
        for i in w_value_address_output:
            value = plc.read_w(i, 1)[0]
            rospy.sleep(0.01)
            if pre_w_value_output[index] != value:
                rospy.logwarn("Read data W{} = {} from PLC".format(i, value))
                pre_w_value_output[index] = value
            index += 1
        return pre_w_value_output

    def reset_value_all(self):
        global pre_x_value, pre_y_value, pre_w_value_input, pre_w_value_output
        pre_x_value = []
        pre_y_value = []
        pre_w_value_input = []
        pre_w_value_output = []
        for i in range(len(x_value_address)):
            pre_x_value.append(0)
        for i in range(len(y_value_address)):
            pre_y_value.append(0)
        for i in range(len(w_value_address_input)):
            pre_w_value_input.append(0)
        for i in range(len(w_value_address_output)):
            pre_w_value_output.append(0)

    def pub_io(self):
        sensors_msg_dict = {}
        for i in range(len(x_value_address)):
            sensors_msg_dict[str(x_value_address[i])] = pre_x_value[i]
        for i in range(len(y_value_address)):
            sensors_msg_dict[str(y_value_address[i])] = pre_y_value[i]
        for i in range(len(w_value_address_input)):
            sensors_msg_dict[str(w_value_address_input[i])] = pre_w_value_input[
                i
            ]
        for i in range(len(w_value_address_output)):
            sensors_msg_dict[str(w_value_address_output[i])] = (
                pre_w_value_output[i]
            )
        self.std_io_msg.stamp = rospy.Time.now()
        self.std_io_msg.data = json.dumps(sensors_msg_dict, indent=2)
        self.status_io_pub.publish(self.std_io_msg)

    def calculate_pose_offset(self, x_offset, cur_pose_x, cur_pose_y, cur_ori):
        x = cur_pose_x + x_offset * cos(cur_ori)
        y = cur_pose_y + x_offset * sin(cur_ori)
        p = Pose()
        p.position.x = x
        p.position.y = y
        q = quaternion_from_euler(0.0, 0.0, cur_ori)
        p.orientation = Quaternion(*q)
        return p

    def diff_angle(self, current_pose, target_pose):
        try:
            angel_target_pose = euler_from_quaternion(
                [
                    target_pose.orientation.x,
                    target_pose.orientation.y,
                    target_pose.orientation.z,
                    target_pose.orientation.w,
                ]
            )
        except:
            angel_target_pose = euler_from_quaternion(
                [
                    target_pose.pose.orientation.x,
                    target_pose.pose.orientation.y,
                    target_pose.pose.orientation.z,
                    target_pose.pose.orientation.w,
                ]
            )
        angel_target_pose = angel_target_pose[2]
        angle_current_pose = euler_from_quaternion(
            [
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w,
            ]
        )
        angle_current_pose = angle_current_pose[2]
        diff_angle = angel_target_pose - angle_current_pose
        diff_angle = (diff_angle + pi) % (2 * pi) - pi
        diff_angle = degrees(diff_angle)
        return diff_angle

    def get_odom(self):
        self.pose_map2robot = Pose()
        if self.use_tf2:
            try:
                trans = self.tf_buffer.lookup_transform(
                    "map",
                    "base_link",
                    time=rospy.Time(0),
                    timeout=rospy.Duration(5),
                )
                self.trans = [
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    0,
                ]
                self.rot = [
                    0,
                    0,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w,
                ]
                self.pose_map2robot.position.x = self.trans[0]
                self.pose_map2robot.position.y = self.trans[1]
                self.pose_map2robot.position.z = 0
                self.pose_map2robot.orientation.x = 0
                self.pose_map2robot.orientation.y = 0
                self.pose_map2robot.orientation.z = self.rot[2]
                self.pose_map2robot.orientation.w = self.rot[3]
                return True
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
                KeyboardInterrupt,
            ):
                rospy.logwarn("TF exception ")
                return False
        else:
            try:
                self.tf_listener.waitForTransform(
                    "/map", "/base_link", rospy.Time(0), rospy.Duration(4.0)
                )
                # rospy.loginfo("transform found :)")
                self.trans, self.rot = self.tf_listener.lookupTransform(
                    "/map", "/base_link", rospy.Time(0)
                )
                ############self.rot is in quaternion############
                # print("CURRENT POSE :{}/n{}".format(self.trans, self.rot))
                self.pose_map2robot.position.x = self.trans[0]
                self.pose_map2robot.position.y = self.trans[1]
                self.pose_map2robot.position.z = 0
                self.pose_map2robot.orientation.x = 0
                self.pose_map2robot.orientation.y = 0
                self.pose_map2robot.orientation.z = self.rot[2]
                self.pose_map2robot.orientation.w = self.rot[3]
                return True
            except (
                tf.Exception,
                tf.ConnectivityException,
                tf.LookupException,
                KeyboardInterrupt,
            ):
                rospy.logwarn("TF exception")
                return False

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
            else:
                self.send_feedback(self._as, self._asm.module_state)
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
        default=os.path.join(rospkg.RosPack().get_path("matehan"), "cfg"),
    )
    parser.add_option(
        "-r",
        "--robot_config_file",
        dest="robot_config_file",
        default=os.path.join(
            rospkg.RosPack().get_path("amr_config"),
            "cfg",
            "control_system",
            "robot_config.yaml",
        ),
    )
    parser.add_option(
        "--robot_define",
        dest="robot_define",
        default=os.path.join(
            HOME,
            "robot_config",
            "robot_define.yaml",
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
    rospy.init_node("elevator_server", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    ElevatorAction(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
