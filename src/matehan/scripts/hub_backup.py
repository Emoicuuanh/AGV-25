#! /usr/bin/env python
# -*- coding: utf-8 -*-

from logging import debug
from bson.json_util import dumps
import os
import sys
import rospy
import rospkg
import tf
import tf2_ros
import tf2_geometry_msgs
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
from agv_msgs.msg import ErrorRobotToPath
from nav_msgs.msg import Odometry
import json
from std_msgs.msg import Bool, Int16, Int8, String, Float32
from math import sqrt, pow, pi, sin, cos, atan2, degrees, atan
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

import numpy as np

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
from offset_agv import agv_offset
from agv_msgs.msg import DataMatrixStamped

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
    SEND_GOTO_TEMP_POSE = 16
    GOING_TO_TEMP_POSE = 17
    SEND_GOTO_ROTATION_POSE = 18
    GOING_TO_ROTATION_POSE = 19
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
    COLLISION_POSSIBLE = 50
    ROTATE_TO_GOAL_ANGLE = 51
    READ_CART_ERROR = 52
    DETECT_MIRROR_ERROR = 60


class RunType(Enum):
    NONE = -1
    GO_NOMAL = 0
    GO_DOCKING = 1
    GO_OUT_DOCKING = 2
    STOP_ACCURACY = 3
    STOP_BY_CROSS_LINE = 4


PICK = 1
PLACE = 0
ON = 1
OFF = 0
LIFT_UP = 1
LIFT_DOWN = 2
FAKE_QR_CODE = True
FORWARD = 1
BACKWARD = 0
USE_TF_LIDAR = 1
USE_TF_QR_CODE = 0
USE_DOCKING_BY_MIRROR = True


class HubAction(object):
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
        self.control_tf_pub = rospy.Publisher("/control_tf", Int8, queue_size=5)
        self.disable_check_error_qr_code_pub = rospy.Publisher(
            "/disable_check_error_qr_code", Int8Stamped, queue_size=5
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

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        # Subscriber
        rospy.Subscriber(
            "/current_control_tf", Int8, self.current_control_tf_cb
        )

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

        rospy.Subscriber("/robot_pose", Pose, self.robot_pose_cb)
        rospy.Subscriber("/data_gls621", DataMatrixStamped, self.data_gls_cb)
        rospy.Subscriber(
            "/current_traffic_control_type",
            StringStamped,
            self.current_traffic_control_type_cb,
        )

        # Service client
        try:
            rospy.wait_for_service("ReadQrCode", 30)
            try:
                self.get_qr_code = rospy.ServiceProxy("ReadQrCode", QrCode)
                rospy.loginfo("ReadQrCode service: is running")
            except rospy.ServiceException as e:
                rospy.logerr(f"Fail to call ReadQrCode service: {e}")
        except rospy.ROSException as e:
            rospy.logerr(f"ReadQrCode service is not available: {e}")
        # dynamic reconfig client
        self.client_reconfig_movebase = dynamic_reconfigure.client.Client(
            "/move_base/NeoLocalPlanner",
            timeout=300,
            config_callback=self.dynamic_callback,
        )

        self.path_offset_x = rospy.get_param("path_offset_x", 0.0)
        self.path_offset_y = rospy.get_param("path_offset_y", 0.0)
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
        self.type_lift = LIFT_UP
        self.liftup_finish = False
        self.liftup_finish_first_check = False
        self.detect_vrack = False
        self.liftdown_finish = False
        self.liftdown_finish_first_check = False
        self.enable_safety = True
        self.vel_move_base = 0.8
        self.lift_msg = Int8Stamped()
        self.disable_qr_code_msg = Int8Stamped()
        self.last_time_get_lift_up = rospy.get_time()
        self.last_time_get_lift_down = rospy.get_time()
        self.robot_pose_angle = None
        self.path_angle = None
        self.qr_angle = None
        self.vel = Twist()
        self.lift_timer = None

        # 2nd time reset UNABLE_TO_PLACE_CART error
        self.disable_unable_to_place_cart = False

        self.use_new_traffic_control = False

        self.current_control_tf = 100

        # tf
        self.tf_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf_buffer)

    def shutdown(self):
        # self.auto_docking_client.cancel_all_goals()
        self.dynamic_reconfig_movebase(
            self.vel_move_base, publish_safety=True, stop_center_qr=True
        )
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

    def current_control_tf_cb(self, msg):
        self.current_control_tf = msg.data

    def current_traffic_control_type_cb(self, msg):
        try:
            if msg.data == "true":
                self.use_new_traffic_control = True
            else:
                self.use_new_traffic_control = False
        except Exception as e:
            rospy.logerr(f"traffic_control_type_cb error: {e}")

    def error_robot_to_path_cb(self, msg):
        self.error_position = msg.error_position
        self.error_angle = msg.error_angle

    def odom_cb(self, msg):
        self.pose_odom2robot = msg.pose.pose

    def standard_io_cb(self, msg):
        data = json.loads(msg.data)
        if "lift_max_sensor" in data:
            if data["lift_max_sensor"]:
                self.liftup_finish_first_check = True
            if data["lift_max_sensor"] and (
                rospy.get_time() - self.last_time_get_lift_up >= 2
            ):
                self.liftup_finish = True
            if not data["lift_max_sensor"]:
                self.last_time_get_lift_up = rospy.get_time()
                self.liftup_finish = False
                self.liftup_finish_first_check = False
        if "lift_min_sensor" in data:
            if data["lift_min_sensor"]:
                self.liftdown_finish_first_check = True
            if data["lift_min_sensor"] and (
                rospy.get_time() - self.last_time_get_lift_down >= 2
            ):
                self.liftdown_finish = True
            if not data["lift_min_sensor"]:
                self.last_time_get_lift_down = rospy.get_time()
                self.liftdown_finish = False
                self.liftdown_finish_first_check = False
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
        if "detect_vrack" in data:
            self.detect_vrack = data["detect_vrack"]
        # self.detect_vrack = False
        # self.liftup_finish = True
        # self.liftdown_finish = True

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

    def robot_pose_cb(self, msg):
        r_x = msg.orientation.x
        r_y = msg.orientation.y
        r_z = msg.orientation.z
        r_w = msg.orientation.w
        roll, pitch, yaw = euler_from_quaternion((r_x, r_y, r_z, r_w))
        self.robot_pose_angle = yaw

    def data_gls_cb(self, msg):
        self.qr_angle = msg.possition.angle

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
        # self.switch_control_tf(USE_TF_QR_CODE)
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
                if "safety_job_rotation" in hub_dict:
                    safety_job_rotation = hub_dict["safety_job_rotation"]
                else:
                    safety_job_rotation = "ROTATION"
                footprint_dock = hub_dict["footprint_dock"]
                foorprint_undock = hub_dict["foorprint_undock"]
                enable_check_error_when_docking = hub_dict[
                    "enable_check_error_when_docking"
                ]
                distance_turn_off_safety_when_docking = hub_dict[
                    "distance_turn_off_safety_when_docking"
                ]
                vel_docking_hub = hub_dict["max_vel_docking"]
            # Waiting path config
            waiting_path_cfg = os.path.join(
                self.config_path, "waiting_path.json"
            )
            with open(waiting_path_cfg) as j:
                self.waiting_path_dict = json.load(j)
                self.return_pose_dict = self.waiting_path_dict["waypoints"][0][
                    "position"
                ]
            # Docking path config
            docking_path_cfg = os.path.join(
                self.config_path, "docking_path.json"
            )
            with open(docking_path_cfg) as j:
                self.docking_path_dict = json.load(j)
            # UnDocking path config
            undocking_path_cfg = os.path.join(
                self.config_path, "undocking_path.json"
            )
            with open(undocking_path_cfg) as j:
                self.undocking_path_dict = json.load(j)
            # Temp path config
            temp_path_cfg = os.path.join(self.config_path, "docking_path.json")
            with open(temp_path_cfg) as j:
                self.temp_path_dict = json.load(j)
            # rotation path config
            rotation_path_cfg = os.path.join(
                self.config_path, "waiting_path.json"
            )
            with open(rotation_path_cfg) as j:
                self.rotation_path_dict = json.load(j)
        except Exception as e:
            rospy.logerr("Read config file error: {}".format(e))
            self._as.set_aborted("Read config file error")
            return

        # Load data from action
        data_dict = json.loads(goal.data)
        rospy.logwarn(data_dict)
        self.enable_safety = True
        try:
            hub_pose_x = data_dict["params"]["position"]["x"]
            hub_pose_y = data_dict["params"]["position"]["y"]

            waiting_pose_x = data_dict["params"]["waiting_position"]["x"]
            waiting_pose_y = data_dict["params"]["waiting_position"]["y"]

            # Add this to add offset to docking path
            waiting_pose = [waiting_pose_x, waiting_pose_y]
            hub_pose = [hub_pose_x, hub_pose_y]

            hub_offset = agv_offset(
                waiting_pose, hub_pose, self.path_offset_x, self.path_offset_y
            )
            self.path_angle = self.get_path_angle(waiting_pose, hub_pose)

            hub_pose = hub_offset.calculate_offset(hub_pose)
            waiting_pose = hub_offset.calculate_offset(waiting_pose)
            # Overwrite hub_pose_x and hub_pose_y
            hub_pose_x = hub_pose[0]
            hub_pose_y = hub_pose[1]
            waiting_pose_x = waiting_pose[0]
            waiting_pose_y = waiting_pose[1]

            self.type = "HUB"
            self.name = data_dict["params"]["name"]
            self.cell = data_dict["params"]["cell"]
            self.cart = data_dict["params"]["cart"]
            self.lot = data_dict["params"]["lot"]
            if "properties" in data_dict["params"]:
                if "Safety" in data_dict["params"]["properties"]:
                    if data_dict["params"]["properties"]["Safety"] == "Disable":
                        self.enable_safety = False
                if "Invert" in data_dict["params"]["properties"]:
                    self.direction = data_dict["params"]["properties"]["invert"]
            if "invert" in data_dict["params"]:
                self.direction = data_dict["params"]["invert"]
        except:
            hub_pose_x = data_dict["params"]["position"]["position"]["x"]
            hub_pose_y = data_dict["params"]["position"]["position"]["y"]

            # Add this to add offset to docking path
            waiting_pose = [self.trans[0], self.trans[1]]
            hub_pose = [hub_pose_x, hub_pose_y]

            hub_offset = agv_offset(
                waiting_pose, hub_pose, self.path_offset_x, self.path_offset_y
            )
            self.path_angle = self.get_path_angle(waiting_pose, hub_pose)

            hub_pose = hub_offset.calculate_offset(hub_pose)
            waiting_pose = hub_offset.calculate_offset(waiting_pose)
            # Overwrite hub_pose_x and hub_pose_y
            hub_pose_x = hub_pose[0]
            hub_pose_y = hub_pose[1]
            waiting_pose_x = waiting_pose[0]
            waiting_pose_y = waiting_pose[1]

            waiting_pose_x = self.trans[0]
            waiting_pose_y = self.trans[1]
            self.type = "HUB"
            self.name = "AGV 01"
            self.cell = 0
            self.cart = "VRACK"
            self.lot = "1"
            # use_server = False
        if USE_DOCKING_BY_MIRROR:
            self.direction = FORWARD
        else:
            self.direction = BACKWARD
        rospy.logwarn("direction: {}".format(self.direction))
        if use_server:
            FAKE_QR_CODE = True
        else:
            FAKE_QR_CODE = True
        if self.direction == FORWARD:
            cur_orient = atan2(
                hub_pose_y - waiting_pose_y, hub_pose_x - waiting_pose_x
            )
        else:
            cur_orient = atan2(
                waiting_pose_y - hub_pose_y, waiting_pose_x - hub_pose_x
            )

        # Calculate waiting goal
        self.waiting_goal = StringGoal()
        waiting_pose = self.calculate_pose_offset(
            0,
            waiting_pose_x,
            waiting_pose_y,
            cur_orient,
        )
        rospy.logwarn(" cur_orient docking: {}".format(cur_orient))
        if USE_DOCKING_BY_MIRROR:
            odom_pose = self.wait_until_pose_available()
            waiting_pose = self.transform_pose_map_to_odom(waiting_pose)
            waiting_pose.position.x = odom_pose.position.x
            waiting_pose.position.y = odom_pose.position.y
            #  _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            # rospy.logwarn("yaw before : {}".format(yaw))
            # if self.direction == FORWARD:
            #     # Cộng thêm 180 độ vào orientation
            #     quat = waiting_pose.orientation
            #     _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            #     yaw = self.normalize_angle(yaw + pi)
            #     rospy.logwarn("yaw after set direction : {}".format(yaw))
            #     q_new = quaternion_from_euler(0, 0, yaw)
            #     waiting_pose.orientation = Quaternion(*q_new)
        self.waiting_path_dict["params"] = {}
        self.waiting_path_dict["params"]["only_use_qr"] = True
        self.waiting_path_dict["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(waiting_pose, self.return_pose_dict)
        )
        self.waiting_goal.data = json.dumps(self.waiting_path_dict, indent=2)
        rospy.logwarn(
            "Waiting goal position:\n{}".format(
                json.dumps(self.waiting_path_dict, indent=2)
            )
        )

        # Calculate docking goal
        self.docking_goal = StringGoal()
        docking_pose = self.calculate_pose_offset(
            0,
            hub_pose_x,
            hub_pose_y,
            cur_orient,
        )
        if USE_DOCKING_BY_MIRROR:
            docking_pose = self.transform_pose_map_to_odom(docking_pose)
        self.docking_path_dict["params"] = {}
        self.docking_path_dict["params"]["only_use_qr"] = True
        self.docking_path_dict["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(waiting_pose, self.return_pose_dict)
        )
        self.docking_path_dict["waypoints"][1]["position"] = copy.deepcopy(
            obj_to_dict(docking_pose, self.return_pose_dict)
        )
        self.docking_goal.data = json.dumps(self.docking_path_dict, indent=2)
        rospy.logwarn(
            "Docking goal position:\n{}".format(
                (json.dumps(self.docking_path_dict, indent=2))
            )
        )

        # Calculate undocking goal
        self.undocking_goal = StringGoal()
        self.undocking_path_dict["params"] = {}
        self.undocking_path_dict["params"]["only_use_qr"] = True
        self.undocking_path_dict["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(docking_pose, self.return_pose_dict)
        )

        self.undocking_path_dict["waypoints"][1]["position"] = copy.deepcopy(
            obj_to_dict(waiting_pose, self.return_pose_dict)
        )
        if "param_test" not in self.undocking_path_dict:
            self.undocking_path_dict["param_test"] = {}
        self.undocking_path_dict["param_test"][
            "need_to_wait_receive_new_path"
        ] = True
        self.undocking_goal.data = json.dumps(
            self.undocking_path_dict, indent=2
        )
        rospy.logwarn(
            "UnDocking goal position:\n{}".format(
                (json.dumps(self.undocking_path_dict, indent=2))
            )
        )
        self.temp_goal = StringGoal()
        self.temp_path_dict["params"] = {}
        self.temp_path_dict["params"]["only_use_qr"] = True
        self.temp_path_dict["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(waiting_pose, self.return_pose_dict)
        )
        self.temp_path_dict["waypoints"][1]["position"] = copy.deepcopy(
            obj_to_dict(docking_pose, self.return_pose_dict)
        )
        self.temp_goal.data = json.dumps(self.temp_path_dict, indent=2)
        rospy.logwarn(
            "Temp goal position:\n{}".format(
                (json.dumps(self.temp_path_dict, indent=2))
            )
        )

        self.rotation_goal = StringGoal()
        self.rotation_path_dict["params"] = {}
        self.rotation_path_dict["params"]["only_use_qr"] = True
        self.rotation_path_dict["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(waiting_pose, self.return_pose_dict)
        )
        self.rotation_goal.data = json.dumps(self.rotation_path_dict, indent=2)
        rospy.logwarn(
            "rotation_goal position:\n{}".format(
                (json.dumps(self.rotation_path_dict, indent=2))
            )
        )

        pick_or_place = data_dict["params"]["pick_or_place"]
        if pick_or_place:
            goal_type = PICK
        else:
            goal_type = PLACE

        r = rospy.Rate(15)
        success = False
        _state = MainState.INIT
        _prev_state = MainState.NONE
        feedback_msg = ""
        _state_when_pause = MainState.NONE
        _state_when_error = MainState.NONE
        _state_when_emg_agv = MainState.NONE
        _state_bf_error = MainState.NONE
        self._asm.reset_flag()
        self._asm.action_running = True
        self.pose_map2robot = None
        self.safety_job_name = None
        self.pre_safety_job_name = None
        self.cmd_vel_msg = Twist()
        distance_to_hub = 0
        self.error_position = 0
        self.error_angle = 0
        self.step = 0
        self.get_first_time_error = True
        first_go_to_waiting = True
        last_time_pub_safety_job = rospy.get_time()
        while not rospy.is_shutdown():
            if not self.get_odom():
                continue
            distance_to_hub = distance_two_points(
                self.pose_map2robot.position.x,
                self.pose_map2robot.position.y,
                hub_pose_x,
                hub_pose_y,
            )
            if self._as.is_preempt_requested() or self._asm.reset_action_req:
                rospy.logerr(
                    "reset_action_req : {}".format(self._asm.reset_action_req)
                )
                rospy.loginfo("%s: Preempted" % self._action_name)
                self._as.set_preempted()
                success = False
                self.send_feedback(
                    self._as, GoalStatus.to_string(GoalStatus.PREEMPTED)
                )
                # self.auto_docking_client.cancel_all_goals()
                self.dynamic_reconfig_movebase(
                    self.vel_move_base, publish_safety=True, stop_center_qr=True
                )
                self.moving_control_client.cancel_all_goals()
                self.stop_lift_timer()
                break

            if self._asm.module_status != ModuleStatus.ERROR:
                self._asm.error_code = ""
            if _state != MainState.PAUSED and self._asm.error_code == "":
                self._asm.module_status = ModuleStatus.RUNNING

            if _prev_state != _state:
                # Record log
                self.db.recordLog(
                    "Action state: {} -> {}".format(
                        _prev_state.toString(), _state.toString()
                    ),
                    rospy.get_name(),
                    LogLevel.INFO.toString(),
                )
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
                rospy.logwarn("DIRECTION: {}".format(self.direction))
                self.vel_move_base = rospy.get_param(
                    "/move_base/NeoLocalPlanner/max_vel_x", 0.8
                )
                print("self.vel_move_base: ", self.vel_move_base)
                self.dynamic_reconfig_movebase(
                    vel_docking_hub, publish_safety=False, stop_center_qr=False
                )
                _state = MainState.SEND_GOTO_WAITING
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # State: SEND_GOTO_WAITING
            elif _state == MainState.SEND_GOTO_WAITING:
                self.moving_control_client.send_goal(
                    self.waiting_goal,
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
                    if USE_DOCKING_BY_MIRROR:
                        _state = MainState.SEND_GOTO_TEMP_POSE
                    else:
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
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # """
            #   #####  ####### ####### #######         ####### ####### #     # ######          ######  #######  #####  #######
            #  #     # #     #    #    #     #            #    #       ##   ## #     #         #     # #     # #     # #
            #  #       #     #    #    #     #            #    #       # # # # #     #         #     # #     # #       #
            #  #  #### #     #    #    #     #            #    #####   #  #  # ######          ######  #     #  #####  #####
            #  #     # #     #    #    #     #            #    #       #     # #               #       #     #       # #
            #  #     # #     #    #    #     #            #    #       #     # #               #       #     # #     # #
            #   #####  #######    #    #######            #    ####### #     # #               #       #######  #####  #######
            #                                  #######                                 #######
            # """
            # State: SEND_GOTO_TEMP_POSE
            elif _state == MainState.SEND_GOTO_TEMP_POSE:
                if not self.compute_goals_from_mirror():
                    _state_when_error = _state
                    _state = MainState.DETECT_MIRROR_ERROR
                    continue
                self.moving_control_client.send_goal(
                    self.temp_goal,
                    feedback_cb=self.moving_control_fb,
                )
                self.moving_control_result = -1
                self.last_moving_control_fb = rospy.get_time()
                _state = MainState.GOING_TO_TEMP_POSE
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # State: GOING_TO_TEMP_POSE
            elif _state == MainState.GOING_TO_TEMP_POSE:
                if self.moving_control_result == GoalStatus.SUCCEEDED:
                    _state = MainState.SEND_GOTO_ROTATION_POSE
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
                    _state_bf_error = MainState.SEND_GOTO_TEMP_POSE
                    _state_when_error = _state
                    _state = MainState.MOVING_ERROR
                if rospy.get_time() - self.last_moving_control_fb >= 5.0:
                    rospy.logerr("/moving control disconnected!")
                    self.send_feedback(
                        self._as, GoalStatus.to_string(GoalStatus.ABORTED)
                    )
                    _state_bf_error = MainState.SEND_GOTO_TEMP_POSE
                    _state_when_error = _state
                    _state = MainState.MOVING_DISCONNECTED
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # State: SEND_GOTO_TEMP_POSE
            elif _state == MainState.SEND_GOTO_ROTATION_POSE:
                # if not self.compute_goals_from_mirror():
                #     _state_when_error = _state
                #     _state = MainState.DETECT_MIRROR_ERROR
                #     continue
                self.moving_control_client.send_goal(
                    self.rotation_goal,
                    feedback_cb=self.moving_control_fb,
                )
                self.moving_control_result = -1
                self.last_moving_control_fb = rospy.get_time()
                _state = MainState.GOING_TO_ROTATION_POSE
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # State: GOING_TO_TEMP_POSE
            elif _state == MainState.GOING_TO_ROTATION_POSE:
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
                    _state_bf_error = MainState.SEND_GOTO_TEMP_POSE
                    _state_when_error = _state
                    _state = MainState.MOVING_ERROR
                if rospy.get_time() - self.last_moving_control_fb >= 5.0:
                    rospy.logerr("/moving control disconnected!")
                    self.send_feedback(
                        self._as, GoalStatus.to_string(GoalStatus.ABORTED)
                    )
                    _state_bf_error = MainState.SEND_GOTO_TEMP_POSE
                    _state_when_error = _state
                    _state = MainState.MOVING_DISCONNECTED
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # State: SEND_DOCKING_HUB
            elif _state == MainState.SEND_DOCKING_HUB:
                rospy.sleep(1)
                if not self.compute_goals_from_mirror():
                    _state_when_error = _state
                    _state = MainState.DETECT_MIRROR_ERROR
                    continue
                first_go_to_waiting = False
                self.dynamic_reconfig_movebase(
                    vel_docking_hub, publish_safety=False, stop_center_qr=False
                )
                check_go_in = True
                self.moving_control_client.send_goal(
                    self.docking_goal,
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
                if self.direction == FORWARD:
                    if self.enable_safety:
                        self.safety_job_name = safety_job_docking_forward
                    else:
                        self.safety_job_name = ""
                else:
                    if self.enable_safety:
                        self.safety_job_name = safety_job_docking_backward
                    else:
                        self.safety_job_name = ""

                rospy.loginfo(
                    f"Distance to hub: {distance_to_hub} \n Vrack: {self.detect_vrack} "
                )

                if distance_to_hub < distance_turn_off_safety_when_docking:
                    self.safety_job_name = ""
                if enable_check_error_when_docking:
                    if (
                        distance_to_hub > dist_check_go_in
                        and distance_to_hub < dist_check_go_in + 0.4
                    ):
                        if (
                            self.detect_vrack
                            and goal_type == PLACE
                            and not self.disable_unable_to_place_cart
                        ):
                            _state_bf_error = MainState.SEND_DOCKING_HUB
                            _state_when_error = _state
                            _state = MainState.UNABLE_PLACE_CART
                            self.moving_control_client.cancel_all_goals()
                            continue
                    if (
                        distance_to_hub > dist_check_go_in
                        and distance_to_hub < dist_check_go_in + 0.2
                    ):
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
                    _state = MainState.ROTATE_TO_GOAL_ANGLE
                    # _state = MainState.CHECK_CART
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

            # State: ROTATE_TO_GOAL_ANGLE
            elif _state == MainState.ROTATE_TO_GOAL_ANGLE:
                _state = MainState.CHECK_CART
                # rospy.logerr(f"angle path: {self.path_angle}")
                # rospy.logerr(f"angle robot: {self.robot_pose_angle}")

                # angle_err = (
                #     self.path_angle - self.robot_pose_angle
                # )  # Robot is go in matehan/hub backward
                # rospy.logerr(f"angle in hub error 1: {angle_err}")

                # angle_err = atan(sin(angle_err) / cos(angle_err))
                # rospy.logerr(f"angle in hub error 2: {angle_err}")

                # succeed = self.rotate_to_goal(angle_err)
                # if succeed:
                #     _state = MainState.CHECK_CART
                # else:
                #     rospy.logerr(f"rotating to goal - result:{succeed}")

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
                    rospy.logwarn("goal type: PICK")
                    if not FAKE_QR_CODE:
                        resp = self.get_qr_code(2)
                        qr_data = resp.Res
                        rospy.logwarn("Get qr code: {}".format(qr_data))
                        if qr_data != "" and qr_data == self.cart:
                            self.cart = qr_data
                            _state = MainState.LIFT_MAX

                        elif (
                            qr_data != ""
                            and qr_data != self.cart
                            and qr_data != "CONNECT_ERROR"
                        ):

                            # _state_when_error = _state
                            # _state = MainState.WRONG_CART
                            _state = MainState.LIFT_MAX
                            rospy.logerr(
                                f"HUB: WRONG_CART: recv-{self.cart}; pick-{qr_data}"
                            )

                        elif qr_data == "":
                            # _state_when_error = _state
                            # _state = MainState.NO_CART
                            _state = MainState.LIFT_MAX
                            rospy.logerr(f"HUB: NO_CART recv-{self.cart}")

                        else:
                            _state = MainState.LIFT_MAX
                            rospy.logerr(f"HUB: READ_CART ERROR")
                            # _state_when_error = _state
                            # _state = MainState.READ_CART_ERROR
                    else:
                        _state = MainState.LIFT_MAX
                else:
                    rospy.logwarn("goal type: PLACE")
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
                if self.liftup_finish_first_check:
                    if not FAKE_QR_CODE and self.server_config != None:
                        if self.upDateCart(
                            self.type, self.name, self.cell, "", ""
                        ) and self.upDateCart(
                            "AGV", self.data, 0, self.cart, self.lot
                        ):
                            _state = MainState.SEND_GOTO_OUT_OF_HUB
                        else:
                            # _state = MainState.UPDATE_CART_ERROR
                            rospy.logwarn("UPDATE_CART_ERROR --> RETRY")
                    else:
                        _state = MainState.SEND_GOTO_OUT_OF_HUB
                    if self.lot == "":
                        self.db.saveStatusCartData(
                            "status_cart", "have_empty_cart"
                        )
                    else:
                        self.db.saveStatusCartData(
                            "status_cart", "have_full_cart"
                        )
                    if not self.liftup_finish:
                        self.type_lift = LIFT_UP
                        self.start_lift_timer()
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
                if self.liftdown_finish_first_check:
                    if not FAKE_QR_CODE and self.server_config != None:
                        if self.upDateCart(
                            self.type, self.name, self.cell, self.cart, self.lot
                        ) and self.upDateCart("AGV", self.data, 0, "", ""):
                            if self.use_new_traffic_control:
                                _state = MainState.DONE
                            else:
                                _state = MainState.SEND_GOTO_OUT_OF_HUB
                        else:
                            # _state = MainState.UPDATE_CART_ERROR
                            rospy.logwarn("UPDATE_CART_ERROR --> RETRY")
                    else:
                        if self.use_new_traffic_control:
                            _state = MainState.DONE
                        else:
                            _state = MainState.SEND_GOTO_OUT_OF_HUB
                    self.db.saveStatusCartData("status_cart", "no_cart")
                    if not self.liftdown_finish:
                        self.type_lift = LIFT_DOWN
                        self.start_lift_timer()
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
                if self.liftup_finish_first_check:
                    _state = MainState.SEND_DOCKING_HUB
                    if not self.liftup_finish:
                        self.type_lift = LIFT_UP
                        self.start_lift_timer()
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

            # State: LIFT_MIN
            elif _state == MainState.LIFT_MIN_FIRST:
                if self.liftdown_finish_first_check:
                    _state = MainState.SEND_DOCKING_HUB
                    if not self.liftdown_finish:
                        self.type_lift = LIFT_DOWN
                        self.start_lift_timer()
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

            # State: LIFT_MIN
            elif _state == MainState.LIFT_MIN_END:
                if self.liftdown_finish_first_check:
                    _state = MainState.DONE
                    if not self.liftdown_finish:
                        self.type_lift = LIFT_DOWN
                        self.start_lift_timer()
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

            # State: SEND_GOTO_OUT_OF_HUB
            elif _state == MainState.SEND_GOTO_OUT_OF_HUB:
                check_go_in = False
                self.moving_control_client.send_goal(
                    self.undocking_goal, feedback_cb=self.moving_control_fb
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
                self.dynamic_reconfig_movebase(
                    vel_docking_hub, publish_safety=False, stop_center_qr=True
                )
                if self.direction == FORWARD:
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
                            # self.followline_run_pause_pub.publish(
                            #     StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                            # )
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
                            # self.pub_reset_param_followline.publish(EmptyStamped(stamp=rospy.Time.now()))
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
                        if self.direction == BACKWARD:
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

            # .##.....##..#######..##.....##.####.##....##..######...........########.########..########...#######..########.
            # .###...###.##.....##.##.....##..##..###...##.##....##..........##.......##.....##.##.....##.##.....##.##.....##
            # .####.####.##.....##.##.....##..##..####..##.##................##.......##.....##.##.....##.##.....##.##.....##
            # .##.###.##.##.....##.##.....##..##..##.##.##.##...####.........######...########..########..##.....##.########.
            # .##.....##.##.....##..##...##...##..##..####.##....##..........##.......##...##...##...##...##.....##.##...##..
            # .##.....##.##.....##...##.##....##..##...###.##....##..........##.......##....##..##....##..##.....##.##....##.
            # .##.....##..#######.....###....####.##....##..######...#######.########.##.....##.##.....##..#######..##.....##

            elif _state == MainState.DETECT_MIRROR_ERROR:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/hub_server: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = _state_when_error

            elif _state == MainState.UNABLE_PLACE_CART:
                # rospy.logwarn(self.error_position)
                # rospy.logerr(self.error_angle)
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = (
                    "/hub_server: {}".format(_state.toString())
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
                    self.disable_unable_to_place_cart = False
                    self._asm.reset_flag()
                    # self.moving control_reset_error_pub.publish(
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
                    "/hub_server: {}".format(_state.toString())
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
                    # self.moving control_reset_error_pub.publish(
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
                    "/hub_server: {}".format(_state.toString())
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
                    # self.moving control_reset_error_pub.publish(
                    #     EmptyStamped(stamp=rospy.Time.now())
                    # )
                    # _state = _state_bf_error
                    self.error_position = 0
                    self.error_angle = 0
                    if _state_when_error == MainState.GOING_TO_OUT_OF_HUB:
                        _state = MainState.SEND_GOTO_OUT_OF_HUB
                    else:
                        _state = MainState.SEND_GOTO_WAITING
                    self.moving_control_error_code = ""

            # State: MOVING_DISCONNECTED
            elif _state == MainState.MOVING_DISCONNECTED:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/hub_server: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = _state_bf_error
                    self.moving_control_error_code = ""

            # State: LIFT_POSITION_WRONG
            elif _state == MainState.LIFT_POSITION_WRONG:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/hub_server: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = _state_when_error

            # State: NO_CART
            elif _state == MainState.NO_CART:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/hub_server: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = _state_when_error

            # State: WRONG_CART
            elif _state == MainState.WRONG_CART:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/hub_server: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = _state_when_error

            # State: READ_CART_ERROR
            elif _state == MainState.READ_CART_ERROR:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/hub_server: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = _state_when_error

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
                self.dynamic_reconfig_movebase(
                    self.vel_move_base, publish_safety=True, stop_center_qr=True
                )
                success = True

                # Reset disable unable_to_place_cart
                self.disable_unable_to_place_cart = False
                break
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
        self._asm.action_running = False
        if success:
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

    def wait_until_pose_available(self):
        while not rospy.is_shutdown():
            pose = self.get_current_pose_in_odom()
            if pose is not None:
                return pose
            rospy.sleep(0.1)

    def get_current_pose_in_odom(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                "odom", "base_link", rospy.Time(0), rospy.Duration(1.0)
            )

            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z

            pose.orientation = trans.transform.rotation

            return pose
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn("Cannot get robot pose in odom: %s", e)
            return None

    def transform_pose_map_to_odom(self, pose_in_map):
        # Tạo PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = pose_in_map

        try:
            transformed = self.tf_buffer.transform(
                pose_stamped, "odom", rospy.Duration(1.0)
            )
            return transformed.pose
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn("Transform failed: %s", e)
            return None

    def normalize_angle(self, angle):
        """Đưa angle về khoảng [-pi, pi]"""
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def average_angles(self, angles):
        """
        Trung bình góc (radian) trong khoảng -pi..pi theo phương pháp vector.
        """
        sin_sum = np.sum(np.sin(angles))
        cos_sum = np.sum(np.cos(angles))
        return atan2(sin_sum, cos_sum)

    def get_mirror(
        self,
        frame_global="odom",
        frame_local="center_hub",
        num_samples=10,
        delay=0.05,
    ):
        x_list = []
        y_list = []
        yaw_list = []

        if self.use_tf2:
            for _ in range(num_samples):
                try:
                    trans = self.tf_buffer.lookup_transform(
                        frame_global,
                        frame_local,
                        time=rospy.Time(0),
                        timeout=rospy.Duration(1),
                    )
                    t = trans.transform.translation
                    r = trans.transform.rotation
                    x_list.append(t.x)
                    y_list.append(t.y)

                    euler = euler_from_quaternion([r.x, r.y, r.z, r.w])
                    yaw_list.append(euler[2])

                    rospy.sleep(delay)
                except (
                    tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException,
                    KeyboardInterrupt,
                ):
                    rospy.logwarn("TF exception during averaging")
                    return False
        else:
            for _ in range(num_samples):
                try:
                    self.tf_listener.waitForTransform(
                        frame_global,
                        frame_local,
                        rospy.Time(0),
                        rospy.Duration(1.0),
                    )
                    t, r = self.tf_listener.lookupTransform(
                        frame_global, frame_local, rospy.Time(0)
                    )
                    x_list.append(t[0])
                    y_list.append(t[1])
                    euler = euler_from_quaternion(r)
                    yaw_list.append(euler[2])
                    rospy.sleep(delay)
                except (
                    tf.Exception,
                    tf.ConnectivityException,
                    tf.LookupException,
                    KeyboardInterrupt,
                ):
                    rospy.logwarn("TF exception during averaging")
                    return False

        # Trung bình x, y
        x_avg = np.mean(x_list)
        y_avg = np.mean(y_list)

        # Trung bình góc yaw
        yaw_avg = self.average_angles(yaw_list)

        # Tạo quaternion từ yaw trung bình
        q_avg = quaternion_from_euler(0, 0, yaw_avg)

        # Lưu vào biến thành viên
        self.trans_mirror = [x_avg, y_avg, 0.0]
        self.rot_mirror = [q_avg[0], q_avg[1], q_avg[2], q_avg[3]]

        return True

    def compute_positions_from_mirror(self):
        if not self.get_mirror():
            return False
        x_m, y_m = self.trans_mirror[0], self.trans_mirror[1]
        yaw_m = euler_from_quaternion(self.rot_mirror)

        rospy.logwarn("position mirror: {}".format(self.trans_mirror))
        rospy.logwarn("orient mirror: {}".format(yaw_m))

        cos_yaw = cos(yaw_m[2])
        sin_yaw = sin(yaw_m[2])

        # Hub cách mirror 0.5 m ngược hướng
        x_hub = x_m - 0.42 * cos_yaw
        y_hub = y_m - 0.42 * sin_yaw
        yaw_hub = yaw_m[2]

        # Waiting cách mirror 1.0 m cùng hướng
        x_wait = x_m + 1.2 * cos_yaw
        y_wait = y_m + 1.2 * sin_yaw
        yaw_wait = yaw_m[2]

        # Waiting cách mirror 1.0 m cùng hướng
        x_temp = x_m + 0.8 * cos_yaw
        y_temp = y_m + 0.8 * sin_yaw
        yaw_temp = yaw_m[2]

        return {
            "hub": (x_hub, y_hub, yaw_hub),
            "waiting": (x_wait, y_wait, yaw_wait),
            "temp": (x_temp, y_temp, yaw_temp),
        }

    def compute_goals_from_mirror(self):
        # Tính vị trí hub và waiting từ mirror
        # TODO: set distance offset
        positions = self.compute_positions_from_mirror()
        if not positions:
            rospy.logwarn("Failed to compute positions from mirror")
            return False

        hub_x, hub_y, hub_yaw = positions["hub"]
        wait_x, wait_y, wait_yaw = positions["waiting"]
        temp_x, temp_y, temp_yaw = positions["temp"]
        # TODO: co the tinh huong o day
        if self.direction == FORWARD:
            hub_yaw += pi
            wait_yaw += pi
            temp_yaw += pi
            hub_yaw = self.normalize_angle(hub_yaw)
            temp_yaw = self.normalize_angle(temp_yaw)

        # Tạo pose hub
        hub_pose = self.calculate_pose_offset(0, hub_x, hub_y, hub_yaw)
        # Tạo pose waiting
        waiting_pose = self.calculate_pose_offset(0, wait_x, wait_y, wait_yaw)
        # Tạo pose temp
        temp_pose = self.calculate_pose_offset(0, temp_x, temp_y, temp_yaw)

        # Tạo waiting_goal
        self.waiting_path_dict["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(waiting_pose, self.return_pose_dict)
        )
        self.waiting_goal.data = json.dumps(self.waiting_path_dict, indent=2)
        rospy.logwarn(
            "Waiting goal position:\n{}".format(self.waiting_goal.data)
        )

        # Tạo docking_goal (waypoint 0: waiting, waypoint 1: hub)
        self.docking_path_dict["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(waiting_pose, self.return_pose_dict)
        )
        self.docking_path_dict["waypoints"][1]["position"] = copy.deepcopy(
            obj_to_dict(hub_pose, self.return_pose_dict)
        )
        self.docking_goal.data = json.dumps(self.docking_path_dict, indent=2)
        rospy.logwarn(
            "Docking goal position:\n{}".format(self.docking_goal.data)
        )

        # Tạo undocking_goal (ngược lại docking)
        self.undocking_path_dict["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(hub_pose, self.return_pose_dict)
        )
        self.undocking_path_dict["waypoints"][1]["position"] = copy.deepcopy(
            obj_to_dict(waiting_pose, self.return_pose_dict)
        )
        self.undocking_goal.data = json.dumps(
            self.undocking_path_dict, indent=2
        )
        rospy.logwarn(
            "UnDocking goal position:\n{}".format(self.undocking_goal.data)
        )
        # Tạo Temp goal (ngược lại docking)
        odom_pose = self.wait_until_pose_available()
        self.temp_path_dict["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(odom_pose, self.return_pose_dict)
        )
        self.temp_path_dict["waypoints"][1]["position"] = copy.deepcopy(
            obj_to_dict(temp_pose, self.return_pose_dict)
        )
        self.temp_goal.data = json.dumps(self.temp_path_dict, indent=2)
        rospy.logwarn("Temp goal position:\n{}".format(self.temp_goal.data))

        # Tạo rotation goal (ngược lại docking)
        self.rotation_path_dict["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(temp_pose, self.return_pose_dict)
        )
        self.rotation_goal.data = json.dumps(self.rotation_path_dict, indent=2)
        rospy.logwarn(
            "rotation goal position:\n{}".format(self.rotation_goal.data)
        )
        return True

    def switch_control_tf(self, type):
        while not rospy.is_shutdown():
            self.control_tf_pub.publish(Int8(data=type))
            if self.current_control_tf == type:
                rospy.sleep(0.1)
                break
            rospy.sleep(0.1)

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

    def dynamic_reconfig_movebase(self, vel_x, publish_safety, stop_center_qr):
        new_config = {
            "max_vel_x": vel_x,
            "max_vel_trans": vel_x,
            "publish_safety": publish_safety,
            "stop_center_qr": stop_center_qr,
        }
        for i in range(3):
            self.client_reconfig_movebase.update_configuration(new_config)
            rospy.sleep(0.1)

    def handle_lift_publish(self, event):
        if self.type_lift == LIFT_UP:
            if self.liftup_finish:
                rospy.loginfo("Lift up completed, stopping timer.")
                self.stop_lift_timer()  # Dừng và xóa timer
                return

            self.lift_msg.stamp = rospy.Time.now()
            self.lift_msg.data = LIFT_UP
            self.pub_lift_cmd.publish(self.lift_msg)
        else:
            if self.liftdown_finish:
                rospy.loginfo("Lift down completed, stopping timer.")
                self.stop_lift_timer()  # Dừng và xóa timer
                return

            self.lift_msg.stamp = rospy.Time.now()
            self.lift_msg.data = LIFT_DOWN
            self.pub_lift_cmd.publish(self.lift_msg)

    def start_lift_timer(self):
        if self.lift_timer is None:
            rospy.loginfo("Starting lift timer.")
            self.lift_timer = rospy.Timer(
                rospy.Duration(0.1), self.handle_lift_publish
            )
        else:
            rospy.logwarn("Lift timer is already running.")

    def stop_lift_timer(self):
        if self.lift_timer is not None:
            rospy.loginfo("Stopping lift timer.")
            self.lift_timer.shutdown()
            self.lift_timer = None

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

    def get_odom(self, frame_global="map", frame_local="base_link"):
        self.pose_map2robot = Pose()
        if self.use_tf2:
            try:
                trans = self.tf_buffer.lookup_transform(
                    frame_global,
                    frame_local,
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
                    frame_global,
                    frame_local,
                    rospy.Time(0),
                    rospy.Duration(4.0),
                )
                self.trans, self.rot = self.tf_listener.lookupTransform(
                    frame_global, frame_local, rospy.Time(0)
                )
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

    def rotate_to_goal(self, angle):
        error_angle = angle  # - self.theta

        if np.abs(error_angle) < 0.001:
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
            self.cmd_vel_pub.publish(self.vel)
            return True

        if error_angle > 0:
            self.vel.angular.z = np.clip(0.1 * error_angle, 0.02, 0.08)
        else:
            self.vel.angular.z = np.clip(0.1 * error_angle, -0.08, -0.02)
        self.vel.linear.x = 0.0
        self.cmd_vel_pub.publish(self.vel)
        return False

    def get_path_angle(self, waiting_pose, docking_pose):
        angle = atan2(
            waiting_pose[1] - docking_pose[1], waiting_pose[0] - docking_pose[0]
        )  # Because agv going backward in hub so angle of path is hub_point -> waiting_point
        return angle

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
            # rospy.logwarn_throttle(
            #     2,
            #     "error_position = {}".format(
            #         self.min_sensor_1 - self.min_sensor_2
            #     ),
            # )
            # rospy.logwarn_throttle(
            #     2,
            #     "error_angle= {}".format(
            #         self.min_sensor_1 - self.max_sensor_2
            #     ),
            # )
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
    rospy.init_node("hub_server", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    HubAction(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
