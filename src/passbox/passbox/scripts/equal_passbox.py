#!/usr/bin/env python3
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.exceptions import ModbusException
import rospy
import time
import threading
import wareshare_modbus
from wareshare_modbus import check_connect, disconnect, clear_all_output_bits
import yaml
import socket
from datetime import datetime
import os
import sys
import rospy
import rospkg
import numpy as np
import math
HOME = os.path.expanduser("~")
import tf
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import copy
import actionlib
import dynamic_reconfigure.client
from dynamic_reconfigure.server import Server
import requests
from offset_agv import agv_offset
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
from mirror_detect.srv import HubService, HubServiceResponse, HubServiceRequest
from cognex_qr_code.srv import *
from agv_msgs.msg import ErrorRobotToPath

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
from std_stamped_msgs.msg import (
    StringAction,
    StringStamped,
    StringResult,
    StringFeedback,
    StringGoal,
    Int8Stamped,
    EmptyStamped,
)
from module_manager import ModuleServer, ModuleClient, ModuleStatus

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
    SEND_GOTO_WAITING = 14
    GOING_TO_WAITING = 15
    SEND_GOTO_OUT_OF_HUB = 23
    GOING_TO_OUT_OF_HUB = 24
    MOVING_DISCONNECTED = 28
    INIT = 29
    NO_CART = 31
    OPTICAL_SENSOR_ERROR = 32
    EMG_AGV = 33
    ALIGNMENT_SENSOR = 35
    LIFT_MIN_END = 36
    LIFT_MIN_FIRST = 37
    LIFT_MAX_FIRST = 38
    UNABLE_PLACE_CART = 47
    WRONG_CART = 48
    COLLISION_POSSIBLE = 50
    EMG_PASSBOX = 60
    NETWORK_ERROR = 71
    REQUEST_ENTER_PASSBOX = 86
    SEND_GOTO_TEMP_POSE = 90
    GOING_TO_TEMP_POSE = 91
    SEND_ROTATE_FIND_MIRROR = 92
    ROTATE_FIND_MIRROR = 93
    DETECT_MIRROR_ERROR = 94
    GOING_TO_WAITING_LIFT = 96
    WAIT_RESET_IO = 108

PICK = 1
PLACE = 0
ON = 1
OFF = 0
LIFT_UP = 1
LIFT_DOWN = 2
FORWARD = 1
BACKWARD = 0
USE_DOCKING_BY_MIRROR = False
###WARESHARE###
# --- ĐỊA CHỈ BIT ---
# INPUT WAREHOUSE, OUTPUT AGV (warehouse gửi tín hiệu đến AGV)
OPEN_DIRTY_SIDE = 1
CLOSE_DIRTY_SIDE = 2
PAUSE_DIRTY_SIDE = 3
EMG_WHEN_DIRTY_SIDE = 4
OPEN_CLEAN_SIDE = 5
CLOSE_CLEAN_SIDE = 6
PAUSE_CLEAN_SIDE = 7
EMG_WHEN_CLEAN_SIDE = 8

# OUTPUT WAREHOUSE, INPUT AGV (AGV gửi tín hiệu về warehouse)
DONE_OPEN_DIRTY_SIDE = 2
DONE_CLOSE_DIRTY_SIDE = 1
DONE_OPEN_CLEAN_SIDE = 3
DONE_CLOSE_CLEAN_SIDE = 4
EMG_DIRTY_SIDE = 5
EMG_CLEAN_SIDE = 6

class PassboxEqualAction(object):
    _feedback = StringFeedback()
    _result = StringResult()

    def __init__(self, name, *args, **kwargs):
        self.init_variable(*args, **kwargs)

        # Initialize ModuleServer first to avoid AttributeError
        self._asm = ModuleServer(name)

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

        #Publisher
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
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        # Subscriber
        rospy.Subscriber("/lidar_info", StringStamped, self.lidar_info_cb)
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
        rospy.Subscriber("/robot_pose", Pose, self.robot_pose_cb)
        rospy.Subscriber(
            "/current_traffic_control_type",
            StringStamped,
            self.current_traffic_control_type_cb,
        )
        rospy.Subscriber(
            "/current_control_tf", Int8, self.current_control_tf_cb
        )
        # Service client
        self.get_qr_code = rospy.ServiceProxy("ReadQrCode", QrCode)
        # hub_service (mirror detect)
        try:
            rospy.wait_for_service("hub_service", 30)
            try:
                self.call_hub_service = rospy.ServiceProxy("hub_service", HubService)
                rospy.loginfo("hub_service: is running")
            except rospy.ServiceException as e:
                rospy.logerr(f"Fail to create hub_service proxy: {e}")
        except rospy.ROSException as e:
            rospy.logerr(f"hub_service service is not available: {e}")
        # dynamic reconfig client
        self.client_reconfig_movebase = dynamic_reconfigure.client.Client(
            "/move_base/NeoLocalPlanner",
            timeout=30,
            config_callback=self.dynamic_callback,
        )

        self.init_server()
        # Loop
        self.loop()

    def dynamic_callback(self, config):
        rospy.loginfo("Dynamic reconfigure callback")
        pass

    def init_variable(self, *args, **kwargs):
        self.config_path = kwargs["config_path"]
        self.robot_config_file = kwargs["robot_config_file"]
        self.server_config_file = kwargs["robot_define"]
        self.use_tf2 = False
        self.tf_listener = tf.TransformListener()
        self.last_moving_control_fb = rospy.get_time()
        self.moving_control_result = -1
        self.moving_control_error_code = ""
        # Database
        db_address = rospy.get_param("/mongodb_address", "localhost")
        self.db = mongodb(db_address)
        self.emg_status = True
        self.type_lift = LIFT_UP
        self.liftup_finish = False
        self.liftup_finish_first_check = False
        self.detect_vrack = False
        self.liftdown_finish = False
        self.liftdown_finish_first_check = False
        self.enable_safety = False
        self.vel_move_base = 0.8
        self.wareshare_ip = rospy.get_param("~wareshare_ip", "192.168.1.200")
        self.wareshare_port = rospy.get_param("~wareshare_port", 502)
        rospy.loginfo("Connecting to Wareshare: {}:{}".format(self.wareshare_ip, self.wareshare_port))
        self.wareshare = ModbusTcpClient(self.wareshare_ip, self.wareshare_port)
        if self.wareshare.connect():
            rospy.loginfo("Connected to Wareshare success")
        else:
            rospy.logerr("Connected to Wareshare failed")

        self.first_emg_agv = -1

        self.lift_msg = Int8Stamped()
        self.disable_qr_code_msg = Int8Stamped()
        self.std_io_msg = StringStamped()
        self.last_time_get_lift_up = rospy.get_time()
        self.last_time_get_lift_down = rospy.get_time()
        self.lift_timer = None

        self.robot_pose_angle = None
        self.robot_odom_angle = None
        self.path_angle = None
        self.qr_angle = None
        self.vel = Twist()

        self.resetTimeoutError = False
        self.is_plc_connect_fail = False

        self.use_new_traffic_control = False
        self.current_control_tf = 100

        self.data_run = StringStamped()
        self.data_run.data = "RUN"
        self.mode_robot = ""

        # Additional missing variable initializations
        self.cmd_vel_msg = Twist()
        self.dirty_or_clean = True  # True for dirty side, False for clean side
        self.get_first_time_error = True
        self.step = 0
        self.safety_job_name = None
        self.pre_safety_job_name = None
        self.error_position = 0.0
        self.error_angle = 0.0
        self.status_robot = ""
        self.start_1 = False
        self.start_2 = False
        self.stop_1 = False
        self.stop_2 = False
        self.pose_odom2robot = Pose()
        self.server_config = None
        self.data = None

        # tf2 buffer (dùng cho mirror: lookup odom->center_hub)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(0.1))
        self.tf2_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Lidar info for coordinate conversion (mirror mode)
        self.lidar_info = None
        self.lidar_scale = None
        self.lidar_offset_x = None
        self.lidar_offset_y = None
        self.lidar_angle = None
        self.has_lidar_info = False

        # Mirror transform samples
        self.trans_mirror = [0.0, 0.0, 0.0]
        self.rot_mirror = [0.0, 0.0, 0.0, 1.0]

        # Path offsets (dùng bởi agv_offset trong execute_cb)
        self.path_offset_x = rospy.get_param("path_offset_x", 0.0)
        self.path_offset_y = rospy.get_param("path_offset_y", 0.0)
        # Path/hub geometry
        self.initial_hub_to_lift_distance = 0.0
        self.initial_lift_to_waiting_distance = 0.0
        self.initial_hub_to_waiting_distance = 0.0
        # Mirror offsets (default, sẽ được overwrite khi load hub.json)
        self.mirror_offsets = {
            "hub":     {"x_offset": -0.42, "y_offset": 0.0},
            "waiting": {"x_offset":  0.8,  "y_offset": 0.0},
            "temp":    {"x_offset":  0.8,  "y_offset": 0.0},
        }
        self.mirror_offsets_lift = {
            "hub":     {"x_offset": -0.42, "y_offset": 0.0},
            "waiting": {"x_offset":  0.8,  "y_offset": 0.0},
            "temp":    {"x_offset":  0.8,  "y_offset": 0.0},
        }
        self.length_hub = 1.0  # default, overwrite from hub.json
        self.choose_config_docking = False #True khi docking lift false khi docking hub


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
        # Lấy góc yaw từ quaternion trong odom
        r_x = msg.pose.pose.orientation.x
        r_y = msg.pose.pose.orientation.y
        r_z = msg.pose.pose.orientation.z
        r_w = msg.pose.pose.orientation.w
        roll, pitch, yaw = euler_from_quaternion((r_x, r_y, r_z, r_w))
        self.robot_odom_angle = yaw

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

    def lidar_info_cb(self, msg):
        """Callback function for /lidar_info topic"""
        try:
            lidar_data = json.loads(msg.data)
            if (
                "scale" in lidar_data
                and "offset" in lidar_data
                and "x" in lidar_data["offset"]
                and "y" in lidar_data["offset"]
                and "angle" in lidar_data
            ):
                self.lidar_scale = float(lidar_data["scale"])
                self.lidar_offset_x = float(lidar_data["offset"]["x"])
                self.lidar_offset_y = float(lidar_data["offset"]["y"])
                self.lidar_angle = float(lidar_data["angle"])
                self.has_lidar_info = True
                self.lidar_info = lidar_data
            else:
                rospy.logwarn("Incomplete lidar info received, missing required fields")
                self.has_lidar_info = False
        except (json.JSONDecodeError, ValueError, KeyError) as e:
            rospy.logerr(f"Error parsing lidar info: {e}")
            self.has_lidar_info = False

    def robot_pose_cb(self, msg):
        r_x = msg.orientation.x
        r_y = msg.orientation.y
        r_z = msg.orientation.z
        r_w = msg.orientation.w
        roll, pitch, yaw = euler_from_quaternion((r_x, r_y, r_z, r_w))
        self.robot_pose_angle = yaw

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
        global USE_DOCKING_BY_MIRROR
        while True:
            if self.get_odom():
                break
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

                # Load length_hub parameter
                if "length_hub" in hub_dict:
                    self.length_hub = hub_dict["length_hub"]
                    rospy.loginfo(f"Loaded length_hub: {self.length_hub}")
                else:
                    self.length_hub = 1.0  # Default value
                    rospy.logwarn(f"length_hub not found in config, using default: {self.length_hub}")

                if "mirror_offsets" in hub_dict:
                    self.mirror_offsets = hub_dict["mirror_offsets"]
                else:
                    # Default values for backward compatibility
                    self.mirror_offsets = {
                        "hub": {"x_offset": -0.42, "y_offset": 0.0},
                        "waiting": {"x_offset": 0.8, "y_offset": 0.0},
                        "temp": {"x_offset": 0.8, "y_offset": 0.0}
                    }
            # Waiting path config
            waiting_path_cfg = os.path.join(
                self.config_path, "waiting_path.json"
            )
            with open(waiting_path_cfg) as j:
                self.waiting_path_dict = json.load(j)
                self.return_pose_dict = self.waiting_path_dict["waypoints"][0]["position"]
                return_pose_dict = self.return_pose_dict
                self.inside_path_dict = copy.deepcopy(self.waiting_path_dict)
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
        self.direction = BACKWARD
        self.enable_safety = False

        # floor_equal=True: AGV và hub cùng tầng (dùng inside_pose)
        # floor_equal=False: AGV và hub khác tầng (dùng waiting_pose để đến lift)
        floor_equal = data_dict["params"]["floor_equal"]

        # Direction mặc định: PICK = FORWARD (quay đầu vào hub), PLACE = BACKWARD (lùi vào hub)
        # Có thể bị override bởi property "invert" bên dưới
        pick_or_place = data_dict["params"]["pick_or_place"]
        if pick_or_place:
            goal_type = PICK
            self.direction = FORWARD   # mặc định PICK: quay đầu vào hub
            _state = MainState.INIT
        else:
            goal_type = PLACE
            self.direction = BACKWARD  # mặc định PLACE: lùi vào hub
            _state = MainState.INIT

        try:
            hub_pose_x = data_dict["params"]["position"]["x"]
            hub_pose_y = data_dict["params"]["position"]["y"]
            waiting_pose_x = data_dict["params"]["waiting_position"]["x"]
            waiting_pose_y = data_dict["params"]["waiting_position"]["y"]
            inside_pose_x = data_dict["params"]["inside_position"]["x"]
            inside_pose_y = data_dict["params"]["inside_position"]["y"]

            if floor_equal:
                # Tính distance từ hub đến lift_pose ban đầu
                self.initial_hub_to_waiting_distance = distance_two_points(
                    hub_pose_x, hub_pose_y, inside_pose_x, inside_pose_y
                )
                rospy.loginfo(f"Initial hub to waiting distance: {self.initial_hub_to_waiting_distance}")

                # Add this to add offset to docking path
                hub_pose = [hub_pose_x, hub_pose_y]
                inside_pose = [inside_pose_x, inside_pose_y]

                hub_offset = agv_offset(
                    inside_pose, hub_pose, self.path_offset_x, self.path_offset_y
                )
                self.path_angle = self.get_path_angle(hub_pose, inside_pose)
                inside_pose = hub_offset.calculate_offset(inside_pose)
                hub_pose = hub_offset.calculate_offset(hub_pose)
                inside_pose_x = inside_pose[0]
                inside_pose_y = inside_pose[1]
                hub_pose_x = hub_pose[0]
                hub_pose_y = hub_pose[1]

            else:
                # Tính distance từ hub đến lift_pose ban đầu
                self.initial_hub_to_lift_distance = distance_two_points(
                    hub_pose_x, hub_pose_y, waiting_pose_x, waiting_pose_y
                )
                rospy.loginfo(f"Initial hub to lift distance: {self.initial_hub_to_lift_distance}")
                # Add this to add offset to docking path
                hub_pose = [hub_pose_x, hub_pose_y]
                waiting_pose = [waiting_pose_x, waiting_pose_y]

                hub_offset = agv_offset(
                    waiting_pose, hub_pose, self.path_offset_x, self.path_offset_y
                )
                self.path_angle = self.get_path_angle(hub_pose, waiting_pose)
                waiting_pose = hub_offset.calculate_offset(waiting_pose)
                hub_pose = hub_offset.calculate_offset(hub_pose)
                hub_pose_x = hub_pose[0]
                hub_pose_y = hub_pose[1]


            self.type = "PASSBOX_EQUAL"
            self.name = data_dict["params"]["name"]
            self.cell = 0  # data_dict["params"]["cell"]
            self.cart = data_dict["params"]["cart"]
            self.lot = data_dict["params"]["lot"]
            self.disable_detect_vrack = False
            self.disable_check_cart = False
            self.disable_lift = False
            if "properties" in data_dict["params"]:
                if "Safety" in data_dict["params"]["properties"]:
                    if data_dict["params"]["properties"]["Safety"] == "Disable":
                        self.enable_safety = False
                if "disable_lift" in data_dict["params"]["properties"]:
                    self.disable_lift = data_dict["params"]["properties"]["disable_lift"]
                    rospy.loginfo(f"self.disable_lift {self.disable_lift}")
                # Add new properties for disabling vrack detection and check_cart
                if "disable_detect_vrack" in data_dict["params"]["properties"]:
                    self.disable_detect_vrack = data_dict["params"]["properties"]["disable_detect_vrack"]
                    rospy.loginfo(f"self.disable_detect_vrack {self.disable_detect_vrack}")
                else:
                    self.disable_detect_vrack = False

                if "disable_check_cart" in data_dict["params"]["properties"]:
                    self.disable_check_cart = data_dict["params"]["properties"]["disable_check_cart"]
                    rospy.loginfo(f"self.disable_check_cart {self.disable_check_cart}")
                else:
                    self.disable_check_cart = False

                # Add property to control USE_DOCKING_BY_MIRROR
                if "use_mirror" in data_dict["params"]["properties"]:
                    USE_DOCKING_BY_MIRROR = data_dict["params"]["properties"]["use_mirror"]
                    rospy.loginfo(f"USE_DOCKING_BY_MIRROR from properties: {USE_DOCKING_BY_MIRROR}")
                else:
                    # Giữ giá trị default nếu không có trong properties
                    rospy.loginfo(f"USE_DOCKING_BY_MIRROR default: {USE_DOCKING_BY_MIRROR}")

                # Add offset support from properties
                if "offset_x" in data_dict["params"]["properties"]:
                    try:
                        additional_offset_x = float(data_dict["params"]["properties"]["offset_x"])
                        rospy.logwarn(f"Additional offset_x from properties: {additional_offset_x}")
                        # Apply additional offset to mirror offsets if they exist
                        if hasattr(self, 'mirror_offsets'):
                            self.mirror_offsets["hub"]["x_offset"] += additional_offset_x
                            self.mirror_offsets["waiting"]["x_offset"] += additional_offset_x
                            self.mirror_offsets["temp"]["x_offset"] += additional_offset_x
                    except (ValueError, TypeError) as e:
                        rospy.logerr(f"Invalid offset_x value in properties: {e}")

                if "offset_y" in data_dict["params"]["properties"]:
                    try:
                        additional_offset_y = float(data_dict["params"]["properties"]["offset_y"])
                        rospy.logwarn(f"Additional offset_y from properties: {additional_offset_y}")
                        # Apply additional offset to mirror offsets if they exist
                        if hasattr(self, 'mirror_offsets'):
                            self.mirror_offsets["hub"]["y_offset"] += additional_offset_y
                            self.mirror_offsets["waiting"]["y_offset"] += additional_offset_y
                            self.mirror_offsets["temp"]["y_offset"] += additional_offset_y
                    except (ValueError, TypeError) as e:
                        rospy.logerr(f"Invalid offset_y value in properties: {e}")
                cell_number = None
                try:
                    # Lấy cell number từ position hoặc cell
                    if "ceil" in data_dict["params"]["position"]:
                        cell_number = data_dict["params"]["position"]["ceil"]
                    elif "cell" in data_dict["params"]:
                        cell_number = data_dict["params"]["cell"]

                    if cell_number is not None:
                        rospy.loginfo(f"Detected cell number: {cell_number}")

                        # Check for ceil-specific offset_x
                        offset_x_key = f"offset_x_ceil_{cell_number}"
                        if offset_x_key in data_dict["params"]["properties"]:
                            try:
                                additional_offset_x = float(data_dict["params"]["properties"][offset_x_key])
                                rospy.logwarn(f"Additional {offset_x_key} from properties: {additional_offset_x}")
                                # Apply additional offset to mirror offsets if they exist
                                if hasattr(self, 'mirror_offsets'):
                                    self.mirror_offsets["hub"]["x_offset"] += additional_offset_x
                                    self.mirror_offsets["waiting"]["x_offset"] += additional_offset_x
                                    self.mirror_offsets["temp"]["x_offset"] += additional_offset_x
                            except (ValueError, TypeError) as e:
                                rospy.logerr(f"Invalid {offset_x_key} value in properties: {e}")

                        # Check for ceil-specific offset_y
                        offset_y_key = f"offset_y_ceil_{cell_number}"
                        if offset_y_key in data_dict["params"]["properties"]:
                            try:
                                additional_offset_y = float(data_dict["params"]["properties"][offset_y_key])
                                rospy.logwarn(f"Additional {offset_y_key} from properties: {additional_offset_y}")
                                # Apply additional offset to mirror offsets if they exist
                                if hasattr(self, 'mirror_offsets'):
                                    self.mirror_offsets["hub"]["y_offset"] += additional_offset_y
                                    self.mirror_offsets["waiting"]["y_offset"] += additional_offset_y
                                    self.mirror_offsets["temp"]["y_offset"] += additional_offset_y
                            except (ValueError, TypeError) as e:
                                rospy.logerr(f"Invalid {offset_y_key} value in properties: {e}")
                except Exception as e:
                    rospy.logerr(f"Error processing ceil-specific properties: {e}")

                # Log final offset values after applying properties
                if hasattr(self, 'mirror_offsets'):
                    rospy.logwarn(f"Final mirror offsets after properties: {self.mirror_offsets}")

            else:
                # Set default values if no properties
                self.disable_detect_vrack = False
                self.disable_check_cart = False
                self.disable_lift = False

            # Convert coordinates if USE_DOCKING_BY_MIRROR and lidar info available
            if USE_DOCKING_BY_MIRROR and self.has_lidar_info:
                # Convert hub pose
                original_hub_x, original_hub_y = hub_pose_x, hub_pose_y
                converted_hub_x, converted_hub_y = self.convert_qr_to_lidar_coordinate(hub_pose_x, hub_pose_y)

                if converted_hub_x is not None and converted_hub_y is not None:
                    hub_pose_x = converted_hub_x
                    hub_pose_y = converted_hub_y
                    rospy.loginfo(f"Converted hub pose: ({original_hub_x:.3f}, {original_hub_y:.3f}) -> ({hub_pose_x:.3f}, {hub_pose_y:.3f})")
                else:
                    rospy.logwarn("Failed to convert hub pose coordinates, using original")

##############################################################################3
                original_inside_x, original_inside_y = inside_pose_x, inside_pose_y
                converted_inside_x, converted_inside_y = self.convert_qr_to_lidar_coordinate(inside_pose_x, inside_pose_y)

                if converted_inside_x is not None and converted_inside_y is not None:
                    inside_pose_x = converted_inside_x
                    inside_pose_y = converted_inside_y
                    rospy.loginfo(f"Converted inside pose: ({original_inside_x:.3f}, {original_inside_y:.3f}) -> ({inside_pose_x:.3f}, {inside_pose_y:.3f})")
                else:
                    rospy.logwarn("Failed to convert inside pose coordinates, using original")
###############################################################################
                original_waiting_x, original_waiting_y = waiting_pose_x, waiting_pose_y
                converted_waiting_x, converted_waiting_y = self.convert_qr_to_lidar_coordinate(waiting_pose_x, waiting_pose_y)

                if converted_waiting_x is not None and converted_waiting_y is not None:
                    waiting_pose_x = converted_waiting_x
                    waiting_pose_y = converted_waiting_y
                    rospy.loginfo(f"Converted waiting pose: ({original_waiting_x:.3f}, {original_waiting_y:.3f}) -> ({waiting_pose_x:.3f}, {waiting_pose_y:.3f})")
                else:
                    rospy.logwarn("Failed to convert waiting pose coordinates, using original")
            else:
                if USE_DOCKING_BY_MIRROR:
                    rospy.logwarn("USE_DOCKING_BY_MIRROR is True but no lidar info available, using original coordinates")

        except:
            hub_pose_x = data_dict["params"]["position"]["x"]
            hub_pose_y = data_dict["params"]["position"]["y"]
            waiting_pose_x = self.trans[0]
            waiting_pose_y = self.trans[1]
            self.type = "PASSBOX_EQUAL"
            self.name = "AGV 01"
            self.cell = 0
            self.cart = "VRACK"
            self.lot = "1"
            use_server = False
        if use_server:
            FAKE_QR_CODE = False
        else:
            FAKE_QR_CODE = True
        # PICK: quay đầu vào hub (FORWARD), PLACE: lùi vào hub (BACKWARD)
        # direction có thể đã bị override bởi property "invert"
        if floor_equal:
            if self.direction == FORWARD:
                cur_orient = atan2(
                    hub_pose_y - inside_pose_y, hub_pose_x - inside_pose_x
                )
            else:  # BACKWARD
                cur_orient = atan2(
                    inside_pose_y - hub_pose_y, inside_pose_x - hub_pose_x
                )
        else:
            if self.direction == FORWARD:
                cur_orient = atan2(
                    hub_pose_y - waiting_pose_y, hub_pose_x - waiting_pose_x
                )
            else:  # BACKWARD
                cur_orient = atan2(
                    waiting_pose_y - hub_pose_y, waiting_pose_x - hub_pose_x
                )

        # ============================================================
        # CALCULATE INSIDE GOAL
        # ============================================================
        self.inside_goal = StringGoal()
        inside_pose = self.calculate_pose_offset(
            0,
            inside_pose_x,
            inside_pose_y,
            cur_orient,
        )
        rospy.logwarn(" cur_orient docking: {}".format(cur_orient))
        if USE_DOCKING_BY_MIRROR:
            odom_pose = self.wait_until_pose_available()
            if odom_pose is None:
                rospy.logerr("Failed to get odom_pose")
                self._as.set_aborted("Failed to get odom_pose")
                return
            inside_pose = self.transform_pose_map_to_odom(inside_pose)
            inside_pose.position.x = odom_pose.position.x
            inside_pose.position.y = odom_pose.position.y
        self.inside_path_dict["params"] = {}
        self.inside_path_dict["params"]["only_use_qr"] = True
        if USE_DOCKING_BY_MIRROR:
            self.inside_path_dict["params"]["use_mirror"] = True
        self.inside_path_dict["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(inside_pose, self.return_pose_dict)
        )
        self.inside_goal.data = json.dumps(self.inside_path_dict, indent=2)
        rospy.logwarn(
            "Waiting goal position:\n{}".format(
                json.dumps(self.inside_path_dict, indent=2)
            )
        )

        # ============================================================
        # CALCULATE WAITING GOAL
        # ============================================================
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
        self.waiting_path_dict["params"] = {}
        self.waiting_path_dict["params"]["only_use_qr"] = True
        if USE_DOCKING_BY_MIRROR:
            self.waiting_path_dict["params"]["use_mirror"] = True
        self.waiting_path_dict["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(waiting_pose, self.return_pose_dict)
        )
        self.waiting_goal.data = json.dumps(self.waiting_path_dict, indent=2)
        rospy.logwarn(
            "Waiting goal position:\n{}".format(
                json.dumps(self.waiting_path_dict, indent=2)
            )
        )

        if floor_equal:
            # ============================================================
            # CALCULATE DOCKING GOAL
            # ============================================================
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
            if USE_DOCKING_BY_MIRROR:
                self.docking_path_dict["params"]["use_mirror"] = True
            self.docking_path_dict["waypoints"][0]["position"] = copy.deepcopy(
                obj_to_dict(inside_pose, self.return_pose_dict)
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

            # ============================================================
            # CALCULATE UNDOCKING GOAL
            # ============================================================
            self.undocking_goal = StringGoal()
            self.undocking_path_dict["params"] = {}
            self.undocking_path_dict["params"]["only_use_qr"] = True
            if USE_DOCKING_BY_MIRROR:
                self.undocking_path_dict["params"]["use_mirror"] = True
            self.undocking_path_dict["waypoints"][0]["position"] = copy.deepcopy(
                obj_to_dict(docking_pose, self.return_pose_dict)
            )

            self.undocking_path_dict["waypoints"][1]["position"] = copy.deepcopy(
                obj_to_dict(inside_pose, self.return_pose_dict)
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

            # ============================================================
            # CALCULATE TEMP GOAL (same as docking; mirror sẽ override)
            # ============================================================
            self.temp_goal = StringGoal()
            self.temp_path_dict["params"] = {}
            self.temp_path_dict["params"]["only_use_qr"] = True
            if USE_DOCKING_BY_MIRROR:
                self.temp_path_dict["params"]["use_mirror"] = True
            self.temp_path_dict["waypoints"][0]["position"] = copy.deepcopy(
                obj_to_dict(inside_pose, self.return_pose_dict)
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
            if USE_DOCKING_BY_MIRROR:
                self.rotation_path_dict["params"]["use_mirror"] = True
            self.rotation_path_dict["waypoints"][0]["position"] = copy.deepcopy(
                obj_to_dict(inside_pose, self.return_pose_dict)
            )
            self.rotation_goal.data = json.dumps(self.rotation_path_dict, indent=2)
            rospy.logwarn(
                "rotation_goal position:\n{}".format(
                    (json.dumps(self.rotation_path_dict, indent=2))
                )
            )
        else:
            # ============================================================
            # CALCULATE DOCKING GOAL
            # ============================================================
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
            if USE_DOCKING_BY_MIRROR:
                self.docking_path_dict["params"]["use_mirror"] = True
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

            # ============================================================
            # CALCULATE UNDOCKING GOAL
            # ============================================================
            self.undocking_goal = StringGoal()
            self.undocking_path_dict["params"] = {}
            self.undocking_path_dict["params"]["only_use_qr"] = True
            if USE_DOCKING_BY_MIRROR:
                self.undocking_path_dict["params"]["use_mirror"] = True
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

            # ============================================================
            # CALCULATE TEMP GOAL (same as docking; mirror sẽ override)
            # ============================================================
            self.temp_goal = StringGoal()
            self.temp_path_dict["params"] = {}
            self.temp_path_dict["params"]["only_use_qr"] = True
            if USE_DOCKING_BY_MIRROR:
                self.temp_path_dict["params"]["use_mirror"] = True
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
            if USE_DOCKING_BY_MIRROR:
                self.rotation_path_dict["params"]["use_mirror"] = True
            self.rotation_path_dict["waypoints"][0]["position"] = copy.deepcopy(
                obj_to_dict(waiting_pose, self.return_pose_dict)
            )
            self.rotation_goal.data = json.dumps(self.rotation_path_dict, indent=2)
            rospy.logwarn(
                "rotation_goal position:\n{}".format(
                    (json.dumps(self.rotation_path_dict, indent=2))
                )
            )

        r = rospy.Rate(15)
        success = False
        _prev_state = MainState.NONE
        feedback_msg = ""
        self._asm.reset_flag()
        self._asm.action_running = True # kiểm tra server có còn hoạt động không
        self.pose_map2robot = None
        is_preemted = False # Kiểm tra xem server có bị preemted không
        first_check_timeout = True # Check timeout first time with plc
        _state_when_network_timeout = MainState.NONE # Lưu state tại thời điểm timeout trước khi set state mất kết nối plc
        first_go_to_waiting = True  # Flag for first time going to waiting position
        disable_auto_get_center_tape = False  # Flag for auto center tape detection
        # Reconnect nếu bị disconnect từ action trước (disconnect() gọi cuối mỗi action)
        if not check_connect(self.wareshare):
            rospy.logwarn("PLC disconnected, reconnecting...")
            self.wareshare.connect()
        if check_connect(self.wareshare):
            rospy.sleep(1)
            rospy.logwarn("connect passbox success first time")
        else:
            rospy.logwarn("connect passbox false first time")
        while not rospy.is_shutdown():
            try:
                if not check_connect(self.wareshare):
                    if first_check_timeout:
                        if (
                            _state == MainState.GOING_TO_OUT_OF_HUB
                            or _state == MainState.DOCKING_TO_HUB
                        ):
                            self.moving_control_run_pause_pub.publish(
                                StringStamped(
                                    stamp=rospy.Time.now(), data="PAUSE"
                                )
                            )
                        # Chỉ lưu state nếu chưa ở trạng thái lỗi
                        if _state != MainState.NETWORK_ERROR:
                            _state_when_network_timeout = _state
                        first_check_timeout = False
                        rospy.sleep(0.5)
                    _state = MainState.NETWORK_ERROR

            except Exception as e:
                rospy.logerr(e)

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

            # Log state transitions
            if _state != _prev_state:
                rospy.logwarn("=" * 60)
                rospy.logwarn("STATE TRANSITION: {} -> {}".format(
                    _prev_state.toString() if _prev_state != MainState.NONE else "NONE",
                    _state.toString()
                ))
                rospy.logwarn("=" * 60)
                _prev_state = _state


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
            # ################# set safety job #################
            self.safety_job_name = ""
            msg = StringStamped()
            msg.stamp = rospy.Time.now()
            msg.data = self.safety_job_name
            self.safety_job_pub.publish(msg)
            # ################# end set safety job #################
            # """
            # .####.##....##.####.########
            # ..##..###...##..##.....##...
            # ..##..####..##..##.....##...
            # ..##..##.##.##..##.....##...
            # ..##..##..####..##.....##...
            # ..##..##...###..##.....##...
            # .####.##....##.####....##...
            # """
            # """
            # State: INIT
            if _state == MainState.INIT:
                self.enable_safety = False
                self.vel_move_base = rospy.get_param(
                    "/move_base/NeoLocalPlanner/max_vel_x",0.8
                )
                self.dynamic_reconfig_movebase(
                    vel_docking_hub, publish_safety=False, stop_center_qr=False
                )
                clear_all_output_bits(self.wareshare)
                if goal_type == PICK:
                    _state = MainState.LIFT_MIN_FIRST
                else:
                    _state = MainState.LIFT_MAX_FIRST

                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED
            # ============================================================
            # State: SEND_GOTO_TEMP_POSE  (chỉ dùng khi USE_DOCKING_BY_MIRROR)
            # ============================================================
            # .#####..#######.#.....#.######..........#######.#######.#.....#.######..
            # #.....#.#.......##....#.#.....#............#....#.......##...##.#.....#.
            # #.......#.......#.#...#.#.....#............#....#.......#.#.#.#.#.....#.
            # .#####..#####...#..#..#.#.....#............#....#####...#..#..#.######..
            # ......#.#.......#...#.#.#.....#............#....#.......#.....#.#.......
            # #.....#.#.......#....##.#.....#............#....#.......#.....#.#.......
            # .#####..#######.#.....#.######.............#....#######.#.....#.#.......
            # ................................#######.................................
            # ........######..#######..#####..#######.
            # ........#.....#.#.....#.#.....#.#.......
            # ........#.....#.#.....#.#.......#.......
            # ........######..#.....#..#####..#####...
            # ........#.......#.....#.......#.#.......
            # ........#.......#.....#.#.....#.#.......
            # ........#.......#######..#####..#######.
            # #######.................................
            elif _state == MainState.SEND_GOTO_TEMP_POSE:
                rospy.loginfo("Send goto temp pose")
                if floor_equal:
                    self.send_request_get_mirror(
                        self.calculate_pose_offset(
                            0.4,
                            hub_pose_x,
                            hub_pose_y,
                            atan2(
                                inside_pose_y - hub_pose_y,
                                inside_pose_x - hub_pose_x,
                            ),
                        ),
                        self.length_hub,
                        True,
                        True
                    )
                    rospy.sleep(1)
                    if not self.compute_goals_from_mirror({
                        "waiting_goal":        self.inside_goal,
                        "waiting_path_dict":   self.inside_path_dict,
                        "docking_goal":        self.docking_goal,
                        "docking_path_dict":   self.docking_path_dict,
                        "undocking_goal":      self.undocking_goal,
                        "undocking_path_dict": self.undocking_path_dict,
                        "temp_goal":           self.temp_goal,
                        "temp_path_dict":      self.temp_path_dict,
                        "rotation_goal":       self.rotation_goal,
                        "rotation_path_dict":  self.rotation_path_dict,
                    },self.initial_hub_to_waiting_distance):
                        # Tính góc từ hub đến waiting
                        target_angle = atan2(
                            inside_pose_y - hub_pose_y,
                            inside_pose_x - hub_pose_x
                        )

                        # Lấy góc hiện tại của robot
                        current_angle = self.robot_pose_angle

                        # Tính góc lệch
                        angle_diff = self.normalize_angle(target_angle - current_angle)
                        angle_diff_deg = abs(degrees(angle_diff))

                        rospy.logwarn(f"Target angle for detect mirror: {degrees(target_angle):.2f} deg")
                        rospy.logwarn(f"Current angle for detect mirror: {degrees(current_angle):.2f} deg")
                        rospy.logwarn(f"Angle difference: {angle_diff_deg:.2f} deg")
                        if angle_diff_deg > 30 and angle_diff_deg < 150:
                            rospy.logwarn("Angle detect mirror difference requires rotation, rotating robot...")
                            # Chuyển sang trạng thái xoay robot
                            _state = MainState.SEND_ROTATE_FIND_MIRROR
                        else:
                            _state_when_error = _state
                            _state = MainState.DETECT_MIRROR_ERROR
                            continue
                    else:
                        self.send_request_get_mirror(
                            self.calculate_pose_offset(
                                0.4,
                                hub_pose_x,
                                hub_pose_y,
                                atan2(
                                    inside_pose_y - hub_pose_y,
                                    inside_pose_x - hub_pose_x,
                                ),
                            ),
                            self.length_hub,
                            False,
                        )
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

                else:
                    self.send_request_get_mirror(
                        self.calculate_pose_offset(
                            0.4,
                            hub_pose_x,
                            hub_pose_y,
                            atan2(
                                waiting_pose_y - hub_pose_y,
                                waiting_pose_x - hub_pose_x,
                            ),
                        ),
                        self.length_hub,
                        True,
                        True
                    )
                    rospy.sleep(1)
                    if not self.compute_goals_from_mirror({
                        "waiting_goal":        self.waiting_goal,
                        "waiting_path_dict":   self.waiting_path_dict,
                        "docking_goal":        self.docking_goal,
                        "docking_path_dict":   self.docking_path_dict,
                        "undocking_goal":      self.undocking_goal,
                        "undocking_path_dict": self.undocking_path_dict,
                        "temp_goal":           self.temp_goal,
                        "temp_path_dict":      self.temp_path_dict,
                        "rotation_goal":       self.rotation_goal,
                        "rotation_path_dict":  self.rotation_path_dict,
                    },self.initial_hub_to_lift_distance):
                        # Tính góc từ hub đến waiting
                        target_angle = atan2(
                            waiting_pose_y - hub_pose_y,
                            waiting_pose_x - hub_pose_x
                        )

                        # Lấy góc hiện tại của robot
                        current_angle = self.robot_pose_angle

                        # Tính góc lệch
                        angle_diff = self.normalize_angle(target_angle - current_angle)
                        angle_diff_deg = abs(degrees(angle_diff))

                        rospy.logwarn(f"Target angle for detect mirror: {degrees(target_angle):.2f} deg")
                        rospy.logwarn(f"Current angle for detect mirror: {degrees(current_angle):.2f} deg")
                        rospy.logwarn(f"Angle difference: {angle_diff_deg:.2f} deg")
                        if angle_diff_deg > 30 and angle_diff_deg < 150:
                            rospy.logwarn("Angle detect mirror difference requires rotation, rotating robot...")
                            # Chuyển sang trạng thái xoay robot
                            _state = MainState.SEND_ROTATE_FIND_MIRROR
                        else:
                            _state_when_error = _state
                            _state = MainState.DETECT_MIRROR_ERROR
                            continue
                    else:
                        self.send_request_get_mirror(
                            self.calculate_pose_offset(
                                0.4,
                                hub_pose_x,
                                hub_pose_y,
                                atan2(
                                    waiting_pose_y - hub_pose_y,
                                    waiting_pose_x - hub_pose_x,
                                ),
                            ),
                            self.length_hub,
                            False,
                        )
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
            # .#####..#######.#.....#.######..........######..#######.#######....#....
            # #.....#.#.......##....#.#.....#.........#.....#.#.....#....#......#.#...
            # #.......#.......#.#...#.#.....#.........#.....#.#.....#....#.....#...#..
            # .#####..#####...#..#..#.#.....#.........######..#.....#....#....#.....#.
            # ......#.#.......#...#.#.#.....#.........#...#...#.....#....#....#######.
            # #.....#.#.......#....##.#.....#.........#....#..#.....#....#....#.....#.
            # .#####..#######.#.....#.######..........#.....#.#######....#....#.....#.
            # ................................#######.................................
            # #######.#######.........#######.###.#.....#.######..........#.....#.###.
            # ...#....#...............#........#..##....#.#.....#.........##...##..#..
            # ...#....#...............#........#..#.#...#.#.....#.........#.#.#.#..#..
            # ...#....#####...........#####....#..#..#..#.#.....#.........#..#..#..#..
            # ...#....#...............#........#..#...#.#.#.....#.........#.....#..#..
            # ...#....#...............#........#..#....##.#.....#.........#.....#..#..
            # ...#....#######.........#.......###.#.....#.######..........#.....#.###.
            # ................#######.............................#######.............
            # ######..######..#######.######..
            # #.....#.#.....#.#.....#.#.....#.
            # #.....#.#.....#.#.....#.#.....#.
            # ######..######..#.....#.######..
            # #...#...#...#...#.....#.#...#...
            # #....#..#....#..#.....#.#....#..
            # #.....#.#.....#.#######.#.....#.
            # ................................
            elif _state == MainState.SEND_ROTATE_FIND_MIRROR:
                if floor_equal:
                    self.moving_control_client.send_goal(
                        self.inside_goal,
                        feedback_cb=self.moving_control_fb,
                    )
                else:
                    self.moving_control_client.send_goal(
                        self.waiting_goal,
                    feedback_cb=self.moving_control_fb,
                )
                self.moving_control_result = -1
                self.last_moving_control_fb = rospy.get_time()
                _state = MainState.ROTATE_FIND_MIRROR
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # ######..#######.#######....#....#######.#######.........#######.###.#.....#.
            # #.....#.#.....#....#......#.#......#....#...............#........#..##....#.
            # #.....#.#.....#....#.....#...#.....#....#...............#........#..#.#...#.
            # ######..#.....#....#....#.....#....#....#####...........#####....#..#..#..#.
            # #...#...#.....#....#....#######....#....#...............#........#..#...#.#.
            # #....#..#.....#....#....#.....#....#....#...............#........#..#....##.
            # #.....#.#######....#....#.....#....#....#######.........#.......###.#.....#.
            # ................................................#######.....................
            # ######..........#.....#.###.######..######..#######.######..
            # #.....#.........##...##..#..#.....#.#.....#.#.....#.#.....#.
            # #.....#.........#.#.#.#..#..#.....#.#.....#.#.....#.#.....#.
            # #.....#.........#..#..#..#..######..######..#.....#.######..
            # #.....#.........#.....#..#..#...#...#...#...#.....#.#...#...
            # #.....#.........#.....#..#..#....#..#....#..#.....#.#....#..
            # ######..........#.....#.###.#.....#.#.....#.#######.#.....#.
            # ........#######.............................................

            elif _state == MainState.ROTATE_FIND_MIRROR:
                if self.moving_control_result == GoalStatus.SUCCEEDED:
                    _state = MainState.SEND_GOTO_TEMP_POSE
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
                    _state_bf_error = MainState.SEND_ROTATE_FIND_MIRROR
                    _state_when_error = _state
                    _state = MainState.MOVING_ERROR
                if rospy.get_time() - self.last_moving_control_fb >= 5.0:
                    rospy.logerr("/moving control disconnected!")
                    self.send_feedback(
                        self._as, GoalStatus.to_string(GoalStatus.ABORTED)
                    )
                    _state_bf_error = MainState.SEND_ROTATE_FIND_MIRROR
                    _state_when_error = _state
                    _state = MainState.MOVING_DISCONNECTED
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED
            # .#####..#######.###.#.....#..#####..........#######.#######.........#######.
            # #.....#.#.....#..#..##....#.#.....#............#....#.....#............#....
            # #.......#.....#..#..#.#...#.#..................#....#.....#............#....
            # #..####.#.....#..#..#..#..#.#..####............#....#.....#............#....
            # #.....#.#.....#..#..#...#.#.#.....#............#....#.....#............#....
            # #.....#.#.....#..#..#....##.#.....#............#....#.....#............#....
            # .#####..#######.###.#.....#..#####.............#....#######............#....
            # ....................................#######.................#######.........
            # #######.#.....#.######..........######..#######..#####..#######.
            # #.......##...##.#.....#.........#.....#.#.....#.#.....#.#.......
            # #.......#.#.#.#.#.....#.........#.....#.#.....#.#.......#.......
            # #####...#..#..#.######..........######..#.....#..#####..#####...
            # #.......#.....#.#...............#.......#.....#.......#.#.......
            # #.......#.....#.#...............#.......#.....#.#.....#.#.......
            # #######.#.....#.#...............#.......#######..#####..#######.
            # ........................#######.................................
            elif _state == MainState.GOING_TO_TEMP_POSE:
                rospy.loginfo("Go to temp pose")
                if self.moving_control_result == GoalStatus.SUCCEEDED:
                    _state = MainState.SEND_GOTO_WAITING
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

            #.....#.#######.#######.#.....#.#######.######..#....#.........#######.######..
            ##....#.#..........#....#..#..#.#.....#.#.....#.#...#..........#.......#.....#.
            #.#...#.#..........#....#..#..#.#.....#.#.....#.#..#...........#.......#.....#.
            #..#..#.#####......#....#..#..#.#.....#.######..###............#####...######..
            #...#.#.#..........#....#..#..#.#.....#.#...#...#..#...........#.......#...#...
            #....##.#..........#....#..#..#.#.....#.#....#..#...#..........#.......#....#..
            #.....#.#######....#.....##.##..#######.#.....#.#....#.........#######.#.....#.
            elif _state == MainState.NETWORK_ERROR:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/passbox_server: {}".format(
                    _state.toString()
                )

                if not check_connect(self.wareshare):
                    rospy.logerr_throttle(5.0, "Connect to passbox error. Attempting to reconnect...")
                    try:
                        self.wareshare.disconnect() # Đảm bảo đóng socket cũ trước khi tạo cái mới
                        self.wareshare.connect()
                    except Exception as e:
                        rospy.logerr_throttle(5.0, "Reconnection attempt failed: {}".format(e))

                rospy.sleep(1)

                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    # Nếu đã kết nối lại, cho phép quay về state trước đó
                    if check_connect(self.wareshare) and _state_when_network_timeout != MainState.NONE:
                        rospy.logwarn("Manual reset NETWORK_ERROR --> state: {}".format(
                            _state_when_network_timeout.toString()
                        ))
                        first_check_timeout = True
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
                if check_connect(self.wareshare) and self.mode_robot == "AUTO":
                    self._asm.reset_flag()
                    first_check_timeout = True
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

                # --------------------------------------------------------
                # Kiểm tra lỗi di chuyển
                # --------------------------------------------------------
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
                    _state = MainState.MOVING_ERROR
                    _state_bf_error = MainState.DOCKING_TO_HUB
                    _state_when_error = _state
            # ============================================================
            # State: WAIT_RESET_IO
            # ============================================================
            # elif _state == MainState.WAIT_RESET_IO:
            #     _state = MainState.INIT

            # """
            # ..######..########.##....##.########.........######...#######..########..#######.........##......##....###....####.########.####.##....##..######..
            # .##....##.##.......###...##.##.............##....##.##.....##....##....##.....##.........##..##..##...##.##....##.....##.....##..###...##.##....##.
            # .##.......##.......####..##.##.............##.......##.....##....##....##.....##.........##..##..##..##...##...##.....##.....##..####..##.##.......
            # ..######..######...##.##.##.##.....#######.##...####.##.....##....##....##.....##.........##..##..##.##.....##..##.....##.....##..##.##.##.##...####
            # .......##.##.......##..####.##.............##....##.##.....##....##....##.....##.........##..##..##.#########..##.....##.....##..##..####.##....##.
            # .##....##.##.......##...###.##.............##....##.##.....##....##....##.....##.........##..##..##.##.....##..##.....##.....##..##...###.##....##.
            # ..######..########.##....##.########........######...#######.....##.....#######..#######..###..###..##.....##.####....##....####.##....##..######..
            # """

            # State: SEND_GOTO_WAITING
            elif _state == MainState.SEND_GOTO_WAITING:
                rospy.loginfo("Send goto waiting")
                if floor_equal:
                    self.moving_control_client.send_goal(
                        self.inside_goal,
                        feedback_cb=self.moving_control_fb,
                    )
                else:
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
            # ""

            # State: SEND_GOTO_WAITING
            elif _state == MainState.GOING_TO_WAITING:
                rospy.logwarn("Go to waiting")
                rospy.logwarn(self.moving_control_error_code)
                if self.enable_safety and first_go_to_waiting:
                    self.safety_job_name = safety_job_rotation
                else:
                    self.safety_job_name = ""
                if self.moving_control_result == GoalStatus.SUCCEEDED:
                    _state = MainState.SEND_DOCKING_HUB
                # --------------------------------------------------------
                # Kiểm tra lỗi di chuyển
                # --------------------------------------------------------
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
                    _state = MainState.MOVING_ERROR
                    _state_bf_error = MainState.SEND_GOTO_WAITING
                    _state_when_error = _state
                # --------------------------------------------------------
                # Kiểm tra timeout (mất kết nối với moving_control)
                # --------------------------------------------------------
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
            #.....#.#######.#.....#.###.#.....#..#####..........#######.######..######..
            ##...##.#.....#.#.....#..#..##....#.#.....#.........#.......#.....#.#.....#.
            #.#.#.#.#.....#.#.....#..#..#.#...#.#...............#.......#.....#.#.....#.
            #..#..#.#.....#.#.....#..#..#..#..#.#..####.........#####...######..######..
            #.....#.#.....#..#...#...#..#...#.#.#.....#.........#.......#...#...#...#...
            #.....#.#.....#...#.#....#..#....##.#.....#.........#.......#....#..#....#..
            #.....#.#######....#....###.#.....#..#####..........#######.#.....#.#.....#.

            elif _state == MainState.MOVING_ERROR:
                self.moving_control_result = -1
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = (
                    "/passbox_server: {}".format(_state.toString())
                    + self.moving_control_error_code
                )
                self.cmd_vel_msg.angular.z = 0
                self.cmd_vel_msg.linear.x = 0
                self.pub_vel.publish(self.cmd_vel_msg)
                if self._asm.reset_error_request:
                    self.moving_control_client.cancel_all_goals()
                    rospy.sleep(0.1)
                    rospy.logwarn(
                        "Reset error --> state: {}".format(
                            _state_bf_error.toString()
                        )
                    )
                    self._asm.reset_flag()
                    if _state_when_error == MainState.GOING_TO_OUT_OF_HUB:
                        _state = MainState.SEND_GOTO_OUT_OF_HUB
                    else:
                        _state = MainState.SEND_GOTO_WAITING
                    self.moving_control_error_code = ""

            #.....#.#######.#.....#.###.#.....#..#####..........######..###..#####..
            ##...##.#.....#.#.....#..#..##....#.#.....#.........#.....#..#..#.....#.
            #.#.#.#.#.....#.#.....#..#..#.#...#.#...............#.....#..#..#.......
            #..#..#.#.....#.#.....#..#..#..#..#.#..####.........#.....#..#...#####..
            #.....#.#.....#..#...#...#..#...#.#.#.....#.........#.....#..#........#.
            #.....#.#.....#...#.#....#..#....##.#.....#.........#.....#..#..#.....#.
            #.....#.#######....#....###.#.....#..#####..........######..###..#####..

            elif _state == MainState.MOVING_DISCONNECTED:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/passbox_server: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = _state_bf_error
                    self.moving_control_error_code = ""
            # """
            # .##.......####.########.########.........##.....##.####.##....##.........########.####.########...######..########
            # .##........##..##..........##............###...###..##..###...##.........##........##..##.....##.##....##....##...
            # .##........##..##..........##............####.####..##..####..##.........##........##..##.....##.##..........##...
            # .##........##..######......##............##.###.##..##..##.##.##.........######....##..########...######.....##...
            # .##........##..##..........##............##.....##..##..##..####.........##........##..##...##.........##....##...
            # .##........##..##..........##............##.....##..##..##...###.........##........##..##....##..##....##....##...
            # .########.####.##..........##....#######.##.....##.####.##....##.#######.##.......####.##.....##..######.....##...
            # """
            # State: LIFT_MIN_FIRST
            elif _state == MainState.LIFT_MIN_FIRST:
                rospy.logwarn("lift min first state")
                if self.liftdown_finish:
                    _state = MainState.REQUEST_ENTER_PASSBOX
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
                    _state = MainState.PAUSE
            # """
            # .##.......####.########.########.........##.....##....###....##.....##.........########.####.########...######..########
            # .##........##..##..........##............###...###...##.##....##...##..........##........##..##.....##.##....##....##...
            # .##........##..##..........##............####.####..##...##....##.##...........##........##..##.....##.##..........##...
            # .##........##..######......##............##.###.##.##.....##....###............######....##..########...######.....##...
            # .##........##..##..........##............##.....##.#########...##.##...........##........##..##...##.........##....##...
            # .##........##..##..........##............##.....##.##.....##..##...##..........##........##..##....##..##....##....##...
            # .########.####.##..........##....#######.##.....##.##.....##.##.....##.#######.##.......####.##.....##..######.....##...
            # """

            # State: LIFT_MAX_FIRST

            elif _state == MainState.LIFT_MAX_FIRST:
                rospy.logwarn("lift max first state")
                if self.liftup_finish:
                    _state = MainState.REQUEST_ENTER_PASSBOX
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
            # .########..########..#######..##.....##.########..######..########.........########.##....##.########.########.########.........########.....###.....######...######..########...#######..##.....##
            # .##.....##.##.......##.....##.##.....##.##.......##....##....##............##.......###...##....##....##.......##.....##....##.....##...##.##...##....##.##....##.##.....##.##.....##.##.....##
            # .##.....##.##.......##.....##.##.....##.##.......##..........##............##.......####..##....##....##.......##.....##....##.....##..##...##..##.......##.......##.....##.##.....##.##.....##
            # .########..######...##.....##.##.....##.######....######.....##............######...##.##.##....##....######...########.....########..##.....##..######...######..########..##.....##.##.....##
            # .##...##...##.......##..##.##.##.....##.##.............##....##............##.......##..####....##....##.......##...##......##.....##.#########.......##.......##.##.....##.##.....##..##...##.
            # .##....##..##.......##....##..##.....##.##.......##....##....##............##.......##...###....##....##.......##....##.....##.....##.##.....##.##....##.##....##.##.....##.##.....##...##.##..
            # .##.....##.########..#####.##..#######..########..######.....##....#######.########.##....##....##....########.##.....##....########..##.....##..######...######..########...#######.....###...
            # """
            # State: REQUEST_ENTER_PASSBOX
            elif _state == MainState.REQUEST_ENTER_PASSBOX:
                rospy.logwarn("request enter passbox state")
                if floor_equal:
                    wareshare_modbus.write_output_bit(self.wareshare, OPEN_CLEAN_SIDE, 1)
                    if(wareshare_modbus.read_input_bit(self.wareshare,DONE_OPEN_CLEAN_SIDE)):
                        rospy.logerr("tien vao")
                        _state = MainState.SEND_GOTO_TEMP_POSE
                else:
                    wareshare_modbus.write_output_bit(self.wareshare, OPEN_DIRTY_SIDE, 1)
                    if(wareshare_modbus.read_input_bit(self.wareshare,DONE_OPEN_DIRTY_SIDE)):
                        _state = MainState.SEND_GOTO_TEMP_POSE


            # """
            # ..######..########.##....##.########.........########...#######...######..##....##.####.##....##..######..........##.....##.##.....##.########.
            # .##....##.##.......###...##.##.....##.........##.....##.##.....##.##....##.##...##...##..###...##.##....##.........##.....##.##.....##.##.....##
            # .##.......##.......####..##.##.....##.........##.....##.##.....##.##.......##..##....##..####..##.##...............##.....##.##.....##.##.....##
            # ..######..######...##.##.##.##.....##.........##.....##.##.....##.##.......#####.....##..##.##.##.##...####.........#########.##.....##.########.
            # .......##.##.......##..####.##.....##.........##.....##.##.....##.##.......##..##....##..##..####.##....##..........##.....##.##.....##.##.....##
            # .##....##.##.......##...###.##.....##.........##.....##.##.....##.##....##.##...##...##..##...###.##....##..........##.....##.##.....##.##.....##
            # ..######..########.##....##.########..#######.########...#######...######..##....##.####.##....##..######...#######.##.....##..#######..########.
            # """
            # State: SEND_DOCKING_HUB
            elif _state == MainState.SEND_DOCKING_HUB:
                rospy.logwarn("send docking hub state")
                if floor_equal:
                    if USE_DOCKING_BY_MIRROR:
                        self.send_request_get_mirror(
                            self.calculate_pose_offset(
                                0.4,
                                hub_pose_x,
                                hub_pose_y,
                                atan2(
                                    inside_pose_y - hub_pose_y,
                                    inside_pose_x - hub_pose_x,
                                ),
                            ),
                            self.length_hub,
                            True,
                        )
                        rospy.sleep(1)
                        if not self.compute_goals_from_mirror({
                            "waiting_goal":        self.inside_goal,
                            "waiting_path_dict":   self.inside_path_dict,
                            "docking_goal":        self.docking_goal,
                            "docking_path_dict":   self.docking_path_dict,
                            "undocking_goal":      self.undocking_goal,
                            "undocking_path_dict": self.undocking_path_dict,
                            "temp_goal":           self.temp_goal,
                            "temp_path_dict":      self.temp_path_dict,
                            "rotation_goal":       self.rotation_goal,
                            "rotation_path_dict":  self.rotation_path_dict,
                        }, self.initial_hub_to_waiting_distance):
                            _state_when_error = _state
                            _state = MainState.DETECT_MIRROR_ERROR
                            continue
                        else:
                            self.send_request_get_mirror(
                                self.calculate_pose_offset(
                                    0.4,
                                    hub_pose_x,
                                    hub_pose_y,
                                    atan2(
                                        inside_pose_y - hub_pose_y,
                                        inside_pose_x - hub_pose_x,
                                    ),
                                ),
                                self.length_hub,
                                False,
                            )
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
                else:
                    if USE_DOCKING_BY_MIRROR:
                        self.send_request_get_mirror(
                            self.calculate_pose_offset(
                                0.4,
                                hub_pose_x,
                                hub_pose_y,
                                atan2(
                                    waiting_pose_y - hub_pose_y,
                                    waiting_pose_x - hub_pose_x,
                                ),
                            ),
                            self.length_hub,
                            True,
                        )
                        rospy.sleep(1)
                        if not self.compute_goals_from_mirror({
                            "waiting_goal":        self.waiting_goal,
                            "waiting_path_dict":   self.waiting_path_dict,
                            "docking_goal":        self.docking_goal,
                            "docking_path_dict":   self.docking_path_dict,
                            "undocking_goal":      self.undocking_goal,
                            "undocking_path_dict": self.undocking_path_dict,
                            "temp_goal":           self.temp_goal,
                            "temp_path_dict":      self.temp_path_dict,
                            "rotation_goal":       self.rotation_goal,
                            "rotation_path_dict":  self.rotation_path_dict,
                        }, self.initial_hub_to_lift_distance):
                            _state_when_error = _state
                            _state = MainState.DETECT_MIRROR_ERROR
                            continue
                        else:
                            self.send_request_get_mirror(
                                self.calculate_pose_offset(
                                    0.4,
                                    hub_pose_x,
                                    hub_pose_y,
                                    atan2(
                                        waiting_pose_y - hub_pose_y,
                                        waiting_pose_x - hub_pose_x,
                                    ),
                                ),
                                self.length_hub,
                                False,
                            )
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
                rospy.loginfo("Docking to hub")
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
                if enable_check_error_when_docking and not self.disable_lift:
                    if (
                        distance_to_hub > dist_check_go_in
                        and distance_to_hub < dist_check_go_in + 0.4
                    ):
                        # Use the new property to control vrack detection
                        if (
                            not self.disable_detect_vrack  # Add this condition
                            and self.detect_vrack
                            and goal_type == PLACE
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
                rospy.logwarn("check cart state")
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
                rospy.logwarn("lift max state")
                if self.liftup_finish:
                    if self.server_config != None:
                        if self.upDateCart(
                            self.type, self.name, self.cell, "", ""
                        ) and self.upDateCart(
                            "AGV", self.data, 0, self.cart, self.lot
                        ):
                            rospy.sleep(1)
                            _state = MainState.SEND_GOTO_OUT_OF_HUB
                        else:
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
                rospy.logwarn("lift min state")
                if self.liftdown_finish:
                    if self.server_config != None:
                        if self.upDateCart(
                            self.type, self.name, self.cell, self.cart, self.lot
                        ) and self.upDateCart("AGV", self.data, 0, "", ""):
                            rospy.sleep(1)
                            _state = MainState.SEND_GOTO_OUT_OF_HUB
                        else:
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
             #####..#######.#.....#.######......#####..#######....#######.#.....#.#######.
            #.....#.#.......##....#.#.....#....#.....#.#.....#....#.....#.#.....#....#....
            #.......#.......#.#...#.#.....#....#.......#.....#....#.....#.#.....#....#....
             #####..#####...#..#..#.#.....#....#..####.#.....#....#.....#.#.....#....#....
                  #.#.......#...#.#.#.....#....#.....#.#.....#....#.....#.#.....#....#....
            #.....#.#.......#....##.#.....#....#.....#.#.....#....#.....#.#.....#....#....
             #####..#######.#.....#.######......#####..#######....#######..#####.....#....
            # State: SEND_GOTO_OUT_OF_HUB
            elif _state == MainState.SEND_GOTO_OUT_OF_HUB:
                rospy.logwarn("send goto out of hub state")
                self.moving_control_client.send_goal(self.undocking_goal, feedback_cb=self.moving_control_fb,)
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
                rospy.logwarn("going to out of hub state")
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
                            continue

                if self.moving_control_result == GoalStatus.SUCCEEDED:
                    clear_all_output_bits(self.wareshare)
                    if floor_equal:
                        wareshare_modbus.write_output_bit(self.wareshare, CLOSE_CLEAN_SIDE, 1)
                        rospy.loginfo("bat bit CLOSE_CLEAN_SIDE")
                        time.sleep(3)
                        wareshare_modbus.write_output_bit(self.wareshare, CLOSE_CLEAN_SIDE, 0)
                    else:
                        wareshare_modbus.write_output_bit(self.wareshare, CLOSE_DIRTY_SIDE, 1)
                        rospy.loginfo("bat bit CLOSE_DIRTY_SIDE")
                        time.sleep(3)
                        wareshare_modbus.write_output_bit(self.wareshare, CLOSE_DIRTY_SIDE, 0)
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
            # .########...#######..##....##.########
            # .##.....##.##.....##.###...##.##......
            # .##.....##.##.....##.####..##.##......
            # .##.....##.##.....##.##.##.##.######..
            # .##.....##.##.....##.##..####.##......
            # .##.....##.##.....##.##...###.##......
            # .########...#######..##....##.########
            # """
            # State: DONE
            elif _state == MainState.DONE:
                rospy.logwarn("done state")
                self.dynamic_reconfig_movebase(self.vel_move_base,  publish_safety=False, stop_center_qr=False)
                if goal_type == PLACE:
                    success = True
                    break
                elif goal_type == PICK:
                    success = True
                    break
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
                    rospy.logerr("RESUME state")
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="RUN")
                    )
                    _state = _state_when_pause
            # State: DETECT_MIRROR_ERROR
            elif _state == MainState.DETECT_MIRROR_ERROR:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/passbox_server: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = _state_when_error
        rospy.logwarn("Close connect to plc")
        wareshare_modbus.disconnect(self.wareshare)
        rospy.logwarn("Closed connect to plc when finish")
        self._asm.action_running = False
        # ################# hardcode tắt safety job #################
        self.safety_job_name = ""
        msg = StringStamped()
        msg.stamp = rospy.Time.now()
        msg.data = self.safety_job_name
        self.safety_job_pub.publish(msg)
        # ################# end hardcode tắt safety job #################
        if success:
            rospy.loginfo("%s: Succeeded" % self._action_name)
            self._as.set_succeeded(self._result)
        self.disable_qr_code_msg.stamp = rospy.Time.now()
        self.disable_qr_code_msg.data = 0
        self.disable_check_error_qr_code_pub.publish(self.disable_qr_code_msg)
        self.send_request_get_mirror(
            self.calculate_pose_offset(
                0.4,
                hub_pose_x,
                hub_pose_y,
                atan2(
                    waiting_pose_y - hub_pose_y,
                    waiting_pose_x - hub_pose_x,
                ),
            ),
            self.length_hub,
            False,
        )

    """
    ######## ##     ## ##    ##  ######  ######## ####  #######  ##    ##
    ##       ##     ## ###   ## ##    ##    ##     ##  ##     ## ###   ##
    ##       ##     ## ####  ## ##          ##     ##  ##     ## ####  ##
    ######   ##     ## ## ## ## ##          ##     ##  ##     ## ## ## ##
    ##       ##     ## ##  #### ##          ##     ##  ##     ## ##  ####
    ##       ##     ## ##   ### ##    ##    ##     ##  ##     ## ##   ###
    ##        #######  ##    ##  ######     ##    ####  #######  ##    ##
    """

    def dynamic_reconfig_movebase(self, vel_x, publish_safety, stop_center_qr):
        new_config = {
            "max_vel_x": vel_x,
            "max_vel_trans": vel_x,
            "publish_safety": publish_safety,
            "stop_center_qr": stop_center_qr
        }
        for i in range(3):
            self.client_reconfig_movebase.update_configuration(new_config)
            rospy.sleep(0.1)

    def load_config(self):
        try:
            # Server config
            if os.path.exists(self.server_config_file):
                with open(self.server_config_file) as file:
                    self.server_config = yaml.load(file, Loader=yaml.Loader)
                    rospy.loginfo("Server config file:")
                    rospy.loginfo(
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

    def upDateCart(self, type, name, cell, cart_no, lot_no, agv_name=""):
        data = {
            "function": "UPDATE_CART",
            "type": type,
            "name": name,
            "cell": cell,
            "cart": cart_no,
            "lot": lot_no,
            "agv": agv_name,
        }
        try:
            response = requests.post(
                self.api_url + "functions/agvapi",
                data=json.dumps(data),
                headers=self.header,
                timeout=2,
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
    ##     ## #### ########  ########   #######  ########
    ###   ###  ##  ##     ## ##     ## ##     ## ##     ##
    ####.####  ##  ##     ## ##     ## ##     ## ##     ##
    ## ### ##  ##  ########  ########  ##     ## ########
    ##     ##  ##  ##   ##   ##   ##   ##     ## ##   ##
    ##     ##  ##  ##    ##  ##    ##  ##     ## ##    ##
    ##     ## #### ##     ## ##     ##  #######  ##     ##
    """

    def normalize_angle(self, angle):
        """Đưa angle về khoảng [-pi, pi]"""
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def average_angles(self, angles):
        """Trung bình góc (radian) theo phương pháp vector."""
        sin_sum = np.sum(np.sin(angles))
        cos_sum = np.sum(np.cos(angles))
        return atan2(sin_sum, cos_sum)

    def rotate_reach_angle(self,angle):
        error_angle = angle
        if np.abs(error_angle) < 0.01:
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
            self.cmd_vel_pub.publish(self.vel)
            return True

        if error_angle > 0:
            self.vel.angular.z = np.clip(0.1 * error_angle, 0.02, 0.12)
        else:
            self.vel.angular.z = np.clip(0.1 * error_angle, -0.12, -0.02)
        self.vel.linear.x = 0.0
        self.cmd_vel_pub.publish(self.vel)
        return False

    def rotate_to_goal(self, angle):
        error_angle = angle

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

    def handle_lift_publish(self, event):
        if self.type_lift == LIFT_UP:
            if self.liftup_finish:
                rospy.loginfo("Lift up completed, stopping timer.")
                self.stop_lift_timer()
                return
            self.lift_msg.stamp = rospy.Time.now()
            self.lift_msg.data = LIFT_UP
            self.pub_lift_cmd.publish(self.lift_msg)
        else:
            if self.liftdown_finish:
                rospy.loginfo("Lift down completed, stopping timer.")
                self.stop_lift_timer()
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


    def get_path_angle(self, hub_pose, waiting_pose):
        """Tính góc từ waiting_pose đến hub_pose."""
        try:
            if isinstance(hub_pose, Pose):
                dx = hub_pose.position.x - waiting_pose.position.x
                dy = hub_pose.position.y - waiting_pose.position.y
            else:
                dx = hub_pose[0] - waiting_pose[0]
                dy = hub_pose[1] - waiting_pose[1]
            return atan2(dy, dx)
        except Exception as e:
            rospy.logerr(f"get_path_angle error: {e}")
            return 0.0

    def get_mirror(
        self,
        frame_global="odom",
        frame_local="center_hub",
        num_samples=10,
        delay=0.05,
    ):
        """Lấy transform odom→center_hub, trung bình nhiều mẫu."""
        x_list, y_list, yaw_list = [], [], []

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
                except Exception as e:
                    rospy.logwarn("Transform failed: %s", e)
                    return False
        else:
            for _ in range(num_samples):
                try:
                    self.tf_listener.waitForTransform(
                        frame_global, frame_local, rospy.Time(0), rospy.Duration(1.0)
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
                    tf.Exception, tf.ConnectivityException,
                    tf.LookupException, KeyboardInterrupt,
                ):
                    rospy.logwarn("TF exception during get_mirror averaging")
                    return False

        x_avg = np.mean(x_list)
        y_avg = np.mean(y_list)
        yaw_avg = self.average_angles(yaw_list)
        q_avg = quaternion_from_euler(0, 0, yaw_avg)
        self.trans_mirror = [x_avg, y_avg, 0.0]
        self.rot_mirror = [q_avg[0], q_avg[1], q_avg[2], q_avg[3]]
        return True

    def compute_positions_from_mirror(self, initial_distance):
        """Tính vị trí hub, waiting, temp từ vị trí gương.

        Args:
            initial_distance (float): Khoảng cách ban đầu từ điểm chuẩn đến điểm đích
                - floor_equal=True  : self.initial_hub_to_waiting_distance (hub → inside)
                - floor_equal=False : self.initial_lift_to_waiting_distance (lift → waiting)
        """
        if not self.get_mirror():
            return False
        x_m, y_m = self.trans_mirror[0], self.trans_mirror[1]
        yaw_m = euler_from_quaternion(self.rot_mirror)

        rospy.logwarn("position mirror: {}".format(self.trans_mirror))
        rospy.logwarn("orient mirror: {}".format(yaw_m))

        cos_yaw = cos(yaw_m[2])
        sin_yaw = sin(yaw_m[2])

        if self.choose_config_docking:
            hub_x_offset = self.mirror_offsets_lift["hub"]["x_offset"]
            hub_y_offset = self.mirror_offsets_lift["hub"]["y_offset"]
        else:
            hub_x_offset = self.mirror_offsets["hub"]["x_offset"]
            hub_y_offset = self.mirror_offsets["hub"]["y_offset"]
        # Hub position
        x_hub = x_m + hub_x_offset * cos_yaw - hub_y_offset * sin_yaw
        y_hub = y_m + hub_x_offset * sin_yaw + hub_y_offset * cos_yaw
        yaw_hub = yaw_m[2]

        # Waiting position
        if self.choose_config_docking:
            wait_x_offset = self.mirror_offsets_lift["waiting"]["x_offset"]
            wait_y_offset = self.mirror_offsets_lift["waiting"]["y_offset"]
        else:
            wait_x_offset = self.mirror_offsets["waiting"]["x_offset"]
            wait_y_offset = self.mirror_offsets["waiting"]["y_offset"]
        x_wait = x_m + wait_x_offset * cos_yaw - wait_y_offset * sin_yaw
        y_wait = y_m + wait_x_offset * sin_yaw + wait_y_offset * cos_yaw
        yaw_wait = yaw_m[2]

        # Temp position
        if self.choose_config_docking:
            temp_x_offset = self.mirror_offsets_lift["temp"]["x_offset"]
            temp_y_offset = self.mirror_offsets_lift["temp"]["y_offset"]
        else:
            temp_x_offset = self.mirror_offsets["temp"]["x_offset"]
            temp_y_offset = self.mirror_offsets["temp"]["y_offset"]
        x_temp = x_m + temp_x_offset * cos_yaw - temp_y_offset * sin_yaw
        y_temp = y_m + temp_x_offset * sin_yaw + temp_y_offset * cos_yaw
        yaw_temp = yaw_m[2]

        # Undocking position (dùng initial_distance do caller truyền vào)
        undocking_y_offset = wait_y_offset
        undocking_x_offset = wait_x_offset + (
            initial_distance - wait_x_offset + hub_x_offset
        )
        rospy.loginfo(f"compute_positions_from_mirror: initial_distance={initial_distance:.3f}, undocking_x_offset={undocking_x_offset:.3f}")
        x_undocking = x_m + undocking_x_offset * cos_yaw - undocking_y_offset * sin_yaw
        y_undocking = y_m + undocking_x_offset * sin_yaw + undocking_y_offset * cos_yaw
        yaw_undocking = yaw_m[2]

        rospy.loginfo(f"Undocking offsets: x={undocking_x_offset:.3f}, y={undocking_y_offset:.3f}")
        rospy.loginfo(f"Undocking position: x={x_undocking:.3f}, y={y_undocking:.3f}")

        return {
            "hub":      (x_hub,      y_hub,      yaw_hub),
            "waiting":  (x_wait,     y_wait,     yaw_wait),
            "temp":     (x_temp,     y_temp,     yaw_temp),
            "undocking":(x_undocking, y_undocking, yaw_undocking),
        }

    def compute_goals_from_mirror(self, goal_config, initial_distance):
        """Tính và cập nhật các goal (docking/undocking/waiting/temp) từ gương.

        Args:
            goal_config (dict): Chứa các StringGoal và path_dict cần cập nhật:
                Required keys:
                    "waiting_goal"        : StringGoal cho waiting
                    "waiting_path_dict"   : dict path cho waiting
                    "docking_goal"        : StringGoal cho docking
                    "docking_path_dict"   : dict path cho docking
                    "undocking_goal"      : StringGoal cho undocking
                    "undocking_path_dict" : dict path cho undocking
                    "temp_goal"           : StringGoal cho temp
                    "temp_path_dict"      : dict path cho temp
                    "rotation_goal"       : StringGoal cho rotation
                    "rotation_path_dict"  : dict path cho rotation
        """
        positions = self.compute_positions_from_mirror(initial_distance)
        if not positions:
            rospy.logwarn("Failed to compute positions from mirror")
            return False

        hub_x,       hub_y,       hub_yaw       = positions["hub"]
        wait_x,      wait_y,      wait_yaw      = positions["waiting"]
        temp_x,      temp_y,      temp_yaw      = positions["temp"]
        undocking_x, undocking_y, undocking_yaw = positions["undocking"]

        # Xử lý hướng FORWARD nếu cần
        if self.direction == FORWARD:
            hub_yaw       += pi
            wait_yaw      += pi
            temp_yaw      += pi
            undocking_yaw += pi
            hub_yaw       = self.normalize_angle(hub_yaw)
            temp_yaw      = self.normalize_angle(temp_yaw)
            undocking_yaw = self.normalize_angle(undocking_yaw)

        hub_pose       = self.calculate_pose_offset(0, hub_x,       hub_y,       hub_yaw)
        waiting_pose   = self.calculate_pose_offset(0, wait_x,      wait_y,      wait_yaw)
        temp_pose      = self.calculate_pose_offset(0, temp_x,      temp_y,      temp_yaw)
        undocking_pose = self.calculate_pose_offset(0, undocking_x, undocking_y, undocking_yaw)

        rpd = self.return_pose_dict


        # Waiting goal (1 waypoint)
        goal_config["waiting_path_dict"]["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(waiting_pose, rpd)
        )
        goal_config["waiting_goal"].data = json.dumps(goal_config["waiting_path_dict"], indent=2)
        rospy.logwarn("Mirror waiting goal:\n{}".format(goal_config["waiting_goal"].data))

        # Docking goal: waypoint0=waiting, waypoint1=hub
        goal_config["docking_path_dict"]["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(waiting_pose, rpd)
        )
        goal_config["docking_path_dict"]["waypoints"][1]["position"] = copy.deepcopy(
            obj_to_dict(hub_pose, rpd)
        )
        goal_config["docking_goal"].data = json.dumps(goal_config["docking_path_dict"], indent=2)
        rospy.logwarn("Mirror docking goal:\n{}".format(goal_config["docking_goal"].data))

        # Undocking goal: waypoint0=hub, waypoint1=undocking
        goal_config["undocking_path_dict"]["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(hub_pose, rpd)
        )
        goal_config["undocking_path_dict"]["waypoints"][1]["position"] = copy.deepcopy(
            obj_to_dict(undocking_pose, rpd)
        )
        goal_config["undocking_goal"].data = json.dumps(goal_config["undocking_path_dict"], indent=2)
        rospy.logwarn("Mirror undocking goal:\n{}".format(goal_config["undocking_goal"].data))

        # Temp goal: waypoint0=odom_pose, waypoint1=temp
        odom_pose = self.wait_until_pose_available()
        goal_config["temp_path_dict"]["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(odom_pose, rpd)
        )
        goal_config["temp_path_dict"]["waypoints"][1]["position"] = copy.deepcopy(
            obj_to_dict(temp_pose, rpd)
        )
        goal_config["temp_goal"].data = json.dumps(goal_config["temp_path_dict"], indent=2)
        rospy.logwarn("Mirror temp goal:\n{}".format(goal_config["temp_goal"].data))

        # Rotation goal (1 waypoint)
        goal_config["rotation_path_dict"]["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(temp_pose, rpd)
        )
        goal_config["rotation_goal"].data = json.dumps(goal_config["rotation_path_dict"], indent=2)
        rospy.logwarn("Mirror rotation goal:\n{}".format(goal_config["rotation_goal"].data))

        # Path angle
        self.path_angle = self.get_path_angle(hub_pose, waiting_pose)
        return True


    def send_request_get_mirror(self, pose_target,length ,enable_detect , use_scan_merge=False):
        """Gọi hub_service để bật/tắt detect gương."""
        req = HubServiceRequest()
        req.pose_target = pose_target
        req.enable_detect = enable_detect
        req.length = length
        req.use_scan_merge = use_scan_merge
        try:
            res = self.call_hub_service(req)
            rospy.loginfo(f"hub_service call success: {res.success}, use_scan_merge: {use_scan_merge}")
            return res.success
        except rospy.ServiceException as e:
            rospy.logerr(f"hub_service call failed: {e}")
            return False

    def wait_until_pose_available(self):
        """Chờ đến khi lấy được pose hiện tại trong frame odom."""
        while not rospy.is_shutdown():
            pose = self.get_current_pose_in_odom()
            if pose is not None:
                return pose
            rospy.sleep(0.1)

    def get_current_pose_in_odom(self):
        """Lấy vị trí hiện tại của robot trong frame odom."""
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
        except Exception as e:
            rospy.logwarn("get_current_pose_in_odom failed: %s", e)
            return None

    def transform_pose_map_to_odom(self, pose_in_map):
        """Chuyển đổi Pose từ frame map sang frame odom."""
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time(0)
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = pose_in_map
        try:
            transformed = self.tf_buffer.transform(
                pose_stamped, "odom", rospy.Duration(1.0)
            )
            return transformed.pose
        except Exception as e:
            rospy.logwarn("transform_pose_map_to_odom failed: %s", e)
            return None

    def convert_qr_to_lidar_coordinate(self, qr_x, qr_y):
        """Chuyển tọa độ QR sang tọa độ lidar (dùng khi USE_DOCKING_BY_MIRROR)."""
        if not self.has_lidar_info:
            return None, None
        cos_angle = math.cos(math.radians(self.lidar_angle))
        sin_angle = math.sin(math.radians(self.lidar_angle))
        lidar_x = (qr_x * self.lidar_scale * cos_angle) - (qr_y * self.lidar_scale * sin_angle) + self.lidar_offset_x
        lidar_y = (qr_x * self.lidar_scale * sin_angle) + (qr_y * self.lidar_scale * cos_angle) + self.lidar_offset_y
        rospy.logdebug(f"Convert QR({qr_x}, {qr_y}) -> Lidar({lidar_x}, {lidar_y})")
        return lidar_x, lidar_y

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
        default=os.path.join(rospkg.RosPack().get_path("amr_config"), "cfg"),
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
    rospy.init_node("passbox_equal_server", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    PassboxEqualAction(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
