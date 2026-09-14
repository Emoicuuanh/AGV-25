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
import math
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
from agv_msgs.msg import DataMatrixStamped
from nav_msgs.msg import Odometry
import json
from std_msgs.msg import Bool, Int16, Int8, String, Float32
from math import sqrt, pow, pi, sin, cos, atan2, degrees, atan
from docking.srv import DockService, DockServiceResponse, DockServiceRequest
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

import numpy as np
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

HOME = expanduser("~")


class MainState(EnumString):
    NONE = -1
    SEND_DOCKING_HUB = 0
    DOCKING_TO_HUB = 1
    DONE = 8
    MOVING_ERROR = 10
    PAUSED = 12
    WAITING = 13
    SEND_GOTO_WAITING = 14
    GOING_TO_WAITING = 15
    SEND_GOTO_TEMP_POSE = 16
    GOING_TO_TEMP_POSE = 17
    MOVING_DISCONNECTED = 28
    INIT = 29
    LECH_TAM = 34
    ALIGNMENT_SENSOR = 35
    COLLISION_POSSIBLE = 50
    ROTATE_TO_GOAL_ANGLE = 51
    DETECT_MIRROR_ERROR = 60


class RunType(Enum):
    NONE = -1
    GO_NOMAL = 0
    GO_DOCKING = 1
    GO_OUT_DOCKING = 2
    STOP_ACCURACY = 3
    STOP_BY_CROSS_LINE = 4


ON = 1
OFF = 0
USE_DOCKING_BY_MIRROR = True


class DockingCharger(object):
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
        self.moving_control_client = actionlib.SimpleActionClient(
            "/moving_control", StringAction
        )
        self.moving_control_client.wait_for_server()
        rospy.on_shutdown(self.shutdown)
        # Publisher
        self.pub_enable_detect_mirror = rospy.Publisher(
            "/docking/perform_detection", Bool, queue_size=10
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
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)

        # Subscriber
        rospy.Subscriber("/lidar_info", StringStamped, self.lidar_info_cb)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
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

        # dynamic reconfig client
        self.client_reconfig_movebase = dynamic_reconfigure.client.Client(
            "/move_base/NeoLocalPlanner",
            timeout=300,
            config_callback=self.dynamic_callback,
        )
        try:
            rospy.wait_for_service("hub_service", 30)
            try:
                self.call_hub_service = rospy.ServiceProxy(
                    "dock_service", DockService
                )
                rospy.loginfo("hub_service: is running")
            except rospy.ServiceException as e:
                rospy.logerr(f"Fail to create hub_service proxy: {e}")
        except rospy.ROSException as e:
            rospy.logerr(f"hub_service service is not available: {e}")

        # ModuleServer
        self._asm = ModuleServer(name)
        # Loop
        self.loop()

    def init_variable(self, *args, **kwargs):
        self.config_path = kwargs["config_path"]
        self.robot_config_file = kwargs["robot_config_file"]
        self.server_config_file = kwargs["robot_define"]
        self.use_tf2 = False
        self.tf_listener = tf.TransformListener(cache_time=rospy.Duration(0.1))
        #
        self.last_moving_control_fb = rospy.get_time()
        self.moving_control_result = -1

        self.label_qr_x = 0.0
        self.label_qr_y = 0.0
        #
        self.moving_control_error_code = ""
        # Database
        db_address = rospy.get_param("/mongodb_address")
        print_debug(db_address)
        self.db = mongodb(db_address)

        # self.vel_move_base = rospy.get_param("/move_base/NeoLocalPlanner/max_vel_x")
        self.vel_move_base = 0.8
        self.enable_safety = True
        self.last_time_read_qr = rospy.get_time()
        self.robot_pose_angle = None
        self.robot_odom_angle = None
        self.path_angle = None
        self.qr_angle = None
        self.vel = Twist()

        # tf
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(0.1))
        self.tf2_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Lidar info for coordinate conversion
        self.lidar_info = None
        self.lidar_scale = None
        self.lidar_offset_x = None
        self.lidar_offset_y = None
        self.lidar_angle = None
        self.has_lidar_info = False

    def shutdown(self):
        # self.auto_docking_client.cancel_all_goals()
        self.moving_control_client.cancel_all_goals()
        self.dynamic_reconfig_movebase(
            self.vel_move_base, publish_safety=True, stop_center_qr=True
        )

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

    def lidar_info_cb(self, msg):
        """Callback function for /lidar_info topic"""
        try:
            lidar_data = json.loads(msg.data)
            # rospy.loginfo(f"Received lidar info: {lidar_data}")
            
            # Kiểm tra có đủ thông tin cần thiết không
            if ("scale" in lidar_data and 
                "offset" in lidar_data and 
                "x" in lidar_data["offset"] and 
                "y" in lidar_data["offset"] and
                "angle" in lidar_data):
                
                self.lidar_scale = float(lidar_data["scale"])
                self.lidar_offset_x = float(lidar_data["offset"]["x"])
                self.lidar_offset_y = float(lidar_data["offset"]["y"])
                self.lidar_angle = float(lidar_data["angle"])  # Giá trị angle theo radian
                self.has_lidar_info = True
                self.lidar_info = lidar_data
                
                # rospy.loginfo(f"Updated lidar conversion params: scale={self.lidar_scale}, "
                #             f"offset=({self.lidar_offset_x}, {self.lidar_offset_y}), "
                #             f"angle={self.lidar_angle}")
            else:
                rospy.logwarn("Incomplete lidar info received, missing required fields")
                self.has_lidar_info = False
                
        except (json.JSONDecodeError, ValueError, KeyError) as e:
            rospy.logerr(f"Error parsing lidar info: {e}")
            self.has_lidar_info = False

    def data_qr_cb(self, msg):
        self.last_time_read_qr = rospy.get_time()
        self.label_qr_x = msg.lable.x / 1000
        self.label_qr_y = msg.lable.y / 1000

    def dynamic_callback(config, level):
        # rospy.loginfo("Suceeed change vel of robot")
        pass

    def odom_cb(self, msg):
        # Lấy góc yaw từ quaternion trong odom
        r_x = msg.pose.pose.orientation.x
        r_y = msg.pose.pose.orientation.y
        r_z = msg.pose.pose.orientation.z
        r_w = msg.pose.pose.orientation.w
        roll, pitch, yaw = euler_from_quaternion((r_x, r_y, r_z, r_w))
        self.robot_odom_angle = yaw

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
        hub_type = "docking_charger"
        try:
            # Hub config
            hub_cfg_file = os.path.join(self.config_path, hub_type + ".json")
            with open(hub_cfg_file) as j:
                hub_dict = json.load(j)
                # dist_check_go_in = hub_dict["dist_check_go_in"]
                # max_error_sensor_in_hub = hub_dict["error_sensor_in_hub"]
                # max_error_angle_sensor_in_hub = hub_dict[
                #     "error_angle_sensor_in_hub"
                # ]
                # max_error_sensor_out_of_hub = hub_dict[
                #     "error_sensor_out_of_hub"
                # ]
                # max_error_angle_sensor_out_of_hub = hub_dict[
                #     "error_angle_sensor_out_of_hub"
                # ]
                safety_job_docking = hub_dict["safety_job_docking"]
                if "safety_job_rotation" in hub_dict:
                    safety_job_rotation = hub_dict["safety_job_rotation"]
                else:
                    safety_job_rotation = "ROTATION"
                # footprint_dock = hub_dict["footprint_dock"]
                # enable_check_error_when_docking = hub_dict[
                #     "enable_check_error_when_docking"
                # ]
                distance_turn_off_safety_when_docking = hub_dict[
                    "distance_turn_off_safety_when_docking"
                ]

                vel_docking_charger = hub_dict["max_vel_docking"]

                # Load mirror offsets (with defaults for backward compatibility)
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
        except Exception as e:
            rospy.logerr("Read config file error: {}".format(e))
            self._as.set_aborted("Read config file error")
            return

        # Load data from action
        data_dict = json.loads(goal.data)
        rospy.logwarn(data_dict)
        if rospy.get_time() - self.last_time_read_qr < 0.2:
            start_in_qr = True
        else:
            start_in_qr = False
        self.enable_safety = True
        try:
            hub_pose_x = data_dict["params"]["position"]["x"]
            hub_pose_y = data_dict["params"]["position"]["y"]
            waiting_pose_x = data_dict["params"]["waiting_position"]["x"]
            waiting_pose_y = data_dict["params"]["waiting_position"]["y"]
        except:
            hub_pose_x = data_dict["params"]["position"]["position"]["x"]
            hub_pose_y = data_dict["params"]["position"]["position"]["y"]
            if start_in_qr:
                waiting_pose_x = self.label_qr_x
                waiting_pose_y = self.label_qr_y
            else:
                waiting_pose_x = self.trans[0]
                waiting_pose_y = self.trans[1]
        if "properties" in data_dict["params"]:
            # Add property to control USE_DOCKING_BY_MIRROR
            if "use_mirror" in data_dict["params"]["properties"]:
                USE_DOCKING_BY_MIRROR = data_dict["params"]["properties"]["use_mirror"]
                rospy.loginfo(f"USE_DOCKING_BY_MIRROR from properties: {USE_DOCKING_BY_MIRROR}")
            else:
                # Giữ giá trị default nếu không có trong properties
                rospy.loginfo(f"USE_DOCKING_BY_MIRROR default: {USE_DOCKING_BY_MIRROR}")

            if "Safety" in data_dict["params"]["properties"]:
                if data_dict["params"]["properties"]["Safety"] == "Disable":
                    self.enable_safety = False

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
                    
            # Log final offset values after applying properties
            if hasattr(self, 'mirror_offsets'):
                rospy.logwarn(f"Final mirror offsets after properties: {self.mirror_offsets}")

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
            
            # Convert waiting pose
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

        waiting_pose = [waiting_pose_x, waiting_pose_y]
        hub_pose = [hub_pose_x, hub_pose_y]
        self.path_angle = self.get_path_angle(hub_pose, waiting_pose)

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
            waiting_pose = self.transform_pose_map_to_odom(waiting_pose)
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

        # Calculate undocking goal
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
        self.cmd_vel_msg = Twist()
        distance_to_hub = 0
        first_go_to_waiting = True
        last_time_pub_safety_job = rospy.get_time()
        self.pre_safety_job_name = None

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
                self.moving_control_client.cancel_all_goals()
                self.dynamic_reconfig_movebase(
                    self.vel_move_base, publish_safety=True, stop_center_qr=True
                )

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
            # RS485_Serial State:
            if _state == MainState.INIT:
                if USE_DOCKING_BY_MIRROR:
                    _state = MainState.SEND_GOTO_TEMP_POSE
                else:
                    _state = MainState.SEND_GOTO_WAITING
                self.vel_move_base = rospy.get_param(
                    "/move_base/NeoLocalPlanner/max_vel_x", 0.8
                )
                print("self.vel_move_base: ", self.vel_move_base)
                self.dynamic_reconfig_movebase(
                    vel_docking_charger,
                    publish_safety=False,
                    stop_center_qr=False,
                )

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
                    self.waiting_goal,
                    feedback_cb=self.moving_control_fb,
                )
                self.moving_control_result = -1
                self.last_moving_control_fb = rospy.get_time()
                _state = MainState.GOING_TO_WAITING
                if self._asm.pause_req:
                    # self.moving_control_client.cancel_all_goals()
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
                    _state = MainState.SEND_DOCKING_HUB

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
                    # self.moving_control_client.cancel_all_goals()
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
                # self.publish_detection(True)
                self.send_request_get_mirror(
                    self.calculate_pose_offset(
                        -0.4,
                        hub_pose_x,
                        hub_pose_y,
                        atan2(
                            waiting_pose_y - hub_pose_y,
                            waiting_pose_x - hub_pose_x,
                        ),
                    ),
                    True,
                    True
                )
                rospy.sleep(1)
                if not self.compute_goals_from_mirror():
                    _state_when_error = _state
                    _state = MainState.DETECT_MIRROR_ERROR
                    continue
                else:
                    # self.publish_detection(False)
                    self.send_request_get_mirror(
                        self.calculate_pose_offset(
                            -0.4,
                            hub_pose_x,
                            hub_pose_y,
                            atan2(
                                waiting_pose_y - hub_pose_y,
                                waiting_pose_x - hub_pose_x,
                            ),
                        ),
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

            # State: GOING_TO_TEMP_POSE
            elif _state == MainState.GOING_TO_TEMP_POSE:
                if self.enable_safety and first_go_to_waiting:
                    self.safety_job_name = safety_job_rotation
                else:
                    self.safety_job_name = ""
                if self.moving_control_result == GoalStatus.SUCCEEDED:
                    _state = MainState.SEND_GOTO_WAITING
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
                    _state_bf_error = MainState.SEND_GOTO_TEMP_POSE
                    _state_when_error = _state
                    _state = MainState.MOVING_ERROR
                    continue
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
                if USE_DOCKING_BY_MIRROR:
                    # self.publish_detection(True)
                    self.send_request_get_mirror(
                        self.calculate_pose_offset(
                            -0.4,
                            hub_pose_x,
                            hub_pose_y,
                            atan2(
                                waiting_pose_y - hub_pose_y,
                                waiting_pose_x - hub_pose_x,
                            ),
                        ),
                        True,
                    )
                    rospy.sleep(1)
                    if not self.compute_goals_from_mirror():
                        _state_when_error = _state
                        _state = MainState.DETECT_MIRROR_ERROR
                        continue
                    else:
                        # self.publish_detection(False)
                        self.send_request_get_mirror(
                            self.calculate_pose_offset(
                                -0.4,
                                hub_pose_x,
                                hub_pose_y,
                                atan2(
                                    waiting_pose_y - hub_pose_y,
                                    waiting_pose_x - hub_pose_x,
                                ),
                            ),
                            False,
                        )
                first_go_to_waiting = False
                self.dynamic_reconfig_movebase(
                    vel_docking_charger,
                    publish_safety=False,
                    stop_center_qr=False,
                )
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
                    # self.moving_control_client.cancel_all_goals()
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
                if self.enable_safety:
                    self.safety_job_name = safety_job_docking
                else:
                    self.safety_job_name = ""
                if distance_to_hub < distance_turn_off_safety_when_docking:
                    self.safety_job_name = ""
                if self.moving_control_result == GoalStatus.SUCCEEDED:
                    _state = MainState.ROTATE_TO_GOAL_ANGLE
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
                    # self.moving_control_client.cancel_all_goals()
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
                    # self.moving_control_client.cancel_all_goals()
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED
                    continue

            # State: MOVING_ERROR

            elif _state == MainState.DETECT_MIRROR_ERROR:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/docking_charger: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = _state_when_error

            elif _state == MainState.MOVING_ERROR:
                self.moving_control_result = -1
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = (
                    "/docking_charger: {}".format(_state.toString())
                    + self.moving_control_error_code
                )
                # self.cmd_vel_msg.angular.z = 0
                # self.cmd_vel_msg.linear.x = 0
                # self.pub_vel.publish(self.cmd_vel_msg)
                if self._asm.reset_error_request:
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
                    _state = MainState.SEND_GOTO_WAITING
                    self.moving_control_error_code = ""
            # State: COLLISION_POSSIBLE
            elif _state == MainState.COLLISION_POSSIBLE:
                self.moving_control_result = -1
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = (
                    "/docking_charger: {}".format(_state.toString())
                    + self.moving_control_error_code
                )
                # self.cmd_vel_msg.angular.z = 0
                # self.cmd_vel_msg.linear.x = 0
                # self.pub_vel.publish(self.cmd_vel_msg)
                if self._asm.reset_error_request:
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
                    _state = MainState.SEND_GOTO_WAITING
                    self.moving_control_error_code = ""

            # State: MOVING_DISCONNECTED
            elif _state == MainState.MOVING_DISCONNECTED:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/docking_charger: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = _state_bf_error
                    self.moving_control_error_code = ""

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

            # State: ROTATE_TO_GOAL_ANGLE
            elif _state == MainState.ROTATE_TO_GOAL_ANGLE:
                rospy.logerr(f"angle path: {self.path_angle}")
                if USE_DOCKING_BY_MIRROR:
                    rospy.logerr(f"angle odom: {self.robot_odom_angle}")
                    angle_err = (
                        self.path_angle - self.robot_odom_angle
                    )
                else:
                    rospy.logerr(f"angle robot: {self.robot_pose_angle}")
                    angle_err = (
                        self.path_angle - self.robot_pose_angle
                    )  # Robot is go in matehan/hub backward
                rospy.logerr(f"angle in hub error 1: {angle_err}")

                # Sử dụng normalize_angle thay vì atan(sin/cos)
                angle_err = self.normalize_angle(angle_err)
                rospy.logerr(f"angle in hub error 2: {angle_err}")

                succeed = self.rotate_to_goal(angle_err)
                if succeed:
                    _state = MainState.DONE
                else:
                    rospy.logerr(f"rotating to goal - result:{succeed}")

                if self._asm.pause_req:
                    # self.moving_control_client.cancel_all_goals()
                    self._asm.reset_flag()
                    _state_when_pause = _state
                    _state = MainState.PAUSED
                    continue

            # State: DONE
            elif _state == MainState.DONE:
                rospy.sleep(1)
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    _state_when_pause = _state
                    _state = MainState.PAUSED
                    continue
                success = True
                self.dynamic_reconfig_movebase(
                    self.vel_move_base, publish_safety=True, stop_center_qr=True
                )
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

    """
    ######## ##     ## ##    ##  ######  ######## ####  #######  ##    ##
    ##       ##     ## ###   ## ##    ##    ##     ##  ##     ## ###   ##
    ##       ##     ## ####  ## ##          ##     ##  ##     ## ####  ##
    ######   ##     ## ## ## ## ##          ##     ##  ##     ## ## ## ##
    ##       ##     ## ##  #### ##          ##     ##  ##     ## ##  ####
    ##       ##     ## ##   ### ##    ##    ##     ##  ##     ## ##   ###
    ##        #######  ##    ##  ######     ##    ####  #######  ##    ##
    """

    def convert_qr_to_lidar_coordinate(self, qr_x, qr_y):
        """
        Convert QR coordinate to lidar coordinate
        Args:
            qr_x: Original x coordinate from QR
            qr_y: Original y coordinate from QR
        Returns:
            (lidar_x, lidar_y): Converted coordinates or None if no lidar info
        """
        if not self.has_lidar_info:
            return None, None
                       
        # Công thức convert
        cos_angle = math.cos(math.radians(self.lidar_angle))
        sin_angle = math.sin(math.radians(self.lidar_angle))
        
        lidar_x = (qr_x * self.lidar_scale * cos_angle) - (qr_y * self.lidar_scale * sin_angle) + self.lidar_offset_x
        lidar_y = (qr_x * self.lidar_scale * sin_angle) + (qr_y * self.lidar_scale * cos_angle) + self.lidar_offset_y
        
        rospy.logdebug(f"Convert QR({qr_x}, {qr_y}) -> Lidar({lidar_x}, {lidar_y})")
        
        return lidar_x, lidar_y

    def send_request_get_mirror(self, pose_target, enable_detect, use_scan_merge=False):
        req = DockServiceRequest()
        req.pose_target = pose_target
        req.enable_detect = enable_detect
        req.use_scan_merge = use_scan_merge  # Thêm option use_scan_merge
        try:
            res = self.call_hub_service(req)
            rospy.loginfo(f"Service call success: {res.success}, use_scan_merge: {use_scan_merge}")
            return res.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def publish_detection(self, value):
        msg = Bool()
        msg.data = value
        rospy.loginfo(f"Publishing perform_detection = {msg.data}")
        self.pub_enable_detect_mirror.publish(msg)

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
        frame_local="dock",
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
        # yaw_avg -= 5 * pi / 180

        # Đặt offset lệch gương trên hub
        forward_offset = 0  # điều chỉnh theo trục lệch thực tế của bạn
        lateral_offset = 0

        # Offset forward/backward
        x_corrected = x_avg + forward_offset * cos(yaw_avg)
        y_corrected = y_avg + forward_offset * sin(yaw_avg)

        # Offset trái/phải
        x_corrected += lateral_offset * cos(yaw_avg + pi / 2)
        y_corrected += lateral_offset * sin(yaw_avg + pi / 2)

        # Tạo quaternion từ yaw trung bình
        q_avg = quaternion_from_euler(0, 0, yaw_avg)

        # Lưu vào biến thành viên
        self.trans_mirror = [x_corrected, y_corrected, 0.0]
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

        # Hub position calculation with configurable 2D offsets
        hub_x_offset = self.mirror_offsets["hub"]["x_offset"]
        hub_y_offset = self.mirror_offsets["hub"]["y_offset"]
        x_hub = x_m + hub_x_offset * cos_yaw - hub_y_offset * sin_yaw
        y_hub = y_m + hub_x_offset * sin_yaw + hub_y_offset * cos_yaw
        yaw_hub = yaw_m[2]

        # Waiting position calculation with configurable 2D offsets
        wait_x_offset = self.mirror_offsets["waiting"]["x_offset"]
        wait_y_offset = self.mirror_offsets["waiting"]["y_offset"]
        x_wait = x_m + wait_x_offset * cos_yaw - wait_y_offset * sin_yaw
        y_wait = y_m + wait_x_offset * sin_yaw + wait_y_offset * cos_yaw
        yaw_wait = yaw_m[2]

        # Temp position calculation with configurable 2D offsets
        temp_x_offset = self.mirror_offsets["temp"]["x_offset"]
        temp_y_offset = self.mirror_offsets["temp"]["y_offset"]
        x_temp = x_m + temp_x_offset * cos_yaw - temp_y_offset * sin_yaw
        y_temp = y_m + temp_x_offset * sin_yaw + temp_y_offset * cos_yaw
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

        # Tính hướng từ hub_pose tới waiting_pose (đảo ngược thứ tự tham số)
        self.path_angle = self.get_path_angle(hub_pose, waiting_pose)

        return True

    def dynamic_reconfig_movebase(self, vel_x, publish_safety, stop_center_qr):
        new_config = {
            "max_vel_x": vel_x,
            "max_vel_trans": vel_x,
            "publish_safety": publish_safety,
            "stop_center_qr": stop_center_qr,
        }
        count = 0
        while True:
            self.client_reconfig_movebase.update_configuration(new_config)
            rospy.sleep(0.1)
            count += 1
            if count > 2:
                break

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

    def get_path_angle(self, from_pose, to_pose):
        """
        Tính góc từ from_pose tới to_pose
        Args:
            from_pose: Pose object (điểm xuất phát)
            to_pose: Pose object (điểm đích)
        Returns:
            angle: góc theo radian từ from_pose tới to_pose
        """
        # Kiểm tra nếu input là Pose objects
        if hasattr(from_pose, 'position'):
            from_x = from_pose.position.x
            from_y = from_pose.position.y
        else:
            # Nếu là array [x, y]
            from_x = from_pose[0]
            from_y = from_pose[1]
            
        if hasattr(to_pose, 'position'):
            to_x = to_pose.position.x
            to_y = to_pose.position.y
        else:
            # Nếu là array [x, y]
            to_x = to_pose[0]
            to_y = to_pose[1]
        
        angle = atan2(to_y - from_y, to_x - from_x)
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
            #     "error_2_sensor = {}".format(
            #         self.min_sensor_1 - self.min_sensor_2
            #     ),
            # )
            # rospy.logwarn_throttle(
            #     2,
            #     "error_angle_sensor= {}".format(
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
        default=os.path.join(
            rospkg.RosPack().get_path("docking_charger"), "cfg"
        ),
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
    rospy.init_node("docking_charger_server", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    DockingCharger(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()

