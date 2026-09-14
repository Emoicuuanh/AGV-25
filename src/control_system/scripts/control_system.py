#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys

python3 = True if sys.hexversion > 0x03000000 else False
import json

if python3:
    import urllib.parse
# else:
#     from urlparse import urlparse
import yaml
import threading
import requests
import rospy
import rospkg
import tf
import copy
import time
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    PoseArray,
    PoseWithCovarianceStamped,
)
from std_msgs.msg import Int8, Int16, String
from std_stamped_msgs.msg import (
    StringStamped,
    Int8Stamped,
    StringFeedback,
    StringResult,
    StringAction,
    StringActionGoal,
    EmptyStamped,
    Float32Stamped,
    UInt32Stamped,
)
from std_stamped_msgs.srv import StringService, StringServiceResponse
from cartographer_ros_msgs.srv import (
    GetScoreGlobalMatching,
    GetScoreGlobalMatchingRequest,
    GetScoreGlobalMatchingResponse,
)
from os.path import expanduser
from safety_msgs.msg import SafetyStatus
from math import sqrt
from cartographer_ros_msgs.srv import WriteState, WriteStateRequest
import math
HOME = expanduser("~")

common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)

slam_manager_dir = os.path.join(
    rospkg.RosPack().get_path("slam_manager"), "scripts"
)
if not os.path.isdir(slam_manager_dir):
    slam_manager_dir = os.path.join(
        rospkg.RosPack().get_path("slam_manager"), "release"
    )
sys.path.insert(0, slam_manager_dir)
from slam_manager import MainState as SlamState

from mongodb import mongodb, LogLevel, MissionStatus
from module_manager import ModuleServer, ModuleClient, ModuleStatus
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
    get_line_info,
    YamlDumper,
    obj_to_dict,
    pose_dict_template,
    # get_ubuntu_serial_hash,
)


class RobotStatus(EnumString):
    NONE = -1
    PAUSED = 0
    WAITING = 1
    RUNNING = 2
    EMG = 4
    ERROR = 5
    WAITING_INIT_POSE = 6


class DetailStatus(EnumString):
    NONE = -1
    ALL_OK = 0
    AUTO_CHARGING = 1
    MANUAL_CHARGING = 2
    IO_BOARD_ERROR = 5
    LOAD_MAP_ERROR = 6
    MAPPING_ERROR = 7
    FATAL_ERROR = 8
    GENERAL_ERROR = 9
    SAFETY_STOP = 10
    ODOMETRY_ERROR = 11
    IO_BOARD_DISCONNECT = 12
    BATTERY_LOW = 13
    BATTERY_EMPTY = 14
    MOTOR_OFF = 15
    SAFETY_DISABLED = 38
    MOTOR_OVER_CURRENT = 23
    MOTOR_CONTROL_ERROR = 24
    LOAD_STATIC_MAP_ERROR = 25
    LOADED_STATIC_MAP = 26


class LedStatus(EnumString):
    STARTING = 0
    SHUTTING_DOWN = 1
    MANUAL = 2
    MAPPING = 3
    WAIT_RESPOND = 4
    PAUSED = 5
    EMG = 6
    SAFETY_STOP = 7
    GENERAL_ERROR = 10
    WAITING = 14
    FATAL_ERROR = 15
    BATTERY_LOW = 16
    BATTERY_EMPTY = 17
    MOTOR_OFF = 18
    RUNNING_NORMAL = 19
    SAFETY_DISABLED = 20
    WAITING_INIT_POSE = 21
    CHARGING_READY = 22
    MOTOR_OVER_CURRENT = 23
    MOTOR_CONTROL_ERROR = 24
    MAINTENANCE = 25
    EDIT_MAP = 26


class RobotMode(EnumString):
    NONE = -1
    MANUAL = 0
    AUTO = 1
    MAPPING = 2
    WAIT_RESPOND = 3
    EDITING_MAP = 4


class MainState(EnumString):
    NONE = -1
    INIT = 0
    WAIT_MANUAL = 1
    MANUAL = 2
    WAIT_AUTO = 3
    AUTO = 4
    WAIT_MAPPING = 5
    MAPPING = 6
    LOADING_MAP = 7
    INIT_LOAD_MAP = 8
    LOAD_MAP_ERROR = 9
    MAPPING_ERROR = 10
    INIT_MAPPING = 11
    INIT_LOAD_STATIC_MAP = 12
    LOADING_STATIC_MAP = 13
    LOAD_STATIC_MAP_ERROR = 14
    LOADED_STATIC_MAP = 15


class CurrentMap:
    map_dir = ""
    map_file = ""


SENSOR_DEACTIVATE = 0
SENSOR_ACTIVATE = 1
SWITCH_MANUAL_MODE = 0
SWITCH_AUTO_MODE = 1

# tf type
USE_TF_LIDAR = 1
USE_TF_QR_CODE = 0


class ControlSystem(object):
    def __init__(self, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        # Initial
        if not self.load_config():
            return
        rospy.on_shutdown(self.shutdown)
        # Publisher
        # Trong hàm __init__(), thêm publisher
        self.pub_lidar_info = rospy.Publisher(
            "/lidar_info", StringStamped, queue_size=10
        )
        self.pub_lidar_convert_position = rospy.Publisher(
            "/lidar_convert_position", StringStamped, queue_size=10
        )
        self.control_tf_pub = rospy.Publisher("/control_tf", Int8, queue_size=5)
        self.pub_init_server = rospy.Publisher(
            "/init_server", EmptyStamped, queue_size=1
        )
        self.pub_battery = rospy.Publisher(
            "/battery", Int8Stamped, queue_size=10
        )
        self.pub_robot_mode = rospy.Publisher(
            "/robot_mode", StringStamped, queue_size=10
        )
        self.pub_current_map = rospy.Publisher(
            "/current_map", StringStamped, queue_size=10
        )
        self.pub_current_server_map = rospy.Publisher(
            "/current_map_server", StringStamped, queue_size=10
        )
        self.pub_robot_status = rospy.Publisher(
            "/robot_status", StringStamped, queue_size=10
        )
        self.pub_mission_control = rospy.Publisher(
            "/mission_control", StringStamped, queue_size=10
        )
        self.pub_mission_run_pause = rospy.Publisher(
            "/mission_manager/run_pause_req", StringStamped, queue_size=10
        )
        self.pub_mapping_req = rospy.Publisher(
            "/mapping_request", StringStamped, queue_size=5
        )
        self.pub_map_load_req = rospy.Publisher(
            "/map_load_request", StringStamped, queue_size=5, latch=True
        )
        self.pub_static_map_load_req = rospy.Publisher(
            "/static_map_load_request", StringStamped, queue_size=5, latch=True
        )
        self.pub_map_save_req = rospy.Publisher(
            "/map_save_request", StringStamped, queue_size=5
        )
        self.pub_temp_map_save_req = rospy.Publisher(
            "/temp_savemap_request", StringStamped, queue_size=5
        )
        self.pub_led_status = rospy.Publisher(
            "/led_status", StringStamped, queue_size=5, latch=True
        )
        self.pub_trigger = rospy.Publisher(
            "/trigger_complex", StringStamped, queue_size=5
        )
        self.pub_mission_reset_error = rospy.Publisher(
            "/mission_manager/reset_error", EmptyStamped, queue_size=5
        )
        self.pub_stop_moving = rospy.Publisher(
            "/stop_moving", EmptyStamped, queue_size=5
        )
        self.pub_request_start_mission = rospy.Publisher(
            "/request_start_mission", StringStamped, queue_size=5
        )
        self.pub_mission_fr_server = rospy.Publisher(
            "/mission_from_server", StringStamped, queue_size=5
        )
        self.pub_mission_fr_server_latch = rospy.Publisher(
            "/mission_from_server_latch",
            StringStamped,
            queue_size=5,
            latch=True,
        )
        self.pub_request_sound = rospy.Publisher(
            "/request_sound", StringStamped, queue_size=5
        )
        self.traffic_control_type_pub = rospy.Publisher(
            "/current_traffic_control_type", StringStamped, queue_size=10
        )
        self.pub_save_editing_map = rospy.Publisher(
            "/save_editing_map", StringStamped, queue_size=5
        )
        rospy.Service(
            "/save_editing_map", StringService, self.save_editing_map_handle
        )
        # Common module

        # Optional module
        for i in self.module_config:
            client_module = None
            if python3:
                client_module = ModuleClient(
                    list(i.keys())[0], list(i.values())[0]
                )
                if list(i.keys())[0] == "mission_manager":
                    self.mission_manager_module = client_module
                elif list(i.keys())[0] == "slam_manager":
                    self.slam_manager_module = client_module
            else:
                client_module = ModuleClient(i.keys()[0], i.values()[0])
                if i.keys()[0] == "mission_manager":
                    self.mission_manager_module = client_module
                elif i.keys()[0] == "slam_manager":
                    self.slam_manager_module = client_module
            self.module_list.append(client_module)

        # Subscriber
        rospy.Subscriber(
            "/current_control_tf", Int8, self.current_control_tf_cb
        )
        rospy.Subscriber(
            "/goal_send_to_server",
            PoseStamped,
            self.goal_send_to_server_callback,
        )
        rospy.Subscriber(
            "/pause_by_traffic_status",
            StringStamped,
            self.pause_by_traffic_status_cb,
        )
        rospy.Subscriber(
            "/status_moving_traffic_control",
            StringStamped,
            self.status_planning_path_cb,
        )
        if self.option_config["use_qr"] and not self.simulation:
            from agv_msgs.msg import DataMatrixStamped

            # Subscribe to QR code data
            rospy.Subscriber("/data_gls621", DataMatrixStamped, self.data_qr_cb)

        rospy.Subscriber("current_path", Path, self.path_callback)
        rospy.Subscriber(
            "/init_server",
            EmptyStamped,
            self.init_server_cb,
        )
        rospy.Subscriber(
            "/mission_manager/module_status",
            StringStamped,
            self.mission_status_cb,
        )
        # rospy.Subscriber(
        #     "/moving_control/goal",
        #     StringActionGoal,
        #     self.moving_control_action_cb,
        # )
        rospy.Subscriber(
            "/current_goal_monving_control",
            PoseStamped,
            self.current_moving_control_goal_cb,
        )
        if self.option_config["use_followline"] and not self.simulation:
            from follow_line.msg import FollowLineActionGoal

            rospy.Subscriber(
                "/followline_action/goal",
                FollowLineActionGoal,
                self.followline_action_cb,
            )
        rospy.Subscriber(
            "/error_control_motor_right", Int16, self.right_vel_error_cb
        )
        rospy.Subscriber(
            "/error_control_motor_left", Int16, self.left_vel_error_cb
        )
        rospy.Subscriber("/request_mode", StringStamped, self.request_mode_cb)
        rospy.Service("/request_mode", StringService, self.request_mode_handle)
        rospy.Subscriber(
            "/request_savemap", StringStamped, self.request_savemap_cb
        )
        rospy.Service(
            "/request_savemap", StringService, self.request_savemap_handle
        )
        rospy.Subscriber(
            "/request_temp_savemap", StringStamped, self.request_temp_savemap_cb
        )
        rospy.Service(
            "/request_temp_savemap",
            StringService,
            self.request_temp_savemap_handle,
        )
        rospy.Subscriber(
            "/request_change_map", StringStamped, self.request_change_map_cb
        )
        rospy.Service(
            "/request_change_map", StringService, self.request_change_map_handle
        )
        rospy.Subscriber(
            "/request_load_static_map",
            StringStamped,
            self.request_load_static_map_cb,
        )
        rospy.Service(
            "/request_load_static_map",
            StringService,
            self.request_load_static_map_handle,
        )
        rospy.Subscriber(
            "/request_reload_map", StringStamped, self.request_reload_map_cb
        )
        rospy.Service(
            "/request_reload_map", StringService, self.request_reload_map_handle
        )
        rospy.Subscriber(
            "/request_run_stop", StringStamped, self.request_run_stop_cb
        )
        rospy.Subscriber(
            "/request_start_mission",
            StringStamped,
            self.request_start_mission_cb,
        )
        rospy.Subscriber("/standard_io", StringStamped, self.standard_io_cb)
        rospy.Subscriber("/arduino_error", StringStamped, self.arduino_error_cb)
        rospy.Subscriber("/reset_error", EmptyStamped, self.reset_error_cb)
        rospy.Subscriber("/fake_battery", Int8, self.fake_battery_cb)
        rospy.Subscriber(
            "/arduino_driver/float_param/battery_percent",
            Float32Stamped,
            self.real_battery_cb,
        )
        rospy.Subscriber(
            "/arduino_driver/float_param/battery_ampe",
            Float32Stamped,
            self.battery_ampe_cb,
        )
        rospy.Subscriber(
            "/arduino_driver/float_param/battery_voltage",
            Float32Stamped,
            self.battery_voltage_cb,
        )
        rospy.Subscriber(
            "/trigger_mission_from_server",
            StringStamped,
            self.trigger_mission_from_server_cb,
        )
        rospy.Subscriber(
            "/initialpose", PoseWithCovarianceStamped, self.initialpose_cb
        )
        rospy.Subscriber("/safety_status", SafetyStatus, self.safety_status_cb)
        rospy.Subscriber(
            "/moving_control/module_status",
            StringStamped,
            self.moving_control_module_status_cb,
        )
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.Subscriber(
            "/log_distance_move", UInt32Stamped, self.distance_move_record_cb
        )
        rospy.Subscriber("/log_lift", UInt32Stamped, self.lift_record_cb)
        rospy.Subscriber(
            "/request_working_stt", StringStamped, self.working_stt_cb
        )
        rospy.Subscriber(
            "/set_status_to_waitting_init_pose",
            EmptyStamped,
            self.set_status_to_waitting_init_pose_cb,
        )
        # Service
        rospy.Service(
            "/stop_mission_request", StringService, self.handle_stop_mission
        )
        self.get_score_slam = rospy.ServiceProxy(
            "/get_score_global_matching", GetScoreGlobalMatching
        )
        rospy.Service(
            "/request_save_static_map",
            StringService,
            self.request_save_static_map_srv,
        )
        rospy.Service(
            "/rename_pbstream", StringService, self.rename_pbstream_handle
        )
        # Server
        self.init_server()
        # Thead for server
        self.thread_server = threading.Thread(
            name="connect_server", target=self.connect_server
        )
        self.thread_server.daemon = True
        self.thread_server.start()

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
        self.tf_listener = tf.TransformListener()
        self.odom_frame = "odom"
        self.robot_frame = "base_footprint"
        self.map_frame = "map"
        self.current_pose = None
        # Config
        self.map_requesting = ""
        self.current_map_file = kwargs["current_map_file"]
        self.robot_config_file = kwargs["robot_config_file"]
        self.server_config_file = kwargs["robot_define"]
        self.current_map_cfg = CurrentMap()
        self.current_map_cfg.map_file = "mkac"
        self.current_map_cfg.map_dir = HOME + "/tmp/ros/maps/"
        # State
        self.slam_state = SlamState.NONE
        self.mission_status = ModuleStatus.WAITING
        self.running_mission_id = ""
        self.last_done_mission_id = ""
        self.led_status = LedStatus.STARTING.toString()
        self.main_state = MainState.INIT
        self.prev_state = MainState.NONE
        self.robot_mode = RobotMode.AUTO
        self.robot_mode_req = RobotMode.NONE
        self.robot_status = RobotStatus.NONE
        self.detail_status = DetailStatus.ALL_OK.toString()
        self.detail_status_addition = ""
        self.detail_status_not_dispay_hmi = ""
        self.pause_detail_status = ""
        self.sound_status = None
        self.std_io_status = {"emg_button": False}
        self.status_dict = {
            "status": RobotStatus.NONE.toString(),
            "detail": DetailStatus.NONE.toString(),
        }
        self.module_list = []
        self.error_code = ""
        self.pre_error_code = ""
        self.special_matching_state = ""
        self.arduino_error_msg = StringStamped()
        self.arduino_error_cnt = 0
        self.load_map_cnt = 0
        self.mapping_cnt = 0
        # Flag
        self.save_map_request = False
        self.reset_error_request = False
        self.mission_from_server_req = False
        self.init_pose_received = False
        self.battery_percent = 90
        self.battery_ampe = 0
        self.battery_voltage = 0
        self.last_battery_fake = rospy.get_time()
        self.last_std_io_msg = rospy.get_time()
        self.last_odom_msg = rospy.get_time()
        self.time_start = rospy.get_time()
        # Button
        self.emg_button = SENSOR_DEACTIVATE
        self.start_1_button = SENSOR_DEACTIVATE
        self.start_2_button = SENSOR_DEACTIVATE
        self.stop_1_button = SENSOR_DEACTIVATE
        self.stop_2_button = SENSOR_DEACTIVATE
        self.auto_manual_sw = SENSOR_DEACTIVATE
        self.motor_enable_sw = SENSOR_DEACTIVATE
        self.update_status_button = True
        # Safety
        self.last_safety_time = rospy.get_time()
        self.is_safety = False
        # Ros params
        self.pose_initiated = rospy.get_param("/pose_initiated", False)
        # Other
        self.moving_control_status_dict = {"status": "NONE"}
        # Server
        self.agv_db_object_id = None
        # Bumper
        self.bumper = SENSOR_DEACTIVATE
        # Motor
        self.control_motor_left_error = False
        self.control_motor_right_error = False
        self.motor_left_over_ampe = False
        self.motor_right_over_ampe = False
        # Pose
        self.tf_listener = tf.TransformListener()
        self.followline_goal = ""
        self.moving_control_goal = ""
        self.pause_by_cross_function = False
        self.current_action_type = ""
        # Database
        db_address = rospy.get_param("/mongodb_address")
        # print_debug(db_address)
        self.db = mongodb(db_address)
        # server
        self.working_status = "PRODUCTION"
        self.reset_get_mission = False
        self.map_server_name = ""
        self.odom_moved_record = 0
        self.lift_record = 0
        self.detail_status_update_to_server = ""
        self.message_update_to_server = ""
        self.robot_status_update_to_server = ""
        self.last_detail_status_update_to_server = ""
        self.last_robot_status_update_to_server = ""
        self.last_message_update_to_server = ""
        self.addtion_error_resset_qr_code = ""
        self.list_action_move = [
            "move",
            "hub",
            "matehand",
            "docking_charger",
            "un_docking",
            "elevator",
        ]
        self.reset_when_mission_error = True
        self.error_code_number = -1

        # handle resume when pause
        self.label_pose = Pose()
        self.first_pose_in_path = Pose()
        self.last_pose_in_path = Pose()
        self.last_time_read_qr = rospy.get_time()
        self.need_to_reset_by_qr_code = False

        # Traffic
        self.status_exe_path = ""
        self.clear_path_display = False
        self.data_traffic_control = ""
        self.use_new_traffic_control = False
        self.need_receive_new_path = True
        self.current_server_map = ""

        # handle reset stop, cancle, finish, restart from HMI
        self.current_mission_id_running = ""
        self.reset_mission_request_type = ""
        self.mission_id_reset_request = ""
        self.server_error_msg = ""
        self.mission_manager_module = None
        self.slam_manager_module = None
        self.sound_config = None  # TODO: Check if None

        # localization
        self.relocalization = False

        # switch to tf qr first time
        self.current_control_tf = 100

    def init_server(self):
        if self.server_config == None:
            rospy.loginfo_throttle(30, "Server was not configured!")
            return
        self.api_url = self.server_config["server_address"]
        self.header = {
            "X-Parse-Application-Id": "APPLICATION_ID",
            "X-Parse-Master-Key": "YOUR_MASTER_KEY",
            "Content-Type": "application/json",
        }

        # Get AGV object ID in Database
        param = urllib.parse.urlencode(
            {"where": json.dumps({"name": self.server_config["agv_name"]})}
        )
        rospy.logwarn(
            "AGV name for system: {}".format(self.server_config["agv_name"])
        )
        try:
            agv_db_obj = requests.get(
                self.api_url + "classes/agv", params=param, headers=self.header
            )
            rospy.logwarn("Request server result:\n{}".format(agv_db_obj))
            # print_debug(json.dumps(json.loads(agv_db_obj.text), indent=2))
            self.agv_db_object_id = json.loads(agv_db_obj.text)["results"][0][
                "objectId"
            ]
            rospy.logwarn("AGV obj db id: {}".format(self.agv_db_object_id))
        except Exception as e:
            self.agv_db_object_id = None
            rospy.logerr(e)
            self.server_error_msg = "SERVER_DISCONNECTED"

    def shutdown(self):
        # Kill all node
        print("Shutdown: {}".format(rospy.get_name()))

    """
    ######## ##     ## ##    ##  ######  ######## ####  #######  ##    ##
    ##       ##     ## ###   ## ##    ##    ##     ##  ##     ## ###   ##
    ##       ##     ## ####  ## ##          ##     ##  ##     ## ####  ##
    ######   ##     ## ## ## ## ##          ##     ##  ##     ## ## ## ##
    ##       ##     ## ##  #### ##          ##     ##  ##     ## ##  ####
    ##       ##     ## ##   ### ##    ##    ##     ##  ##     ## ##   ###
    ##        #######  ##    ##  ######     ##    ####  #######  ##    ##
    """

    def switch_control_tf(self, type):
        if self.current_control_tf == type:
            return True
        else:
            self.control_tf_pub.publish(Int8(data=type))
            return False

    def publish_traffic_control_type(self):
        msg = StringStamped()
        msg.stamp = rospy.Time.now()
        msg.data = "true" if self.use_new_traffic_control else "false"
        self.traffic_control_type_pub.publish(msg)

    def update_path_display_in_app(self, data):
        try:
            self.delete_existing_objects()
            # response = requests.post(self.api_url + "classes/traffic_path",, headers=header, json=data)  # Send the updated data
            response = requests.post(
                self.api_url + "classes/traffic_path",
                json=data,
                headers=self.header,
                # timeout=1,
            )

            # Check the status code for success (200, 201)
            if response.status_code in [200, 201]:
                print("Success! Object created or collection 'path' updated.")
                # Parse the response JSON for more details
                response_data = (
                    response.json()
                )  # This converts the response to a dictionary
                print(json.dumps(response_data, indent=2))

                # Optionally, check for specific success indicators in the response body
                if response_data.get("success") is True:
                    print(
                        "The operation was marked as successful in the response body."
                    )
                else:
                    print(
                        "The operation succeeded, but no 'success' field was found in the response."
                    )
            else:
                print(
                    f"Failed to create object, status code: {response.status_code}"
                )
                print("Response:", response.text)  # Print the detailed response

        except requests.exceptions.RequestException as e:
            print("An error occurred while trying to create the collection.")
            print(e)

    def connect_server(self):
        # Get mission from server
        last_update_pose = rospy.get_time()
        last_send_traffic_control = rospy.get_time()
        begin_request_mission = rospy.get_time()
        mission_queue_id = ""
        pre_mission_queue_id = ""
        allow_update_mission = False
        data = ""
        r = rospy.Rate(10.0)
        while True:
            begin_time = rospy.get_time()
            if self.robot_status == RobotStatus.WAITING:
                if self.clear_path_display:
                    self.clear_path_display = False
                    self.delete_existing_objects()
            # Update position to server
            if self.agv_db_object_id != None:
                now = rospy.get_time()
                update_when_change_state = False
                if (
                    self.detail_status_update_to_server
                    != self.last_detail_status_update_to_server
                    or self.robot_status_update_to_server
                    != self.last_robot_status_update_to_server
                    or self.message_update_to_server
                    != self.last_message_update_to_server
                ):
                    update_when_change_state = True
                if now - last_update_pose >= 2 or update_when_change_state:
                    last_update_pose = now
                    if self.current_pose != None:
                        pose = {
                            "x": self.current_pose.position.x,
                            "y": self.current_pose.position.y,
                            "z": 0.0,
                        }
                        # Trong hàm connect_server(), phần update position to server
                        data = {
                            "function": "UPDATE_AGV",
                            "agv": self.server_config["agv_name"],
                            "status": {
                                "status": self.robot_status_update_to_server,
                                "detail": self.detail_status_update_to_server,
                                "message": self.message_update_to_server,
                            },
                            "position": pose,
                            "battery": {
                                "capacity": self.battery_percent,
                                "voltage": round(self.battery_voltage, 2),
                                "current": round(self.battery_ampe, 2),
                            },
                            "params": {
                                "odo": self.odom_moved_record,
                                "liftup": self.lift_record,
                                "working_status": self.working_status,
                                "lidar_map": self.current_map_cfg.map_file if self.current_control_tf == 1 else "",
                            },
                            "version": 6,
                        }
                        rospy.logwarn(data)
                        try:
                            response_update_pose = requests.post(
                                self.api_url + "functions/agvapi",
                                data=json.dumps(data),
                                headers=self.header,
                                timeout=1,
                            )
                            result_update_pose = json.loads(
                                response_update_pose.text
                            )
                            if "result" in result_update_pose:
                                result_str = result_update_pose["result"]
                                # rospy.logwarn(result_str)

                                # Kiểm tra nếu bên trong result có các khóa cần thiết
                                if "map" in result_str:
                                    self.current_server_map = result_str["map"]
                                else:
                                    rospy.logerr(
                                        "Missing keys map in response 'result'"
                                    )
                                if "build_in_traffic" in result_str:
                                    self.use_new_traffic_control = (
                                        not result_str["build_in_traffic"]
                                    )

                                else:
                                    rospy.logerr(
                                        "Missing keys build_in_traffic in response 'result'"
                                    )

                                # Xử lý thông tin lidar
                                if "lidar" in result_str:
                                    lidar_data = result_str["lidar"]
                                    rospy.loginfo(f"Received lidar data: {lidar_data}")
                                    
                                    # Publish lidar info ra topic
                                    lidar_msg = StringStamped()
                                    lidar_msg.stamp = rospy.Time.now()
                                    lidar_msg.data = json.dumps(lidar_data, indent=2)
                                    self.pub_lidar_info.publish(lidar_msg)
                                    
                                    rospy.loginfo(f"Published lidar info: {lidar_msg.data}")
                                    # Tính toán và publish lidar position nếu current_control_tf == 1
                                    if self.current_control_tf == 0:
                                        try:
                                            # Lấy current robot position (QR coordinates)
                                            QR_x = self.current_pose.position.x
                                            QR_y = self.current_pose.position.y
                                            
                                            # Lấy thông tin từ lidar data
                                            scale = float(lidar_data["scale"])
                                            offset_x = float(lidar_data["offset"]["x"])
                                            offset_y = float(lidar_data["offset"]["y"])
                                            angle = float(lidar_data["angle"])  # angle in radians
                                            
                                            # Tính toán lidar position theo công thức
                                            cos_angle = math.cos(math.radians(angle))
                                            sin_angle = math.sin(math.radians(angle))
                                            
                                            position_lidar_x = (QR_x * scale * cos_angle) - (QR_y * scale * sin_angle) + offset_x
                                            position_lidar_y = (QR_x * scale * sin_angle) + (QR_y * scale * cos_angle) + offset_y
                                            
                                            # Tạo message để publish
                                            lidar_position_data = {
                                                "original_position": {
                                                    "x": QR_x,
                                                    "y": QR_y
                                                },
                                                "lidar_convert_position": {
                                                    "x": position_lidar_x,
                                                    "y": position_lidar_y
                                                },
                                                "conversion_params": {
                                                    "scale": scale,
                                                    "offset": {"x": offset_x, "y": offset_y},
                                                    "angle": angle
                                                }
                                            }
                                            
                                            lidar_position_msg = StringStamped()
                                            lidar_position_msg.stamp = rospy.Time.now()
                                            lidar_position_msg.data = json.dumps(lidar_position_data, indent=2)
                                            self.pub_lidar_convert_position.publish(lidar_position_msg)
                                            
                                            rospy.loginfo(f"Published lidar position: QR({QR_x:.3f}, {QR_y:.3f}) -> Lidar({position_lidar_x:.3f}, {position_lidar_y:.3f})")
                                            
                                        except (KeyError, ValueError, TypeError) as e:
                                            rospy.logerr(f"Error calculating lidar position: {e}")
                                    else:
                                        rospy.logdebug(f"current_control_tf != 0 ({self.current_control_tf}), skipping lidar position calculation")
                                else:
                                    rospy.logwarn("No lidar data in server response")
                            else:
                                rospy.logerr("Missing 'result' in response")
                            rospy.loginfo(
                                "Update status current Pose - Position: [x: %f, y: %f, z: %f], Orientation: [x: %f, y: %f, z: %f, w: %f]",
                                self.current_pose.position.x,
                                self.current_pose.position.y,
                                self.current_pose.position.z,
                                self.current_pose.orientation.x,
                                self.current_pose.orientation.y,
                                self.current_pose.orientation.z,
                                self.current_pose.orientation.w,
                            )

                            if (
                                self.detail_status_update_to_server
                                != self.last_detail_status_update_to_server
                            ):
                                rospy.logwarn(
                                    "detail_status_update_to_server: {}".format(
                                        self.detail_status_update_to_server
                                    )
                                )
                            if (
                                self.robot_status_update_to_server
                                != self.last_robot_status_update_to_server
                            ):
                                rospy.logwarn(
                                    "robot_status_update_to_server: {}".format(
                                        self.robot_status_update_to_server
                                    )
                                )
                            if (
                                self.message_update_to_server
                                != self.last_message_update_to_server
                            ):
                                rospy.logwarn(
                                    "message_update_to_server: {}".format(
                                        self.message_update_to_server
                                    )
                                )
                            self.last_detail_status_update_to_server = (
                                self.detail_status_update_to_server
                            )
                            self.last_robot_status_update_to_server = (
                                self.robot_status_update_to_server
                            )
                            self.last_message_update_to_server = (
                                self.message_update_to_server
                            )
                        except requests.exceptions.RequestException as e:
                            rospy.logerr("update robot status fail")
                            rospy.logerr_throttle(10.0, e)
            else:
                rospy.logerr_throttle(
                    10.0,
                    "cannot update robot status because agv_db_object_id is None",
                )
            # get mission
            if (
                self.robot_status == RobotStatus.WAITING
                and self.robot_mode == RobotMode.AUTO
            ):
                if mission_queue_id != pre_mission_queue_id:
                    pre_mission_queue_id = mission_queue_id
                    allow_update_mission = True
                    data = ""
                if rospy.get_time() - begin_request_mission > 2.0:
                    if self.agv_db_object_id != None:
                        # Update mission done
                        # if self.reset_get_mission:
                        #     self.reset_get_mission = False
                        # allow_update_mission = False
                        # them obtion nhu data_reset = "done", "cancle" , "restart"
                        # allow_update_mission = True de allow update
                        if mission_queue_id != "" and allow_update_mission:
                            if self.last_done_mission_id == mission_queue_id:
                                rospy.logwarn(
                                    "Mission done id: {}".format(
                                        self.last_done_mission_id
                                    )
                                )
                                data = {
                                    "function": "UPDATE_MISSION",
                                    "missionId": self.last_done_mission_id,
                                    "progress": "done",
                                }
                            else:
                                if (
                                    self.mission_id_reset_request
                                    != self.current_mission_id_running
                                ):
                                    self.reset_get_mission = False
                                if self.reset_get_mission:
                                    self.reset_get_mission = False
                                    rospy.logwarn(
                                        "Mission id reset by HMI: {}, data_type: {}".format(
                                            self.current_mission_id_running,
                                            self.reset_mission_request_type,
                                        )
                                    )
                                    data = {
                                        "function": "UPDATE_MISSION",
                                        "missionId": self.current_mission_id_running,
                                        "progress": self.reset_mission_request_type,
                                    }
                            if data != "":
                                try:
                                    response_update_mission = requests.post(
                                        self.api_url + "functions/agvapi",
                                        data=json.dumps(data),
                                        headers=self.header,
                                        timeout=1,
                                    )
                                    result_update_mission = json.loads(
                                        response_update_mission.text
                                    )
                                    if (
                                        "result" in result_update_mission
                                        and result_update_mission["result"]
                                        == "OK"
                                    ):
                                        allow_update_mission = False
                                        mission_queue_id = ""
                                        pre_mission_queue_id = ""
                                        rospy.logwarn(
                                            "Update mission done succeed!"
                                        )
                                    else:
                                        rospy.logerr(
                                            "UPDAVE MISSION FAIL --> CANNOT GET NEW MISSION"
                                        )
                                except (
                                    requests.exceptions.RequestException
                                ) as e:
                                    rospy.logerr_throttle(10.0, e)
                                    rospy.logerr(
                                        "UPDAVE MISSION FAIL --> CANNOT GET NEW MISSION"
                                    )
                                    # Nếu không update được mision done hiện tại thì
                                    # chưa gọi request tiếp mission
                        if not allow_update_mission:
                            try:
                                data = {
                                    "agv": self.server_config["agv_name"],
                                }
                                response = requests.post(
                                    self.api_url + "functions/get_mission",
                                    data=json.dumps(data),
                                    headers=self.header,
                                    timeout=5,
                                )  # TOCHECK
                                _temp = json.loads(response.text)
                                mission_json = json.dumps(_temp, indent=2)
                                rospy.loginfo(
                                    "Mission from server:\n{}".format(
                                        mission_json
                                    )
                                )
                                # print_debug(_temp["result"])
                                if (
                                    "result" in _temp
                                    and _temp["result"] != None
                                    and "actions" in _temp["result"]
                                    and len(_temp["result"]["actions"]) > 0
                                ):
                                    for i in range(
                                        len(_temp["result"]["actions"])
                                    ):
                                        if (
                                            "waypoints"
                                            in _temp["result"]["actions"][i]
                                        ):
                                            try:
                                                self.map_server_name = _temp[
                                                    "result"
                                                ]["actions"][i]["waypoints"][0][
                                                    "map"
                                                ]
                                                break
                                            except:
                                                pass

                                    self.pub_mission_fr_server.publish(
                                        StringStamped(
                                            stamp=rospy.Time.now(),
                                            data=mission_json,
                                        )
                                    )
                                    self.pub_mission_fr_server_latch.publish(
                                        StringStamped(
                                            stamp=rospy.Time.now(),
                                            data=mission_json,
                                        )
                                    )
                                    # TODO: get from ros param
                                    mission_queue_id = _temp["result"][
                                        "mission_queue_id"
                                    ]
                                    self.current_mission_id_running = (
                                        mission_queue_id
                                    )
                                    rospy.set_param(
                                        "/current_mission_queue_id",
                                        mission_queue_id,
                                    )
                                    rospy.logwarn(
                                        "Received new mission_queue_id : {}".format(
                                            mission_queue_id
                                        )
                                    )
                                    self.db.saveStatusCartData(
                                        "status_cart", "no_cart"
                                    )
                            except requests.exceptions.RequestException as e:
                                rospy.logerr_throttle(10.0, e)
                                # rospy.logerr("Reset agv_db_object_id")
                                # self.agv_db_object_id = None

                    elif self.server_config != None:  # Retry init server
                        rospy.logerr_throttle(
                            10.0,
                            "cannot get and update mission because agv_db_object_id is None",
                        )
                        self.pub_init_server.publish(
                            EmptyStamped(stamp=rospy.Time.now())
                        )
                    begin_request_mission = rospy.get_time()
            else:
                rospy.logerr_throttle(
                    10.0,
                    "cannot get and update mission because agv mode is manual or status is error",
                )

            # """
            #  ####### ######     #    ####### ####### ###  #####
            #     #    #     #   # #   #       #        #  #     #
            #     #    #     #  #   #  #       #        #  #
            #     #    ######  #     # #####   #####    #  #
            #     #    #   #   ####### #       #        #  #
            #     #    #    #  #     # #       #        #  #     #
            #     #    #     # #     # #       #       ###  #####

            # """
            # Traffic control

            if self.use_new_traffic_control:
                if (
                    self.robot_status == RobotStatus.RUNNING
                    and self.robot_mode == RobotMode.AUTO
                ) or (
                    self.robot_status == RobotStatus.PAUSED
                    and self.robot_mode == RobotMode.AUTO
                ):
                    if self.data_traffic_control != "":
                        if (
                            "STOP" in self.data_traffic_control
                            and self.current_action_type != "un_docking"
                        ):
                            self.status_exe_path = self.data_traffic_control
                            if self.robot_status == RobotStatus.RUNNING:
                                rospy.logwarn("Stop by traffic control")
                                self.pause_robot_by_server()
                                self.pause_detail_status = (
                                    "Pause by traffic control"
                                )
                                self.data_traffic_control = ""

                            elif self.robot_status != RobotStatus.RUNNING:
                                if (
                                    self.moving_control_status_dict["status"]
                                    == ModuleStatus.RUNNING.toString()
                                ):
                                    rospy.logwarn("Stop by traffic control")
                                    self.pause_robot_by_server()
                                    self.pause_detail_status = (
                                        "Pause by traffic control"
                                    )
                                    self.data_traffic_control = ""

                        else:
                            self.status_exe_path = ""
                            if self.robot_status == RobotStatus.PAUSED:
                                if self.pause_by_cross_function:
                                    rospy.logwarn("Resume by traffic control")
                                    self.resume_running_by_server()
                                    self.data_traffic_control = ""

            else:
                self.status_exe_path = ""
                if self.server_config != None and (
                    (
                        self.robot_status == RobotStatus.RUNNING
                        and self.robot_mode == RobotMode.AUTO
                    )
                    or (
                        self.robot_status == RobotStatus.PAUSED
                        and self.robot_mode == RobotMode.AUTO
                    )
                ):
                    time_send_traffic_control = rospy.get_time()
                    if (
                        time_send_traffic_control - last_send_traffic_control
                        >= 0.3
                    ):
                        # if self.is_near_intersect_area(dist_check=2.5):
                        if self.current_pose != None:
                            pose = {
                                "x": self.current_pose.position.x,
                                "y": self.current_pose.position.y,
                                "z": 0.0,
                            }
                            data = {
                                "function": "TRAFFIC_CONTROL",
                                "agv": self.server_config["agv_name"],
                                "position": pose,
                            }
                            try:
                                response_traffic_control = requests.post(
                                    self.api_url + "functions/agvapi",
                                    data=json.dumps(data),
                                    headers=self.header,
                                    timeout=1,
                                )
                                response_traffic_control = json.loads(
                                    response_traffic_control.text
                                )
                                if response_traffic_control["result"] == "STOP":
                                    if self.robot_status == RobotStatus.RUNNING:
                                        rospy.logwarn("Stop by traffic control")
                                        self.pause_robot_by_server()
                                        self.pause_detail_status = (
                                            "Pause by traffic control"
                                        )
                                    elif (
                                        self.robot_status != RobotStatus.RUNNING
                                    ):
                                        if (
                                            self.moving_control_status_dict[
                                                "status"
                                            ]
                                            == ModuleStatus.RUNNING.toString()
                                        ):
                                            rospy.logwarn(
                                                "Stop by traffic control"
                                            )
                                            self.pause_robot_by_server()
                                            self.pause_detail_status = (
                                                "Pause by traffic control"
                                            )

                                else:
                                    if self.robot_status == RobotStatus.PAUSED:
                                        if self.pause_by_cross_function:
                                            rospy.logwarn(
                                                "Resume by traffic control"
                                            )
                                            self.resume_running_by_server()
                                last_send_traffic_control = (
                                    time_send_traffic_control
                                )
                                rospy.logwarn(
                                    "last pose send traffic control: {}".format(
                                        self.current_pose
                                    )
                                )
                            except requests.exceptions.RequestException as e:
                                rospy.logerr_throttle(10.0, e)
                                rospy.logerr("request traffic control fail")
                            if rospy.get_time() - begin_time > 0.5:
                                rospy.logerr(
                                    "send traffic control time out: {}".format(
                                        rospy.get_time() - begin_time
                                    )
                                )
                        # else:
                        #     rospy.logwarn_throttle(1, "agv not near cross line")
            if rospy.get_time() - begin_time > 1:
                rospy.logerr(
                    "Connect server consume time: {}".format(
                        rospy.get_time() - begin_time
                    )
                )
            r.sleep()

    def is_near_intersect_area(self, dist_check=2):
        try:
            rospy.logwarn_throttle(
                1,
                "current_action_type: {},current_pose: {}, pose_check: {}".format(
                    self.current_action_type,
                    self.current_pose,
                    self.moving_control_goal,
                ),
            )
            if self.current_action_type == "move":
                if self.moving_control_goal != "":
                    # for i in range(len(self.moving_control_goal)):
                    #     _pose = self.moving_control_goal[i]["position"]
                    #     goal_check = dict_to_obj(_pose, Pose())
                    #     distance = distance_two_pose(
                    #         self.current_pose, goal_check
                    #     )
                    #     if distance < dist_check:
                    #         return True
                    # return False
                    distance = distance_two_pose(
                        self.current_pose, self.moving_control_goal
                    )
                    if distance < dist_check:
                        return True
                    return False

            elif self.current_action_type in self.list_action_move:
                if self.followline_goal != "":
                    distance = distance_two_pose(
                        self.current_pose, self.followline_goal
                    )
                    if distance < dist_check:
                        return True
                    else:
                        return False
                else:
                    if self.moving_control_goal != "":
                        distance = distance_two_pose(
                            self.current_pose, self.moving_control_goal
                        )
                        if distance < dist_check:
                            return True
                        return False

            return False
        except Exception as e:
            rospy.logerr("error check near intersection: {}".format(e))
            return False

    def dump_config(self):
        # TODO: Only dump when load map successful
        with open(self.current_map_file, "w") as file:
            documents = yaml.dump(self.current_map_cfg, file)

    def load_config(self):
        try:
            # Current map
            print_debug(
                "Check current_map_file: {}".format(self.current_map_file)
            )
            if os.path.exists(self.current_map_file):
                with open(self.current_map_file) as file:
                    config_dict = yaml.load(file, Loader=yaml.Loader)
                    if config_dict == None:
                        rospy.logwarn("Current map file empty")
                        self.dump_config()
                    else:
                        config_dict.map_dir = (
                            self.current_map_cfg.map_dir
                        )  # Overwrite map_dir for different user name
                        self.current_map_cfg = config_dict
                        self.dump_config()
                        print_info("Current map file:")
                        print(
                            yaml.dump(
                                self.current_map_cfg,
                                Dumper=YamlDumper,
                                default_flow_style=False,
                            )
                        )
            else:
                print_warn(
                    "Check current_map_file not exist. Write default value"
                )
                self.dump_config()
            # Robot config
            if os.path.exists(self.robot_config_file):
                with open(self.robot_config_file) as file:
                    config_dict = yaml.load(file, Loader=yaml.Loader)
                    if config_dict == None:
                        rospy.logerr("Robot config file empty")
                    else:
                        self.robot_config = config_dict
                        self.battery_config = self.robot_config["battery"]
                        self.module_config = self.robot_config["module_list"]
                        self.option_config = self.robot_config["option"]
                        self.sound_config = self.robot_config["sound"]
                        self.error_code_remap_config = self.robot_config[
                            "error_code_remap"
                        ]
                        if "motor" in self.robot_config:
                            self.motor_config = self.robot_config["motor"]
                            rospy.Subscriber(
                                "/ampe_left",
                                Float32Stamped,
                                self.ampe_motor_left_cb,
                            )
                            rospy.Subscriber(
                                "/ampe_right",
                                Float32Stamped,
                                self.ampe_motor_right_cb,
                            )
                        if "use_followline" not in self.option_config:
                            self.option_config["use_followline"] = False
                        if "use_qr" not in self.option_config:
                            self.option_config["use_qr"] = False
                        if "check_init_pose" not in self.option_config:
                            self.option_config["check_init_pose"] = True
                        if "use_slam" not in self.option_config:
                            self.option_config["use_slam"] = True
                        if "use_bumper" not in self.option_config:
                            self.option_config["use_bumper"] = False

                        # self.server_config = (
                        #     self.robot_config["server"]
                        #     if "server" in self.robot_config
                        #     else None
                        # )
                        print_info("Robot config file:")
                        print(
                            yaml.dump(
                                self.robot_config,
                                Dumper=YamlDumper,
                                default_flow_style=False,
                            )
                        )
            # TODO: add file robot config in folder robot config to define agv name, server
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
            rospy.logerr("Load config: {}".format(e))
            return False
        return True

    def set_pose_init(self, value):
        self.pose_initiated = value
        rospy.set_param("/pose_initiated", value)

    def pause_robot(self):
        self.pause_by_cross_function = False
        self.pub_mission_run_pause.publish(
            StringStamped(stamp=rospy.Time.now(), data="PAUSE")
        )

    def pause_robot_by_server(self):
        rospy.logwarn("Pause by cross function")
        self.pause_by_cross_function = True
        self.pub_mission_run_pause.publish(
            StringStamped(stamp=rospy.Time.now(), data="PAUSE_BY_SERVER")
        )

    def stop_moving(self):
        self.pub_stop_moving.publish(EmptyStamped(stamp=rospy.Time.now()))

    def resume_running(self):
        if self.need_to_reset_by_qr_code:
            if (
                self.point_to_line_distance_pose(
                    self.first_pose_in_path,
                    self.last_pose_in_path,
                    self.label_pose,
                )
                < 0.6
            ) and (rospy.get_time() - self.last_time_read_qr < 1):
                self.pub_mission_run_pause.publish(
                    StringStamped(stamp=rospy.Time.now(), data="RUN")
                )
                self.pause_detail_status = ""
                self.need_to_reset_by_qr_code = False
        else:
            self.pub_mission_run_pause.publish(
                StringStamped(stamp=rospy.Time.now(), data="RUN")
            )
            self.pause_detail_status = ""

    def resume_running_by_server(self):
        rospy.logwarn("Resume running by cross function")
        self.pub_mission_run_pause.publish(
            StringStamped(stamp=rospy.Time.now(), data="RUN")
        )
        self.pause_detail_status = ""

    def load_map(self, map_file, abs_path=True, reload_map=False):
        msg = StringStamped()
        msg.stamp = rospy.Time.now()
        if abs_path:
            msg.data = self.current_map_cfg.map_dir + map_file
        else:
            msg.data = map_file
        rospy.logwarn(msg.data)
        if not reload_map:
            self.pub_map_load_req.publish(msg)
            rospy.loginfo("load_map: {}".format(msg.data))
        else:
            self.pub_map_load_req.publish(msg)
            rospy.loginfo("reload_map: {}".format(msg.data))

    def load_static_map(self, map_file, abs_path=True, reload_map=False):
        msg = StringStamped()
        msg.stamp = rospy.Time.now()
        if abs_path:
            msg.data = self.current_map_cfg.map_dir + map_file
        else:
            msg.data = map_file
        rospy.logwarn(msg.data)
        if not reload_map:
            self.pub_static_map_load_req.publish(msg)
            rospy.loginfo("load_map: {}".format(msg.data))
        else:
            self.pub_static_map_load_req.publish(msg)
            rospy.loginfo("reload_map: {}".format(msg.data))

    def check_map(self, map_file):
        # Service
        rospy.loginfo("Check map file: {}".format(map_file))
        check_map_srv = rospy.ServiceProxy("check_map", StringService)
        try:
            result = check_map_srv(map_file)
            if result.respond == "OK":
                return True
            else:
                return False
        except Exception as e:
            rospy.logerr("Check map error: {}".format(e))
            return False

    def label_msg_to_pose(self, qr_msg):
        qr_pose = Pose()
        qr_pose.position.x = (qr_msg.lable.x) / 1000
        qr_pose.position.y = (qr_msg.lable.y) / 1000
        return qr_pose

    def point_to_line_distance_pose(self, pose_A, pose_B, pose_C):
        # Extract positions from Pose objects
        if pose_A == Pose() or pose_B == Pose():
            return 0
        if self.need_receive_new_path:
            return 0
        x1, y1 = pose_A.position.x, pose_A.position.y
        x2, y2 = pose_B.position.x, pose_B.position.y
        xC, yC = pose_C.position.x, pose_C.position.y

        # Check if A and B are the same point
        if x1 == x2 and y1 == y2:
            # Return Euclidean distance from C to A
            return sqrt((xC - x1) ** 2 + (yC - y1) ** 2)

        # Compute the perpendicular distance from C to the line AB
        numerator = abs((y2 - y1) * xC - (x2 - x1) * yC + x2 * y1 - y2 * x1)
        denominator = sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)

        if denominator == 0:
            return float("inf")  # Should not happen after the A == B check

        distance = numerator / denominator
        return distance

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

    def goal_send_to_server_callback(self, msg):
        self.need_receive_new_path = True

    def pause_by_traffic_status_cb(self, msg):
        self.data_traffic_control = msg.data

    def status_planning_path_cb(self, msg):
        self.status_exe_path = msg.data

    def path_callback(self, msg):
        # New format list for the goals
        self.clear_path_display = True
        new_format_goals = []

        # Iterate over each pose in the path message
        for pose_stamped in msg.poses:
            position = pose_stamped.pose.position  # Extract position
            orientation = pose_stamped.pose.orientation  # Extract orientation

            # Create new formatted goal
            new_goal = {
                "position": {
                    "x": position.x,
                    "y": position.y,
                    "z": 0.0,  # Assuming you want z to be 0.0
                },
                "orientation": {
                    "x": orientation.x,
                    "y": orientation.y,
                    "z": orientation.z,  # Assuming you want to keep the original z
                    "w": orientation.w,  # Assuming you want to keep the original w
                },
            }

            # Append to new format list
            new_format_goals.append(new_goal)

        # Create the data dictionary to send, including the map and AGV name
        data = {
            "poses": new_format_goals,
            "map": self.map_server_name,
            "agv": self.server_config["agv_name"],
        }

        # Log the data being sent
        rospy.logerr("Data being sent:\n%s", json.dumps(data, indent=2))

        # Call the update function to send the data to the app
        self.update_path_display_in_app(data)

        if len(msg.poses) == 0:
            rospy.loginfo("No poses in the path.")
            return

        if len(msg.poses) == 1:
            self.first_pose_in_path = msg.poses[0].pose
            self.last_pose_in_path = msg.poses[0].pose
        else:
            self.first_pose_in_path = msg.poses[0].pose
            self.last_pose_in_path = msg.poses[-1].pose
        self.first_pose_in_path.position.x = round(
            self.first_pose_in_path.position.x, 3
        )
        self.first_pose_in_path.position.y = round(
            self.first_pose_in_path.position.y, 3
        )

        self.last_pose_in_path.position.x = round(
            self.last_pose_in_path.position.x, 3
        )
        self.last_pose_in_path.position.y = round(
            self.last_pose_in_path.position.y, 3
        )
        self.need_receive_new_path = False

    def data_qr_cb(self, msg):
        self.label_pose = self.label_msg_to_pose(msg)
        self.last_time_read_qr = rospy.get_time()

    def working_stt_cb(self, msg):
        self.working_status = msg.data

    def distance_move_record_cb(self, msg):
        self.odom_moved_record = msg.data

    def lift_record_cb(self, msg):
        self.lift_record = msg.data

    def init_server_cb(self, msg):
        if self.server_config != None:  # Retry init server
            self.init_server()
            if self.agv_db_object_id != None:
                rospy.logwarn("Re-init server")

    def mission_status_cb(self, msg):
        data_dict = json.loads(msg.data)
        self.current_action_type = data_dict["current_action_type"]

    def moving_control_action_cb(self, msg):
        data_dict = json.loads(msg.goal.data)
        moving_control_goal = data_dict["waypoints"]
        # New format list
        new_format_goals = []

        # Iterate over each waypoint
        for waypoint in moving_control_goal:
            position = waypoint["position"]["position"]  # Extract position
            orientation = waypoint["position"][
                "orientation"
            ]  # Extract orientation

            # Create new formatted goal
            new_goal = {
                "position": {
                    "x": position["x"],
                    "y": position["y"],
                    "z": 0.0,  # Assuming you want z to be 0.0
                },
                "orientation": {
                    "x": orientation["x"],
                    "y": orientation["y"],
                    "z": orientation[
                        "z"
                    ],  # Assuming you want to keep the original z
                    "w": orientation[
                        "w"
                    ],  # Assuming you want to keep the original w
                },
            }

            # Append to new format list
            new_format_goals.append(new_goal)
        data = {
            "poses": new_format_goals,
            "map": self.map_server_name,
            "agv": self.server_config["agv_name"],
        }
        # data = {"poses": new_format_goals, "map": "Tendo_1F"}  # Thêm trường "poses"
        rospy.logerr("Data being sent:\n%s", json.dumps(data, indent=2))
        self.update_path_display_in_app(data)

    def delete_existing_objects(self):
        try:
            # Query for objects with the same "agv" value
            query_url = self.api_url + "classes/traffic_path"
            query_params = {
                "where": json.dumps({"agv": self.server_config["agv_name"]})
            }

            # Send a GET request to find matching objects
            response = requests.get(
                query_url, params=query_params, headers=self.header
            )

            if response.status_code == 200:
                objects = response.json().get("results", [])

                # Loop through the objects and delete each one
                for obj in objects:
                    object_id = obj["objectId"]
                    delete_url = (
                        f"{self.api_url}classes/traffic_path/{object_id}"
                    )

                    delete_response = requests.delete(
                        delete_url, headers=self.header
                    )

                    if delete_response.status_code == 200:
                        print(
                            f"Successfully deleted object with ID: {object_id}"
                        )
                    else:
                        print(
                            f"Failed to delete object with ID: {object_id}, Status Code: {delete_response.status_code}"
                        )
            else:
                print(
                    f"Failed to query objects, Status Code: {response.status_code}"
                )
                print("Response:", response.text)

        except requests.exceptions.RequestException as e:
            print("An error occurred while querying or deleting objects.")
            print(e)

    def current_moving_control_goal_cb(self, msg):
        self.moving_control_goal = msg.pose

    def followline_action_cb(self, msg):
        self.followline_goal = msg.goal.target_pose.pose

    def ampe_motor_left_cb(self, msg):
        if msg.data > self.motor_config["max_ampe"]:
            self.motor_left_over_ampe = True
        else:
            self.motor_left_over_ampe = False

    def ampe_motor_right_cb(self, msg):
        if msg.data > self.motor_config["max_ampe"]:
            self.motor_right_over_ampe = True
        else:
            self.motor_right_over_ampe = False

    def left_vel_error_cb(self, msg):
        if msg.data == 1:
            self.control_motor_left_error = True
        else:
            self.control_motor_left_error = False

    def right_vel_error_cb(self, msg):
        if msg.data == 1:
            self.control_motor_right_error = True
        else:
            self.control_motor_right_error = False

    def request_mode_cb(self, msg):
        self.robot_mode_req = RobotMode[msg.data]
        rospy.loginfo("Request mode: {}".format(self.robot_mode_req.toString()))
        if self.robot_mode_req == RobotMode.MAPPING:
            self.mapping_cnt += 1
            print_debug("Mapping request time: {}".format(self.mapping_cnt))

    def request_mode_handle(self, req):
        try:
            rospy.loginfo("Request mode service handle: {}".format(req.request))
            self.request_mode_cb(StringStamped(data=req.request))
            return StringServiceResponse("SUCCESS")
        except KeyError:
            rospy.logerr("Invalid robot mode requested: {}".format(req.request))
            return StringServiceResponse("FAILED")

    def request_savemap_cb(self, msg):
        if not self.option_config["use_slam"]:
            return
        rospy.loginfo("Request save map: {}".format(msg.data))
        try:
            self.map_requesting = json.loads(msg.data)["name"]
            map_name = self.current_map_cfg.map_dir + self.map_requesting
            msg.data = map_name
            self.pub_map_save_req.publish(msg)
            self.save_map_request = True
        except Exception as e:
            rospy.logerr("request_savemap_cb: {}".format(e))

    def request_savemap_handle(self, req):
        try:
            rospy.loginfo(
                "Request save map service handle: {}".format(req.request)
            )
            self.request_savemap_cb(StringStamped(data=req.request))
            return StringServiceResponse("SUCCESS")
        except Exception as e:
            rospy.logerr("request_savemap_handle error: {}".format(e))
            return StringServiceResponse("FAILED")

    def request_temp_savemap_cb(self, msg):
        if not self.option_config["use_slam"]:
            return
        rospy.loginfo("Temp save map request: {}".format(msg.data))
        try:
            self.map_requesting = json.loads(msg.data)["name"]
            map_name = self.current_map_cfg.map_dir + self.map_requesting
            msg.data = map_name
            self.pub_temp_map_save_req.publish(msg)
        except Exception as e:
            rospy.logerr("request_temp_savemap_cb: {}".format(e))

    def request_temp_savemap_handle(self, req):
        try:
            rospy.loginfo(
                "Request temp save map service handle: {}".format(req.request)
            )
            self.request_temp_savemap_cb(StringStamped(data=req.request))
            return StringServiceResponse("SUCCESS")
        except Exception as e:
            rospy.logerr("request_temp_savemap_handle error: {}".format(e))
            return StringServiceResponse("FAILED")

    def request_change_map_cb(self, msg):
        if not self.option_config["use_slam"]:
            return
        rospy.loginfo("Request change map: {}".format(msg.data))
        file_check = self.current_map_cfg.map_dir + msg.data
        if self.check_map(file_check):
            self.main_state = MainState.INIT_LOAD_MAP
            self.current_map_cfg.map_file = msg.data
            self.dump_config()
        else:
            rospy.logerr(
                "The requesting map is not exist: {}".format(file_check)
            )

    def request_change_map_handle(self, req):
        try:
            rospy.loginfo(
                "Request change map service handle: {}".format(req.request)
            )
            self.request_change_map_cb(StringStamped(data=req.request))
            return StringServiceResponse("SUCCESS")
        except Exception as e:
            rospy.logerr("request_change_map_handle error: {}".format(e))
            return StringServiceResponse("FAILED")

    def request_load_static_map_cb(self, msg):
        if not self.option_config["use_slam"]:
            return
        rospy.loginfo("Request load static map: {}".format(msg.data))
        file_check = self.current_map_cfg.map_dir + msg.data
        if self.check_map(file_check):
            self.main_state = MainState.INIT_LOAD_STATIC_MAP
            self.current_map_cfg.map_file = msg.data
            self.dump_config()
        else:
            rospy.logerr(
                "The requesting map is not exist: {}".format(file_check)
            )

    def request_load_static_map_handle(self, req):
        try:
            rospy.loginfo(
                "Request load static map service handle: {}".format(req.request)
            )
            self.request_load_static_map_cb(StringStamped(data=req.request))
            return StringServiceResponse("SUCCESS")
        except Exception as e:
            rospy.logerr("request_load_static_map_handle error: {}".format(e))
            return StringServiceResponse("FAILED")

    def set_status_to_waitting_init_pose_cb(self, msg):
        self.relocalization = True

    def test_connection_cb(self, msg):
        rospy.loginfo("Test connection topic received")
        msg.stamp = rospy.Time.now()
        self.test_connection_res_pub.publish(msg)

    def rename_pbstream_handle(self, req):
        try:
            # Expecting req.request to be a JSON string: '{"old_name": "abc", "new_name": "def"}'
            params = json.loads(req.request)
            old_name = params.get("old_name")
            new_name = params.get("new_name")
            old_path = self.current_map_cfg.map_dir + old_name + ".pbstream"
            new_path = self.current_map_cfg.map_dir + new_name + ".pbstream"
            if os.path.exists(old_path):
                os.rename(old_path, new_path)
                rospy.loginfo("Renamed {} to {}".format(old_path, new_path))
                return StringServiceResponse("SUCCESS")
            else:
                rospy.logerr("File does not exist: {}".format(old_path))
                return StringServiceResponse("FAILED")
        except Exception as e:
            rospy.logerr("Rename pbstream error: {}".format(e))
            return StringServiceResponse("FAILED")

    def set_status_to_waitting_init_pose_cb(self, msg):
        self.relocalization = True

    def save_editing_map_handle(self, req):
        self.pub_save_editing_map.publish(
            StringStamped(stamp=rospy.Time.now(), data=req.request)
        )
        rospy.loginfo(
            "Save editing map service request: {}".format(req.request)
        )
        return StringServiceResponse("OK")

    def request_save_static_map_srv(self, req):
        try:
            map_file = req.request
            write_srv = rospy.ServiceProxy("/write_state", WriteState)
            write_req = WriteStateRequest()
            write_req.filename = (
                self.current_map_cfg.map_dir + map_file + ".pbstream"
            )
            rospy.loginfo(write_req.filename)
            rospy.loginfo(
                "Calling /write_state to save: %s", write_req.filename
            )
            write_srv(write_req)
            rospy.loginfo("Saved pbstream.")
            return StringServiceResponse("SUCCESS")
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call /write_state: %s", e)
            return StringServiceResponse("FAILED")

    def request_reload_map_cb(self, msg):
        if not self.option_config["use_slam"]:
            return
        rospy.loginfo("Request reload map: {}".format(msg.data))
        file_check = self.current_map_cfg.map_dir + msg.data
        if self.check_map(file_check):
            self.main_state = MainState.INIT_LOAD_MAP
        else:
            rospy.logerr(
                "The requesting map is not exist: {}".format(file_check)
            )

    def request_reload_map_handle(self, req):
        try:
            rospy.loginfo(
                "Request reload map service handle: {}".format(req.request)
            )
            self.request_reload_map_cb(StringStamped(data=req.request))
            return StringServiceResponse("SUCCESS")
        except Exception as e:
            rospy.logerr("request_reload_map_handle error: {}".format(e))
            return StringServiceResponse("FAILED")

    def request_run_stop_cb(self, msg):
        # TODO: Change in app
        # self.pub_mission_run_pause.publish(msg)

        if msg.data == "STOP":
            msg.data = "PAUSE"
        if msg.data == "PAUSE":
            self.pause_detail_status = "Pause by request pause"
            self.pause_robot()
            rospy.logwarn("Pause by request pause")
        elif msg.data == "RUN":
            self.resume_running()
            rospy.logwarn("resume by request resume")

    def handle_run_pause(self, req):
        self.pub_mission_run_pause.publish(
            StringStamped(stamp=rospy.Time.now(), data=req.request)
        )
        return StringServiceResponse("OK")

    def handle_stop_mission(self, req):
        self.reset_get_mission = True
        self.mission_id_reset_request = self.current_mission_id_running
        self.reset_mission_request_type = req.request
        rospy.loginfo("Stop all mission when hold PAUSE button")
        rospy.loginfo("command request by hmi: {}".format(req.request))
        self.pub_request_start_mission.publish(
            StringStamped(stamp=rospy.Time.now(), data="STOP")
        )
        return StringServiceResponse("OK")

    def trigger_mission_from_server_cb(self, msg):
        self.mission_from_server_req = True

    def initialpose_cb(self, msg):
        self.init_pose_received = True

    def request_start_mission_cb(self, msg):
        if msg.data == "START":
            if self.robot_mode == RobotMode.AUTO:
                self.pub_mission_control.publish(msg)
        else:
            self.reset_get_mission = True
            self.pub_mission_control.publish(msg)

    def standard_io_cb(self, msg):
        try:
            self.std_io_status = json.loads(msg.data)
            if "bumper" in self.std_io_status:
                self.bumper = self.std_io_status["bumper"]
            if "emg_button" in self.std_io_status:
                self.emg_button = self.std_io_status["emg_button"]
            if self.update_status_button:
                self.update_status_button = False
                if "start_1_button" in self.std_io_status:
                    self.start_1_button = self.std_io_status["start_1_button"]
                if "start_2_button" in self.std_io_status:
                    self.start_2_button = self.std_io_status["start_2_button"]
                if "stop_1_button" in self.std_io_status:
                    self.stop_1_button = self.std_io_status["stop_1_button"]
                if "stop_2_button" in self.std_io_status:
                    self.stop_2_button = self.std_io_status["stop_2_button"]
                if "auto_manual_sw" in self.std_io_status:
                    self.auto_manual_sw = self.std_io_status["auto_manual_sw"]
                if "motor_enable_sw" in self.std_io_status:
                    self.motor_enable_sw = self.std_io_status["motor_enable_sw"]
        except:
            rospy.logerr("standard_io error")
        self.last_std_io_msg = rospy.get_time()

    def odom_cb(self, msg):
        self.last_odom_msg = rospy.get_time()

    def arduino_error_cb(self, msg):
        self.arduino_error_msg = msg

    def reset_error_cb(self, msg):
        self.reset_error_request = True

    def fake_battery_cb(self, msg):
        self.battery_percent = msg.data
        self.last_battery_fake = rospy.get_time()

    def real_battery_cb(self, msg):
        # TODO: Get lowest value ultil reset
        if rospy.get_time() - self.last_battery_fake > 1.0:
            try:
                # self.battery_percent = int(msg.data)
                self.battery_percent = 100
            except Exception as e:
                rospy.logerr_throttle(10, e)

    def battery_ampe_cb(self, msg):
        self.battery_ampe = msg.data

    def battery_voltage_cb(self, msg):
        self.battery_voltage = msg.data

    def led_control_alive_cb(self, msg):
        self.last_led_alive = rospy.get_time()

    def safety_status_cb(self, msg):
        self.last_safety_time = rospy.get_time()
        safety_fields = list(msg.fields)  # [1, 2, 3, ...]
        if len(safety_fields) > 0:
            if safety_fields[0] == 1:
                self.is_safety = True
            else:
                self.is_safety = False
        else:
            self.is_safety = False

    def update_feedback(self):
        msg = StringStamped()
        msg.stamp = rospy.Time.now()
        # Publish robot_mode
        msg.data = self.robot_mode.toString()
        self.pub_robot_mode.publish(msg)
        # Publish robot_status
        self.status_dict["working_status"] = self.working_status
        self.status_dict["status"] = self.robot_status.toString()
        self.status_dict["state"] = self.main_state.toString()
        self.status_dict["mode"] = self.robot_mode.toString()
        self.status_dict["sound"] = (
            "OFF" if self.sound_status == "" else self.sound_status
        )
        self.status_dict["detail"] = self.detail_status
        self.status_dict["error_code"] = self.error_code  # TODO: Display in app
        self.status_dict["special_matching_state"] = self.special_matching_state
        msg.data = json.dumps(self.status_dict, indent=2)
        self.pub_robot_status.publish(msg)
        # Current map
        msg.data = self.current_map_cfg.map_file
        if self.option_config["use_slam"]:
            self.pub_current_map.publish(msg)
        msg.data = self.current_server_map
        self.pub_current_server_map.publish(msg)
        msg = Int8Stamped()
        msg.data = self.battery_percent
        if msg.data > 100:
            msg.data = 100
        if msg.data < 0:
            msg.data = 0
        msg.stamp = rospy.Time.now()
        self.pub_battery.publish(msg)

    def update_error_code(
        self, module_disconnected="", msg="", error_code_remap=""
    ):
        if module_disconnected != "":
            self.error_code = '"{}" module disconnected'.format(
                module_disconnected
            )
        elif msg != "":
            self.error_code = msg

        if error_code_remap in self.error_code_remap_config:
            self.error_code_number = self.error_code_remap_config[
                error_code_remap
            ]

    def clear_error_code(self):
        self.error_code = ""

    def moving_control_module_status_cb(self, msg):
        try:
            self.moving_control_status_dict = json.loads(msg.data)
        except Exception as e:
            rospy.logerr("moving_control_module_status_cb: {}".format(e))

    """
    ##        #######   #######  ########
    ##       ##     ## ##     ## ##     ##
    ##       ##     ## ##     ## ##     ##
    ##       ##     ## ##     ## ########
    ##       ##     ## ##     ## ##
    ##       ##     ## ##     ## ##
    ########  #######   #######  ##
    """

    def check_condition_reset_err(self):
        if self.robot_mode_req == RobotMode.MAPPING:
            self.robot_mode_req = RobotMode.NONE
            self.main_state = MainState.INIT_MAPPING
        if self.slam_state == SlamState.LOCALIZING:
            self.main_state = MainState.MANUAL
            self.robot_mode_req = RobotMode.NONE
        if self.slam_state == SlamState.SWITCH_LOCALIZING:
            self.main_state = MainState.LOADING_MAP
        if self.slam_state == SlamState.SWITCH_MAPPING:
            self.main_state = MainState.WAIT_MAPPING
        # if self.slam_state == SlamState.MAPPING:
        #     self.main_state = MainState.MAPPING
        #     self.robot_mode_req = RobotMode.NONE
        if self.robot_mode_req == RobotMode.MANUAL:
            self.robot_mode_req = RobotMode.NONE
            self.main_state = MainState.INIT_LOAD_MAP

    def loop(self):
        rospy.sleep(2.0)
        begin_load_map = rospy.get_time()
        last_pub_time = rospy.get_time()
        begin_hold_stop_button = rospy.get_time()
        begin_load_map_error = rospy.get_time()
        begin_check_mode_change = rospy.get_time()
        begin_check_status_change = rospy.get_time()
        last_sound_time = rospy.get_time()
        begin_loop = rospy.get_time()
        trigger_button_time = rospy.get_time()
        trigger_button = ""
        first_check_mode = True
        pushed_stop_mission = False
        odom_ready = True
        prev_led_stt = None
        prev_sound_stt = None
        prev_auto_man_switch = SWITCH_MANUAL_MODE
        last_auto_man_switch = SWITCH_MANUAL_MODE
        current_safety_job = ""
        pause_by_error = False
        last_robot_status = RobotStatus.NONE
        last_robot_mode = RobotMode.NONE
        last_start_1_button = SENSOR_DEACTIVATE
        last_start_2_button = SENSOR_DEACTIVATE
        last_stop_1_button = SENSOR_DEACTIVATE
        last_stop_2_button = SENSOR_DEACTIVATE
        pre_start_1_button = SENSOR_DEACTIVATE
        pre_start_2_button = SENSOR_DEACTIVATE
        pre_stop_1_button = SENSOR_DEACTIVATE
        pre_stop_2_button = SENSOR_DEACTIVATE
        time_update_button = rospy.get_time()
        last_module_disconnected = ""
        r = rospy.Rate(10.0)
        success_switch_tf_qr_first_time = False
        while not rospy.is_shutdown():
            if self.option_config["use_qr"]:
                # switch to tf qr code first time
                if not success_switch_tf_qr_first_time:
                    rospy.logwarn("switch use tf qr code first time")
                    success_switch_tf_qr_first_time = self.switch_control_tf(
                        USE_TF_LIDAR
                    )
            self.update_status_button = True
            if self.main_state != self.prev_state:
                rospy.loginfo(
                    "Main state: {} -> {}".format(
                        self.prev_state.toString(), self.main_state.toString()
                    )
                )
                self.prev_state = self.main_state

            self.detail_status = DetailStatus.ALL_OK.toString()
            self.detail_status_addition = ""
            self.detail_status_not_dispay_hmi = ""

            # State
            if "state" in self.slam_manager_module.module_status_dict:
                self.slam_state = SlamState[
                    self.slam_manager_module.module_status_dict["state"]
                ]
            # Status
            if self.mission_manager_module != None:
                if "status" in self.mission_manager_module.module_status_dict:
                    self.mission_status = ModuleStatus[
                        self.mission_manager_module.module_status_dict["status"]
                    ]
                # Running mission id
                if (
                    "running_mission_id"
                    in self.mission_manager_module.module_status_dict
                ):
                    self.running_mission_id = (
                        self.mission_manager_module.module_status_dict[
                            "running_mission_id"
                        ]
                    )
                if (
                    "last_done_mission_id"
                    in self.mission_manager_module.module_status_dict
                ):
                    self.last_done_mission_id = (
                        self.mission_manager_module.module_status_dict[
                            "last_done_mission_id"
                        ]
                    )
            # Safety job
            try:
                safety_disabled = False
                if (
                    "use_safety" in self.option_config
                    and self.option_config["use_safety"]
                ):
                    current_safety_job = json.loads(
                        rospy.get_param("/scan_safety_node_ng/job")
                    )
                    if len(current_safety_job["safety"]) == 0:
                        safety_disabled = True
            except:
                pass

            # Update current position
            temp_pose = None
            temp_pose = lockup_pose(
                self.tf_listener, self.map_frame, self.robot_frame
            )
            # print_info(temp_pose)
            if temp_pose != None:
                self.current_pose = temp_pose

            """
             ######  ########    ###    ######## ########
            ##    ##    ##      ## ##      ##    ##
            ##          ##     ##   ##     ##    ##
             ######     ##    ##     ##    ##    ######
                  ##    ##    #########    ##    ##
            ##    ##    ##    ##     ##    ##    ##
             ######     ##    ##     ##    ##    ########
            """

            # Switch mode
            # State: INIT
            if self.main_state == MainState.INIT:
                # slam_manager is ready
                if self.slam_state == SlamState.MAPPING:
                    self.main_state = MainState.MAPPING
                    self.robot_mode_req = (
                        RobotMode.NONE
                    )  # Clear initial value if mode is set
                    continue
                elif self.slam_state == SlamState.LOCALIZING:
                    if (
                        self.auto_manual_sw == SENSOR_DEACTIVATE
                        and "use_auto_man_sw" in self.option_config
                        and self.option_config["use_auto_man_sw"]
                    ):
                        self.main_state = MainState.MANUAL
                    else:
                        self.main_state = MainState.AUTO
                    continue
                elif self.slam_state == SlamState.SWITCH_LOCALIZING:
                    if (
                        self.auto_manual_sw == SENSOR_DEACTIVATE
                        and "use_auto_man_sw" in self.option_config
                        and self.option_config["use_auto_man_sw"]
                    ):
                        self.main_state = MainState.WAIT_MANUAL
                    else:
                        self.main_state = MainState.WAIT_AUTO
                    continue
                elif self.slam_state == SlamState.SWITCH_MAPPING:
                    self.main_state = MainState.WAIT_MAPPING
                    continue
                # Check static map
                if self.slam_state == SlamState.STATIC_MAP_LOADED:
                    self.main_state = MainState.LOADED_STATIC_MAP
                    self.robot_mode_req = (
                        RobotMode.NONE
                    )  # Clear initial value if mode is set
                    continue
                # slam_manager is not ready
                elif self.robot_mode_req == RobotMode.MANUAL:
                    self.robot_mode_req = RobotMode.NONE
                    self.pause_robot()
                    self.main_state = MainState.WAIT_MANUAL
                    if self.slam_state != SlamState.LOCALIZING:
                        self.main_state = MainState.INIT_LOAD_MAP
                elif rospy.get_time() - begin_loop > 2.0:
                    if self.slam_state != SlamState.LOCALIZING:
                        self.main_state = MainState.INIT_LOAD_MAP
            # State: WAIT_MAPPING
            elif self.main_state == MainState.WAIT_MAPPING:
                self.robot_mode = RobotMode.WAIT_RESPOND
                if self.slam_state == SlamState.MAPPING:
                    self.main_state = MainState.MAPPING
                elif self.slam_state == SlamState.ERROR_MAPPING:
                    self.main_state = MainState.MAPPING_ERROR
                # TODO: Handle when fake_slam but push "Create map"
                # elif self.slam_state == SlamState.LOCALIZING:
                #     self.main_state = MainState.MAPPING_ERROR
                # Timeout when slam_manager not run

                if (
                    rospy.get_time()
                    - self.slam_manager_module.last_module_status
                    > 2.0
                ):  # Sometime > 1.0 because get_node_list() take 1 sec
                    self.main_state = MainState.LOAD_MAP_ERROR
            # State: WAIT_MANUAL
            elif self.main_state == MainState.WAIT_MANUAL:
                # TODO: Handle /mission_status timeout
                self.robot_mode = RobotMode.WAIT_RESPOND
                self.main_state = MainState.MANUAL
            # State: WAIT_AUTO
            elif self.main_state == MainState.WAIT_AUTO:
                self.robot_mode = RobotMode.WAIT_RESPOND
                if self.slam_state == SlamState.LOCALIZING:
                    self.main_state = MainState.AUTO
                else:
                    print_error("Not yet switch to AUTO")
            # State: MANUAL
            elif self.main_state == MainState.MANUAL:
                self.robot_mode = RobotMode.MANUAL
                if self.init_pose_received:
                    self.init_pose_received = False
                    self.set_pose_init(True)
                if self.robot_mode_req == RobotMode.AUTO or (
                    self.auto_manual_sw == SWITCH_AUTO_MODE
                    # and last_auto_man_switch == SWITCH_MANUAL_MODE
                    and not self.simulation
                ):
                    self.robot_mode_req = RobotMode.NONE
                    self.main_state = MainState.WAIT_AUTO
                    continue
                elif self.robot_mode_req == RobotMode.MAPPING:
                    self.robot_mode_req = RobotMode.NONE
                    self.main_state = MainState.INIT_MAPPING
                    continue
                if self.slam_state == SlamState.ERROR_LOCALIZING and odom_ready:
                    self.main_state = MainState.LOAD_MAP_ERROR
                    continue
                if self.slam_state == SlamState.STATIC_MAP_LOADED:
                    self.main_state = MainState.LOADED_STATIC_MAP
                self.check_condition_reset_err()
            # State: AUTO
            elif self.main_state == MainState.AUTO:
                self.robot_mode = RobotMode.AUTO
                if self.init_pose_received:
                    self.init_pose_received = False
                    self.set_pose_init(True)
                if self.robot_mode_req == RobotMode.MANUAL or (
                    self.auto_manual_sw == SENSOR_DEACTIVATE
                    and not self.simulation
                ):
                    self.robot_mode_req = RobotMode.NONE
                    # Stop robot
                    # TODO: add pause
                    if last_robot_status != RobotStatus.ERROR:
                        # if self.current_action_type == "move":
                        #     self.need_to_reset_by_qr_code = True
                        self.pause_robot()
                        self.pause_detail_status = (
                            "Pause by switch mode to MANUAL"
                        )
                    self.main_state = MainState.WAIT_MANUAL
                elif self.robot_mode_req == RobotMode.MAPPING:
                    self.robot_mode_req = RobotMode.NONE
                    self.main_state = MainState.INIT_MAPPING
                if self.slam_state == SlamState.ERROR_LOCALIZING and odom_ready:
                    self.main_state = MainState.LOAD_MAP_ERROR
                if self.slam_state == SlamState.STATIC_MAP_LOADED:
                    self.main_state = MainState.LOADED_STATIC_MAP
            # State: MAPPING
            elif self.main_state == MainState.MAPPING:
                self.robot_mode = RobotMode.MAPPING
                if self.robot_mode_req == RobotMode.MANUAL:
                    print_debug("MAPPING and request MANUAL")
                    self.robot_mode_req = RobotMode.NONE
                    self.main_state = MainState.INIT_LOAD_MAP
                    continue
                if (
                    self.save_map_request
                ):  # When stop MAPPING and force switch to MANUAL

                    self.robot_mode = RobotMode.WAIT_RESPOND
                    if (
                        self.current_map_cfg.map_dir + self.map_requesting
                        == self.slam_manager_module.module_status_dict[
                            "current_map"
                        ]
                    ):
                        rospy.loginfo("MAPPING stop -> load new map")
                        self.save_map_request = False
                        self.current_map_cfg.map_file = self.map_requesting
                        self.dump_config()
                        self.main_state = MainState.INIT_LOAD_MAP
                        continue
                if self.slam_state == SlamState.STATIC_MAP_LOADED:
                    self.main_state = MainState.LOADED_STATIC_MAP
                # Khi đang MAPPING mà /slam_manager disconnected, lúc kết nối lại cần reset
                self.check_condition_reset_err()
            # State: INIT_LOAD_MAP
            elif self.main_state == MainState.INIT_LOAD_MAP:
                map_file_check = (
                    self.current_map_cfg.map_dir + self.current_map_cfg.map_file
                )
                print_debug("Check map: {}".format(map_file_check))
                if self.check_map(map_file_check):
                    self.load_map_cnt += 1
                    print_debug(
                        "Load map request time: {}".format(self.load_map_cnt)
                    )
                    self.load_map(self.current_map_cfg.map_file)
                    begin_load_map = rospy.get_time()
                    self.main_state = MainState.LOADING_MAP
                else:
                    self.main_state = MainState.LOAD_MAP_ERROR
            # State: INIT_LOAD_STATIC_MAP
            elif self.main_state == MainState.INIT_LOAD_STATIC_MAP:
                map_file_check = (
                    self.current_map_cfg.map_dir + self.current_map_cfg.map_file
                )
                self.load_static_map(self.current_map_cfg.map_file)
                self.main_state = MainState.LOADING_STATIC_MAP
            # State: LOADING_STATIC_MAP
            elif self.main_state == MainState.LOADING_STATIC_MAP:
                self.robot_mode = RobotMode.WAIT_RESPOND
                if self.slam_state == SlamState.STATIC_MAP_LOADED:
                    self.main_state = MainState.LOADED_STATIC_MAP
                if self.slam_state == SlamState.ERROR_LOAD_STATIC_MAP:
                    self.main_state = MainState.LOAD_STATIC_MAP_ERROR
            # State: LOAD_STATIC_MAP_ERROR
            elif self.main_state == MainState.LOAD_STATIC_MAP_ERROR:
                # TODO: Load prev map, load other map
                self.robot_mode = RobotMode.MANUAL
                self.detail_status = (
                    DetailStatus.LOAD_STATIC_MAP_ERROR.toString()
                )
            # State: LOADED_STATIC_MAP
            elif self.main_state == MainState.LOADED_STATIC_MAP:
                self.robot_mode = RobotMode.EDITING_MAP
                self.detail_status = DetailStatus.LOADED_STATIC_MAP.toString()
                if self.robot_mode_req == RobotMode.MAPPING:
                    self.robot_mode_req = RobotMode.NONE
                    self.main_state = MainState.INIT_MAPPING
                if self.robot_mode_req == RobotMode.MANUAL:
                    self.robot_mode_req = RobotMode.NONE
                    self.pause_robot()
                    self.main_state = MainState.WAIT_MANUAL
            # State: INIT_MAPPING
            elif self.main_state == MainState.INIT_MAPPING:
                self.pub_mapping_req.publish(StringStamped())
                self.main_state = MainState.WAIT_MAPPING
            # State: LOADING_MAP
            elif self.main_state == MainState.LOADING_MAP:
                self.robot_mode = RobotMode.WAIT_RESPOND
                if self.slam_state == SlamState.LOCALIZING:
                    if rospy.get_time() - begin_load_map > 2.0:
                        if (
                            first_check_mode
                            and self.auto_manual_sw == SWITCH_AUTO_MODE
                        ):
                            self.main_state = MainState.AUTO
                        else:
                            self.main_state = MainState.MANUAL
                        self.robot_mode_req = RobotMode.NONE
                        first_check_mode = False
                if self.slam_state == SlamState.ERROR_LOCALIZING:
                    self.main_state = MainState.LOAD_MAP_ERROR
                # Timeout when slam_manager not run
                if (
                    rospy.get_time()
                    - self.slam_manager_module.last_module_status
                    > 2.0
                ):  # Sometime > 1.0 because get_node_list() take 1 sec
                    self.main_state = MainState.LOAD_MAP_ERROR
            # State: LOAD_MAP_ERROR
            elif self.main_state == MainState.LOAD_MAP_ERROR:
                # TODO: Load prev map, load other map
                self.robot_mode = RobotMode.MANUAL
                self.detail_status = DetailStatus.LOAD_MAP_ERROR.toString()
                self.check_condition_reset_err()
                # Auto reset
                auto_reset_load_map = False
                if rospy.get_time() - begin_load_map_error > 30.0:
                    auto_reset_load_map = True
                    rospy.logwarn("Auto reset LOAD_MAP_ERROR")
                if self.reset_error_request or auto_reset_load_map:
                    self.reset_error_request = False
                    self.main_state = MainState.INIT_LOAD_MAP
            # State: MAPPING_ERROR
            elif self.main_state == MainState.MAPPING_ERROR:
                self.robot_mode = RobotMode.MAPPING
                self.detail_status = DetailStatus.MAPPING_ERROR.toString()
                self.check_condition_reset_err()
                if self.reset_error_request:
                    self.reset_error_request = False
                    self.main_state = MainState.INIT_MAPPING

            # Set timeout
            if (
                self.main_state == MainState.LOAD_MAP_ERROR
                and self.prev_state != MainState.LOAD_MAP_ERROR
            ):
                begin_load_map_error = rospy.get_time()

            """
             ######  ########    ###    ######## ##     ##  ######
            ##    ##    ##      ## ##      ##    ##     ## ##    ##
            ##          ##     ##   ##     ##    ##     ## ##
             ######     ##    ##     ##    ##    ##     ##  ######
                  ##    ##    #########    ##    ##     ##       ##
            ##    ##    ##    ##     ##    ##    ##     ## ##    ##
             ######     ##    ##     ##    ##     #######   ######
            """
            # Check module check disconnect
            module_disconnected = ""
            special_matching_error = ""
            special_matching_state = ""
            special_error_led = ""
            special_error_sound = ""
            special_state_led = ""
            special_state_sound = ""
            module_error_code = ""
            pause_if_error = True
            self.reset_when_mission_error = True
            display_error = True
            self.error_code_number = -1
            for m in self.module_list:
                if not m.module_alive and (
                    m.handle_when_sim or not self.simulation
                ):
                    self.update_error_code(module_disconnected=m.display_name)
                    module_disconnected = m.display_name
                    display_error = m.display_error
                    self.error_code_number = m.error_code_number_default
                    if not display_error:
                        self.detail_status_addition = m.display_name
                    break
            for (
                m
            ) in (
                self.module_list
            ):  # Không để trong cùng vòng lặp đc vì nếu module trước lỗi mà module sau disconnect
                if m.special_matching_error != "":
                    special_matching_error = m.special_matching_error
                    special_error_led = m.special_error_led
                    special_error_sound = m.special_error_sound
                    self.reset_when_mission_error = m.reset_when_mission_error
                    self.update_error_code(msg=special_matching_error)
                    break
            for m in self.module_list:
                if m.error_code != "":
                    module_error_code = m.error_code
                    self.error_code_number = m.error_code_number
                    pause_if_error = m.pause_if_error
                    break
            for m in self.module_list:
                if m.special_matching_state != "":
                    special_matching_state = m.special_matching_state
                    special_state_led = m.special_state_led
                    special_state_sound = m.special_state_sound
                    break
            self.special_matching_state = special_matching_state
            if module_disconnected == "" and special_matching_error == "":
                self.clear_error_code()

            # Update robot_status and detail_status
            if (
                rospy.get_time() - self.last_std_io_msg > 1.0
                and not self.simulation
            ):
                self.robot_status = RobotStatus.ERROR
                self.detail_status = DetailStatus.FATAL_ERROR.toString()
                self.update_error_code(
                    msg=DetailStatus.IO_BOARD_DISCONNECT.toString()
                )
                # rospy.logwarn("IO_BOARD_DISCONNECT")
                odom_ready = False
            elif (
                rospy.Time.now().to_sec()
                - self.arduino_error_msg.stamp.to_sec()
                < 0.5
                and not self.simulation
            ):
                self.arduino_error_cnt += 1
                if self.arduino_error_cnt > 30:
                    self.robot_status = RobotStatus.ERROR
                    self.detail_status = DetailStatus.FATAL_ERROR.toString()
                    self.update_error_code(
                        msg=DetailStatus.IO_BOARD_ERROR.toString()
                    )
                    # rospy.logwarn("IO_BOARD_ERROR")
                    odom_ready = False
            elif (
                self.emg_button == SENSOR_DEACTIVATE
                or (
                    self.bumper == SENSOR_DEACTIVATE
                    and self.option_config["use_bumper"]
                )
            ) and not self.simulation:
                self.robot_status = RobotStatus.EMG
                # TODO: update error code number here
                if (
                    last_robot_status != self.robot_status
                    and last_robot_status != RobotStatus.ERROR
                ):
                    self.pause_robot()
                    if self.emg_button == SENSOR_DEACTIVATE:
                        self.pause_detail_status = "Pause by push EMG button"
                    else:
                        self.pause_detail_status = (
                            "Pause by detect bumper collision"
                        )
            elif (
                self.motor_enable_sw == SENSOR_DEACTIVATE
                and not self.simulation
                and "use_motor_release" in self.option_config
                and self.option_config["use_motor_release"]
            ):
                self.robot_status = RobotStatus.ERROR
                self.detail_status = DetailStatus.MOTOR_OFF.toString()
                self.update_error_code(
                    msg=self.detail_status, error_code_remap=self.detail_status
                )
                if self.current_action_type == "move":
                    self.need_to_reset_by_qr_code = False
                # TODO: update error code number here
            elif module_disconnected != "":
                # TOCHECK: module_disconnected but special_matching_error display
                if display_error:
                    # TODO: update error code number here
                    self.robot_status = RobotStatus.ERROR
                    self.detail_status = (
                        DetailStatus.FATAL_ERROR.toString()
                    )  # Will be overwritten by update_feedback()
                else:
                    if (
                        self.main_state == MainState.LOAD_MAP_ERROR
                        or self.main_state == MainState.MAPPING_ERROR
                    ):
                        self.robot_status = RobotStatus.ERROR
                        # self.detail_status = DetailStatus.LOAD_MAP_ERROR.toString()
                        self.update_error_code(msg=self.detail_status)
                    elif special_matching_error != "":
                        self.robot_status = RobotStatus.ERROR
                        self.detail_status = special_matching_error
                    elif module_error_code != "":
                        self.robot_status = RobotStatus.ERROR
                        self.detail_status = (
                            DetailStatus.GENERAL_ERROR.toString()
                        )
                        self.update_error_code(msg=module_error_code)
                    elif (
                        self.battery_percent
                        < self.battery_config["empty_threshold"]
                        and special_matching_state
                        != DetailStatus.AUTO_CHARGING.toString()
                        and special_matching_state
                        != DetailStatus.MANUAL_CHARGING.toString()
                    ):
                        # TODO: update error code number here
                        self.robot_status = RobotStatus.ERROR
                        self.detail_status = (
                            DetailStatus.BATTERY_EMPTY.toString()
                        )
                        self.update_error_code(
                            msg=DetailStatus.BATTERY_EMPTY.toString(),
                            error_code_remap=self.detail_status,
                        )
                    elif (
                        self.motor_left_over_ampe or self.motor_right_over_ampe
                    ):
                        # TODO: update error code number here
                        self.robot_status = RobotStatus.ERROR
                        self.detail_status = (
                            DetailStatus.MOTOR_OVER_CURRENT.toString()
                        )
                        self.update_error_code(
                            msg=DetailStatus.MOTOR_OVER_CURRENT.toString(),
                            error_code_remap=self.detail_status,
                        )
                    # elif (
                    #     self.control_motor_left_error or self.control_motor_right_error
                    # ):
                    #     self.robot_status = RobotStatus.ERROR
                    #     self.detail_status = DetailStatus.MOTOR_CONTROL_ERROR.toString()
                    #     self.update_error_code(
                    #         msg=DetailStatus.MOTOR_CONTROL_ERROR.toString()
                    #     )
                    elif (
                        not self.pose_initiated
                        and self.option_config["check_init_pose"]
                    ):
                        self.robot_status = RobotStatus.WAITING_INIT_POSE
                    elif self.mission_status == ModuleStatus.WAITING:
                        self.robot_status = RobotStatus.WAITING
                    elif self.mission_status == ModuleStatus.PAUSED:
                        self.robot_status = RobotStatus.PAUSED
                    elif self.mission_status == ModuleStatus.RUNNING:
                        self.robot_status = RobotStatus.RUNNING
            # elif rospy.get_time() - self.last_odom_msg > 1.0:
            #     # If motor disconnect, lỗi FATAL_ERROR sẽ được ưu tiên hơn nên
            #     # không chạy đến đoạn này
            #     self.robot_status = RobotStatus.ERROR
            #     self.detail_status = DetailStatus.FATAL_ERROR.toString()
            #     self.update_error_code(
            #         msg=DetailStatus.ODOMETRY_ERROR.toString()
            #     )
            #     self.set_pose_init(False)
            elif (
                self.main_state == MainState.LOAD_MAP_ERROR
                or self.main_state == MainState.MAPPING_ERROR
            ):
                self.robot_status = RobotStatus.ERROR
                # self.detail_status = DetailStatus.LOAD_MAP_ERROR.toString()
                self.update_error_code(msg=self.detail_status)
            elif special_matching_error != "":
                self.robot_status = RobotStatus.ERROR
                self.detail_status = special_matching_error
            elif module_error_code != "":
                self.robot_status = RobotStatus.ERROR
                self.detail_status = DetailStatus.GENERAL_ERROR.toString()
                self.update_error_code(msg=module_error_code)
            elif (
                self.battery_percent < self.battery_config["empty_threshold"]
                and special_matching_state
                != DetailStatus.AUTO_CHARGING.toString()
                and special_matching_state
                != DetailStatus.MANUAL_CHARGING.toString()
            ):
                # TODO: update error code number here
                self.robot_status = RobotStatus.ERROR
                self.detail_status = DetailStatus.BATTERY_EMPTY.toString()
                self.update_error_code(
                    msg=DetailStatus.BATTERY_EMPTY.toString(),
                    error_code_remap=self.detail_status,
                )
            elif self.motor_left_over_ampe or self.motor_right_over_ampe:
                # TODO: update error code number here
                self.robot_status = RobotStatus.ERROR
                self.detail_status = DetailStatus.MOTOR_OVER_CURRENT.toString()
                self.update_error_code(
                    msg=DetailStatus.MOTOR_OVER_CURRENT.toString(),
                    error_code_remap=self.detail_status,
                )
            elif self.server_error_msg != "":
                self.robot_status = RobotStatus.ERROR
                self.detail_status = self.server_error_msg
                self.update_error_code(msg=self.server_error_msg)
            # elif (
            #     self.control_motor_left_error or self.control_motor_right_error
            # ):
            #     self.robot_status = RobotStatus.ERROR
            #     self.detail_status = DetailStatus.MOTOR_CONTROL_ERROR.toString()
            #     self.update_error_code(
            #         msg=DetailStatus.MOTOR_CONTROL_ERROR.toString()
            #     )
            elif (
                not self.pose_initiated
                and self.option_config["check_init_pose"]
            ):
                self.robot_status = RobotStatus.WAITING_INIT_POSE
            elif self.mission_status == ModuleStatus.WAITING:
                self.robot_status = RobotStatus.WAITING
            elif self.mission_status == ModuleStatus.PAUSED:
                self.robot_status = RobotStatus.PAUSED
            elif self.mission_status == ModuleStatus.RUNNING:
                self.robot_status = RobotStatus.RUNNING
            # Xử lý tình huống FATAL_ERROR có mứu ưu tiên cao hơn ODOMETRY_ERROR
            if rospy.get_time() - self.last_odom_msg > 5.0:
                self.set_pose_init(False)

            if module_disconnected != "" and last_module_disconnected == "":
                rospy.logerr(
                    "Module disconnected: {}".format(module_disconnected)
                )

            if not (
                self.detail_status
                == DetailStatus.IO_BOARD_DISCONNECT.toString()
                or self.detail_status == DetailStatus.IO_BOARD_ERROR.toString()
            ):
                odom_ready = True

            # Normal status, no need update robot_status

            if self.robot_status != RobotStatus.ERROR:
                if (
                    self.battery_percent < self.battery_config["low_threshold"]
                    and special_matching_state
                    != DetailStatus.AUTO_CHARGING.toString()
                    and special_matching_state
                    != DetailStatus.MANUAL_CHARGING.toString()
                ):
                    self.detail_status = DetailStatus.BATTERY_LOW.toString()
                elif (
                    safety_disabled
                    and self.robot_status == RobotStatus.RUNNING
                    and special_matching_state
                    != DetailStatus.AUTO_CHARGING.toString()
                    and special_matching_state
                    != DetailStatus.MANUAL_CHARGING.toString()
                    and special_matching_state == "GOING_TO_POS"
                ):
                    self.detail_status = DetailStatus.SAFETY_DISABLED.toString()
                elif (
                    self.is_safety
                    or rospy.get_time() - self.last_safety_time > 0.5
                ) and special_matching_state == "GOING_TO_POS":  # Hot fix SAFETY_STOP slow
                    self.detail_status = DetailStatus.SAFETY_STOP.toString()
                elif special_matching_state != "":
                    self.detail_status = special_matching_state
                elif self.current_action_type in self.list_action_move:
                    self.detail_status_not_dispay_hmi = "GOING_TO_POS"
            self.addtion_error_resset_qr_code = ""
            # Clear Arduino Error
            if (
                rospy.Time.now().to_sec()
                - self.arduino_error_msg.stamp.to_sec()
                > 5.0
            ):
                self.arduino_error_cnt = 0

            # Get reason robot pause
            if self.robot_status == RobotStatus.ERROR:
                keyword = "LOST LABEL"
                if keyword in self.error_code:
                    end_index = self.error_code.find(keyword) + len(keyword)
                    result = self.error_code[:end_index]
                    self.pause_detail_status = "Pause by " + result
                else:
                    self.pause_detail_status = "Pause by " + self.error_code
            elif self.robot_status == RobotStatus.WAITING:
                self.pause_detail_status = ""
                self.need_to_reset_by_qr_code = False
                if self.relocalization:
                    self.set_pose_init(False)
                    self.relocalization = False
                    continue
            elif self.robot_status == RobotStatus.PAUSED:
                self.detail_status_not_dispay_hmi = ""
                # rospy.logwarn_throttle(5, "Pause by: {}".format(self.pause_detail_status))
                if (
                    self.pause_detail_status == "MOTOR_OFF"
                    and self.motor_enable_sw != SENSOR_DEACTIVATE
                ):
                    self.pause_detail_status = "Pause by switch mode to MANUAL"
                self.detail_status = self.pause_detail_status
                if self.need_to_reset_by_qr_code:
                    self.addtion_error_resset_qr_code = "need to read QR CODE in path from ({}, {}) to ({}, {}) to RESET ERROR".format(
                        self.first_pose_in_path.position.x,
                        self.first_pose_in_path.position.y,
                        self.last_pose_in_path.position.x,
                        self.last_pose_in_path.position.y,
                    )

            # Get final status update to server
            if self.robot_status == RobotStatus.ERROR:
                self.detail_status_update_to_server = self.detail_status
            else:
                if (
                    self.detail_status == DetailStatus.ALL_OK.toString()
                    and self.robot_mode == RobotMode.MANUAL
                ):
                    self.detail_status_update_to_server = (
                        self.robot_mode.toString()
                    )
                else:
                    if self.detail_status_not_dispay_hmi != "":
                        self.detail_status_update_to_server = (
                            self.detail_status_not_dispay_hmi
                        )
                    else:
                        self.detail_status_update_to_server = self.detail_status

            self.robot_status_update_to_server = self.robot_status.toString()
            if self.addtion_error_resset_qr_code != "":
                self.detail_status += " - " + self.addtion_error_resset_qr_code
            if self.status_exe_path != "":
                self.detail_status += " - " + self.status_exe_path
            if self.detail_status_addition != "":
                self.detail_status += " - " + self.detail_status_addition
            self.message_update_to_server = self.detail_status
            if self.robot_status == RobotStatus.ERROR:
                if "QR LABEL" in self.error_code:
                    self.error_code += " - " + "need to read QR CODE to reset"
                self.error_code += " (ERROR_CODE: {})".format(
                    self.error_code_number
                )
                if self.error_code != self.pre_error_code:
                    rospy.logerr(
                        "ERROR CODE CHANGE: {} -> {}".format(
                            self.pre_error_code, self.error_code
                        )
                    )
                    self.pre_error_code = self.error_code

            """
            ##       ######## ########
            ##       ##       ##     ##
            ##       ##       ##     ##
            ##       ######   ##     ##
            ##       ##       ##     ##
            ##       ##       ##     ##
            ######## ######## ########
            """
            # Error
            if self.working_status == "PRODUCTION":
                if (
                    self.detail_status == DetailStatus.FATAL_ERROR.toString()
                ):  # Lỗi kết nối với IO, cần phân biệt với các lỗi khác và có ưu tiên cao nhất
                    self.led_status = LedStatus.FATAL_ERROR.toString()
                elif self.robot_status == RobotStatus.EMG:
                    self.led_status = LedStatus.EMG.toString()
                elif (
                    special_error_led != ""
                ):  # LIFT_STUCK, NO_CART, CHARGING_ERROR, MOVING_ERROR...
                    self.led_status = special_error_led
                elif (
                    self.detail_status == DetailStatus.MOTOR_OFF.toString()
                ):  # Lỗi có thể xử lý bằng tay
                    self.led_status = LedStatus.MOTOR_OFF.toString()
                elif (
                    self.robot_status == RobotStatus.ERROR
                ):  # Những lỗi cần tới tablet để xử lý
                    self.led_status = LedStatus.GENERAL_ERROR.toString()
                # Other status
                elif (
                    self.main_state == MainState.WAIT_MAPPING
                    or self.main_state == MainState.WAIT_MANUAL
                    or self.main_state == MainState.WAIT_AUTO
                    or self.main_state == MainState.LOADING_MAP
                ):
                    self.led_status = LedStatus.WAIT_RESPOND.toString()
                elif (
                    self.detail_status
                    == DetailStatus.SAFETY_DISABLED.toString()
                ):
                    self.led_status = LedStatus.SAFETY_DISABLED.toString()
                elif (
                    self.detail_status == DetailStatus.AUTO_CHARGING.toString()
                    and self.mission_status == ModuleStatus.WAITING
                    and self.robot_mode == RobotMode.AUTO
                ):
                    self.led_status = LedStatus.CHARGING_READY.toString()
                elif self.detail_status == DetailStatus.SAFETY_STOP.toString():
                    self.led_status = LedStatus.SAFETY_STOP.toString()
                elif (
                    special_state_led != ""
                ):  # AUTO_CHARGING, MANUAL_CHARGING, SAFETY_STOP, GOING_TO_POS, CHARGING_FULL...
                    self.led_status = special_state_led
                elif (
                    self.detail_status == DetailStatus.BATTERY_EMPTY.toString()
                ):
                    self.led_status = LedStatus.BATTERY_EMPTY.toString()
                elif self.robot_status == RobotStatus.WAITING_INIT_POSE:
                    try:
                        response = self.get_score_slam.call(
                            GetScoreGlobalMatchingRequest()
                        )
                        if response.score.data > 0:
                            rospy.loginfo("Succeed localization by cartographer")
                            self.set_pose_init(True)
                        rospy.logwarn("GetScoreGlobalMatchingRequest: {}".format(response.score.data))
                    except:
                        rospy.loginfo("ERROR_CONNECT_TO_CARTOGRAPHER")
                    self.led_status = LedStatus.WAITING_INIT_POSE.toString()
                elif self.detail_status == DetailStatus.BATTERY_LOW.toString():
                    self.led_status = LedStatus.BATTERY_LOW.toString()
                # Mode
                elif self.main_state == MainState.MANUAL:
                    self.led_status = LedStatus.MANUAL.toString()
                elif self.main_state == MainState.MAPPING:
                    self.led_status = LedStatus.MAPPING.toString()
                elif self.robot_status == RobotStatus.PAUSED:
                    self.led_status = LedStatus.PAUSED.toString()
                elif self.robot_status == RobotStatus.WAITING:
                    self.led_status = LedStatus.WAITING.toString()
                elif self.mission_status == ModuleStatus.RUNNING:
                    self.led_status = LedStatus.RUNNING_NORMAL.toString()
                else:
                    rospy.logwarn("Led status was not set")
            else:
                self.led_status = LedStatus.MAINTENANCE.toString()

            # Record log
            if (
                self.robot_status == RobotStatus.ERROR
                and last_robot_status != RobotStatus.ERROR
            ):
                self.db.recordLog(
                    "ERROR: {}".format(self.detail_status),
                    rospy.get_name(),
                    LogLevel.ERROR.toString(),
                )
                rospy.logerr("ERROR: {}".format(self.detail_status))

            # Pause if error
            if (
                self.robot_status == RobotStatus.ERROR
                and last_robot_status != RobotStatus.ERROR
                and last_robot_status != RobotStatus.PAUSED
                and pause_if_error
                # and self.robot_mode == RobotMode.AUTO
            ):
                rospy.logwarn("Pause by error")
                self.pause_robot()
                pause_by_error = True

            # Check clear pause_by_error
            if (
                self.robot_status == RobotStatus.WAITING
                and pause_by_error
                and self.robot_mode == RobotMode.AUTO
            ):
                pause_by_error = False
                rospy.logwarn("Clear pause by error")

            # Resume running if ERROR was cleared, có thể cần phải phân biệt với error khác như MOTOR OFF, tùy trường hợp mới cho reset
            # if (
            #     self.robot_status == RobotStatus.PAUSED
            #     and pause_by_error
            #     and self.robot_mode == RobotMode.AUTO
            # ):
            #     # chỉ được auto reset if not pause bởi MOTOR OFF
            #     if (
            #         self.pause_detail_status != "Pause by switch mode to MANUAL"
            #         and self.pause_detail_status != "MOTOR_OFF"
            #     ):
            #         rospy.logwarn("Resume after error")
            #         self.resume_running()
            #         pause_by_error = False

            # FIXME: Sometime MANUAL or ERROR but Robot still run
            # NOTE: Do khi 1 action đang lõi mà vẫn có thể reset bằng APP ở mode MANUAL
            # Eg: When /un_docking: MOVING_ERROR, mission_manager is PAUSED so do not forward reset cmd
            if (
                self.robot_mode != RobotMode.AUTO
                and last_robot_mode == RobotMode.AUTO
            ):
                begin_check_mode_change = rospy.get_time()
            if (
                self.robot_status != RobotStatus.RUNNING
                and last_robot_status == RobotStatus.RUNNING
            ):
                begin_check_status_change = rospy.get_time()
            if (
                self.robot_mode != RobotMode.AUTO
                and rospy.get_time() - begin_check_mode_change > 1
                # and rospy.get_time() - begin_check_mode_change < 3.0
            ):
                begin_check_mode_change = rospy.get_time()
                if (
                    self.moving_control_status_dict["status"]
                    == ModuleStatus.RUNNING.toString()
                ):
                    self.pause_robot()
                    # self.stop_moving()
                    rospy.logerr("Not yet PAUSED after switch mode to MANUAL")
            if (
                self.robot_status != RobotStatus.RUNNING
                and rospy.get_time() - begin_check_status_change > 1
                # and rospy.get_time() - begin_check_status_change < 3.0
            ):
                begin_check_status_change = rospy.get_time()
                if (
                    self.moving_control_status_dict["status"]
                    == ModuleStatus.RUNNING.toString()
                ):
                    # self.pause_robot()
                    self.pause_robot_by_server()
                    # self.stop_moving()
                    rospy.logerr("Not yet PAUSED after status != RUNNING")

            """
            ########  ##     ## ######## ########  #######  ##    ##
            ##     ## ##     ##    ##       ##    ##     ## ###   ##
            ##     ## ##     ##    ##       ##    ##     ## ####  ##
            ########  ##     ##    ##       ##    ##     ## ## ## ##
            ##     ## ##     ##    ##       ##    ##     ## ##  ####
            ##     ## ##     ##    ##       ##    ##     ## ##   ###
            ########   #######     ##       ##     #######  ##    ##
            """

            # Button handle
            # Hold PAUSE button to stop mission
            # Không check Mode khi gửi stop mission vì không chắc chắn lúc đó
            # mission_manager có đang hoạt động không
            # if self.robot_mode == RobotMode.AUTO:
            #     if (
            #         self.stop_1_button == SENSOR_ACTIVATE
            #         and last_stop_1_button == SENSOR_DEACTIVATE
            #     ) or (
            #         self.stop_2_button == SENSOR_ACTIVATE
            #         and last_stop_2_button == SENSOR_DEACTIVATE
            #     ):
            #         pushed_stop_mission = False
            #     if (
            #         self.stop_1_button == SENSOR_ACTIVATE
            #         and last_stop_1_button == SENSOR_ACTIVATE
            #     ) or (
            #         self.stop_2_button == SENSOR_ACTIVATE
            #         and last_stop_2_button == SENSOR_ACTIVATE
            #     ):
            #         if rospy.get_time() - begin_hold_stop_button >= 3.0:
            #             if pushed_stop_mission == False:
            #                 self.reset_get_mission = True
            #                 pushed_stop_mission = True
            #                 rospy.loginfo(
            #                     "Stop all mission when hold PAUSE button"
            #                 )
            #                 self.pub_request_start_mission.publish(
            #                     StringStamped(
            #                         stamp=rospy.Time.now(), data="STOP"
            #                     )
            #                 )
            #     else:
            #         begin_hold_stop_button = rospy.get_time()
            # Reset error
            if (
                self.robot_status == RobotStatus.ERROR
                and self.robot_mode == RobotMode.MANUAL
            ):
                self.reset_error_request = False
            if (
                self.robot_status == RobotStatus.ERROR
                and self.robot_mode == RobotMode.AUTO
            ):  # and self.robot_mode == RobotMode.AUTO:
                if (
                    self.start_1_button == SENSOR_ACTIVATE
                    and last_start_1_button == SENSOR_DEACTIVATE
                    or self.start_2_button == SENSOR_ACTIVATE
                    and last_start_2_button == SENSOR_DEACTIVATE
                ) or self.reset_error_request:
                    self.reset_error_request = False
                    # TODO: handle reset mission error here  check reset_when_mission_error
                    # TODO: need to test
                    if self.reset_when_mission_error:
                        self.pub_mission_reset_error.publish(
                            EmptyStamped(stamp=rospy.Time.now())
                        )
                        rospy.loginfo("Reset error when push RUN button")
                    else:
                        rospy.logwarn("DONT ALLOW RESET MISSION ERROR")
            # Trigger
            elif (
                self.robot_status == RobotStatus.WAITING
                and self.robot_mode == RobotMode.AUTO
            ):
                self.switch_control_tf(USE_TF_LIDAR)
                trigger_dict = {}
                trigger_dict["topic"] = ""
                trigger_dict["run_immediately"] = True
                # print_info(self.current_pose)
                if self.current_pose != None:
                    trigger_dict["pose"] = obj_to_dict(
                        self.current_pose, copy.deepcopy(pose_dict_template)
                    )
                    # Only press
                    if (
                        self.start_1_button == SENSOR_ACTIVATE
                        and last_start_1_button == SENSOR_DEACTIVATE
                    ):
                        trigger_dict["topic"] = "FRONT_RUN_BUTTON"
                        trigger_msg = json.dumps(trigger_dict)
                        self.pub_trigger.publish(
                            StringStamped(
                                stamp=rospy.Time.now(), data=trigger_msg
                            )
                        )
                        rospy.loginfo("Send trigger when push front RUN button")
                    elif (
                        self.start_2_button == SENSOR_ACTIVATE
                        and last_start_2_button == SENSOR_DEACTIVATE
                    ):
                        trigger_dict["topic"] = "REAR_RUN_BUTTON"
                        trigger_msg = json.dumps(trigger_dict)
                        self.pub_trigger.publish(
                            StringStamped(
                                stamp=rospy.Time.now(), data=trigger_msg
                            )
                        )
                        rospy.loginfo("Send trigger when push rear RUN button")
                    elif (
                        self.stop_1_button == SENSOR_ACTIVATE
                        and last_stop_1_button == SENSOR_DEACTIVATE
                    ):
                        trigger_dict["topic"] = "FRONT_PAUSE_BUTTON"
                        trigger_msg = json.dumps(trigger_dict)
                        self.pub_trigger.publish(
                            StringStamped(
                                stamp=rospy.Time.now(), data=trigger_msg
                            )
                        )
                        rospy.loginfo(
                            "Send trigger when push front PAUSE button"
                        )
                    elif (
                        self.stop_2_button == SENSOR_ACTIVATE
                        and last_stop_2_button == SENSOR_DEACTIVATE
                    ):
                        trigger_dict["topic"] = "REAR_PAUSE_BUTTON"
                        trigger_msg = json.dumps(trigger_dict)
                        self.pub_trigger.publish(
                            StringStamped(
                                stamp=rospy.Time.now(), data=trigger_msg
                            )
                        )
                        rospy.loginfo(
                            "Send trigger when push rear PAUSE button"
                        )
            # Resume
            elif (
                self.robot_status == RobotStatus.PAUSED
                and self.robot_mode == RobotMode.AUTO
            ):
                if (
                    self.start_1_button == SENSOR_ACTIVATE
                    and last_start_1_button == SENSOR_DEACTIVATE
                    or self.start_2_button == SENSOR_ACTIVATE
                    and last_start_2_button == SENSOR_DEACTIVATE
                ):
                    rospy.loginfo("Resume robot when push RUN button")
                    self.resume_running()
            # Pause
            elif self.robot_status == RobotStatus.RUNNING:
                if (
                    self.stop_1_button == SENSOR_ACTIVATE
                    and last_stop_1_button == SENSOR_DEACTIVATE
                    or self.stop_2_button == SENSOR_ACTIVATE
                    and last_stop_2_button == SENSOR_DEACTIVATE
                ):
                    rospy.loginfo("Pause robot when push PAUSE button")
                    self.pause_robot()
                    self.pause_detail_status = "Pause by push PAUSE button"

            # Publish status
            now = rospy.get_time()
            if now - last_pub_time >= 0.2:
                last_pub_time = now
                self.update_feedback()

            # Publish led status
            if self.led_status != prev_led_stt:
                msg = StringStamped()
                msg.stamp = rospy.Time.now()
                msg.data = self.led_status
                self.pub_led_status.publish(msg)
                prev_led_stt = self.led_status

            """
             ######   #######  ##     ## ##    ## ########
            ##    ## ##     ## ##     ## ###   ## ##     ##
            ##       ##     ## ##     ## ####  ## ##     ##
             ######  ##     ## ##     ## ## ## ## ##     ##
                  ## ##     ## ##     ## ##  #### ##     ##
            ##    ## ##     ## ##     ## ##   ### ##     ##
             ######   #######   #######  ##    ## ########
            """

            # Special sound
            if (
                self.sound_config != None
                and rospy.get_time() - self.time_start > 60
            ):
                if self.sound_config["play_sound_when_error"]:
                    if (
                        self.detail_status
                        == DetailStatus.FATAL_ERROR.toString()
                    ):  # Lỗi kết nối với IO, cần phân biệt với các lỗi khác và có ưu tiên cao nhất
                        self.sound_status = LedStatus.FATAL_ERROR.toString()
                    elif self.robot_status == RobotStatus.EMG:
                        self.sound_status = LedStatus.EMG.toString()
                    elif (
                        special_error_sound != ""
                    ):  # LIFT_STUCK, NO_CART, CHARGING_ERROR, MOVING_ERROR...
                        self.sound_status = special_error_sound
                    elif (
                        self.detail_status
                        == DetailStatus.BATTERY_EMPTY.toString()
                    ):
                        self.sound_status = LedStatus.BATTERY_EMPTY.toString()
                    elif (
                        self.robot_status == RobotStatus.ERROR
                    ):  # Những lỗi cần tới tablet để xử lý
                        self.sound_status = LedStatus.GENERAL_ERROR.toString()
                    elif (
                        self.detail_status
                        == DetailStatus.SAFETY_DISABLED.toString()
                    ):
                        self.sound_status = LedStatus.SAFETY_DISABLED.toString()
                    elif (
                        self.detail_status
                        == DetailStatus.SAFETY_STOP.toString()
                    ):
                        self.sound_status = LedStatus.SAFETY_STOP.toString()
                    elif (
                        special_state_sound != ""
                    ):  # AUTO_CHARGING, MANUAL_CHARGING, SAFETY_STOP, GOING_TO_POS, CHARGING_FULL...
                        self.sound_status = special_state_sound
                    elif self.robot_status == RobotStatus.PAUSED:
                        self.sound_status = LedStatus.PAUSED.toString()
                    else:
                        self.sound_status = ""
                else:
                    if (
                        self.detail_status
                        == DetailStatus.SAFETY_DISABLED.toString()
                    ):
                        self.sound_status = self.detail_status
                    else:
                        self.sound_status = special_state_sound

            if (
                self.sound_status != prev_sound_stt
            ):  # or rospy.get_time() - last_sound_time > 2.0: # Do not send request continuity because it will effect to request from app
                last_sound_time = rospy.get_time()
                self.pub_request_sound.publish(
                    StringStamped(
                        stamp=rospy.Time.now(), data=self.sound_status
                    )
                )
            prev_sound_stt = self.sound_status

            # Test server by topic
            if self.mission_from_server_req:
                self.mission_from_server_req = False
                if (
                    self.robot_mode == RobotMode.AUTO
                    and self.robot_status == RobotStatus.WAITING
                ):
                    # try:
                    if True:
                        mission_fr_server_file = os.path.join(
                            rospkg.RosPack().get_path("control_system"),
                            "mission",
                            "mission.json",
                        )
                        with open(mission_fr_server_file) as j:
                            print_warn(mission_fr_server_file)
                            mission_dict = json.load(j)
                            mission_json = json.dumps(mission_dict, indent=2)
                            # print_debug(mission_json)
                            self.pub_mission_fr_server.publish(
                                StringStamped(
                                    stamp=rospy.Time.now(), data=mission_json
                                )
                            )
                            self.pub_mission_fr_server_latch.publish(
                                StringStamped(
                                    stamp=rospy.Time.now(), data=mission_json
                                )
                            )
                    # except Exception as e:
                    #     rospy.logerr("Test mission from server: {}".format(e))

            last_start_1_button = self.start_1_button
            last_start_2_button = self.start_2_button
            last_stop_1_button = self.stop_1_button
            last_stop_2_button = self.stop_2_button
            last_auto_man_switch = self.auto_manual_sw
            last_robot_status = self.robot_status
            last_robot_mode = self.robot_mode
            last_module_disconnected = module_disconnected
            r.sleep()


def parse_opts():
    from optparse import OptionParser

    parser = OptionParser()
    parser.add_option(
        "-s",
        "--simulation",
        action="store_true",
        dest="simulation",
        default=False,
        help='type "-s" if simulation',
    )
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
        "--current_map_file",
        dest="current_map_file",
        default=os.path.join(HOME + "/tmp/ros/maps/current_map.yaml"),
    )
    parser.add_option(
        "-r",
        "--robot_config_file",
        dest="robot_config_file",
        default=os.path.join(
            rospkg.RosPack().get_path("control_system"),
            "cfg",
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
    # if not get_ubuntu_serial_hash():
    #     print_error("Control system: License verification failed.")
    #     return
    os.system("mkdir -p {}/tmp/ros/maps/".format(HOME))
    (options, args) = parse_opts()
    log_level = None
    if options.log_debug:
        log_level = rospy.DEBUG

    rospy.init_node("control_system", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    ControlSystem(**vars(options))


if __name__ == "__main__":
    main()
