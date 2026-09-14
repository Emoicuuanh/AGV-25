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

HOME = expanduser("~")


class MainState(EnumString):
    NONE = -1
    SEND_UNDOCKING = 0
    UNDOCKING = 1
    DONE = 8
    MOVING_ERROR = 10
    PAUSED = 12
    WAITING = 13
    MOVING_DISCONNECTED = 28
    INIT = 29
    LECH_TAM = 34
    ALIGNMENT_SENSOR = 35
    COLLISION_POSSIBLE = 50


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


class UnDockingCharger(object):
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
        self.safety_job_pub = rospy.Publisher(
            "/safety_job_name", StringStamped, queue_size=5
        )
        self.moving_control_run_pause_pub = rospy.Publisher(
            "/moving_control/run_pause_req", StringStamped, queue_size=5
        )
        self.moving_control_reset_error_pub = rospy.Publisher(
            "/moving_control/reset_error", EmptyStamped, queue_size=5
        )

        # Subscriber
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

        # dynamic reconfig client
        self.client_reconfig_movebase = dynamic_reconfigure.client.Client(
            "/move_base/NeoLocalPlanner",
            timeout=300,
            config_callback=self.dynamic_callback,
        )

        # ModuleServer
        self._asm = ModuleServer(name)
        # Loop
        self.loop()

    def init_variable(self, *args, **kwargs):
        self.config_path = kwargs["config_path"]
        self.robot_config_file = kwargs["robot_config_file"]
        self.server_config_file = kwargs["robot_define"]
        self.use_tf2 = False
        self.tf_listener = tf.TransformListener(cache_time=rospy.Duration(0.3))
        #
        self.last_moving_control_fb = rospy.get_time()
        self.moving_control_result = -1
        #
        self.moving_control_error_code = ""
        # Database
        db_address = rospy.get_param("/mongodb_address")
        print_debug(db_address)
        self.db = mongodb(db_address)

        # self.vel_move_base = rospy.get_param("/move_base/NeoLocalPlanner/max_vel_x")
        self.last_time_read_qr = rospy.get_time()

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(0.3))
        self.tf2_listener = tf2_ros.TransformListener(self.tf_buffer)

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

    def dynamic_callback(config, level):
        # rospy.loginfo("Suceeed change vel of robot")
        pass

    def odom_cb(self, msg):
        self.pose_odom2robot = msg.pose.pose

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
        hub_type = "undocking_charger"
        try:
            # Hub config
            hub_cfg_file = os.path.join(self.config_path, hub_type + ".json")
            with open(hub_cfg_file) as j:
                hub_dict = json.load(j)
                x_offset = hub_dict["offset_x_direction"]

                vel_undocking_charger = hub_dict["max_vel_undocking"]
                # max_error_sensor_in_hub = hub_dict["error_sensor_in_hub"]
                # max_error_angle_sensor_in_hub = hub_dict[
                #     "error_angle_sensor_in_hub"
                # ]
                safety_job_docking = hub_dict["safety_job_docking"]
                # footprint_dock = hub_dict["footprint_dock"]
                # enable_check_error_when_docking = hub_dict[
                #     "enable_check_error_when_docking"
                # ]
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

        cur_pose_x = self.trans[0]
        cur_pose_y = self.trans[1]

        robot_orient = euler_from_quaternion([0, 0, self.rot[2], self.rot[3]])
        cur_orient = robot_orient[2]

        waiting_pose = self.calculate_pose_offset(
            0,
            cur_pose_x,
            cur_pose_y,
            cur_orient,
        )
        undocking_pose = self.calculate_pose_offset(
            x_offset,
            cur_pose_x,
            cur_pose_y,
            cur_orient,
        )
        if USE_DOCKING_BY_MIRROR:
            waiting_pose = self.wait_until_pose_available()
            quat = [
                waiting_pose.orientation.x,
                waiting_pose.orientation.y,
                waiting_pose.orientation.z,
                waiting_pose.orientation.w,
            ]

            # Chuyển sang euler (roll, pitch, yaw)
            roll, pitch, yaw = euler_from_quaternion(quat)

            undocking_pose = self.calculate_pose_offset(
                x_offset,
                waiting_pose.position.x,
                waiting_pose.position.y,
                yaw,
            )
        # Calculate undocking goal
        undocking_goal = StringGoal()
        undocking_path_dict["params"] = {}
        undocking_path_dict["params"]["only_use_qr"] = True
        if USE_DOCKING_BY_MIRROR:
            undocking_path_dict["params"]["use_mirror"] = True
        undocking_path_dict["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(waiting_pose, return_pose_dict)
        )

        undocking_path_dict["waypoints"][1]["position"] = copy.deepcopy(
            obj_to_dict(undocking_pose, return_pose_dict)
        )
        undocking_goal.data = json.dumps(undocking_path_dict, indent=2)
        rospy.logwarn(
            "UnDocking goal position:\n{}".format(
                (json.dumps(undocking_path_dict, indent=2))
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
        last_time_pub_safety_job = rospy.get_time()
        self.pre_safety_job_name = None
        undocking_done = False

        while not rospy.is_shutdown():
            if not self.get_odom():
                continue
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
            # State:
            if _state == MainState.INIT:
                _state = MainState.SEND_UNDOCKING
                self.vel_move_base = rospy.get_param(
                    "/move_base/NeoLocalPlanner/max_vel_x", 0.8
                )
                print("self.vel_move_base: ", self.vel_move_base)
                self.dynamic_reconfig_movebase(
                    vel_undocking_charger,
                    publish_safety=False,
                    stop_center_qr=False,
                )
                # print(f"vel_undocking_charger: {vel_undocking_charger}")
                if self._asm.pause_req:
                    # self.moving_control_client.cancel_all_goals()
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # """
            #  ######  ######## ##    ## ########          ##     ## ##    ## ########   #######   ######  ##    ## #### ##    ##  ######
            # ##    ## ##       ###   ## ##     ##         ##     ## ###   ## ##     ## ##     ## ##    ## ##   ##   ##  ###   ## ##    ##
            # ##       ##       ####  ## ##     ##         ##     ## ####  ## ##     ## ##     ## ##       ##  ##    ##  ####  ## ##
            #  ######  ######   ## ## ## ##     ##         ##     ## ## ## ## ##     ## ##     ## ##       #####     ##  ## ## ## ##   ####
            #       ## ##       ##  #### ##     ##         ##     ## ##  #### ##     ## ##     ## ##       ##  ##    ##  ##  #### ##    ##
            # ##    ## ##       ##   ### ##     ##         ##     ## ##   ### ##     ## ##     ## ##    ## ##   ##   ##  ##   ### ##    ##
            #  ######  ######## ##    ## ########  #######  #######  ##    ## ########   #######   ######  ##    ## #### ##    ##  ######
            # """
            # State: SEND_UNDOCKING
            elif _state == MainState.SEND_UNDOCKING:
                self.moving_control_client.send_goal(
                    undocking_goal,
                    feedback_cb=self.moving_control_fb,
                )
                self.moving_control_result = -1
                self.last_moving_control_fb = rospy.get_time()
                _state = MainState.UNDOCKING
                if self._asm.pause_req:
                    # self.moving_control_client.cancel_all_goals()
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # """
            # ##     ## ##    ## ########   #######   ######  ##    ## #### ##    ##  ######
            # ##     ## ###   ## ##     ## ##     ## ##    ## ##   ##   ##  ###   ## ##    ##
            # ##     ## ####  ## ##     ## ##     ## ##       ##  ##    ##  ####  ## ##
            # ##     ## ## ## ## ##     ## ##     ## ##       #####     ##  ## ## ## ##   ####
            # ##     ## ##  #### ##     ## ##     ## ##       ##  ##    ##  ##  #### ##    ##
            # ##     ## ##   ### ##     ## ##     ## ##    ## ##   ##   ##  ##   ### ##    ##
            #  #######  ##    ## ########   #######   ######  ##    ## #### ##    ##  ######
            # """
            # State: UNDOCKING
            elif _state == MainState.UNDOCKING:
                self.safety_job_name = safety_job_docking
                if self.moving_control_result == GoalStatus.SUCCEEDED:
                    # print("aaaaaaaaaaaaaaa")
                    undocking_done = True
                    _state = MainState.DONE
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
                    _state_bf_error = MainState.SEND_UNDOCKING
                    _state_when_error = _state
                    _state = MainState.MOVING_ERROR
                    # self.moving_control_client.cancel_all_goals()
                    continue
                # print(f"bbbbbbbbbbbbb :{self.last_moving_control_fb}")
                if rospy.get_time() - self.last_moving_control_fb >= 5.0:
                    rospy.logerr("/moving control disconnected!")
                    self.send_feedback(
                        self._as, GoalStatus.to_string(GoalStatus.ABORTED)
                    )
                    _state_bf_error = MainState.SEND_UNDOCKING
                    _state_when_error = _state
                    _state = MainState.MOVING_DISCONNECTED
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # State: MOVING_ERROR
            elif _state == MainState.MOVING_ERROR:
                self.moving_control_result = -1
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = (
                    "/hub_server: {}".format(_state.toString())
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
                    _state = MainState.SEND_UNDOCKING
                    self.moving_control_error_code = ""
            # State: COLLISION_POSSIBLE
            elif _state == MainState.COLLISION_POSSIBLE:
                self.moving_control_result = -1
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = (
                    "/hub_server: {}".format(_state.toString())
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
                    _state = MainState.SEND_UNDOCKING
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

            # State: PAUSED
            elif _state == MainState.PAUSED:
                self._asm.module_status = ModuleStatus.PAUSED
                if self._asm.resume_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="RUN")
                    )

                    _state = _state_when_pause
                    if undocking_done:
                        _state = MainState.DONE

            # State: DONE
            elif _state == MainState.DONE:
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

        feedback_msg = _state.toString()
        self._asm.module_state = _state.toString()
        self.send_feedback(self._as, feedback_msg)
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
    rospy.init_node("un_docking", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    UnDockingCharger(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
