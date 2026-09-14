#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import json
import threading
import yaml
import argparse
from rosgraph.names import is_private
import rospy
import rospkg
import tf
import copy
import actionlib
from math import pi, degrees, atan2
import numpy as np
import time
from agv_msgs.msg import WaypointGlobal
from agv_msgs.srv import (
    WaypointsPath,
    WaypointsPathResponse,
    ArrayWaypoints,
    ArrayWaypointsResponse,
)
from agv_msgs.msg import DataMatrixStamped
from nav_msgs.msg import Odometry
import dynamic_reconfigure.client
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Twist
from std_msgs.msg import Int8, Header, Float32, Bool
from std_stamped_msgs.msg import (
    StringStamped,
    StringFeedback,
    StringResult,
    StringAction,
    EmptyStamped,
    Int8Stamped,
)
from dynamic_reconfigure.client import Client
from actionlib_msgs.msg import GoalStatus, GoalID, GoalStatusArray
from safety_msgs.msg import SafetyStatus

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
    pose_stamped_array_to_pose_array,
    delta_angle,
    merge_two_dicts,
    print_debug,
    print_error,
    print_info,
    angle_vector_two_point,
    dict_to_pose,
)


class MainState(EnumString):
    NONE = -1
    SEND_GOAL = 0
    MOVING = 1
    RE_SEND_GOAL = 2
    DONE = 6
    ERROR = 7
    STOP_BY_SAFETY = 8
    PAUSED = 9
    WAITING = 10
    WAIT_CLEAR_SAFETY = 11  # TODO
    FIND_QR_CODE = 12
    FIND_QR_CODE_ERROR = 13
    LIFT_DOWN = 14


class ToleranceType(EnumString):
    BOTH = 1
    XY = 2
    YAW = 3
    X_ONLY = 4
    NOT_REACH = 5


class MovingDirection(EnumString):
    INTERPOLATE = 0
    FORWARD = 1
    BACKWARD = 2


FORWARD = True
BACKWARD = False

goal_result = GoalStatus()
VIA_POINT_TOL_XY_BREAK = 0.2
VIA_POINT_TOL_YAW_BREAK = 0.1
NO_NEED_BREAK = Int8(0)
NEED_BREAK = Int8(1)

USE_TF_LIDAR = 1
USE_TF_QR_CODE = 0


class MovingControl(object):
    _feedback = StringFeedback()
    _result = StringResult()

    def __init__(self, name, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        # Action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            StringAction,
            execute_cb=self.moving_control_exec_cb,
            auto_start=False,
        )
        self._as.start()
        # self.planner_setting_client = actionlib.SimpleActionClient('/planner_setting', StringAction)
        rospy.on_shutdown(self.shutdown)
        # Publisher
        self.control_tf_pub = rospy.Publisher("/control_tf", Int8, queue_size=5)
        self.waypoints_following_pub = rospy.Publisher(
            "/waypoints_following", PoseArray, queue_size=5
        )
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        self.income_pose_pub = rospy.Publisher(
            "/income_pose", PoseStamped, queue_size=5
        )
        self.run_pause_pub = rospy.Publisher(
            "/run_pause_req", StringStamped, queue_size=5
        )
        self.pub_lift_cmd = rospy.Publisher(
            "/lift_cart", Int8Stamped, queue_size=10
        )
        rospy.Subscriber("/robot_pose", Pose, self.robot_pose_cb)
        rospy.Subscriber("/stop_moving", EmptyStamped, self.stop_moving_cb)
        rospy.Subscriber(
            "/disable_check_error_qr_code",
            Int8Stamped,
            self.disable_check_error_qr_code_cb,
        )
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/standard_io", StringStamped, self.standard_io_cb)
        rospy.Subscriber("/data_gls621", DataMatrixStamped, self.data_qr_cb)
        rospy.Subscriber(
            "/current_control_tf", Int8, self.current_control_tf_cb
        )
        rospy.Subscriber(
            "/succeed_init_local_planner", Bool, self.init_local_planner_cb
        )
        # Service Server
        self.server_get_wps = rospy.Service(
            "/waypoints_to_global_lidar",
            WaypointsPath,
            self.waypoints_path_lidar_cb,
        )
        self.get_wp_create_path = rospy.Service(
            "/waypoints_to_global_qr", ArrayWaypoints, self.waypoints_path_qr_cb
        )
        self.server_ger_wp_low_speed = rospy.Service(
            "/waypoints_low_speed", ArrayWaypoints, self.waypoints_low_speed_cb
        )

        # ModuleServer
        self._asm = ModuleServer(name)
        self.last_module_status = self._asm.module_status
        # Initial
        self.planner_init(path=kwargs["planner_map"])
        self.load_default_params(path=kwargs["default_params"])
        self.load_planner_setting(path=kwargs["planner_setting"])
        self.loop()

    """

    # #    # # ##### #   ##   #
    # ##   # #   #   #  #  #  #
    # # #  # #   #   # #    # #
    # #  # # #   #   # ###### #
    # #   ## #   #   # #    # #
    # #    # #   #   # #    # ######

    """

    def init_variable(self, *args, **kwargs):
        self.simulation = kwargs["simulation"]
        print_debug("simulation: {}".format(self.simulation))
        # TF
        self.tf_listener = tf.TransformListener()
        self.map_frame = "map"
        self.odom_frame = "odom"
        self.robot_base = "base_footprint"
        self.robot_pose = Pose()
        #
        self.planner_map = None
        self.planner_setting_dict = None
        self.current_local_planner = ""
        self.local_planner_var = {}
        self.default_params_dict = {}
        self.safety_fields = []
        self.last_safety_fields = []
        self.list_slow_point = []
        self.delay_clear_safety = 5.0  # Second
        self.odom_msg = Odometry()

        # Flag vars
        self.is_safety_stop = False
        self.last_state_use_qr_code = False
        self.bool_ignore_wp = False
        self.have_pallet = True
        self.liftdown_finish = False
        self.old_state_near_slow_point = False
        self.move_action_status = -1
        self.move_action_result = -1
        self.prev_move_action_status = -1
        self.offset_start_x = 0.0
        self.offset_start_y = 0.0
        self.last_action_fb = rospy.get_time()
        self.last_scan_safety = rospy.get_time()
        self.last_robot_pose = rospy.get_time()
        self.last_time_get_lift_down = rospy.get_time()
        self.last_time_read_qr = rospy.get_time()
        self.time_init_local_planner = rospy.get_time()
        # TODO: Get from config
        self.current_vel_param = "/move_to_point/max_vel_x"
        self.planner_type_rotate_and_straight = "move_to_point"
        self.path_qr_code = []
        self.list_wp_create_path = []
        self.array_wp_mb = WaypointsPathResponse()
        self.array_wp_slow_speed = ArrayWaypointsResponse()
        self.start_pose_plan = WaypointGlobal()
        self.last_pose_qr_code = PoseStamped()
        self.lift_msg = Int8Stamped()
        self.sensor_lift_min = 1
        self.current_control_tf = 100
        self.disable_check_qr_code = False

    def shutdown(self):
        self.cancel_all_action()
        print("Shutdown: {}".format(rospy.get_name()))

    def cancel_all_action(self):
        for key, value in self.local_planner_var.items():
            self.local_planner_var[key]["action_cancel_pub"].publish(
                GoalID(stamp=rospy.Time.now())
            )

    """

    ###### #    # #    #  ####  ##### #  ####  #    #
    #      #    # ##   # #    #   #   # #    # ##   #
    #####  #    # # #  # #        #   # #    # # #  #
    #      #    # #  # # #        #   # #    # #  # #
    #      #    # #   ## #    #   #   # #    # #   ##
    #       ####  #    #  ####    #   #  ####  #    #

    """

    def switch_control_tf(self, type):
        while not rospy.is_shutdown():
            self.control_tf_pub.publish(Int8(data=type))
            if self.current_control_tf == type:
                break
            rospy.sleep(0.1)

    def switch_local_planner(self, local_planner, global_planner):
        client = Client("move_base", timeout=300)

        params = {
            "base_local_planner": local_planner,
            "base_global_planner": global_planner,
            # Additional parameters for the chosen local planner
        }
        config = client.update_configuration(params)
        time_check = rospy.get_time()
        while not rospy.is_shutdown():
            local_planner_get = rospy.get_param("/move_base/base_local_planner")
            if (
                local_planner_get == local_planner
                and rospy.get_time() - self.time_init_local_planner < 1.0
            ):
                rospy.loginfo(
                    "Switched to {} local planner, {} global planner".format(
                        local_planner, global_planner
                    )
                )
                return True
            if rospy.get_time() - time_check > 2.0:
                rospy.logerr("ERROR switch local planner")
                return False
            rospy.sleep(0.1)

    def load_planner_setting(self, path):
        # try:
        if True:
            with open(path) as file:
                planner_setting = yaml.load(file, Loader=yaml.Loader)
                self.planner_setting_dict = planner_setting[
                    "planner_setting_action"
                ]
                self.safety_vel_set = planner_setting["safety_vel"]["velocity"]
                self.safety_deceleration = planner_setting["safety_vel"][
                    "deceleration"
                ]
                self.obstacle_config = (
                    planner_setting["obstacle_config"]
                    if "obstacle_config" in planner_setting
                    else None
                )
                # List of params which will be set to by planner_setting action
                self.planner_setting_params = []
                for key, value in self.planner_setting_dict.items():
                    for i in value:
                        self.planner_setting_params.append(i)
                print_info(
                    "planner_setting params: {}".format(
                        self.planner_setting_params
                    )
                )
                print_debug("Safety vel set:\n{}".format(self.safety_vel_set))
        # except Exception as e:
        #     rospy.logerr('Error loading: {}'.format(e))

    def planner_init(self, path):
        try:
            with open(path) as file:
                self.planner_map = yaml.load(file, Loader=yaml.Loader)
        except Exception as e:
            rospy.logerr("Error loading: {}".format(e))
        # Loop all moving type and create Publisher, Subscriber
        for key, value in self.planner_map.items():
            action_server_name = value["action_server_name"]
            action_server_msg = value["action_server_msg"]
            action_server_prefix = value["action_server_prefix"]
            if key not in self.local_planner_var:
                # Import action msgs
                _cmd = "from {} import {}, {}, {}".format(
                    action_server_msg,
                    action_server_prefix + "Action",
                    action_server_prefix + "ActionResult",
                    action_server_prefix + "Goal",
                )
                print("cmd -->: " + _cmd)
                exec(_cmd)
                # Create action_client install
                # Similar: self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

                global temp  # Python3 exec() only can change global var
                action_client = None
                _cmd = "global temp; temp = actionlib.SimpleActionClient('{}', {})".format(
                    action_server_name, action_server_prefix + "Action"
                )
                print("cmd -->: " + _cmd)
                exec(_cmd)
                action_client = temp

                # Create action goal
                # move_base_goal = MoveBaseGoal(target_pose = goal)
                action_goal = None
                _cmd = "global temp; temp = {}Goal()".format(
                    action_server_prefix
                )
                print("cmd -->: " + _cmd)
                exec(_cmd)
                action_goal = temp

                # Create subscriber
                # rospy.Subscriber('/move_base/result', MoveToPointActionResult, move_to_point_result_cb)
                _cmd = """rospy.Subscriber('/{}/result', {}ActionResult, self.action_result_cb)""".format(
                    action_server_name, action_server_prefix
                )
                print("cmd -->: " + _cmd)
                exec(_cmd)

                _cmd = """rospy.Subscriber('/{}/status', GoalStatusArray, self.action_status_cb)""".format(
                    action_server_name
                )
                print("cmd -->: " + _cmd)
                exec(_cmd)

                # Create Publisher
                # move_base_cancel_publisher = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
                action_cancel_pub = None
                _cmd = """global temp; temp = rospy.Publisher('/{}/cancel', GoalID, queue_size=5)""".format(
                    action_server_name
                )
                print("cmd -->: " + _cmd)
                exec(_cmd)
                action_cancel_pub = temp

                # Assign object with a key
                planner_dict = {}
                planner_dict["action_client"] = action_client
                planner_dict["action_goal"] = action_goal
                planner_dict["action_cancel_pub"] = action_cancel_pub
                self.local_planner_var[key] = planner_dict
                print("---")
        print("local_planner_var:")
        for key, value in self.local_planner_var.items():
            print(key)
            for k, v in value.items():
                print("  {}: {}".format(k, type(v)))
        print("---")

    def load_default_params(self, path):
        default_params_path = path
        try:
            with open(default_params_path) as file:
                self.default_params_dict = json.load(file)
                print_debug(
                    default_params_path
                    + "\n"
                    + json.dumps(self.default_params_dict, indent=2)
                )
                self.default_lin_vel = self.default_params_dict["params"][
                    "reconfigure"
                ]["forward_vel"]
                self.default_back_vel = self.default_params_dict["params"][
                    "reconfigure"
                ]["backward_vel"]
                self.default_ang_vel = self.default_params_dict["params"][
                    "reconfigure"
                ]["angular_vel"]
                self.default_rot_vel = self.default_params_dict["params"][
                    "reconfigure"
                ]["rotate_vel"]
                self.last_lin_vel_bf_safety = (
                    self.default_lin_vel
                )  # Velocity regardless direction
                self.last_back_vel_bf_safety = (
                    self.default_back_vel
                )  # Velocity regardless direction
                self.last_ang_vel_bf_safety = self.default_ang_vel
                self.last_rot_vel_bf_safety = self.default_rot_vel
                self.set_lin_vel = self.default_lin_vel
                self.set_back_vel = self.default_back_vel
                self.set_ang_vel = self.default_ang_vel
                self.set_rot_vel = self.default_rot_vel
        except Exception as e:
            rospy.logerr("Error load default params: {}".format(e))

    def get_wp_display(self, wp_dict):
        ret = []
        for wp in wp_dict:
            pose_stamped = PoseStamped()
            pose_stamped.pose = dict_to_pose(wp["position"])
            ret.append(pose_stamped)
        return ret

    def send_feedback(self, action, msg):
        self._feedback.data = msg
        action.publish_feedback(self._feedback)

    def create_wp_for_mb(self, array_wp, offset_start_x, offset_start_y):
        self.array_wp_mb.Waypoints = []
        if self.current_goal == 0 and self.bool_ignore_wp:
            self.start_pose_plan.Pose.pose.position.x += self.offset_start_x
            self.start_pose_plan.Pose.pose.position.y += self.offset_start_y
            self.array_wp_mb.Waypoints.append(self.start_pose_plan)
        elif self.current_goal > 0:
            wp_mb = WaypointGlobal()
            wp_mb.Pose.pose = dict_to_pose(
                array_wp[self.current_goal - 1]["position"]
            )
            wp_mb.Pose.pose.position.x += offset_start_x
            wp_mb.Pose.pose.position.y += offset_start_y
            wp_mb.radius = 0.0
            self.array_wp_mb.Waypoints.append(wp_mb)
        else:
            pass

        numble_mb_goal = 0
        for i in range(len(array_wp)):
            if i < self.current_goal:
                continue

            if array_wp[i]["rule_path"] != "qr_code":
                wp_mb = WaypointGlobal()
                wp_mb.Pose.pose = dict_to_pose(array_wp[i]["position"])
                if "data" in array_wp[i]:
                    wp_mb.radius = array_wp[i]["data"]["radius"]
                elif "position" in array_wp[i]:
                    wp_mb.radius = array_wp[i]["position"]["radius"]
                else:
                    wp_mb.radius = 0.0
                self.array_wp_mb.Waypoints.append(wp_mb)
                numble_mb_goal = i
                if i < len(array_wp) - 1:
                    if array_wp[i + 1]["rule_path"] == "qr_code":
                        wp_mb = WaypointGlobal()
                        wp_mb.Pose.pose = dict_to_pose(
                            array_wp[i + 1]["position"]
                        )
                        if "data" in array_wp[i + 1]:
                            wp_mb.radius = array_wp[i + 1]["data"]["radius"]
                        elif "position" in array_wp[i + 1]:
                            wp_mb.radius = array_wp[i + 1]["position"]["radius"]
                        else:
                            wp_mb.radius = 0.0
                        # self.array_wp_mb.Waypoints.append(wp_mb)
            else:
                return numble_mb_goal

        return numble_mb_goal

    def create_wp_for_qr_mb(self, array_wp):
        self.list_wp_create_path = []
        if self.current_goal == 0 and self.bool_ignore_wp:
            self.list_wp_create_path.append(self.start_pose_plan.Pose)
        elif self.current_goal > 0:
            first_pose = PoseStamped()
            first_pose.pose = dict_to_pose(
                array_wp[self.current_goal - 1]["position"]
            )
            self.list_wp_create_path.append(first_pose)
        else:
            pass

        numble_mb_goal = 0
        for i in range(len(array_wp)):
            if i < self.current_goal:
                continue

            if (
                i < len(array_wp) - 1
                and array_wp[i + 1]["rule_path"] == "qr_code"
            ):
                _goal_next = array_wp[i + 1]["position"]
            else:
                _goal_next = None

            if _goal_next is not None:
                if i == 0 and self.bool_ignore_wp:
                    angle_curPose_to_Goal = angle_vector_two_point(
                        dict_to_pose(array_wp[i]["position"]),
                        self.start_pose_plan.Pose.pose,
                    )
                else:
                    angle_curPose_to_Goal = angle_vector_two_point(
                        dict_to_pose(array_wp[i]["position"]),
                        dict_to_pose(array_wp[i - 1]["position"]),
                    )

                angle_Goal_to_nextGoal = angle_vector_two_point(
                    dict_to_pose(array_wp[i + 1]["position"]),
                    dict_to_pose(array_wp[i]["position"]),
                )
                diff_angle = delta_angle(
                    angle_curPose_to_Goal, angle_Goal_to_nextGoal
                )
                if abs(degrees(diff_angle)) > 45:
                    numble_mb_goal = i
                    goal_pose = PoseStamped()
                    goal_pose.pose = dict_to_pose(
                        array_wp[numble_mb_goal]["position"]
                    )
                    self.list_wp_create_path.append(goal_pose)
                    return numble_mb_goal
            else:
                numble_mb_goal = i
                goal_pose = PoseStamped()
                goal_pose.pose = dict_to_pose(
                    array_wp[numble_mb_goal]["position"]
                )
                self.list_wp_create_path.append(goal_pose)
                return numble_mb_goal
        return numble_mb_goal

    def check_pose_near_slow_point(self):
        if len(self.list_slow_point) < 1:
            return False

        for i in range(len(self.list_slow_point)):
            dx = self.robot_pose.position.x - self.list_slow_point[i].position.x
            dy = self.robot_pose.position.y - self.list_slow_point[i].position.y
            dis = np.sqrt(dx * dx + dy * dy)
            if dis < 1.0:
                return True
        return False

    def list_slow_speed_qr(self, array_wp):
        self.list_slow_point = []
        for i in range(len(array_wp)):
            if "data" in array_wp[i]:
                if "properties" in array_wp[i]["data"]:
                    if "Slow_speed" in array_wp[i]["data"]["properties"]:
                        if (
                            array_wp[i]["data"]["properties"]["Slow_speed"]
                            == "True"
                        ):
                            self.list_slow_point.append(
                                dict_to_pose(array_wp[i]["position"])
                            )

    def list_wp_low_speed(self, array_wp):
        self.array_wp_slow_speed.Waypoints = []
        for i in range(len(array_wp)):
            if "data" in array_wp[i]:
                if "isLowSpeed" not in array_wp[i]["data"]:
                    continue
                if array_wp[i]["data"]["isLowSpeed"] == True:
                    wp = PoseStamped()
                    wp.pose = dict_to_pose(array_wp[i]["position"])
                    self.array_wp_slow_speed.Waypoints.append(wp)

    def set_path_rule_for_wp(self, waypoints):
        self.last_state_use_qr_code = False
        for i in range(len(waypoints)):
            waypoints[i]["rule_path"] = ""

        if len(waypoints) > 1:
            for i in range(len(waypoints) - 1):
                for j in range(
                    len(self.path_qr_code)
                ):  # Mảng các mảng waypoints[] trong 1 path_rule
                    for k in range(
                        len(self.path_qr_code[j]) - 1
                    ):  # Các phần tử trong mảng waypoints[] của path_rule
                        # Nếu 2 waypoints liên tiếp trùng với 2 waypoints liên tiếp trong 1 path_rule
                        if (
                            waypoints[i]["name"] == self.path_qr_code[j][k]
                            and waypoints[i + 1]["name"]
                            == self.path_qr_code[j][k + 1]
                        ) or (  # Thứ tự ngược lại
                            waypoints[i]["name"] == self.path_qr_code[j][k + 1]
                            and waypoints[i + 1]["name"]
                            == self.path_qr_code[j][k]
                        ):
                            # waypoints[i]["rule_path"] = "qr_code"
                            waypoints[i + 1]["rule_path"] = "qr_code"
        elif len(waypoints) == 1:
            for j in range(
                len(self.path_qr_code)
            ):  # Mảng các mảng waypoints[] trong 1 path_rule
                for k in range(
                    len(self.path_qr_code[j])
                ):  # Các phần tử trong mảng waypoints[] của path_rule
                    # Nếu 2 waypoints liên tiếp trùng với 2 waypoints liên tiếp trong 1 path_rule
                    if waypoints[0]["name"] == self.path_qr_code[j][k]:
                        waypoints[0]["rule_path"] = "qr_code"
                        self.last_state_use_qr_code = True
        return waypoints

    """

     ####    ##   #      #      #####    ##    ####  #    #
    #    #  #  #  #      #      #    #  #  #  #    # #   #
    #      #    # #      #      #####  #    # #      ####
    #      ###### #      #      #    # ###### #      #  #
    #    # #    # #      #      #    # #    # #    # #   #
     ####  #    # ###### ###### #####  #    #  ####  #    #

    """

    def standard_io_cb(self, msg):
        data = json.loads(msg.data)
        if "lift_min_level" in data:
            self.sensor_lift_min = data["lift_min_level"]
            if self.sensor_lift_min and (
                rospy.get_time() - self.last_time_get_lift_down >= 1
            ):
                self.liftdown_finish = True
            if not self.sensor_lift_min:
                self.last_time_get_lift_down = rospy.get_time()
                self.liftdown_finish = False

    def dynamic_callback(config, level):
        # rospy.loginfo("Suceeed change vel of robot")
        pass

    def dynamic_callback_qr(config, level):
        pass

    def waypoints_path_lidar_cb(self, request):
        return self.array_wp_mb

    def waypoints_path_qr_cb(self, req):
        response = ArrayWaypointsResponse()
        response.Waypoints = self.list_wp_create_path
        return response

    def waypoints_low_speed_cb(self, request):
        return self.array_wp_slow_speed

    def disable_check_error_qr_code_cb(self, msg):
        rospy.logwarn("Disable check qr code")
        if msg.data == 1:
            self.disable_check_qr_code = True
        elif msg.data == 0:
            self.disable_check_qr_code = False

    def stop_moving_cb(self, msg):
        rospy.logwarn("Stop moving request")
        self.cancel_all_action()
        self._asm.pause_req = True

    def robot_pose_cb(self, msg):
        self.robot_pose = msg
        self.last_robot_pose = rospy.get_time()

    def odom_cb(self, msg):
        self.odom_msg = msg

    def action_result_cb(self, msg):
        self.move_action_result = msg.status.status

    def action_status_cb(self, msg):
        # Tránh sử dụng action status vì sau khi SUCCEEDED vẫn tiếp tục bắn về SUCCEEDED vài giây nữa
        # if not 'self.prev_move_action_status' in vars(__builtins__):
        #     self.prev_move_action_status = -1
        if len(msg.status_list) > 0:
            self.move_action_status = msg.status_list[
                len(msg.status_list) - 1
            ].status
            if self.prev_move_action_status != self.move_action_status:
                # rospy.loginfo('move_base_status: %s' % self.move_action_status)
                self.prev_move_action_status = self.move_action_status

    def action_fb(self, data):
        self.last_action_fb = rospy.get_time()

    def data_qr_cb(self, msg):
        self.last_time_read_qr = rospy.get_time()

    def init_local_planner_cb(self, msg):
        self.time_init_local_planner = rospy.get_time()

    def current_control_tf_cb(self, msg):
        self.current_control_tf = msg.data

    """

    #####    ##   #####    ##   #    #    #    # #####  #####    ##   ##### ######
    #    #  #  #  #    #  #  #  ##  ##    #    # #    # #    #  #  #    #   #
    #    # #    # #    # #    # # ## #    #    # #    # #    # #    #   #   #####
    #####  ###### #####  ###### #    #    #    # #####  #    # ######   #   #
    #      #    # #   #  #    # #    #    #    # #      #    # #    #   #   #
    #      #    # #    # #    # #    #     ####  #      #####  #    #   #   ######

    """

    def dynamic_reconfig_movebase_qr(self, slow_speed):
        client_reconfig_movebase = dynamic_reconfigure.client.Client(
            "/move_base/NeoLocalPlanner",
            timeout=300,
            config_callback=self.dynamic_callback_qr,
        )
        new_config = {"slow_speed": slow_speed}
        for i in range(3):
            client_reconfig_movebase.update_configuration(new_config)
            rospy.sleep(0.1)

    def dynamic_reconfig_movebase(self, rule_direction):
        rospy.loginfo("update dynamic reconfig")
        client = dynamic_reconfigure.client.Client(
            "/move_base/AgfLocalPlannerROS",
            timeout=300,
            config_callback=self.dynamic_callback,
        )
        if not self.have_pallet:
            max_vel_x = 0.7
            percent_vel = 0.2
        else:
            max_vel_x = 0.4
            percent_vel = 0.3
        new_config = {
            "max_vel_x": max_vel_x,
            "percent_vel": percent_vel,
            "rule_direction": rule_direction,
        }
        client.update_configuration(new_config)

    """
    ######## ##     ## ########  ######  ##     ## ######## ########
    ##        ##   ##  ##       ##    ## ##     ##    ##    ##
    ##         ## ##   ##       ##       ##     ##    ##    ##
    ######      ###    ######   ##       ##     ##    ##    ######
    ##         ## ##   ##       ##       ##     ##    ##    ##
    ##        ##   ##  ##       ##    ## ##     ##    ##    ##
    ######## ##     ## ########  ######   #######     ##    ########
    """

    def moving_control_exec_cb(self, goal):
        try:
            data_dict = json.loads(goal.data)
            if "path_follow_qr" in data_dict:
                self.path_qr_code = data_dict["path_follow_qr"]
            else:
                self.path_qr_code = []
            goal_accept = True
            rospy.loginfo("------------------Received goal:------------------")
            rospy.loginfo(json.dumps(data_dict, indent=2))
            if "params" not in data_dict:
                data_dict["params"] = {}
            if "reconfigure" not in data_dict["params"]:
                data_dict["params"]["reconfigure"] = {}
            # Set params in reconfigure from database params
            # TODO: check overwrite_planner_setting
            if "distance_threshold" in data_dict["params"]:
                data_dict["params"]["reconfigure"]["xy_tolerance"] = data_dict[
                    "params"
                ]["distance_threshold"]
                rospy.logwarn('"distance_threshold" not in data_dict params')
            # Get param from defaut param, prioritize for global param in root json tree
            data_dict["params"] = merge_two_dicts(
                self.default_params_dict["params"], data_dict["params"]
            )
            data_dict["params"]["reconfigure"] = merge_two_dicts(
                self.default_params_dict["params"]["reconfigure"],
                data_dict["params"]["reconfigure"],
            )
            # print_warn('Merge default params:')
            # print(json.dumps(data_dict, indent=2))
            # Get param from global param, prioritize for local param in each waypoint
            for i in range(len(data_dict["waypoints"])):
                if "params" not in data_dict["waypoints"][i]:
                    data_dict["waypoints"][i]["params"] = {}
                # TOCHECK: why not mege not empty ['reconfigure'] dict? Only copy this dict if not exist
                if "reconfigure" not in data_dict["waypoints"][i]["params"]:
                    data_dict["waypoints"][i]["params"]["reconfigure"] = {}
                # print_debug('''waypoint_{}'s params before merge:\n{}'''.format(i+1, json.dumps(data_dict['waypoints'][i]['params'], indent=2)))
                data_dict["waypoints"][i]["params"] = merge_two_dicts(
                    data_dict["params"], data_dict["waypoints"][i]["params"]
                )
                data_dict["waypoints"][i]["params"]["reconfigure"] = (
                    merge_two_dicts(
                        data_dict["params"]["reconfigure"],
                        data_dict["waypoints"][i]["params"]["reconfigure"],
                    )
                )
                # print_debug('''waypoint_{}'s params after merge:\n{}'''.format(i+1, json.dumps(data_dict['waypoints'][i]['params'], indent=2)))
            # print_warn('Merge local params:')
            # print(json.dumps(data_dict, indent=2))
            frame_id = data_dict["params"][
                "frame_id"
            ]  # Only use global frame_id, not yet use waypoint frame_id
            wp_display = self.get_wp_display(data_dict["waypoints"])
            self.waypoints_following_pub.publish(
                pose_stamped_array_to_pose_array(wp_display, frame_id)
            )
        except Exception as e:
            rospy.logerr("Action goal.data systax error: {}".format(e))
            self.send_feedback(self._as, "ABORTED")
            self._as.set_aborted(text="Action goal.data systax error")
            return

        # Check only use QR code
        # Hub, matehan, charge only use QR code and set this param
        if "only_use_qr" in data_dict["params"]:
            if data_dict["params"]["only_use_qr"] == True:
                only_use_qr = True
            else:
                only_use_qr = False
        else:
            only_use_qr = False

        # set direction for lidar
        if "rule_direction" in data_dict["params"]:
            rule_direction = data_dict["params"]["rule_direction"]
        else:
            rule_direction = 2

        # Set rules for path
        data_dict["waypoints"] = self.set_path_rule_for_wp(
            data_dict["waypoints"]
        )
        rospy.logwarn("Waypoints with path rule:")
        for i in range(len(data_dict["waypoints"])):
            rospy.logwarn(
                "{}: {}".format(
                    data_dict["waypoints"][i]["name"],
                    data_dict["waypoints"][i]["rule_path"],
                )
            )

        # ignore first waypoint
        self.bool_ignore_wp = False
        if len(data_dict["waypoints"]) > 1:
            self.bool_ignore_wp = True
            self.start_pose_plan.Pose.pose = dict_to_pose(
                data_dict["waypoints"][0]["position"]
            )
            # add offset to use move by map lidar
            self.offset_start_x = 0.0
            self.offset_start_y = 0.0
            if "data" in data_dict["waypoints"][0]:
                if "properties" in data_dict["waypoints"][0]["data"]:
                    if (
                        "offset_x"
                        in data_dict["waypoints"][0]["data"]["properties"]
                    ):
                        self.offset_start_x = float(
                            data_dict["waypoints"][0]["data"]["properties"][
                                "offset_x"
                            ]
                        )
            if "data" in data_dict["waypoints"][0]:
                if "properties" in data_dict["waypoints"][0]["data"]:
                    if (
                        "offset_y"
                        in data_dict["waypoints"][0]["data"]["properties"]
                    ):
                        self.offset_start_y = float(
                            data_dict["waypoints"][0]["data"]["properties"][
                                "offset_y"
                            ]
                        )
            if "data" in data_dict["waypoints"][0]:
                self.start_pose_plan.radius = data_dict["waypoints"][0]["data"][
                    "radius"
                ]
            else:
                self.start_pose_plan.radius = 0.0
            data_dict["waypoints"].pop(0)

        # always lift down when run
        if self.sensor_lift_min != 1 and not only_use_qr:
            self.have_pallet = True
            _state = MainState.LIFT_DOWN
        else:
            self.have_pallet = False
            _state = MainState.SEND_GOAL

        success = False
        _prev_state = MainState.NONE
        self.current_goal = 0
        action_goal = None
        action_client = None
        local_planner = ""
        last_action_client = None
        last_local_planner = ""
        current_target_pose = PoseStamped()
        status_msg = StringStamped()
        check_action_status = False
        max_retry_time = 0
        move_base_retry_cnt = 0
        last_loop_time = rospy.get_time()
        total_point = 0
        start_time = rospy.get_time()
        feedback_msg = ""
        self._asm.reset_flag()
        goal_dict = {}
        goal_dict["params"] = {}
        self.is_safety_stop = False
        self.old_state_near_slow_point = False
        self.list_wp_create_path = []

        # Loop
        polling_rate = 20.0
        NOP_TIME = 0.0001
        rate = rospy.Rate(polling_rate)
        t = rospy.get_time()
        diff_time = 1.0 / polling_rate

        while goal_accept and not rospy.is_shutdown():
            self._asm.action_running = True
            if rospy.get_time() - t < diff_time:
                # Must be sleep to prevent high CPU load
                rospy.sleep(NOP_TIME)
                continue
            else:
                # rospy.loginfo("{}".format(round(1.0/(rospy.get_time() - t - NOP_TIME), 2)))
                t = rospy.get_time()
            # Scan safety timeout
            # TOCHECK: vòng lặp bị trễ khi update reconfigure => MOVING_DISCONNECTED
            now = rospy.get_time()
            duration = now - last_loop_time
            if duration > 0.25:
                print_error("Loop time: {}".format(round(duration, 2)))
            last_loop_time = rospy.get_time()
            # Check cancel action
            if self._as.is_preempt_requested() or self._asm.reset_action_req:
                rospy.logwarn(
                    "Is preempt: {}".format(self._as.is_preempt_requested())
                )
                rospy.logwarn(
                    "Is reset_action_req: {}".format(self._asm.reset_action_req)
                )
                rospy.loginfo("%s: Preempted" % self._action_name)
                self.cancel_all_action()
                self._as.set_preempted()
                success = False
                self.send_feedback(self._as, "PREEMPTED")
                self.switch_control_tf(USE_TF_LIDAR)
                break
            # Set module status for action moving
            # this notice error to control_system, HMI
            if self._asm.module_status != ModuleStatus.ERROR:
                self._asm.error_code = ""
            if _state != MainState.PAUSED and _state != MainState.ERROR:
                self._asm.module_status = ModuleStatus.RUNNING
            if _state != _prev_state:
                rospy.loginfo(
                    "Main state: {} -> {}".format(
                        _prev_state.toString(), _state.toString()
                    )
                )
                _prev_state = _state
                feedback_msg = _state.toString()
                self._asm.module_state = _state.toString()
                first_changed_state = True
                # Publish module_status
                status_msg.stamp = rospy.Time.now()
                status_msg.data = json.dumps(
                    {
                        "status": self._asm.module_status.toString(),
                        "state": self._asm.module_state,
                        "error_code": self._asm.error_code,
                    }
                )
                self._asm.module_status_pub.publish(status_msg)
            self.send_feedback(self._as, feedback_msg)
            # log when can't update robot pose
            t_robot_pose = rospy.get_time() - self.last_robot_pose
            if t_robot_pose > 0.2:
                rospy.logerr("Missing robot_pose: {}".format(t_robot_pose))

            # State: LIFT_DOWN:
            if _state == MainState.LIFT_DOWN:
                if self.liftdown_finish:
                    _state = MainState.SEND_GOAL
                else:
                    self.lift_msg.stamp = rospy.Time.now()
                    self.lift_msg.data = 2
                    self.pub_lift_cmd.publish(self.lift_msg)
            # State: SEND_GOAL
            elif _state == MainState.SEND_GOAL:
                rospy.logwarn("Begin SEND_GOAL")
                offset_goal_x = 0.0
                offset_goal_y = 0.0
                offset_start_x = 0.0
                offset_start_y = 0.0
                wp = data_dict["waypoints"][self.current_goal]
                total_point = len(data_dict["waypoints"])
                if wp["rule_path"] == "qr_code" or only_use_qr:
                    # action hub, matehan, docking auto switch local planner
                    if not only_use_qr:
                        while not rospy.is_shutdown():
                            if (
                                rospy.get_param("/move_base/base_local_planner")
                                == "neo_local_planner/NeoLocalPlanner"
                            ):
                                break

                            if self.switch_local_planner(
                                "neo_local_planner/NeoLocalPlanner",
                                "waypoint_global_planner/WaypointGlobalPlanner",
                            ):
                                break
                            rospy.sleep(0.1)
                    use_qr_code = True
                else:
                    while not rospy.is_shutdown():
                        if (
                            rospy.get_param("/move_base/base_local_planner")
                            == "agf_local_planner/AgfLocalPlannerROS"
                        ):
                            break

                        if self.switch_local_planner(
                            "agf_local_planner/AgfLocalPlannerROS",
                            "waypoints_global_planner/WaypointGlobalPlanner",
                        ):
                            break
                        rospy.sleep(0.1)
                    use_qr_code = False
                # print_debug(json.dumps(wp['params'], indent=2))
                if use_qr_code:
                    # set local planner
                    local_planner = "neo_local_planner"
                    # publish use tf lidar map
                    self.switch_control_tf(USE_TF_QR_CODE)
                    # check QR in start point, when change from lidar to QR always read a QR
                    if self.last_state_use_qr_code == False:
                        _state = MainState.FIND_QR_CODE
                        continue
                    else:
                        rospy.logwarn("DEBUG last_state_use_qr_code: True")
                    # create list wp slow speed
                    self.list_slow_speed_qr(data_dict["waypoints"])
                    # create list wp for move_base qr
                    self.current_goal = self.create_wp_for_qr_mb(
                        data_dict["waypoints"]
                    )
                    # set goal for move by qr
                    goal_pose = data_dict["waypoints"][self.current_goal][
                        "position"
                    ]
                    rospy.logerr(self.list_wp_create_path)

                if not use_qr_code:
                    # set local planner
                    local_planner = "agf_local_planner"
                    # publish use tf lidar map
                    self.switch_control_tf(USE_TF_LIDAR)
                    rospy.logwarn(
                        'DEBUG use_qr_code: {}, modify_status not in wp["param"]: {}'.format(
                            use_qr_code, "modify_status" not in wp["params"]
                        )
                    )
                    # offset start pose for move by lidar
                    if self.current_goal > 0:
                        if (
                            "data"
                            in data_dict["waypoints"][self.current_goal - 1]
                        ):
                            if (
                                "properties"
                                in data_dict["waypoints"][
                                    self.current_goal - 1
                                ]["data"]
                            ):
                                if (
                                    "offset_x"
                                    in data_dict["waypoints"][
                                        self.current_goal - 1
                                    ]["data"]["properties"]
                                ):
                                    offset_start_x = float(
                                        data_dict["waypoints"][
                                            self.current_goal - 1
                                        ]["data"]["properties"]["offset_x"]
                                    )
                        if (
                            "data"
                            in data_dict["waypoints"][self.current_goal - 1]
                        ):
                            if (
                                "properties"
                                in data_dict["waypoints"][
                                    self.current_goal - 1
                                ]["data"]
                            ):
                                if (
                                    "offset_y"
                                    in data_dict["waypoints"][
                                        self.current_goal - 1
                                    ]["data"]["properties"]
                                ):
                                    offset_start_y = float(
                                        data_dict["waypoints"][
                                            self.current_goal - 1
                                        ]["data"]["properties"]["offset_y"]
                                    )

                    # create list wp for move_base lidar
                    self.current_goal = self.create_wp_for_mb(
                        data_dict["waypoints"], offset_start_x, offset_start_y
                    )
                    # create list low speed wp for move_base lidar
                    self.list_wp_low_speed(data_dict["waypoints"])
                    # set goal for move by lidar
                    # because diff map lidar and qr code, need offset to robot move to center qr code
                    if "data" in data_dict["waypoints"][self.current_goal]:
                        if (
                            "properties"
                            in data_dict["waypoints"][self.current_goal]["data"]
                        ):
                            if (
                                "offset_x"
                                in data_dict["waypoints"][self.current_goal][
                                    "data"
                                ]["properties"]
                            ):
                                offset_goal_x = float(
                                    data_dict["waypoints"][self.current_goal][
                                        "data"
                                    ]["properties"]["offset_x"]
                                )
                    if "data" in data_dict["waypoints"][self.current_goal]:
                        if (
                            "properties"
                            in data_dict["waypoints"][self.current_goal]["data"]
                        ):
                            if (
                                "offset_y"
                                in data_dict["waypoints"][self.current_goal][
                                    "data"
                                ]["properties"]
                            ):
                                offset_goal_y = float(
                                    data_dict["waypoints"][self.current_goal][
                                        "data"
                                    ]["properties"]["offset_y"]
                                )
                    self.array_wp_mb.Waypoints[
                        -1
                    ].Pose.pose.position.x += offset_goal_x
                    self.array_wp_mb.Waypoints[
                        -1
                    ].Pose.pose.position.y += offset_goal_y
                    goal_pose = data_dict["waypoints"][self.current_goal][
                        "position"
                    ]
                    # call dynamic reconfig move base
                    for i in range(3):
                        self.dynamic_reconfig_movebase(rule_direction)
                        rospy.sleep(0.2)

                # print_info("Next cycle dict: \n{}".format(json.dumps(data_dict['waypoints'], indent=2)))
                wp_display = self.get_wp_display(data_dict["waypoints"])
                del wp_display[: self.current_goal - 1]
                self.waypoints_following_pub.publish(
                    pose_stamped_array_to_pose_array(wp_display, frame_id)
                )

                current_target_pose.header.stamp = rospy.Time.now()
                current_target_pose.header.frame_id = frame_id
                current_target_pose.pose = dict_to_pose(goal_pose)
                current_target_pose.pose.position.x += offset_goal_x
                current_target_pose.pose.position.y += offset_goal_y
                rospy.loginfo(
                    "---------------------------wp: {}/{}---------------------------".format(
                        self.current_goal, total_point
                    )
                )
                rospy.loginfo(
                    "wp name: {}".format(
                        data_dict["waypoints"][self.current_goal]["name"]
                    )
                )
                rospy.loginfo("\n{}".format(current_target_pose.pose))
                if "retries" in wp["params"]:
                    max_retry_time = wp["params"]["retries"]

                self.current_local_planner = local_planner
                # print('local_planner: ' + local_planner)
                # print('---')
                action_goal = self.local_planner_var[local_planner][
                    "action_goal"
                ]
                action_goal.target_pose = current_target_pose
                action_client = self.local_planner_var[local_planner][
                    "action_client"
                ]
                # TOCHECK: Cancel prev goal
                self.move_action_result = -1
                check_action_status = True
                action_client.send_goal(action_goal, feedback_cb=self.action_fb)
                last_action_client = action_client
                last_local_planner = local_planner
                self.last_action_fb = rospy.get_time()
                start_time = rospy.get_time()
                _state = MainState.MOVING
            # State: FIND_QR_CODE
            elif _state == MainState.FIND_QR_CODE:
                self.last_state_use_qr_code = True
                if rospy.get_time() - self.last_time_read_qr < 0.1:
                    _state = MainState.SEND_GOAL
                    continue
                if rospy.get_time() - self.last_time_read_qr > 5.0:
                    if not self.disable_check_qr_code:
                        _state = MainState.FIND_QR_CODE_ERROR
                    else:
                        self.disable_check_qr_code = False
                        _state = MainState.SEND_GOAL
                        continue
                    # _state = MainState.SEND_GOAL
            # State: RE_SEND_GOAL
            elif _state == MainState.RE_SEND_GOAL:
                self.move_action_result = -1
                check_action_status = True
                if (
                    last_action_client != None
                    and last_local_planner != local_planner
                ):
                    print_debug("Cancel all goal of: {}".format(action_client))
                    last_action_client.cancel_all_goals()
                action_client.send_goal(action_goal, feedback_cb=self.action_fb)
                last_action_client = action_client
                last_local_planner = local_planner
                self.last_action_fb = rospy.get_time()
                _state = MainState.MOVING
            # State: MOVING
            elif _state == MainState.MOVING:
                # check slow speed
                if use_qr_code:
                    near_slow_point = self.check_pose_near_slow_point()

                    if (
                        near_slow_point
                        and self.old_state_near_slow_point == False
                    ):
                        self.dynamic_reconfig_movebase_qr(True)
                        self.old_state_near_slow_point = True

                    if self.old_state_near_slow_point and not near_slow_point:
                        self.dynamic_reconfig_movebase_qr(False)
                        self.old_state_near_slow_point = False

                # Pause handle
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    _state = MainState.PAUSED
                    rospy.logwarn("going to PAUSE")
                    if use_qr_code:
                        self.run_pause_pub.publish(
                            StringStamped(stamp=rospy.Time.now(), data="RUN")
                        )
                        rospy.sleep(0.2)
                        self.run_pause_pub.publish(
                            StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                        )
                    else:
                        self.cancel_all_action()
                    continue

                if check_action_status == True:
                    if self.move_action_result == goal_result.SUCCEEDED:
                        rospy.loginfo(
                            "ACCURACY - move_base_status: %s"
                            % self.move_action_result
                        )
                        _state = MainState.DONE

                # Action ABORTED check
                if (
                    check_action_status == True
                    and self.move_action_result == goal_result.ABORTED
                ):
                    move_base_retry_cnt += 1
                    rospy.logwarn(
                        "Retry move_base when ABORTED: {} times".format(
                            move_base_retry_cnt
                        )
                    )
                    if move_base_retry_cnt >= max_retry_time:
                        rospy.logerr(
                            "Error after try {} times".format(max_retry_time)
                        )
                        _state = MainState.ERROR
                        if not use_qr_code:
                            self.cancel_all_action()
                        else:
                            self.run_pause_pub.publish(
                                StringStamped(
                                    stamp=rospy.Time.now(), data="PAUSE"
                                )
                            )
                    else:
                        _state = MainState.RE_SEND_GOAL
                    # check_action_status = False
            # State: DONE
            elif _state == MainState.DONE:
                move_base_retry_cnt = 0
                # Measure running time
                duration = rospy.get_time() - start_time
                rospy.loginfo("Done with %i(s)", duration)
                # Next waypoint
                self.current_goal += 1
                if self.current_goal == len(data_dict["waypoints"]):
                    self.last_state_use_qr_code = False
                    success = True
                    break
                else:
                    # Remove finished waypoint
                    wp_display.pop(0)
                    self.waypoints_following_pub.publish(
                        pose_stamped_array_to_pose_array(wp_display, frame_id)
                    )
                    _state = MainState.SEND_GOAL
            # State: PAUSED
            elif _state == MainState.PAUSED:
                self._asm.module_status = ModuleStatus.PAUSED
                if self._asm.resume_req:
                    self._asm.reset_flag()
                    if not use_qr_code:
                        _state = MainState.RE_SEND_GOAL
                    else:
                        self.run_pause_pub.publish(
                            StringStamped(stamp=rospy.Time.now(), data="RUN")
                        )
                        _state = MainState.MOVING
                    # print_debug("Resume waypoint number: {}".format(self.current_goal+1))
            # State: ERROR
            elif _state == MainState.ERROR:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = self._asm.error_code = (
                    "/moving_control: {}".format(_state.toString())
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = MainState.RE_SEND_GOAL
            # State: FIND_QR_CODE_ERROR
            elif _state == MainState.FIND_QR_CODE_ERROR:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = self._asm.error_code = (
                    "/moving_control: {}".format(_state.toString())
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = MainState.FIND_QR_CODE

            # Update prev state
            # self.last_safety_fields = self.safety_fields
        self.switch_control_tf(USE_TF_LIDAR)
        self._asm.action_running = False
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
        r = rospy.Rate(2.0)
        moving_status_time = rospy.get_time()
        status_msg = StringStamped()
        while not rospy.is_shutdown():
            r.sleep()
            if not self._asm.action_running:
                self._asm.module_status = ModuleStatus.WAITING
                self._asm.module_state = MainState.WAITING.toString()
                self._asm.error_code = ""
            now = rospy.get_time()
            if now - moving_status_time >= 0.5:
                moving_status_time = now
                status_msg.stamp = rospy.Time.now()
                status_msg.data = json.dumps(
                    {
                        "status": self._asm.module_status.toString(),
                        "state": self._asm.module_state,
                        "error_code": self._asm.error_code,
                    }
                )
                self._asm.module_status_pub.publish(status_msg)


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
        "--planner_map",
        dest="planner_map",
        default=os.path.join(
            rospkg.RosPack().get_path("moving_control"),
            "cfg",
            "planner_map.yaml",
        ),
        type=str,
        help="planner_map config file path",
    )
    parser.add_option(
        "--planner_setting",
        dest="planner_setting",
        default=os.path.join(
            rospkg.RosPack().get_path("moving_control"),
            "cfg",
            "planner_setting.yaml",
        ),
        type=str,
        help="planner_setting config file path",
    )
    parser.add_option(
        "--default_params",
        dest="default_params",
        default=os.path.join(
            rospkg.RosPack().get_path("moving_control"),
            "cfg",
            "default_params.json",
        ),
        type=str,
        help="default_param config file path",
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
    rospy.init_node("moving_control", log_level=log_level, disable_signals=True)
    rospy.loginfo("Init node " + rospy.get_name())
    MovingControl(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
