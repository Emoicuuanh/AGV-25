#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import json
import threading
import yaml
import numpy as np
import argparse
from rosgraph.names import is_private
import rospy
import rospkg
import tf
import copy
import actionlib
import math
from math import pi, degrees, hypot, atan2
import time
from nav_msgs.msg import Odometry
import dynamic_reconfigure.client
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Twist
from std_msgs.msg import Int8, Header, Float32, Bool
from agv_msgs.msg import WaypointGlobal
from agv_msgs.srv import (
    WaypointsPath,
    WaypointsPathResponse,
    DirectionMove,
    DirectionMoveResponse,
)
from std_stamped_msgs.msg import (
    StringStamped,
    StringFeedback,
    StringResult,
    StringAction,
    EmptyStamped,
    Int8Stamped,
)
from std_stamped_msgs.srv import StringService, StringServiceResponse
from agv_msgs.msg import DataMatrixStamped
from actionlib_msgs.msg import GoalStatus, GoalID, GoalStatusArray
from safety_msgs.msg import SafetyStatus
from nav_msgs.msg import Path

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
from mongodb import mongodb

from module_manager import ModuleServer, ModuleClient, ModuleStatus
from common_function import (
    EnumString,
    lockup_pose,
    obj_to_dict,
    offset_pose_x,
    pose_stamped_array_to_pose_array,
    distance_two_pose,
    get_yaw,
    delta_angle,
    dict_to_obj,
    merge_two_dicts,
    print_debug,
    print_warn,
    print_error,
    print_info,
    MIN_FLOAT,
    distance_to_line_perpendicular_vs_goal,
    angle_robot_vs_robot_to_goal,
    yaw_to_quaternion,
    pose_dict_template,
    angle_vector_two_point,
    dict_to_pose,
)


class MainState(EnumString):
    NONE = -1
    SEND_GOAL = 0
    MOVING = 1
    RE_SEND_GOAL = 2
    RETURN_TO_RETRY = 3
    DONE = 6
    ERROR = 7
    STOP_BY_SAFETY = 8
    PAUSED = 9
    WAITING = 10
    WAIT_CLEAR_SAFETY = 11  # TODO
    LOST_QR = 12
    WAITING_AT_POSE = 13
    FIND_QR_CODE = 14
    FIND_QR_CODE_ERROR = 15


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
        rospy.on_shutdown(self.shutdown)
        # Publisher
        self.control_tf_pub = rospy.Publisher("/control_tf", Int8, queue_size=5)
        self.run_pause_pub = rospy.Publisher(
            "/run_pause_req", StringStamped, queue_size=5
        )
        self.current_goal_publisher = rospy.Publisher(
            "/current_goal_monving_control", PoseStamped, queue_size=1
        )
        self.waypoints_following_pub = rospy.Publisher(
            "/waypoints_following", PoseArray, queue_size=5
        )
        self.path_publisher = rospy.Publisher(
            "/current_path", Path, queue_size=10
        )
        self.safety_job_pub = rospy.Publisher(
            "/safety_job_by_point_name", StringStamped, queue_size=1
        )
        rospy.Subscriber("/safety_job_name", StringStamped, self.set_safety_cb)

        # Subscriber
        rospy.Subscriber("/lidar_info", StringStamped, self.lidar_info_cb)
        rospy.Subscriber("/data_gls621", DataMatrixStamped, self.data_qr_cb)
        rospy.Subscriber(
            "/disable_check_error_qr_code",
            Int8Stamped,
            self.disable_check_error_qr_code_cb,
        )
        rospy.Subscriber(
            "/current_control_tf", Int8, self.current_control_tf_cb
        )
        rospy.Subscriber(
            "/current_traffic_control_type",
            StringStamped,
            self.current_traffic_control_type_cb,
        )
        rospy.Subscriber("/route_receive", StringStamped, self.route_callback)
        rospy.Subscriber(
            "/mission_manager/module_status",
            StringStamped,
            self.mission_status_cb,
        )
        rospy.Subscriber("/safety_status", SafetyStatus, self.safety_status_cb)
        rospy.Subscriber("/stop_moving", EmptyStamped, self.stop_moving_cb)
        rospy.Subscriber("/robot_pose", Pose, self.robot_pose_cb)
        rospy.Subscriber("~fake_error", EmptyStamped, self.fake_error_cb)
        rospy.Subscriber(
            "/robot_state_direction",
            StringStamped,
            self.robot_state_direction_cb,
        )

        # SERVICE
        rospy.Service(
            "/get_type_move", StringService, self.handle_get_type_move
        )
        rospy.Service(
            "waypoints_to_global",
            WaypointsPath,
            self.handle_get_wp_create_path,
        )
        rospy.Service(
            "get_diriction_move", DirectionMove, self.handle_get_direction_move
        )
        # dynamic reconfig client
        self.client_reconfig_movebase = dynamic_reconfigure.client.Client(
            "/move_base/NeoLocalPlanner",
            timeout=300,
            config_callback=self.dynamic_callback,
        )

        # ModuleServer
        self._asm = ModuleServer(name)
        self.last_module_status = self._asm.module_status
        # Initial
        self.planner_init(path=kwargs["planner_map"])
        self.load_default_params(path=kwargs["default_params"])
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
        #
        self.planner_map = None
        self.planner_setting_dict = None
        self.current_local_planner = ""
        self.moving_direction = FORWARD
        self.dynamic_global_var = {}
        self.default_params_dict = {}
        self.safety_fields = []
        self.list_safety_job_point = []
        self.list_slow_point = []
        self.list_wp_create_path = []
        # Flag vars
        self.is_safety_stop = False
        self.fake_error = False
        self.old_state_near_slow_point = False
        self.reset_safety_point = False
        self.current_safety_job_name_auto_mode = ""
        self.move_action_status = -1
        self.move_action_result = -1
        self.prev_move_action_status = -1
        self.direction_move = 0
        self.direction = FORWARD
        self.last_action_fb = rospy.get_time()
        self.last_scan_safety = rospy.get_time()
        self.safety_name = ""
        # TODO: Get from config

        self.robot_pose = Pose()
        self.msg_safety = StringStamped()

        self.current_action_type = ""
        self.pre_action_type = ""

        # traffic
        self.new_route_data = ""
        self.is_new_route = False
        self.need_to_wait_receive_new_path = False
        self.is_change_path_when_move = True
        self.use_new_traffic_control = False
        self.check_same_goal_when_receive_new_route = False
        self.trigger_resend_goal = False
        self.last_time_send_goal = rospy.get_time()

        # Database
        db_address = rospy.get_param("/mongodb_address")
        print_debug(db_address)
        self.db = mongodb(db_address)

        # type localisation
        self.current_control_tf = 100

        # last local plan use
        self.type_move = "QR"
        self.last_state_use_qr_code = False
        self.use_lidar = True
        self.path_qr_code = []

        # check lost qr
        self.disable_check_qr_code = False
        self.last_time_read_qr = rospy.get_time()

        # Lidar info for coordinate conversion
        self.lidar_info = None
        self.lidar_scale = None
        self.lidar_offset_x = None
        self.lidar_offset_y = None
        self.lidar_angle = None
        self.has_lidar_info = False

    def shutdown(self):
        self.cancel_all_action()
        print("Shutdown: {}".format(rospy.get_name()))

    def cancel_all_action(self):
        for key, value in self.dynamic_global_var.items():
            self.dynamic_global_var[key]["action_cancel_pub"].publish(
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

    def set_path_rule_for_wp(self, waypoints):
        if not waypoints or not isinstance(waypoints, list):
            return waypoints

        # Reset rule_path
        for wp in waypoints:
            wp["rule_path"] = ""

        if len(waypoints) > 1:
            for i in range(len(waypoints) - 1):
                current_wp = waypoints[i]
                current_name = current_wp.get("name")
                next_wp = waypoints[i + 1]
                next_name = next_wp.get("name")

                data = current_wp.get("data", {})
                next_list = data.get("next", [])
                road_type_json = data.get("road_type", "{}")

                try:
                    road_type_dict = json.loads(road_type_json)
                except Exception as e:
                    rospy.logwarn(
                        "Failed to parse road_type JSON: {}".format(e)
                    )
                    road_type_dict = {}

                # if next_name == "pose_17519464978":
                #     rospy.logerr("0 --- check rule path pose_17519464978")
                rospy.logwarn(
                    "current_wp_name: {} ---> next_wp_name: {}".format(
                        current_name, next_name
                    )
                )

                if next_name in next_list:
                    road_info = road_type_dict.get(next_name)
                    if road_info and "road_move_type" in road_info:
                        if road_info["road_move_type"] == "lidar":
                            next_wp["rule_path"] = "lidar"
                        else:
                            next_wp["rule_path"] = "qr"
                        rospy.logerr(
                            "1 --- {} --> {}".format(
                                next_name, road_info["road_move_type"]
                            )
                        )
                    else:
                        next_wp["rule_path"] = "qr"
                        rospy.logerr(
                            "2 --- {} --> qr (default)".format(next_name)
                        )
                else:
                    next_wp["rule_path"] = "qr"
                    rospy.logerr(
                        "3 --- {} not in current next list".format(next_name)
                    )
        else:
            if rospy.get_time() - self.last_time_read_qr < 1:
                waypoints[0]["rule_path"] = "qr"
            else:
                waypoints[0]["rule_path"] = "lidar"

        return waypoints

    def create_wp_for_mb(self, array_wp):
        rospy.loginfo(
            "call create_wp_for_mb with self.current_point: {}".format(
                self.current_point
            )
        )
        self.list_wp_create_path = []
        numble_mb_goal = 1

        # Handle current_point if valid
        if self.current_point >= 0 and self.current_point < len(array_wp):
            wp_data = array_wp[self.current_point]
            offset_x = float(
                wp_data.get("data", {})
                .get("properties", {})
                .get("offset_x", 0.0)
            )
            offset_y = float(
                wp_data.get("data", {})
                .get("properties", {})
                .get("offset_y", 0.0)
            )

            wp_mb = WaypointGlobal()
            pose_stamped = PoseStamped()
            pose_stamped.pose = dict_to_pose(wp_data["position"])
            
            # Convert coordinate if lidar info is available
            if self.has_lidar_info:
                original_x = pose_stamped.pose.position.x
                original_y = pose_stamped.pose.position.y
                
                converted_x, converted_y = self.convert_qr_to_lidar_coordinate(original_x, original_y)
                
                if converted_x is not None and converted_y is not None:
                    pose_stamped.pose.position.x = converted_x
                    pose_stamped.pose.position.y = converted_y
                    rospy.loginfo(f"Converted current point: ({original_x}, {original_y}) -> ({converted_x}, {converted_y})")
                else:
                    rospy.logwarn("Failed to convert current point coordinates")
            else:
                rospy.loginfo("No lidar info available, using original coordinates for current point")
            
            pose_stamped.pose.position.x += offset_x
            pose_stamped.pose.position.y += offset_y
            wp_mb.Pose = pose_stamped
            wp_mb.radius = 0.0

            self.list_wp_create_path.append(wp_mb)
            rospy.logwarn("append current_point pose: {}".format(wp_mb))

        # Loop through remaining waypoints
        for i in range(self.current_point + 1, len(array_wp)):
            wp_data = array_wp[i]
            rule_path = wp_data.get("rule_path", "")
            
            rotate_goal = False
            # Kiểm tra điều kiện đầu tiên trong vòng loop hoặc list_wp_create_path rỗng
            is_first_point = (i == self.current_point + 1) or (len(self.list_wp_create_path) == 0)
            
            if (
                "data" in wp_data
                and "properties" in wp_data["data"]
                # and not is_first_point  # Chỉ check rotate_goal khi không phải điểm đầu
            ):
                if (
                    "rotate_goal" in wp_data["data"]["properties"]
                    and wp_data["data"]["properties"]["rotate_goal"]
                ):
                    # Log thông tin waypoint là rotate_goal
                    wp_name = wp_data.get("name", f"waypoint_{i}")
                    rospy.logwarn("Waypoint {} (index {}) is ROTATE_GOAL".format(wp_name, i))
                    rotate_goal = True

            if rule_path != "qr":
                offset_x = float(
                    wp_data.get("data", {})
                    .get("properties", {})
                    .get("offset_x", 0.0)
                )
                offset_y = float(
                    wp_data.get("data", {})
                    .get("properties", {})
                    .get("offset_y", 0.0)
                )

                wp_mb = WaypointGlobal()
                pose_stamped = PoseStamped()
                pose_stamped.pose = dict_to_pose(wp_data["position"])
                
                # Convert coordinate if lidar info is available
                if self.has_lidar_info:
                    original_x = pose_stamped.pose.position.x
                    original_y = pose_stamped.pose.position.y
                    
                    converted_x, converted_y = self.convert_qr_to_lidar_coordinate(original_x, original_y)
                    
                    if converted_x is not None and converted_y is not None:
                        pose_stamped.pose.position.x = converted_x
                        pose_stamped.pose.position.y = converted_y
                        rospy.loginfo(f"Converted waypoint {i}: ({original_x}, {original_y}) -> ({converted_x}, {converted_y})")
                    else:
                        rospy.logwarn(f"Failed to convert waypoint {i} coordinates")
                else:
                    rospy.logdebug(f"No lidar info available, using original coordinates for waypoint {i}")
                
                pose_stamped.pose.position.x += offset_x
                pose_stamped.pose.position.y += offset_y
                wp_mb.Pose = pose_stamped

                if "data" in wp_data and "radius" in wp_data["data"]:
                    wp_mb.radius = wp_data["data"]["radius"]
                elif "position" in wp_data and "radius" in wp_data["position"]:
                    wp_mb.radius = wp_data["position"]["radius"]
                else:
                    wp_mb.radius = 0.0

                self.list_wp_create_path.append(wp_mb)
                rospy.logwarn("append additional pose: {}".format(wp_mb))
                numble_mb_goal = i
                # nếu là điểm xoay thì return path luôn
                if rotate_goal:
                    return numble_mb_goal
            else:
                # Encounter QR path => stop appending and return
                return numble_mb_goal

        return numble_mb_goal

    def create_wp_for_qr_mb(self, array_wp):
        rospy.loginfo(
            "call create_wp_for_qr_mb with self.current_point: {}".format(
                self.current_point
            )
        )
        self.list_wp_create_path = []

        if self.current_point >= 0:
            wp_mb = WaypointGlobal()
            pose_in_wp = PoseStamped()
            pose_in_wp.pose = dict_to_pose(
                array_wp[self.current_point]["position"]
            )
            wp_mb.Pose = pose_in_wp
            wp_mb.radius = 0.0
            self.list_wp_create_path.append(wp_mb)
            rospy.logwarn(
                "append pose for list_wp_create_path: : {}".format(pose_in_wp)
            )

        numble_mb_goal = 1
        for i in range(len(array_wp)):
            if i <= self.current_point:
                continue
            _goal_current = (
                array_wp[i]["position"]
                if array_wp[i]["rule_path"] == "qr"
                else None
            )
            _goal_next = (
                array_wp[i + 1]["position"]
                if i < len(array_wp) - 1
                and array_wp[i + 1]["rule_path"] == "qr"
                else None
            )

            if _goal_current is not None and _goal_next is not None:
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
                    wp_mb = WaypointGlobal()
                    pose_in_wp = PoseStamped()
                    pose_in_wp.pose = dict_to_pose(
                        array_wp[numble_mb_goal]["position"]
                    )
                    wp_mb.Pose = pose_in_wp
                    wp_mb.radius = 0.0
                    self.list_wp_create_path.append(wp_mb)
                    rospy.logwarn(
                        "append pose for list_wp_create_path: : {}".format(
                            pose_in_wp
                        )
                    )
                    return numble_mb_goal

            elif _goal_current is not None:
                numble_mb_goal = i
                wp_mb = WaypointGlobal()
                pose_in_wp = PoseStamped()
                pose_in_wp.pose = dict_to_pose(
                    array_wp[numble_mb_goal]["position"]
                )
                wp_mb.Pose = pose_in_wp
                wp_mb.radius = 0.0
                self.list_wp_create_path.append(wp_mb)
                rospy.logwarn(
                    "append pose for list_wp_create_path: : {}".format(
                        pose_in_wp
                    )
                )
                return numble_mb_goal

        return numble_mb_goal

    def switch_control_tf(self, type):
        while not rospy.is_shutdown():
            self.control_tf_pub.publish(Int8(data=type))
            if self.current_control_tf == type:
                break
            rospy.sleep(0.1)

    def point_to_segment_distance(self, pose_C, pose_A, pose_B):
        # Chuyển các Pose thành tọa độ x, y
        xC, yC = pose_C.position.x, pose_C.position.y
        x1, y1 = pose_A.position.x, pose_A.position.y
        x2, y2 = pose_B.position.x, pose_B.position.y

        # Vector AB
        dx = x2 - x1
        dy = y2 - y1

        # Nếu A và B là cùng một điểm, trả về khoảng cách từ C đến A
        epsilon = 0.01
        if abs(dx) < epsilon and abs(dy) < epsilon:
            return hypot(xC - x1, yC - y1)

        # Tính hệ số t (vị trí chiếu của C lên đoạn AB theo t từ 0 đến 1)
        t = ((xC - x1) * dx + (yC - y1) * dy) / (dx**2 + dy**2)

        # Giới hạn t trong đoạn [0, 1] để chiếu lên đoạn (không phải đường thẳng vô hạn)
        t = max(0, min(1, t))

        # Tính tọa độ điểm chiếu gần nhất trên đoạn AB
        nearest_x = x1 + t * dx
        nearest_y = y1 + t * dy

        # Tính khoảng cách từ C đến điểm gần nhất này
        return hypot(xC - nearest_x, yC - nearest_y)

    def is_same_position(self, pose1, pose2, xy_epsilon=0.01):
        dx = abs(pose1.position.x - pose2.position.x)
        dy = abs(pose1.position.y - pose2.position.y)
        return dx < xy_epsilon and dy < xy_epsilon

    def get_safety_job_by_points(self):
        if len(self.list_safety_job_point) < 1:
            return False

        for i in range(len(self.list_safety_job_point)):
            pose = dict_to_pose(self.list_safety_job_point[i]["position"])

            dx = self.robot_pose.position.x - pose.position.x
            dy = self.robot_pose.position.y - pose.position.y
            dis = np.sqrt(dx * dx + dy * dy)

            if dis < 0.5:
                safety_str = self.list_safety_job_point[i]["data"][
                    "properties"
                ]["Safety"]

                parts = safety_str.split("-")
                name = parts[0]
                if len(parts) > 1:
                    try:
                        orient_list = [float(ang) for ang in parts[1:]]

                        # Robot yaw in degrees
                        quat = self.robot_pose.orientation
                        (_, _, robot_yaw) = (
                            tf.transformations.euler_from_quaternion(
                                [quat.x, quat.y, quat.z, quat.w]
                            )
                        )
                        robot_yaw_deg = np.degrees(robot_yaw)

                        if self.direction == BACKWARD:  # BACKWARD
                            robot_yaw_deg = (robot_yaw_deg + 180) % 360

                        for target_yaw in orient_list:
                            angle_diff = abs(
                                (robot_yaw_deg - target_yaw + 180) % 360 - 180
                            )
                            if angle_diff < 30:
                                return name

                    except Exception as e:
                        rospy.logwarn(
                            f"Invalid Safety orientation list: {safety_str}"
                        )
                        continue
                else:
                    return name  # Chỉ có tên, không có góc

        return False

    def manager_safety(self):
        safety_job_name = self.get_safety_job_by_points()
        if safety_job_name:
            if safety_job_name == "" or safety_job_name == "Disable":
                self.msg_safety.data = ""
            elif (
                self.safety_name == "FORWARD" or self.safety_name == "BACKWARD"
            ):
                self.msg_safety.data = self.safety_name + "_" + safety_job_name
            elif self.safety_name == "ROTATION":
                self.msg_safety.data = "ROTATION"
            else:
                return
            self.msg_safety.stamp = rospy.Time.now()
            self.safety_job_pub.publish(self.msg_safety)
            self.reset_safety_point = True
        elif self.reset_safety_point:
            self.msg_safety.stamp = rospy.Time.now()
            self.msg_safety.data = self.current_safety_job_name_auto_mode
            self.safety_job_pub.publish(self.msg_safety)
            self.reset_safety_point = False

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

    def dynamic_reconfig_movebase(self, slow_speed):
        new_config = {"slow_speed": slow_speed}
        for i in range(3):
            self.client_reconfig_movebase.update_configuration(new_config)
            rospy.sleep(0.01)

    def check_distance_to_near_cross_line(self, current_pose):
        if current_pose == None:
            return
        if len(self.ori_waypoints) > 0:
            pose_obj = dict_to_pose(self.ori_waypoints[0]["position"])
            distance = distance_two_pose(current_pose, pose_obj)
            if distance < 0.4:
                if len(self.ori_waypoints) > 1:
                    pose_obj_next = dict_to_pose(
                        self.ori_waypoints[1]["position"]
                    )
                    distance_cur_goal_to_next_goal = distance_two_pose(
                        pose_obj_next, pose_obj
                    )
                    if distance_cur_goal_to_next_goal < 1:
                        self.ori_waypoints.pop(0)
                        return
                self.is_near_target_pose = True
            if self.is_near_target_pose and distance > 1:
                self.ori_waypoints.pop(0)
                self.is_near_target_pose = False
                return
            self.current_goal_publisher.publish(
                PoseStamped(
                    header=Header(
                        stamp=rospy.Time.now(),
                        frame_id=self.map_frame,
                    ),
                    pose=pose_obj,
                )
            )

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
            if key not in self.dynamic_global_var:
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

                # Create reconfigure publisher
                reconfigure_pub = None
                # reconfigure_pub = rospy.Publisher("/move_to_point/update_reconfigure", StringStamped, queue_size=1)
                _cmd = """global temp; temp = rospy.Publisher('/{}/update_reconfigure', StringStamped, queue_size=5)""".format(
                    action_server_name
                )
                print("cmd -->: " + _cmd)
                exec(_cmd)
                reconfigure_pub = temp

                # Assign object with a key
                planner_dict = {}
                planner_dict["action_client"] = action_client
                planner_dict["action_goal"] = action_goal
                planner_dict["action_cancel_pub"] = action_cancel_pub
                planner_dict["reconfigure_pub"] = reconfigure_pub
                self.dynamic_global_var[key] = planner_dict
                print("---")
        print("dynamic_global_var:")
        for key, value in self.dynamic_global_var.items():
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

    """

     ####    ##   #      #      #####    ##    ####  #    #
    #    #  #  #  #      #      #    #  #  #  #    # #   #
    #      #    # #      #      #####  #    # #      ####
    #      ###### #      #      #    # ###### #      #  #
    #    # #    # #      #      #    # #    # #    # #   #
     ####  #    # ###### ###### #####  #    #  ####  #    #

    """

    def lidar_info_cb(self, msg):
        """Callback function for /lidar_info topic"""
        try:
            lidar_data = json.loads(msg.data)
            rospy.loginfo(f"Received lidar info: {lidar_data}")
            
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
                
                rospy.loginfo(f"Updated lidar conversion params: scale={self.lidar_scale}, "
                            f"offset=({self.lidar_offset_x}, {self.lidar_offset_y}), "
                            f"angle={self.lidar_angle}")
            else:
                rospy.logwarn("Incomplete lidar info received, missing required fields")
                self.has_lidar_info = False
                
        except (json.JSONDecodeError, ValueError, KeyError) as e:
            rospy.logerr(f"Error parsing lidar info: {e}")
            self.has_lidar_info = False

    def data_qr_cb(self, msg):
        self.last_time_read_qr = rospy.get_time()

    def disable_check_error_qr_code_cb(self, msg):
        rospy.logwarn("Disable check qr code")
        if msg.data == 1:
            self.disable_check_qr_code = True
        elif msg.data == 0:
            self.disable_check_qr_code = False

    def current_control_tf_cb(self, msg):
        self.current_control_tf = msg.data

    def robot_state_direction_cb(self, msg):
        try:
            data_dict = json.loads(msg.data)
            if "state" in data_dict and "direction" in data_dict:
                if data_dict["direction"] == "FORWARD":
                    self.direction = FORWARD
                else:
                    self.direction = BACKWARD
                if data_dict["state"] == "TRANSLATING":
                    if data_dict["direction"] == "FORWARD":
                        self.safety_name = "FORWARD"
                    else:
                        self.safety_name = "BACKWARD"
                else:
                    self.safety_name = "ROTATION"
        except:
            rospy.logerr("robot_state_direction_cb error")

    def set_safety_cb(self, msg):
        self.current_safety_job_name_auto_mode = msg.data

    def current_traffic_control_type_cb(self, msg):
        try:
            if msg.data == "true":
                self.use_new_traffic_control = True
            else:
                self.use_new_traffic_control = False
        except Exception as e:
            rospy.logerr(f"traffic_control_type_cb error: {e}")

    def route_callback(self, msg):
        if self.use_new_traffic_control:
            rospy.logwarn("receive new route")
            self.new_route_data = msg.data
            self.is_new_route = True
        else:
            rospy.logwarn("ignore receive new route because use old traffic")
            self.new_route_data = ""
            self.is_new_route = False

    def dynamic_callback(config, level):
        # rospy.loginfo("Suceeed change vel of robot")
        pass

    def robot_pose_cb(self, msg):
        self.robot_pose = msg

    def mission_status_cb(self, msg):
        data_dict = json.loads(msg.data)
        self.current_action_type = data_dict["current_action_type"]

    def handle_get_type_move(self, req):
        return StringServiceResponse(self.type_move)

    def handle_get_wp_create_path(self, req):
        rospy.logwarn("Receive service request")
        response = WaypointsPathResponse()
        response.Waypoints = self.list_wp_create_path
        response.use_lidar = self.use_lidar
        rospy.logwarn(self.list_wp_create_path)
        return response

    def handle_get_direction_move(self, req):
        rospy.logwarn("Receive get direction move service request")
        response = DirectionMoveResponse()
        response.direction = self.direction_move
        return response

    def stop_moving_cb(self, msg):
        rospy.logwarn("Stop moving request")
        self.cancel_all_action()
        self._asm.pause_req = True

    def action_result_cb(self, msg):
        self.move_action_result = msg.status.status
        print_warn(self.move_action_result)

    def fake_error_cb(self, msg):
        self.fake_error = True

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

    def safety_status_cb(self, msg):
        self.last_scan_safety = rospy.get_time()
        self.safety_fields = list(msg.fields)  # [1, 2, 3, ...]
        if (
            len(self.safety_fields)
            and self.safety_fields[0] == 1
            and not self.simulation
        ):
            self.is_safety_stop = True
        else:
            self.is_safety_stop = False

    def action_fb(self, data):
        self.last_action_fb = rospy.get_time()

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
        # try:
        self.new_route_data = ""
        self.is_new_route = False
        data_dict = json.loads(goal.data)
        need_set_vel_before_move = False
        self.need_to_wait_receive_new_path = False
        self.is_change_path_when_move = False
        self.trigger_resend_goal = False
        self.last_time_send_goal = rospy.get_time()
        if (
            "param_test" in data_dict
            and "need_to_wait_receive_new_path" in data_dict["param_test"]
        ):
            if data_dict["param_test"]["need_to_wait_receive_new_path"]:
                rospy.logerr("need_to_wait_receive_new_path == True")
                self.need_to_wait_receive_new_path = True
                self.is_change_path_when_move = True
        if (
            "param_test" in data_dict
            and "need_set_vel_before_move" in data_dict["param_test"]
        ):
            if data_dict["param_test"]["need_set_vel_before_move"]:
                rospy.logerr("need_set_vel_before_move == True")
                need_set_vel_before_move = True
        # need_set_vel_before_move = True
        if need_set_vel_before_move:
            vel_config = self.db.get_speed_profiles()
            rospy.logwarn("vel_config: {}".format(vel_config))
            if vel_config is not None:
                status_cart_json = json.loads(
                    self.db.loadStatusCartData("status_cart")
                )
                rospy.loginfo(
                    f"Loaded raw JSON: {self.db.loadStatusCartData('status_cart')}"
                )

                status_cart = status_cart_json["status_cart"]
                rospy.logwarn("status_cart: {}".format(status_cart))

                if status_cart == "have_empty_cart":
                    rospy.logwarn("Current is AGV_CARRY_EMPTY_CART")
                    profile = vel_config.get("AGV_CARRY_EMPTY_CART", {})
                elif status_cart == "have_full_cart":
                    rospy.logwarn("Current is AGV_CARRY_FULL_CART")
                    profile = vel_config.get("AGV_CARRY_FULL_CART", {})
                else:
                    rospy.logwarn("Current is AGV_NO_CART")
                    profile = vel_config.get("AGV_NO_CART", {})

                # Chỉ thêm vào new_config nếu trường đó tồn tại
                new_config = {}

                if "max_trans_speed" in profile:
                    rospy.loginfo(
                        f"max_trans_speed: {profile['max_trans_speed']}"
                    )
                    new_config["max_vel_x"] = profile["max_trans_speed"]
                    new_config["max_vel_trans"] = profile["max_trans_speed"]

                if "max_trans_acceleration" in profile:
                    rospy.loginfo(
                        f"max_trans_acceleration: {profile['max_trans_acceleration']}"
                    )
                    new_config["acc_lim_x"] = profile["max_trans_acceleration"]
                    new_config["stop_acc"] = profile["max_trans_acceleration"]

                if "emg_trans_deceleration" in profile:
                    rospy.loginfo(
                        f"emg_trans_deceleration: {profile['emg_trans_deceleration']}"
                    )
                    new_config["emergency_acc_lim_x"] = profile[
                        "emg_trans_deceleration"
                    ]

                if "max_rot_speed" in profile:
                    rospy.loginfo(f"max_rot_speed: {profile['max_rot_speed']}")
                    new_config["max_rot_vel"] = profile["max_rot_speed"]

                if "max_rot_acceleration" in profile:
                    rospy.loginfo(
                        f"max_rot_acceleration: {profile['max_rot_acceleration']}"
                    )
                    new_config["acc_lim_theta"] = profile[
                        "max_rot_acceleration"
                    ]

                if "emg_rot_deceleration" in profile:
                    rospy.loginfo(
                        f"emg_rot_deceleration: {profile['emg_rot_deceleration']}"
                    )
                    new_config["emergency_acc_lim_theta"] = profile[
                        "emg_rot_deceleration"
                    ]

                if new_config:
                    rospy.logwarn("Update config vel")
                    for i in range(3):
                        self.client_reconfig_movebase.update_configuration(
                            new_config
                        )
                        rospy.sleep(0.01)
                else:
                    rospy.logwarn(
                        "Do not Update config vel because not found config in db"
                    )

        #######################################
        goal_accept = True
        print_warn("------------------Received goal:------------------")
        # print(json.dumps(data_dict, indent=2))
        if "params" not in data_dict:
            data_dict["params"] = {}
        if "reconfigure" not in data_dict["params"]:
            data_dict["params"]["reconfigure"] = {}
        # Get param from defaul param, prioritize for global param in root json tree
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
        # except Exception as e:
        #     rospy.logerr("Action goal.data systax error: {}".format(e))
        #     self.send_feedback(self._as, "ABORTED")
        #     self._as.set_aborted(text="Action goal.data systax error")
        #     return

        success = False
        self.list_safety_job_point.clear()
        self.list_slow_point.clear()
        self.list_wp_create_path.clear()
        _state = MainState.SEND_GOAL
        _state_when_pause = MainState.NONE
        _prev_state = MainState.NONE
        self.current_point = 0
        action_goal = None
        action_client = None
        local_planner = ""
        last_action_client = None
        last_local_planner = ""
        current_target_pose = PoseStamped()
        check_action_status = False
        max_retry_time = 0
        move_base_retry_cnt = 0
        last_loop_time = rospy.get_time()
        total_point = 0
        start_time = rospy.get_time()
        feedback_msg = ""
        self._asm.reset_flag()
        is_print_error = False
        safety_timeout = False
        goal_dict = {}
        goal_dict["params"] = {}
        self.is_safety_stop = False
        self.reset_safety_point = False
        self.safety_name = ""
        self.last_state_use_qr_code = False

        # Loop
        polling_rate = 20.0
        NOP_TIME = 0.0001
        rate = rospy.Rate(polling_rate)
        t = rospy.get_time()
        diff_time = 1.0 / polling_rate

        # Check only use QR code
        # Hub, matehan, charge only use QR code and set this param
        only_use_qr = False
        use_mirror = False
        if "only_use_qr" in data_dict["params"]:
            if data_dict["params"]["only_use_qr"] == True:
                only_use_qr = True
            else:
                only_use_qr = False
        else:
            only_use_qr = False

        if "use_mirror" in data_dict["params"]:
            if data_dict["params"]["use_mirror"] == True:
                rospy.logwarn("use_mirror == True")
                use_mirror = True
            else:
                use_mirror = False
        else:
            use_mirror = False

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

        # Use for traffic control purpose
        self.ori_waypoints = copy.deepcopy(data_dict["waypoints"])
        self.is_near_target_pose = False
        self.last_wp_remove_in_path = None

        # Get turn waypoints
        for i in range(len(self.ori_waypoints)):
            if (
                "data" in self.ori_waypoints[i]
                and "properties" in self.ori_waypoints[i]["data"]
            ):
                if "Safety" in self.ori_waypoints[i]["data"]["properties"]:
                    self.list_safety_job_point.append(self.ori_waypoints[i])
                if "Slow_speed" in self.ori_waypoints[i]["data"]["properties"]:
                    if (
                        self.ori_waypoints[i]["data"]["properties"][
                            "Slow_speed"
                        ]
                        == True
                    ):
                        self.list_slow_point.append(
                            dict_to_pose(self.ori_waypoints[i]["position"])
                        )
        self.is_waiting_pose = False
        self.time_waiting_pose = 0
        self.time_begin_check_waiting = rospy.get_time()

        target_position = None
        self.check_same_goal_when_receive_new_route = False
        self.last_goal_in_path = None
        self.first_goal_in_path = None
        if (
            "waypoints" in data_dict
            and isinstance(data_dict["waypoints"], list)
            and len(data_dict["waypoints"]) > 0
        ):
            last_waypoint = data_dict["waypoints"][-1]

            if (
                "position" in last_waypoint
                and "position" in last_waypoint["position"]
            ):
                try:
                    target_position = last_waypoint["position"]["position"]
                except Exception as e:
                    rospy.logerr(
                        f"Error when get last waypoint of origin data_dict: {e}"
                    )

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

            if (
                rospy.get_time() - self.last_scan_safety > 5.0
                and not self.simulation
                and _state == MainState.MOVING
            ):
                safety_timeout = True
                if not is_print_error:
                    rospy.logerr("Safety module disconnected")
                    is_print_error = True
            else:
                is_print_error = False
                safety_timeout = False

            if self._as.is_preempt_requested() or self._asm.reset_action_req:
                rospy.loginfo("%s: Preempted" % self._action_name)
                self.cancel_all_action()
                self._as.set_preempted()
                success = False
                self.send_feedback(self._as, "PREEMPTED")
                # self.switch_control_tf(USE_TF_LIDAR)
                break

            if self._asm.module_status != ModuleStatus.ERROR:
                self._asm.error_code = ""
            if _state != MainState.PAUSED and _state != MainState.ERROR:
                self._asm.module_status = ModuleStatus.RUNNING
                # Handle new route recive
                if (
                    self.use_new_traffic_control
                    and self.is_change_path_when_move
                ):
                    if self.is_new_route and target_position != None:
                        self.is_new_route = False
                        data_dict_new = json.loads(self.new_route_data)

                        # Duyệt ngược waypoints để tìm vị trí khớp

                        if (
                            "waypoints" in data_dict_new
                            and isinstance(data_dict_new["waypoints"], list)
                            and len(data_dict_new["waypoints"]) > 0
                        ):

                            waypoints = data_dict_new["waypoints"]
                            index_to_keep = -1

                            for i in range(
                                len(waypoints) - 1, -1, -1
                            ):  # Duyệt từ cuối về đầu
                                if (
                                    "position" not in waypoints[i]
                                    or "position"
                                    not in waypoints[i]["position"]
                                ):
                                    continue  # Bỏ qua nếu waypoint không có dữ liệu hợp lệ
                                waypoint_pos = waypoints[i]["position"][
                                    "position"
                                ]

                                # Làm tròn 3 chữ số sau dấu phẩy trước khi so sánh
                                if round(waypoint_pos["x"], 3) == round(
                                    target_position["x"], 3
                                ) and round(waypoint_pos["y"], 3) == round(
                                    target_position["y"], 3
                                ):
                                    index_to_keep = i
                                    rospy.logerr("find new data_dict")
                                    break  # Dừng ngay khi tìm thấy phần tử phù hợp

                            # Nếu tìm thấy, chỉ giữ lại phần tử từ 0 đến index_to_keep
                            if index_to_keep != -1:
                                # Get turn waypoints
                                self.need_to_wait_receive_new_path = False
                                data_dict["waypoints"] = data_dict_new[
                                    "waypoints"
                                ][: index_to_keep + 1]
                                self.list_safety_job_point.clear()
                                self.ori_waypoints = copy.deepcopy(
                                    data_dict["waypoints"]
                                )
                                for i in range(len(self.ori_waypoints)):
                                    if (
                                        "data" in self.ori_waypoints[i]
                                        and "properties"
                                        in self.ori_waypoints[i]["data"]
                                    ):
                                        if (
                                            "Safety"
                                            in self.ori_waypoints[i]["data"][
                                                "properties"
                                            ]
                                        ):
                                            self.list_safety_job_point.append(
                                                self.ori_waypoints[i]
                                            )
                                        if (
                                            "Slow_speed"
                                            in self.ori_waypoints[i]["data"][
                                                "properties"
                                            ]
                                        ):
                                            if (
                                                self.ori_waypoints[i]["data"][
                                                    "properties"
                                                ]["Slow_speed"]
                                                == True
                                            ):
                                                self.list_slow_point.append(
                                                    dict_to_pose(
                                                        self.ori_waypoints[i][
                                                            "position"
                                                        ]
                                                    )
                                                )
                                goal_accept = True
                                print_warn(
                                    "------------------Received new goal:------------------"
                                )
                                # print(json.dumps(data_dict, indent=2))
                                if "params" not in data_dict:
                                    data_dict["params"] = {}
                                if "reconfigure" not in data_dict["params"]:
                                    data_dict["params"]["reconfigure"] = {}
                                # Get param from defaul param, prioritize for global param in root json tree
                                data_dict["params"] = merge_two_dicts(
                                    self.default_params_dict["params"],
                                    data_dict["params"],
                                )
                                data_dict["params"]["reconfigure"] = (
                                    merge_two_dicts(
                                        self.default_params_dict["params"][
                                            "reconfigure"
                                        ],
                                        data_dict["params"]["reconfigure"],
                                    )
                                )
                                # print_warn('Merge default params:')
                                # print(json.dumps(data_dict, indent=2))
                                # Get param from global param, prioritize for local param in each waypoint
                                for i in range(len(data_dict["waypoints"])):
                                    if (
                                        "params"
                                        not in data_dict["waypoints"][i]
                                    ):
                                        data_dict["waypoints"][i]["params"] = {}
                                    # TOCHECK: why not mege not empty ['reconfigure'] dict? Only copy this dict if not exist
                                    if (
                                        "reconfigure"
                                        not in data_dict["waypoints"][i][
                                            "params"
                                        ]
                                    ):
                                        data_dict["waypoints"][i]["params"][
                                            "reconfigure"
                                        ] = {}
                                    # print_debug('''waypoint_{}'s params before merge:\n{}'''.format(i+1, json.dumps(data_dict['waypoints'][i]['params'], indent=2)))
                                    data_dict["waypoints"][i]["params"] = (
                                        merge_two_dicts(
                                            data_dict["params"],
                                            data_dict["waypoints"][i]["params"],
                                        )
                                    )
                                    data_dict["waypoints"][i]["params"][
                                        "reconfigure"
                                    ] = merge_two_dicts(
                                        data_dict["params"]["reconfigure"],
                                        data_dict["waypoints"][i]["params"][
                                            "reconfigure"
                                        ],
                                    )
                                    # print_debug('''waypoint_{}'s params after merge:\n{}'''.format(i+1, json.dumps(data_dict['waypoints'][i]['params'], indent=2)))
                                # print_warn('Merge local params:')
                                # print(json.dumps(data_dict, indent=2))
                                frame_id = data_dict["params"][
                                    "frame_id"
                                ]  # Only use global frame_id, not yet use waypoint frame_id
                                self.current_point = 0
                                self.check_same_goal_when_receive_new_route = (
                                    True
                                )
                                _state = MainState.SEND_GOAL
                            else:
                                rospy.logerr(
                                    "Not found target position in new data_dict"
                                )
                                rospy.logwarn(
                                    "Target position: {}".format(
                                        target_position
                                    )
                                )
                                rospy.logwarn(
                                    "Route new receive: {}".format(
                                        data_dict_new
                                    )
                                )
                        else:
                            rospy.logerr(
                                "Fail when check waypoints in new route"
                            )

            if _state != _prev_state:
                rospy.loginfo(
                    "Main state: {} -> {}".format(
                        _prev_state.toString(), _state.toString()
                    )
                )
                _prev_state = _state
                feedback_msg = _state.toString()
                self._asm.module_state = _state.toString()
            self.send_feedback(self._as, feedback_msg)

            if self.fake_error and _state != MainState.SEND_GOAL:
                print_debug("Fake error")
                self.fake_error = False
                _state = MainState.ERROR
                self.cancel_all_action()
            # Nếu treo ở bước SEND_GOAL thì sẽ check thời gian lần đầu nhảy vào bước SEND_GOAL
            if _state != MainState.SEND_GOAL:
                self.last_time_send_goal = rospy.get_time()
            """
               # #       #####  ####### #     # ######           #####  #######    #    #
               # #      #     # #       ##    # #     #         #     # #     #   # #   #
             #######    #       #       # #   # #     #         #       #     #  #   #  #
               # #       #####  #####   #  #  # #     #         #  #### #     # #     # #
             #######          # #       #   # # #     #         #     # #     # ####### #
               # #      #     # #       #    ## #     #         #     # #     # #     # #
               # #       #####  ####### #     # ######           #####  ####### #     # #######
                                                        #######
            """
            if _state == MainState.SEND_GOAL:

                # Pause handle
                if self._asm.pause_req:
                    if self._asm.pause_req_by_server:
                        rospy.logwarn("PAUSE SERVER")
                        self.run_pause_pub.publish(
                            StringStamped(
                                stamp=rospy.Time.now(), data="PAUSE_BY_SERVER"
                            )
                        )
                    else:
                        self.run_pause_pub.publish(
                            StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                        )
                    _state_when_pause = _state
                    _state = MainState.PAUSED
                    self._asm.reset_flag()
                    continue

                if (
                    self.use_new_traffic_control
                    and self.is_change_path_when_move
                    and self.need_to_wait_receive_new_path
                ):
                    self.trigger_resend_goal = True
                    rospy.logwarn_throttle(
                        5, "Wait for receive path from server traffic control"
                    )
                    continue
                else:
                    self.trigger_resend_goal = False

                rospy.logwarn("nhau vao day")
                total_point = len(data_dict["waypoints"])
                wp = data_dict["waypoints"][self.current_point]
                wp_next = None
                if self.current_point < total_point - 1:
                    wp_next = data_dict["waypoints"][self.current_point + 1]
                # check current type
                if wp_next is not None:
                    if wp_next["rule_path"] == "qr" or only_use_qr:
                        use_qr_code = True
                    else:
                        use_qr_code = False
                else:
                    if wp["rule_path"] == "qr" or only_use_qr:
                        use_qr_code = True
                    else:
                        use_qr_code = False

                if use_qr_code:
                    if use_mirror:
                        self.type_move = "MIRROR"
                        self.switch_control_tf(USE_TF_LIDAR)
                        rospy.logwarn("use mirror")
                    else:
                        self.type_move = "QR"
                        self.switch_control_tf(USE_TF_QR_CODE)
                    self.use_lidar = False
                    rospy.logwarn("use_qr_code")
                    # publish use tf lidar map
                    # check QR in start point, when change from lidar to QR always read a QR
                    if self.last_state_use_qr_code == False and not use_mirror:
                        _state = MainState.FIND_QR_CODE
                        continue
                    else:
                        rospy.logwarn("DEBUG last_state_use_qr_code: True")
                    # create list wp for move_base qr
                    self.current_point = (
                        self.create_wp_for_qr_mb(data_dict["waypoints"]) - 1
                    )
                    # set goal for move by qr
                    # goal_pose = data_dict["waypoints"][self.current_goal][
                    #     "position"
                    # ]
                else:
                    self.type_move = "LIDAR"
                    self.use_lidar = True
                    rospy.logwarn("use lidar")
                    self.last_state_use_qr_code = False
                    # publish use tf lidar map
                    self.switch_control_tf(USE_TF_LIDAR)
                    rospy.logwarn(
                        'DEBUG use_qr_code: {}, modify_status not in wp["param"]: {}'.format(
                            use_qr_code, "modify_status" not in wp["params"]
                        )
                    )
                    # create list wp for move_base lidar
                    self.current_point = (
                        self.create_wp_for_mb(data_dict["waypoints"]) - 1
                    )

                rospy.logerr(self.list_wp_create_path)
                # --------------------------------
                #TODO: cần convert tf map qr->lidar cho _current_goal
                _current_goal = Pose()
                if len(self.list_wp_create_path) > 0:
                    # Lấy phần tử cuối cùng và convert từ PoseStamped sang Pose
                    _current_goal = self.list_wp_create_path[-1].Pose.pose
                else:
                    _current_goal = data_dict["waypoints"][self.current_point][
                        "position"
                    ]

                    # rospy.logwarn("Total point: {}".format(total_point))
                    # rospy.logwarn(
                    #     "Current point: {}".format(self.current_point)
                    # )
                    if total_point > 1:
                        if self.current_point < total_point - 1:
                            _current_goal = data_dict["waypoints"][
                                self.current_point + 1
                            ]["position"]
                        elif self.current_point == total_point - 1:
                            _state = MainState.DONE
                            continue
                    else:
                        _current_goal = data_dict["waypoints"][self.current_point][
                            "position"
                        ]
                rospy.logwarn(
                        "Current goal send to move base: {}".format(_current_goal)
                    )
                # set huong di chuyen tu server
                # Hiện tại mặc định là tự chọn chiều nếu có 2 wp liên tiếp là revese thì sẽ cho đi ngược tại path đó
                self.direction_move = 0
                if total_point > 1 and self.current_point == total_point - 2:
                    wp_start = data_dict["waypoints"][self.current_point]
                    wp_end = data_dict["waypoints"][self.current_point + 1]
                    # Lấy pose 2 waypoint
                    pose_start = dict_to_pose(wp_start["position"])
                    pose_end = dict_to_pose(wp_end["position"])

                    # Tính yaw hướng path (radian)
                    yaw_path = atan2(
                        pose_end.position.y - pose_start.position.y,
                        pose_end.position.x - pose_start.position.x,
                    )

                    # Lấy yaw robot hiện tại (radian)
                    quat_robot = self.robot_pose.orientation
                    (_, _, yaw_robot) = (
                        tf.transformations.euler_from_quaternion(
                            [
                                quat_robot.x,
                                quat_robot.y,
                                quat_robot.z,
                                quat_robot.w,
                            ]
                        )
                    )

                    # Tính độ lệch góc giữa robot và hướng path (đơn vị độ)
                    angle_diff_deg = abs(
                        (degrees(yaw_robot) - degrees(yaw_path) + 180) % 360
                        - 180
                    )

                    # Chỉ xét nếu góc lệch nằm trong khoảng 80 đến 100 độ
                    if 45 < angle_diff_deg < 135:
                        is_reverse_in_start_wp = False
                        is_reverse_in_end_wp = False
                        if "data" in wp_start:
                            if "is_reserve" in wp_start["data"]:
                                if wp_start["data"]["is_reserve"] == True:
                                    is_reverse_in_start_wp = True
                        if self.last_wp_remove_in_path is not None:
                            if "data" in self.last_wp_remove_in_path:
                                if (
                                    "is_reserve"
                                    in self.last_wp_remove_in_path["data"]
                                ):
                                    if (
                                        self.last_wp_remove_in_path["data"][
                                            "is_reserve"
                                        ]
                                        == True
                                    ):
                                        is_reverse_in_start_wp = True
                        if "data" in wp_end:
                            if "is_reserve" in wp_end["data"]:
                                if wp_end["data"]["is_reserve"] == True:
                                    is_reverse_in_end_wp = True

                        if is_reverse_in_start_wp and is_reverse_in_end_wp:
                            self.direction_move = 2
                self.last_wp_remove_in_path = None
                # Publish the path
                path_msg = Path()
                path_msg.header.frame_id = "map"  # Set the frame ID
                path_msg.header.stamp = rospy.Time.now()
                if len(self.list_wp_create_path) >= 1:
                    path_msg.poses.append(self.list_wp_create_path[0].Pose)
                    if len(self.list_wp_create_path) > 1:
                        path_msg.poses.append(self.list_wp_create_path[-1].Pose)

                    self.path_publisher.publish(path_msg)
                else:
                    rospy.logwarn(
                        "list_wp_create_path is empty, cannot publish path."
                    )
                # if  same with last then change to state Move and continue
                if self.check_same_goal_when_receive_new_route:
                    self.check_same_goal_when_receive_new_route = False
                    current_waypoint = (
                        self.list_wp_create_path[-1].Pose
                        if self.list_wp_create_path
                        else None
                    )
                    if (
                        self.last_goal_in_path is not None
                        and self.first_goal_in_path is not None
                        and self.is_same_position(
                            current_waypoint.pose,
                            self.last_goal_in_path.pose,
                        )
                    ):
                        distance_robot_to_old_path = (
                            self.point_to_segment_distance(
                                self.robot_pose,
                                self.first_goal_in_path.pose,
                                self.last_goal_in_path.pose,
                            )
                        )
                        rospy.logwarn(
                            "Khoảng cách từ robot đến old path is: {}".format(
                                distance_robot_to_old_path
                            )
                        )
                        if distance_robot_to_old_path < 0.1:
                            check_action_status = True
                            _state = MainState.MOVING
                            continue

                self.last_goal_in_path = (
                    self.list_wp_create_path[-1].Pose
                    if self.list_wp_create_path
                    else None
                )
                self.first_goal_in_path = (
                    self.list_wp_create_path[0].Pose
                    if self.list_wp_create_path
                    else None
                )

                # data_dict["waypoints"][self.current_point]["position"]
                ##########################################################
                # print_info("Next cycle dict: \n{}".format(json.dumps(data_dict['waypoints'], indent=2)))
                wp_display = self.get_wp_display(data_dict["waypoints"])
                del wp_display[: self.current_point - 1]
                self.waypoints_following_pub.publish(
                    pose_stamped_array_to_pose_array(wp_display, frame_id)
                )

                current_target_pose.header.stamp = rospy.Time.now()
                current_target_pose.header.frame_id = frame_id
                current_target_pose.pose = _current_goal
                rospy.loginfo(
                    "---------------------------wp: {}/{}---------------------------".format(
                        self.current_point + 1, total_point
                    )
                )
                rospy.loginfo(current_target_pose.pose)
                rospy.loginfo("---")
                # print_debug(json.dumps(wp, indent=2))
                max_retry_time = wp["params"]["max_retry_time"]
                local_planner = wp["params"]["local_planner"]
                self.current_local_planner = local_planner
                # print('local_planner: ' + local_planner)
                # print('---')
                action_goal = self.dynamic_global_var[local_planner][
                    "action_goal"
                ]
                action_goal.target_pose = current_target_pose
                action_client = self.dynamic_global_var[local_planner][
                    "action_client"
                ]
                # TOCHECK: Cancel prev goal
                self.move_action_result = -1
                check_action_status = True
                # Cancel last action
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
                start_time = rospy.get_time()
                _state = MainState.MOVING
                self.run_pause_pub.publish(
                    StringStamped(stamp=rospy.Time.now(), data="RUN")
                )
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

            # State: FIND_QR_CODE_ERROR
            elif _state == MainState.FIND_QR_CODE_ERROR:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = self._asm.error_code = (
                    "/moving_control: {}".format(_state.toString())
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = MainState.FIND_QR_CODE

            # State: RE_SEND_GOAL
            elif _state == MainState.RE_SEND_GOAL:
                rospy.logwarn("RESEND GOAL")
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
                self.run_pause_pub.publish(
                    StringStamped(stamp=rospy.Time.now(), data="RUN")
                )
            # State: STOP_BY_SAFETY
            elif _state == MainState.STOP_BY_SAFETY:
                self.manager_safety()
                # Check not safety
                if not self.is_safety_stop and not safety_timeout:
                    _state = MainState.MOVING
                # Pause handle
                if self._asm.pause_req:
                    if self._asm.pause_req_by_server:
                        rospy.logwarn("PAUSE SERVER")
                        self.run_pause_pub.publish(
                            StringStamped(
                                stamp=rospy.Time.now(), data="PAUSE_BY_SERVER"
                            )
                        )
                    else:
                        self.run_pause_pub.publish(
                            StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                        )
                    _state_when_pause = _state
                    _state = MainState.PAUSED
                    self._asm.reset_flag()
                    continue

            # State: MOVING
            elif _state == MainState.MOVING:
                self.manager_safety()
                near_slow_point = self.check_pose_near_slow_point()
                if near_slow_point and self.old_state_near_slow_point == False:
                    self.dynamic_reconfig_movebase(True)
                    self.old_state_near_slow_point = True
                if self.old_state_near_slow_point and not near_slow_point:
                    self.dynamic_reconfig_movebase(False)
                    self.old_state_near_slow_point = False

                # Pause handle
                if self._asm.pause_req:
                    if self._asm.pause_req_by_server:
                        rospy.logwarn("PAUSE SERVER")
                        self.run_pause_pub.publish(
                            StringStamped(
                                stamp=rospy.Time.now(), data="PAUSE_BY_SERVER"
                            )
                        )
                    else:
                        self.run_pause_pub.publish(
                            StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                        )
                    _state_when_pause = _state
                    _state = MainState.PAUSED
                    self._asm.reset_flag()
                    continue
                # current_pose = lockup_pose(
                #     self.tf_listener, frame_id, self.robot_base
                # )
                self.check_distance_to_near_cross_line(self.robot_pose)
                # distance = distance_two_pose(current_pose, current_target_pose.pose)
                # Action SUCCEEDED
                if (
                    check_action_status == True
                    and self.move_action_result == goal_result.SUCCEEDED
                ):
                    _state = MainState.DONE
                    rospy.loginfo(
                        "TOLERANCE OK - move_action_result: %s"
                        % self.move_action_result
                    )
                    check_action_status = False
                    continue
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
                        self.cancel_all_action()
                        continue
                    else:
                        _state = MainState.RE_SEND_GOAL
                        continue
                    check_action_status = False
                if self.is_safety_stop or safety_timeout:
                    _state = MainState.STOP_BY_SAFETY

            # State: DONE
            elif _state == MainState.DONE:
                move_base_retry_cnt = 0
                # Measure running time
                duration = rospy.get_time() - start_time
                rospy.loginfo("Done with %i(s)", duration)
                # Next waypoint
                self.current_point += 1
                if total_point > 1:
                    if self.current_point == len(data_dict["waypoints"]) - 1:
                        success = True
                        break
                if self.current_point == len(data_dict["waypoints"]):
                    success = True
                    break
                else:
                    # Remove finished waypoint
                    wp_display.pop(0)
                    self.waypoints_following_pub.publish(
                        pose_stamped_array_to_pose_array(wp_display, frame_id)
                    )
                    if self.is_waiting_pose:
                        self.is_waiting_pose = False
                        self.time_begin_check_waiting = rospy.get_time()
                        _state = MainState.WAITING_AT_POSE
                    else:
                        _state = MainState.SEND_GOAL

            elif _state == MainState.WAITING_AT_POSE:
                if (
                    rospy.get_time() - self.time_begin_check_waiting
                    > self.time_waiting_pose
                ):
                    rospy.logwarn(f"TIMEOUT waiting at pose")
                    _state = MainState.SEND_GOAL

            # State: PAUSED
            elif _state == MainState.PAUSED:
                self._asm.module_status = ModuleStatus.PAUSED
                if self._asm.pause_req:
                    if self._asm.pause_req_by_server:
                        rospy.logwarn("PAUSE SERVER")
                        self.run_pause_pub.publish(
                            StringStamped(
                                stamp=rospy.Time.now(), data="PAUSE_BY_SERVER"
                            )
                        )
                    else:
                        self.run_pause_pub.publish(
                            StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                        )
                    self._asm.reset_flag()
                if self._asm.resume_req:
                    self._asm.reset_flag()
                    self.run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="RUN")
                    )

                    _state = _state_when_pause
                    # print_debug("Resume waypoint number: {}".format(self.current_point+1))
            # State: ERROR
            elif _state == MainState.ERROR:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = self._asm.error_code = (
                    "/moving_control: {}".format(_state.toString())
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = MainState.SEND_GOAL

        self._asm.action_running = False
        # self.switch_control_tf(USE_TF_LIDAR)

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
        r = rospy.Rate(5)
        moving_status_time = rospy.get_time()
        status_msg = StringStamped()
        while not rospy.is_shutdown():
            r.sleep()
            if not self._asm.action_running:
                self._asm.module_status = ModuleStatus.WAITING
                self._asm.module_state = MainState.WAITING.toString()
                self._asm.error_code = ""
            else:
                self.send_feedback(self._as, self._asm.module_state)

            now = rospy.get_time()
            if now - moving_status_time >= 0.2:
                resend_goal = False
                if (
                    self.trigger_resend_goal
                    and rospy.get_time() - self.last_time_send_goal > 5
                ):
                    self.trigger_resend_goal = False
                    self.last_time_send_goal = rospy.get_time()
                    resend_goal = True
                moving_status_time = now
                status_msg.stamp = rospy.Time.now()
                status_msg.data = json.dumps(
                    {
                        "status": self._asm.module_status.toString(),
                        "state": self._asm.module_state,
                        "error_code": self._asm.error_code,
                        "trigger_resend_goal": resend_goal,
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
