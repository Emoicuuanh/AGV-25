#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import json
from bson.json_util import dumps
import yaml
import argparse
import rospy
import rospkg
import tf
import copy
import actionlib
import time
import dynamic_reconfigure.client
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_stamped_msgs.msg import (
    StringStamped,
    StringFeedback,
    StringResult,
    StringAction,
    StringActionResult,
    StringGoal,
    EmptyStamped,
)
from std_msgs.msg import  Int8
from actionlib_msgs.msg import GoalStatus, GoalID, GoalStatusArray

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

from module_manager import ModuleServer, ModuleClient, ModuleStatus
from common_function import (
    EnumString,
    ActionResult,
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
)


class MainState(EnumString):
    NONE = -1
    INIT = 0
    SEND_ACTION_GOAL = 1
    WAIT_RESULT = 2
    UNDOCKING = 4
    ERROR = 7
    UNDOCKING_ERROR = 8
    PAUSED = 9
    DUPLICATE_MARKER = 10
    MARKER_DIR_WRONG = 11
    MISSION_EMPTY = 12
    UNDOCKING_DISCONNECTED = 13
    ACTION_DATA_NULL = 14  # TOCHECK
    MISSING_ACTION_DATA = 15
    SERVER_DATA_ERROR = 16
    ACTION_EXEC_ERROR = 17
    ROBOT_NOT_IN_POSE = 18


class ModuleStatus(EnumString):
    WAITING = 0
    RUNNING = 1
    PAUSED = 2
    UNDOCKING = 3
    ERROR = 4

USE_TF_LIDAR = 1
USE_TF_QR_CODE = 0


class MissionManagerServer(object):
    _feedback = StringFeedback()
    _result = StringResult()

    def __init__(self, name, *args, **kwargs):
        # Action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            StringAction,
            execute_cb=self.mission_manager_exec_cb,
            auto_start=False,
        )
        self._as.start()
        self._as_client = actionlib.SimpleActionClient(
            self._action_name, StringAction
        )
        self._as_client.wait_for_server(timeout=rospy.Duration(2))
        self.undocking_client = actionlib.SimpleActionClient(
            "un_docking", StringAction
        )
        # TODO: Undocking client
        rospy.on_shutdown(self.shutdown)
        # Publisher
        self.control_tf_pub = rospy.Publisher("/control_tf", Int8, queue_size=5)
        self.cancel_charging_pub = rospy.Publisher(
            "/cancel_charging", EmptyStamped, queue_size=5
        )
        self.auto_docking_run_pause_pub = rospy.Publisher(
            "/un_docking/run_pause_req", StringStamped, queue_size=5
        )
        self.auto_docking_reset_error_pub = rospy.Publisher(
            "/un_docking/reset_error", EmptyStamped, queue_size=5
        )
        # Subscriber
        rospy.Subscriber("/current_control_tf", Int8, self.current_control_tf_cb)
        rospy.Subscriber(
            "/mission_control", StringStamped, self.mission_control_cb
        )
        rospy.Subscriber(
            "/mission_from_server", StringStamped, self.mission_from_server_cb
        )
        rospy.Subscriber("/current_map", StringStamped, self.current_map_cb)
        rospy.Subscriber("/robot_status", StringStamped, self.robot_status_cb)
        rospy.Subscriber("/standard_io", StringStamped, self.std_io_cb)
        rospy.Subscriber(
            "/un_docking/result", StringResult, self.un_docking_result_cb
        )
        rospy.Subscriber(
            "/charging_status", StringStamped, self.auto_charging_stt_cb
        )
        rospy.Subscriber(
            "/auto_docking/module_status",
            StringStamped,
            self.auto_docking_module_status_cb,
        )
        # ModuleServer
        self._asm = ModuleServer(name)
        # Initial
        self.init_variable()
        self.read_config(path=kwargs["mission_list"])

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

    def init_variable(self):
        # TF
        self.tf_listener = tf.TransformListener()
        self.map_frame = "map"
        self.odom_frame = "odom"
        self.robot_base = "base_footprint"
        # DB
        db_address = rospy.get_param("/mongodb_address")
        self.check_robot_in_pose_when_start = rospy.get_param(
            "~check_robot_in_pose_when_start", False
        )
        print_debug(db_address)
        self.db = mongodb(db_address)
        self.mission_cfg_dict = {}
        self.mission_client_dict = {}
        self.auto_charging_stt_dict = {}
        self.action_result = -1
        self.last_status_time = rospy.get_time()
        self.last_feedback = rospy.get_time()
        self.last_std_io_msg = rospy.get_time()
        #
        self.current_map = ""
        self.robot_status_msg = StringStamped()
        self.is_auto_charging = False
        self.is_man_charging = False
        #
        self.un_docking_result = -1
        self.module_error_code = ""
        self.auto_docking_error_code = ""
        self.undocking_angle_allow = 0.3
        #
        self.last_mission_done_id = ""
        self.running_mission_id = ""
        self.cur_mission_name = ""
        self.mission_group = ""
        self.current_action_num = 0
        self.is_mission_fr_server = False
        self.current_action_type = ""
        self.current_action_name = ""
        self.total_action = 0

        #
        self.change_action_server_move = False
        self.current_control_tf = 100

    def reset_status(self):
        self.running_mission_id = ""
        self.cur_mission_name = ""
        self.current_action_num = 0
        self.is_mission_fr_server = False
        self.current_action_type = ""
        self.current_action_name = ""
        self.total_action = 0
        self.mission_group = ""

    def read_config(self, path):
        cfg_file = path
        try:
            with open(cfg_file) as file:
                self.mission_cfg_dict = yaml.load(file, Loader=yaml.Loader)
        except Exception as e:
            rospy.logerr("Error loading: {}".format(e))

        for action_type, value in self.mission_cfg_dict.items():
            action_server_name = value["action_server"]
            # Create mission client instance
            # print_debug("Creating mission client")
            # action_client = None
            # _cmd = "action_client = actionlib.SimpleActionClient('{}', StringAction)".format(action_server_name)
            # print('cmd -->: ' + _cmd)
            # exec(_cmd)
            action_client = actionlib.SimpleActionClient(
                action_server_name, StringAction
            )
            self.mission_client_dict[action_type] = {}
            self.mission_client_dict[action_type][
                "action_client"
            ] = action_client  # TOCHECK: Why no need deepcopy?
            # Create ModuleClient in /control_system because module not === action
            # Action server name
            self.mission_client_dict[action_type][
                "action_server"
            ] = action_server_name
            # Common topic
            run_pause_req_pub = rospy.Publisher(
                action_server_name + "/run_pause_req",
                StringStamped,
                queue_size=5,
            )
            self.mission_client_dict[action_type][
                "run_pause_req_pub"
            ] = run_pause_req_pub
            reset_pub = rospy.Publisher(
                action_server_name + "/reset_error", EmptyStamped, queue_size=5
            )
            self.mission_client_dict[action_type]["reset_pub"] = reset_pub
            # Need undocking or not
            self.mission_client_dict[action_type]["need_undocking"] = value[
                "need_undocking"
            ]
            # Module status
            if "module_name" in value:
                self.mission_client_dict[action_type]["module_status_topic"] = (
                    "/" + value["module_name"] + "/module_status"
                )

            # Create action result callback
            # print_debug("Creating action result callback")
            # _cmd = '''rospy.Subscriber('/{}/result', StringActionResult, self.action_result_cb)'''.format(action_server_name)
            # print('cmd -->: ' + _cmd)
            # exec(_cmd)
            # print_debug("Creating action status callback")
            # _cmd = '''rospy.Subscriber('/{}/status', GoalStatusArray, self.action_status_cb)'''.format(action_server_name)
            # print('cmd -->: ' + _cmd)
            # exec(_cmd)

    def shutdown(self):
        for action_type, value in self.mission_client_dict.items():
            value["action_client"].cancel_all_goals()
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
            return
        else:
            self.control_tf_pub.publish(Int8(data=type))

    def send_feedback(self, action, msg):
        self._feedback.data = msg
        action.publish_feedback(self._feedback)

    def get_current_pose_in_map(self):
        received_map = False
        get_pose_successed = False
        current_pose = lockup_pose(
            self.tf_listener, self.map_frame, self.robot_base
        )
        if current_pose == None:
            if not get_pose_successed:
                print_warn("Lockup pose error")
                get_pose_successed = True
            return False
        current_pose_type = ""
        cur_pose_dict = {}
        is_in_docking_pose = False
        # if rospy.get_time() - self.last_current_map < 0.2:
        #     cur_pose_dict = self.db.getCurrentPos(
        #         current_pose, self.current_map, accuracy=0.5
        #     )
        # else:
        #     if not received_map:
        #         print_warn("Current map timeout")
        #         received_map = True
        #     return False
        # print_debug(
        #     "Current pose: {}".format(json.dumps(cur_pose_dict, indent=2))
        # )
        return cur_pose_dict

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

    def std_io_cb(self, msg):
        try:
            self.std_io_dict = json.loads(msg.data)
            if "auto_charging_detect" in self.std_io_dict:
                self.is_auto_charging = self.std_io_dict["auto_charging_detect"]
            if "manual_charging_detect" in self.std_io_dict:
                self.is_man_charging = self.std_io_dict[
                    "manual_charging_detect"
                ]
        except:
            rospy.logerr("standard_io error")
        self.last_std_io_msg = rospy.get_time()

    def un_docking_result_cb(self, msg):
        self.un_docking_result = msg.status.status

    def auto_charging_stt_cb(self, msg):
        try:
            self.auto_charging_stt_dict = json.loads(msg.data)
        except Exception as e:
            rospy.logerr("Auto charging stt: {}".format(e))

    def action_result_cb(self, msg):
        # TODO: separate multi actions
        self.action_result = msg.status.status

    def action_status_cb(self, msg):
        pass

    def auto_docking_module_status_cb(self, msg):
        try:
            self.auto_docking_error_code = json.loads(msg.data)["error_code"]
        except Exception as e:
            rospy.logerr("auto_docking_module_status_cb: {}".format(e))

    def action_feedback_cb(self, msg):
        self.action_feedback = msg
        self.last_feedback = rospy.get_time()

    def mission_control_cb(self, msg):
        if msg.data == "START":
            rospy.loginfo(
                "-------------------Send mission goal-------------------"
            )
            goal = StringGoal()
            self._as_client.send_goal(goal)
            self._asm.pause_req = False

    def mission_from_server_cb(self, msg):
        goal = StringGoal(data=msg.data)
        self._as_client.send_goal(goal)
        self._asm.pause_req = False

    def current_map_cb(self, msg):
        self.current_map = msg.data
        self.last_current_map = rospy.get_time()

    def robot_status_cb(self, msg):
        """
        - mission_manager không quản lý lỗi của module vì ngoài các module
        action còn có các module khác như motor, lidar...
        - mission_manager chỉ xác định lỗi khi disconnect với action
        - Đúng ra mission_manager phải quản lý được các lỗi không phải disconnect
        của các action mình đang quản lý nhưng vì 1 module có thể có nhiều action
        - TOCHECK: Có 1 số lỗi như /un_docking: MOVING_ERROR, mission_manager
        status vẫn là PAUSED
        - DETECT_MARKER_ERROR tại sao vẫn reset được?
        - Để xác định đâu là lỗi của action, đâu là lỗi của ngoại vi, có thể
        kiểm tra /robot_status == ERROR mà mission_manager đang là PAUSED
        """
        self.robot_status_msg = msg

    def module_status_cb(self, data):
        try:
            self.module_error_code = json.load(data.data)["error_code"]
            print_debug(self.module_error_code)
        except Exception as e:
            pass

    """
       ###     ######  ######## ####  #######  ##    ##    ######## ##     ## ########  ######
      ## ##   ##    ##    ##     ##  ##     ## ###   ##    ##        ##   ##  ##       ##    ##
     ##   ##  ##          ##     ##  ##     ## ####  ##    ##         ## ##   ##       ##
    ##     ## ##          ##     ##  ##     ## ## ## ##    ######      ###    ######   ##
    ######### ##          ##     ##  ##     ## ##  ####    ##         ## ##   ##       ##
    ##     ## ##    ##    ##     ##  ##     ## ##   ###    ##        ##   ##  ##       ##    ##
    ##     ##  ######     ##    ####  #######  ##    ##    ######## ##     ## ########  ######
    """

    def mission_manager_exec_cb(self, mission_goal):
        string_goal = mission_goal.data
        load_mission_from_server = False
        if string_goal != "":
            try:
                goal_dict_fr_server = json.loads(string_goal)
                # print_debug(json.dumps(goal_dict_fr_server), "Goal from server")
                load_mission_from_server = True
            except Exception as e:
                rospy.logerr("Mission goal: {}".format(e))
        data_dict = {}
        success = False
        _state = MainState.INIT
        _prev_state = MainState.NONE
        action_index = 0
        total_action = 0
        mission_cnt = 1
        action_goal = None
        action_client = actionlib.SimpleActionClient(
            "", StringAction
        )  # For hint
        start_time = rospy.get_time()
        goal_accept = True
        feedback_msg = ""
        get_pose_successed = False
        received_map = False
        cur_pose_name = ""
        string_stamped_msg = StringStamped()
        self._asm.action_running = True
        un_docking_finish = False
        action_type = ""
        self._asm.reset_flag()
        _state_when_pause = MainState.NONE
        _state_when_error = MainState.NONE
        _state_bf_error = MainState.NONE
        module_status_sub = None
        mission_key = ""
        current_mission_is_undocking = True

        # Loop
        r = rospy.Rate(15.0)
        while goal_accept and not rospy.is_shutdown():
            if (
                _state != MainState.WAIT_RESULT
                and _state != MainState.UNDOCKING
            ):
                self.last_feedback = rospy.get_time()
            if _state != _prev_state:
                rospy.loginfo(
                    "Main state: {} -> {}".format(
                        _prev_state.toString(), _state.toString()
                    )
                )
                _prev_state = _state
                feedback_msg = _state.toString()
                self._asm.module_state = _state.toString()
                # self.db.updateQueueMission(
                #     "{}/{}".format(action_index + 1, total_action), "Doing"
                # )
            self.send_feedback(self._as, feedback_msg)

            if self._as.is_preempt_requested() or self._asm.reset_action_req:
                rospy.loginfo("%s: Preempted" % self._action_name)
                self._as.set_preempted()
                success = False
                self.send_feedback(
                    self._as, GoalStatus.to_string(GoalStatus.PREEMPTED)
                )
                self.db.updateQueueMission(
                    "{}/{}".format(action_index + 1, total_action), "Cancel"
                )
                if "name" in data_dict:
                    self.db.recordMissionLog(
                        mission_key,
                        data_dict["name"],
                        status=MissionStatus.CANCEL.toString(),
                    )
                break

            if (
                self._asm.module_status != ModuleStatus.ERROR
            ):  # Lỗi của module mission_manager chứ k phải của các action
                self._asm.error_code = ""
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    self.mission_client_dict[action_type]["reset_pub"].publish(
                        EmptyStamped(stamp=rospy.Time.now())
                    )
                    rospy.loginfo(
                        "Forward reset error to: {}".format(action_type)
                    )
            # if _state != MainState.PAUSED and _state != MainState.UNDOCKING_ERROR:
            if _state != MainState.PAUSED and self._asm.error_code == "":
                self._asm.module_status = ModuleStatus.RUNNING

            # Update status
            self.cur_mission_name = (
                data_dict["name"] if "name" in data_dict else ""
            )
            self.current_action_num = action_index + 1
            if "actions" in data_dict:
                if len(data_dict["actions"]) and action_index < len(
                    data_dict["actions"]
                ):
                    self.current_action_type = data_dict["actions"][
                        action_index
                    ]["type"]
                    self.current_action_name = data_dict["actions"][
                        action_index
                    ]["name"]
                # if _state == MainState.UNDOCKING or _state == MainState.UNDOCKING_ERROR:
                #     self.current_action_type = "un_docking"
                if current_mission_is_undocking:
                    self.current_action_type = "un_docking"
            self.total_action = total_action

            # State: INIT
            if _state == MainState.INIT:
                # Read DB
                # try:
                if True:
                    ######
                    current_pose = lockup_pose(
                        self.tf_listener, self.map_frame, self.robot_base
                    )
                    if current_pose == None:
                        if not get_pose_successed:
                            print_warn("Lockup pose error")
                            get_pose_successed = True
                        continue
                    current_pose_type = ""
                    cur_pose_dict = {}
                    is_in_docking_pose = False
                    if rospy.get_time() - self.last_current_map < 0.2:
                        cur_pose_dict = self.db.getCurrentPos(
                            current_pose, self.current_map, accuracy=0.5
                        )
                    else:
                        if not received_map:
                            print_warn("Current map timeout")
                            received_map = True
                        continue
                    print_debug(
                        "Current pose: {}".format(
                            json.dumps(cur_pose_dict, indent=2)
                        )
                    )
                    if "name" in cur_pose_dict:
                        cur_pose_name = cur_pose_dict["name"]
                    ###### TODO: Use get_current_pose_in_map()
                    # cur_pose_dict = self.get_current_pose_in_map()
                    # if not cur_pose_dict: continue
                    # if "name" in cur_pose_dict:
                    #     cur_pose_name = cur_pose_dict["name"]
                    ######
                    if not load_mission_from_server:
                        data_dict = self.db.getQueueMission(
                            curPos=cur_pose_name,
                            curMap=self.current_map,
                            checkRobotInPoseWhenStart=self.check_robot_in_pose_when_start,
                        )
                        if data_dict == "ROBOT_NOT_IN_POSE":
                            _state = MainState.ROBOT_NOT_IN_POSE
                            _state_when_error = _state
                            continue
                    else:
                        if goal_dict_fr_server["result"] != None:
                            data_dict_temp = goal_dict_fr_server["result"]
                            # Tạo bản sao của data_dict
                            data_dict = data_dict_temp.copy()

                            # Duyệt qua các action bằng index
                            for i in range(len(data_dict["actions"])):
                                # Kiểm tra nếu là action "Move to position" và không phải action cuối cùng
                                if (
                                    i <= len(data_dict["actions"]) - 1
                                    and data_dict["actions"][i]["type"]
                                    == "move"
                                ):

                                    try:
                                        # Tạo dictionary param_test nếu chưa tồn tại
                                        if (
                                            "param_test"
                                            not in data_dict["actions"][i]
                                        ):
                                            data_dict["actions"][i][
                                                "param_test"
                                            ] = {}
                                            data_dict["actions"][i][
                                                "param_test"
                                            ][
                                                "need_to_wait_receive_new_path"
                                            ] = True
                                            data_dict["actions"][i][
                                                "param_test"
                                            ]["need_set_vel_before_move"] = True
                                    except Exception as e:
                                        print(
                                            f"Error processing action {i+1}: {str(e)}"
                                        )

                            # Kiểm tra kết quả
                            for action in data_dict["actions"]:
                                if action["type"] == "move":
                                    rospy.logwarn(action)

                            self.is_mission_fr_server = True
                            self.running_mission_id = data_dict[
                                "mission_queue_id"
                            ]
                            if "agv_group" in data_dict:
                                self.mission_group = data_dict["agv_group"]
                            if "name" not in data_dict:
                                data_dict["name"] = "from_server"
                        else:
                            success = True
                            rospy.loginfo(
                                "Mission from server empty --> stop queue"
                            )
                            break
                    total_action = 0
                    try:
                        json_string = dumps(data_dict, indent=2, sort_keys=True)
                        data_dict = json.loads(
                            json_string
                        )  # Convert BSON to JSON
                        if not data_dict:  # Empty
                            rospy.loginfo("Mission empty --> stop queue")
                            success = True
                            break
                        rospy.loginfo(
                            "Mission from server:\n{}".format(json_string)
                        )
                        total_action = len(data_dict["actions"])
                    except Exception as e:
                        # Only happen with data from server
                        rospy.logerr("Server data error: {}".format(e))
                        print_debug("Data: {}".format(data_dict))
                        _state = MainState.SERVER_DATA_ERROR
                        continue
                    action_index = 0
                    un_docking_finish = False
                    if module_status_sub != None:
                        module_status_sub.unregister()
                    # Record log
                    self.db.recordLog(
                        "Start mission: {}".format(data_dict["name"]),
                        rospy.get_name(),
                        LogLevel.INFO.toString(),
                    )
                    mission_key = str(rospy.get_time())
                    self.db.recordMissionLog(
                        mission_key,
                        data_dict["name"],
                        status=MissionStatus.RUNNING.toString(),
                    )

                    _state = MainState.SEND_ACTION_GOAL
                # except Exception as e:
                #     rospy.logerr('Database data systax error: {}'.format(e))
                #     self.send_feedback(self._as, GoalStatus.to_string(GoalStatus.ABORTED))
                #     _state = MainState.ERROR
            # State: UNDOCKING
            if _state == MainState.UNDOCKING:
                if rospy.get_time() - self.last_feedback >= 5.0:
                    rospy.logerr("Action /un_docking disconnected!")
                    self.send_feedback(
                        self._as, GoalStatus.to_string(GoalStatus.ABORTED)
                    )
                    _state_bf_error = (
                        MainState.SEND_ACTION_GOAL
                    )  # TOCHCHEK: Check funtion này
                    _state_when_error = _state
                    _state = MainState.UNDOCKING_DISCONNECTED
                if self.un_docking_result == GoalStatus.SUCCEEDED:
                    un_docking_finish = True
                    _state = MainState.SEND_ACTION_GOAL
                elif (
                    self.un_docking_result != GoalStatus.SUCCEEDED
                    and self.un_docking_result != GoalStatus.ACTIVE
                    and self.un_docking_result != -1
                ) or self.auto_docking_error_code != "":
                    rospy.logerr(
                        "Undocking action fail: {}".format(
                            GoalStatus.to_string(self.un_docking_result)
                        )
                    )
                    self.send_feedback(
                        self._as, GoalStatus.to_string(GoalStatus.ABORTED)
                    )
                    _state_bf_error = (
                        MainState.SEND_ACTION_GOAL
                    )  # TOCHCHEK: Check funtion này
                    _state_when_error = _state
                    _state = MainState.UNDOCKING_ERROR
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.auto_docking_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED
            # State: SEND_ACTION_GOAL
            elif _state == MainState.SEND_ACTION_GOAL:

                # Make goal and send
                if len(data_dict["actions"]) == 0:
                    _state = MainState.MISSION_EMPTY
                    continue
                if data_dict["actions"][action_index] == None:
                    _state = MainState.ACTION_DATA_NULL
                    continue
                action_type = data_dict["actions"][action_index]["type"]
                action_client = self.mission_client_dict[action_type][
                    "action_client"
                ]
                need_undocking = self.mission_client_dict[action_type][
                    "need_undocking"
                ]
                self.cur_action_type = action_type
                # Check undocking
                if need_undocking:
                    ######
                    # current_pose = lockup_pose(
                    #     self.tf_listener, self.map_frame, self.robot_base
                    # )
                    # if current_pose == None:
                    #     if not get_pose_successed:
                    #         print_warn("Lockup pose error")
                    #         get_pose_successed = True
                    #     continue
                    current_pose_type = ""
                    cur_pose_dict = {}
                    is_in_docking_pose = False
                    # if rospy.get_time() - self.last_current_map < 0.2:
                    #     cur_pose_dict = self.db.getCurrentPos(
                    #         current_pose, self.current_map, accuracy=0.5
                    #     )
                    # else:
                    #     if not received_map:
                    #         print_warn("Current map timeout")
                    #         received_map = True
                    #     continue
                    print_debug(
                        "Current pose: {}".format(
                            json.dumps(cur_pose_dict, indent=2)
                        )
                    )
                    ###### TODO: Use get_current_pose_in_map()
                    # current_pose_type = ""
                    # is_in_docking_pose = False
                    # cur_pose_dict = self.get_current_pose_in_map()
                    # if not cur_pose_dict: continue
                    ######
                    if "name" in cur_pose_dict:
                        cur_pose_name = cur_pose_dict["name"]
                    if (
                        "type" in cur_pose_dict
                        and cur_pose_dict["type"] != ""
                        and cur_pose_dict["type"] != "position"
                    ):
                        if (
                            "info" in cur_pose_dict
                            and cur_pose_dict["info"] == "duplicate_marker"
                        ):
                            _state = MainState.DUPLICATE_MARKER
                            continue
                        if (
                            "diff_angle" in cur_pose_dict
                            and abs(cur_pose_dict["diff_angle"])
                            > self.undocking_angle_allow
                        ):
                            _state = MainState.MARKER_DIR_WRONG
                            continue

                        rospy.loginfo(
                            "Robot is in docking pose, need un-docking before run action"
                        )
                        current_pose_type = cur_pose_dict["type"]
                        # TODO: Undocking send to /auto_docking with marker type,
                        # send command disconnect auto charging
                        is_in_docking_pose = True  # TESTING

                    if self.is_auto_charging:
                        current_pose_type = "vl_marker_for_mav_charger"  # TODO
                    if (
                        self.is_auto_charging or is_in_docking_pose
                    ) and not un_docking_finish:
                        self.cancel_charging_pub.publish(
                            EmptyStamped(stamp=rospy.Time.now())
                        )
                        undocking_goal = StringGoal()
                        undocking_goal_dict = {}
                        undocking_goal_dict["params"] = {
                            "marker_type": current_pose_type
                        }
                        undocking_goal.data = json.dumps(
                            undocking_goal_dict, indent=2
                        )
                        # module_status_sub = rospy.Subscriber(
                        #     self.mission_client_dict["un_docking"][
                        #         "module_status_topic"
                        #     ],
                        #     StringStamped,
                        #     self.module_status_cb,
                        # )
                        self.undocking_client.send_goal(
                            undocking_goal, feedback_cb=self.action_feedback_cb
                        )
                        self.un_docking_result = -1
                        self.last_feedback = rospy.get_time()
                        _state = MainState.UNDOCKING
                        rospy.loginfo(
                            'Undocking before run action no "{}": "{}"'.format(
                                action_index + 1,
                                self.mission_client_dict[action_type][
                                    "action_server"
                                ],
                            )
                        )
                        continue
                # No need undocking
                goal = StringGoal()
                goal_dict = {}
                if (
                    action_type == "move"
                    or action_type == "move_to_position_mbf"
                ):
                    goal_dict["waypoints"] = data_dict["actions"][action_index][
                        "waypoints"
                    ]
                goal_dict["params"] = data_dict["actions"][action_index][
                    "params"
                ]
                goal_dict["name"] = data_dict["actions"][action_index]["name"]
                goal_dict["type"] = data_dict["actions"][action_index]["type"]
                if "param_test" in data_dict["actions"][action_index]:
                    goal_dict["param_test"] = data_dict["actions"][
                        action_index
                    ]["param_test"]
                # next_action_type for moving_control
                try:
                    if action_index < total_action - 1:
                        goal_dict["next_action_type"] = data_dict["actions"][
                            action_index + 1
                        ]["type"]
                    elif action_index == total_action - 1:
                        goal_dict["next_action_type"] = (
                            "This is the last action"
                        )
                except Exception as e:
                    rospy.logerr("Missing data in action")
                    _state = MainState.MISSING_ACTION_DATA
                    continue
                goal.data = json.dumps(goal_dict, indent=2)
                # print_debug(
                #     """Send action type "{}" goal:\n{}""".format(
                #         action_type, goal.data
                #     )
                # )
                action_client.send_goal(
                    goal, feedback_cb=self.action_feedback_cb
                )  # Only handle feedback of action which call by mission_manager
                self.action_result = -1
                self.last_feedback = rospy.get_time()
                rospy.loginfo(
                    'Send action no "{}" in mission no "{}"'.format(
                        action_index + 1, mission_cnt
                    )
                )
                current_mission_is_undocking = False
                _state = MainState.WAIT_RESULT
            # State: WAIT_RESULT
            elif _state == MainState.WAIT_RESULT:
                # There is only one method "handle action feedback" to check action_server alive.
                if rospy.get_time() - self.last_feedback >= 5.0:
                    rospy.logerr(
                        'Action "{}" disconnected!'.format(
                            action_client.action_client.ns
                        )
                    )
                    self.send_feedback(
                        self._as, GoalStatus.to_string(GoalStatus.ABORTED)
                    )
                    # self._as.set_aborted(text="Action disconnected!")
                    # break
                    _state = MainState.ERROR
                    continue
                action_result = action_client.get_state()
                try:
                    action_result = int(action_result)
                except:
                    rospy.logwarn("Action result is not <int>")
                    action_result = GoalStatus.ACTIVE
                if (
                    action_result < GoalStatus.ACTIVE
                    or GoalStatus.ACTIVE > GoalStatus.LOST
                ):
                    rospy.logwarn(
                        "Action result is out of range: ".format(action_result)
                    )
                    action_result = GoalStatus.ACTIVE

                if action_result == GoalStatus.SUCCEEDED:
                    action_index += 1
                    if module_status_sub != None:
                        module_status_sub.unregister()
                    un_docking_finish = False
                    if action_index < total_action:
                        _state = MainState.SEND_ACTION_GOAL
                    else:
                        self.last_mission_done_id = self.running_mission_id
                        self.running_mission_id = ""
                        self.cur_mission_name = ""
                        self.db.recordLog(
                            "Mission execute successful: {}".format(
                                data_dict["name"]
                            ),
                            rospy.get_name(),
                            LogLevel.INFO.toString(),
                        )
                        self.db.recordMissionLog(
                            mission_key,
                            data_dict["name"],
                            status=MissionStatus.SUCCEEDED.toString(),
                        )
                        self.db.deleteQueueMission()
                        if load_mission_from_server:
                            goal_dict_fr_server["result"] = None
                        # Jump to INIT because each action need check position before start
                        _state = MainState.INIT
                        mission_cnt += 1
                elif (
                    action_result != GoalStatus.SUCCEEDED
                    and action_result != GoalStatus.ACTIVE
                ):
                    rospy.logerr(
                        'Action number {} "{}" fail: {}'.format(
                            action_index + 1,
                            action_client.action_client.ns,
                            GoalStatus.to_string(action_result),
                        )
                    )
                    # rospy.logerr(type(action_result))
                    # rospy.logwarn("Cancel all action!")
                    # self.send_feedback(self._as, GoalStatus.to_string(GoalStatus.ABORTED))
                    # self._as.set_aborted(text="Action fail!")
                    self.db.recordMissionLog(
                        mission_key,
                        data_dict["name"],
                        status=MissionStatus.ERROR.toString(),
                    )
                    _state_bf_error = (
                        MainState.SEND_ACTION_GOAL
                    )  # TOCHCHEK: Check funtion này
                    _state_when_error = _state
                    _state = MainState.ACTION_EXEC_ERROR
                    continue
                if self._asm.pause_req:
                    if self._asm.pause_req_by_server:
                        self.mission_client_dict[action_type][
                            "run_pause_req_pub"
                        ].publish(
                            StringStamped(
                                stamp=rospy.Time.now(), data="PAUSE_BY_SERVER"
                            )
                        )
                    else:
                        self.mission_client_dict[action_type][
                            "run_pause_req_pub"
                        ].publish(
                            StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                        )
                    self._asm.reset_flag()
                    self.db.recordMissionLog(
                        mission_key,
                        data_dict["name"],
                        status=MissionStatus.PAUSED.toString(),
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED
            # State: PAUSED
            elif _state == MainState.PAUSED:
                self._asm.module_status = ModuleStatus.PAUSED
                if self._asm.pause_req:
                    if self._asm.pause_req_by_server:
                        self.mission_client_dict[action_type][
                            "run_pause_req_pub"
                        ].publish(
                            StringStamped(
                                stamp=rospy.Time.now(), data="PAUSE_BY_SERVER"
                            )
                        )
                    else:
                        self.mission_client_dict[action_type][
                            "run_pause_req_pub"
                        ].publish(
                            StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                        )
                    self._asm.reset_flag()
                if self._asm.resume_req:
                    self._asm.reset_flag()
                    if _state_when_pause == MainState.UNDOCKING:
                        self.auto_docking_run_pause_pub.publish(
                            StringStamped(stamp=rospy.Time.now(), data="RUN")
                        )
                    elif _state_when_pause == MainState.WAIT_RESULT:
                        self.mission_client_dict[action_type][
                            "run_pause_req_pub"
                        ].publish(
                            StringStamped(stamp=rospy.Time.now(), data="RUN")
                        )
                    self.db.recordMissionLog(
                        mission_key,
                        data_dict["name"],
                        status=MissionStatus.RUNNING.toString(),
                    )
                    _state = _state_when_pause

            # State: ERROR
            elif _state == MainState.ERROR:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/mission_manager: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    # Resend error action
                    _state = MainState.SEND_ACTION_GOAL
            # State: MISSION_EMPTY
            elif _state == MainState.MISSION_EMPTY:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/mission_manager: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    # Resend error action
                    _state = MainState.INIT
            # State: ACTION_DATA_NULL
            elif _state == MainState.ACTION_DATA_NULL:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/mission_manager: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    # Resend error action
                    _state = MainState.INIT
            # State: ACTION_EXEC_ERROR
            elif _state == MainState.ACTION_EXEC_ERROR:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/mission_manager: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    # Resend error action
                    _state = _state_bf_error
            # State: MISSING_ACTION_DATA
            elif _state == MainState.MISSING_ACTION_DATA:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/mission_manager: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    # Resend error action
                    _state = MainState.INIT
            # State: SERVER_DATA_ERROR
            elif _state == MainState.SERVER_DATA_ERROR:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/mission_manager: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    # Resend error action
                    _state = MainState.INIT
            # ROBOT_NOT_IN_POSE
            elif _state == MainState.ROBOT_NOT_IN_POSE:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/mission_manager: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    # Resend error action
                    _state = MainState.INIT
            # State: UNDOCKING_ERROR
            elif _state == MainState.UNDOCKING_ERROR:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = (
                    "/mission_manager: {}".format(_state.toString())
                    + self.module_error_code
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    # Do not reset mission
                    self.auto_docking_error_code = ""
                    self.auto_docking_reset_error_pub.publish(
                        EmptyStamped(stamp=rospy.Time.now())
                    )
                    _state = _state_when_error
            # State: UNDOCKING_DISCONNECTED
            elif _state == MainState.UNDOCKING_DISCONNECTED:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/mission_manager: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    # Reset mission
                    # _state = MainState.INIT
                    # Có thể quay lại SEND_ACTION_GOAL mà không cần quay lại INIT
                    # vì un_docking_finish chỉ được set khi UNDOCKING successfully
                    _state = _state_bf_error
            # State: DUPLICATE_MARKER
            elif _state == MainState.DUPLICATE_MARKER:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/mission_manager: {}".format(
                    _state.toString()
                )
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    # Reset mission
                    _state = MainState.INIT
            # State: MARKER_DIR_WRONG
            elif _state == MainState.MARKER_DIR_WRONG:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = "/mission_manager: {}".format(
                    _state.toString()
                )

                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    # Reset mission
                    _state = MainState.INIT

            r.sleep()

        self._asm.pause_req = False
        self._asm.action_running = False

        self.reset_status()

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
        rate = rospy.Rate(5)
        msg = StringStamped()
        while not rospy.is_shutdown():
            if not self._asm.action_running:
                self._asm.module_status = ModuleStatus.WAITING
                self._asm.error_code = ""
                # self.switch_control_tf(USE_TF_LIDAR)
            msg.stamp = rospy.Time.now()
            msg.data = json.dumps(
                {
                    "status": self._asm.module_status.toString(),
                    "state": self._asm.module_state,
                    "error_code": self._asm.error_code,
                    "running_mission_id": self.running_mission_id,
                    "last_done_mission_id": self.last_mission_done_id,
                    "current_mission_name": self.cur_mission_name,
                    "current_action_num": self.current_action_num,
                    "is_mission_fr_server": self.is_mission_fr_server,
                    "current_action_type": self.current_action_type,
                    "current_action_name": self.current_action_name,
                    "total_action": self.total_action,
                    "mission_group": self.mission_group,
                }
            )
            self._asm.module_status_pub.publish(msg)
            rate.sleep()


def parse_opt():
    parser = argparse.ArgumentParser()

    opt = parser.parse_args()
    return opt


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
        "--mission_list",
        dest="mission_list",
        default=os.path.join(
            rospkg.RosPack().get_path("mission_manager"),
            "cfg",
            "mission_list.yaml",
        ),
        type=str,
        help="mission_list config file path",
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
    rospy.init_node("mission_manager", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    MissionManagerServer(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
