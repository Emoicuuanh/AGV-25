#! /usr/bin/env python
import rospy
import yaml
import rospkg
import copy
import actionlib
import json
import os
import sys

import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from logging import debug
from bson.json_util import dumps

from actionlib_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseWithCovarianceStamped,Pose
from std_stamped_msgs.msg import StringAction, StringStamped, StringResult, StringFeedback, StringGoal, Int8Stamped, EmptyStamped
from std_stamped_msgs.srv import StringService, StringServiceResponse

from cartographer_ros_msgs.srv import GetScoreGlobalMatching, GetScoreGlobalMatchingRequest, GetScoreGlobalMatchingResponse
from cartographer_ros_msgs.srv import ImproveMap, ImproveMapRequest, ImproveMapResponse

common_func_dir = os.path.join(rospkg.RosPack().get_path('agv_common_library'), 'scripts')
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(rospkg.RosPack().get_path('agv_common_library'), 'release')
sys.path.insert(0, common_func_dir)

from module_manager import ModuleServer, ModuleClient, ModuleStatus
from actionlib_msgs.msg import GoalStatus, GoalID, GoalStatusArray

common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)

from common_function import (
    dict_to_pose,
)

class MainState():
    SEND_INITIALPOSE = 0
    WAIT_OPTIMIZE = 1
    WAIT_COLLECT_DATA = 2
    PAUSED = 9
    SET_RESULT = 10

class ActionServerRelocalization():
    _feedback = StringFeedback()
    _result = StringResult()

    def __init__(self, name , *args, **kwargs):
        self.init_variable(*args, **kwargs)
        # Action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, StringAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        # Publisher
        self.pub_initialpose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.pub_set_status_to_waitting_init_pose = rospy.Publisher("/set_status_to_waitting_init_pose", EmptyStamped, queue_size=1)

        # Subscriber

        # Service
        self.get_score_slam = rospy.ServiceProxy("/get_score_global_matching", GetScoreGlobalMatching)
        self.run_optimize = rospy.ServiceProxy("/improve_map", ImproveMap)

        # ModuleServer
        self._asm = ModuleServer(name)
        # Loop
        self.loop()

    def init_variable(self, *args, **kwargs):
        self.rate = rospy.Rate(5)

        self.main_state = MainState()

        self.increase_degree = rospy.get_param('~increase_degree', 360)
        self.time_wait_one_cycle = rospy.get_param('time_wait_one_cycle', 20)
        self.numble_try = int(360/self.increase_degree)

    def action_feedback_cb(self, msg):
        self.action_feedback = msg
        self.last_feedback = rospy.get_time()

    def add_yaw_to_pose(self, input_pose, degrees):
        """
        Hàm cộng thêm góc yaw vào pose.

        Args:
            input_pose (geometry_msgs/Pose): Pose đầu vào
            degrees (float): Số độ muốn cộng thêm vào góc yaw

        Returns:
            geometry_msgs/Pose: Pose mới với góc yaw đã được cập nhật
        """
        # Lấy quaternion từ pose
        quaternion = [
            input_pose.orientation.x,
            input_pose.orientation.y,
            input_pose.orientation.z,
            input_pose.orientation.w
        ]

        # Chuyển quaternion thành góc Euler (roll, pitch, yaw)
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        # Cộng số độ (chuyển sang radian)
        yaw += math.radians(degrees)

        # Chuẩn hóa góc yaw về khoảng [-π, π]
        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi

        # Chuyển góc Euler mới thành quaternion
        new_quaternion = quaternion_from_euler(roll, pitch, yaw)

        # Tạo pose mới
        new_pose = Pose()
        new_pose.position = input_pose.position  # Giữ nguyên vị trí
        new_pose.orientation.x = new_quaternion[0]
        new_pose.orientation.y = new_quaternion[1]
        new_pose.orientation.z = new_quaternion[2]
        new_pose.orientation.w = new_quaternion[3]

        return new_pose

    def execute_cb(self, goal):
        self._asm.reset_flag()
        _state = self.main_state.SEND_INITIALPOSE

        goal_dict = json.loads(goal.data)
        goal_init = goal_dict["params"]["init_pose"]
        goal_init_pose = dict_to_pose(goal_init)

        msg_goal_initialpose = PoseWithCovarianceStamped()
        msg_get_score = GetScoreGlobalMatchingRequest()
        count_try = 0

        time_start_optimize = rospy.get_time()

        wait_computer_scan_global_matching = False
        bool_localization = False

        while not rospy.is_shutdown():
            # check cancel
            if self._as.is_preempt_requested() or self._asm.reset_action_req:
                self._asm.reset_flag()
                self._as.set_preempted()
                rospy.loginfo("Cancel action localization")
                return

            if _state == self.main_state.SEND_INITIALPOSE:
                msg_goal_initialpose.header.stamp = rospy.Time.now()
                msg_goal_initialpose.header.frame_id = "map"
                msg_goal_initialpose.pose.pose = goal_init_pose
                self.pub_initialpose.publish(msg_goal_initialpose)
                count_try +=1
                time_start_optimize = rospy.get_time()
                self.run_optimize.call(ImproveMapRequest())
                self.pub_set_status_to_waitting_init_pose.publish(EmptyStamped())
                _state = self.main_state.WAIT_OPTIMIZE

            elif  _state == self.main_state.WAIT_OPTIMIZE:
                response = self.get_score_slam.call(GetScoreGlobalMatchingRequest())
                rospy.loginfo(response.score.data)

                # Score > 0 => suceed localization
                if response.score.data < 0.1:
                    bool_localization = False
                else:
                    bool_localization = True

                if bool_localization:
                    _state = self.main_state.SET_RESULT
                elif rospy.get_time() - time_start_optimize < self.time_wait_one_cycle:
                    rospy.loginfo("Waiting computer matching")
                elif count_try >= self.numble_try and not bool_localization:
                    _state = self.main_state.SET_RESULT
                else:
                    goal_init_pose = self.add_yaw_to_pose(goal_init_pose, self.increase_degree)
                    _state = self.main_state.SEND_INITIALPOSE

            elif _state == self.main_state.SET_RESULT:
                if bool_localization:
                    rospy.logwarn("Succeed localization")
                    self._result.status = 1
                else:
                    rospy.logwarn("Fail localization")
                    self._result.status = 0

                self._as.set_succeeded(self._result)
                return

            self._feedback.data = self._action_name + "/" + str(_state)
            self._as.publish_feedback(self._feedback)
            self.rate.sleep()

    def loop(self):
        r = rospy.Rate(1)
        status_msg = StringStamped()
        while not rospy.is_shutdown():
            if not self._asm.action_running:
                self._asm.module_status = ModuleStatus.WAITING
                self._asm.module_state = ModuleStatus.WAITING.toString()
                self._asm.error_code = ""
            status_msg.stamp = rospy.Time.now()
            status_msg.data = json.dumps({"status": self._asm.module_status.toString(),
                                        "state": self._asm.module_state,
                                        "error_code": self._asm.error_code})
            self._asm.module_status_pub.publish(status_msg)
            r.sleep()

def parse_opts():
    from optparse import OptionParser
    parser = OptionParser()
    parser.add_option("-d", "--ros_debug",
                    action="store_true", dest="log_debug", default=False, help="log_level=rospy.DEBUG")
    parser.add_option("-c", "--config_path", dest="config_path",
                    default=os.path.join(rospkg.RosPack().get_path('cartographer_run'), 'cfg'))

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)

def main():
    (options, args) = parse_opts()
    log_level = None
    if options.log_debug:
        log_level = rospy.DEBUG
    rospy.init_node('relocalization_cartographer_server', log_level=log_level)
    rospy.loginfo('Init node ' + rospy.get_name())
    ActionServerRelocalization(rospy.get_name(), **vars(options))

if __name__ == '__main__':
    main()
