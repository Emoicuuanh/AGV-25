#! /usr/bin/env python
# -*- coding: utf-8 -*-

from logging import debug
from bson.json_util import dumps
import os
import sys
import rospy
import rospkg
import copy
import actionlib
import json
from nav_msgs.msg import Odometry, Path
from std_stamped_msgs.msg import StringAction, StringStamped, StringResult, StringFeedback, StringGoal, Int8Stamped, EmptyStamped
from agv_msgs.msg import DataMatrixStamped, CartesianPosition
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped, Point
from actionlib_msgs.msg import GoalStatus, GoalID, GoalStatusArray
from safety_msgs.msg import SafetyStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionResult, MoveBaseActionFeedback, MoveBaseGoal, MoveBaseActionGoal
import math
# from std_stamped_msgs.srv import StringService, StringServiceResponse

common_func_dir = os.path.join(rospkg.RosPack().get_path('agv_common_library'), 'scripts')
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(rospkg.RosPack().get_path('agv_common_library'), 'release')
sys.path.insert(0, common_func_dir)

from module_manager import ModuleServer, ModuleClient, ModuleStatus
from common_function import (
    EnumString,
    lockup_pose,
    dict_to_obj,
    merge_two_dicts,
    print_debug,
    print_info,
    print_warn,
    print_error,
    obj_to_dict,
    dict_to_pose
)
class MainState(EnumString):
    NONE = -1
    INIT = 0
    NORMAL = 1
    WAITING_FOR_INIT = 2
    ERROR = 3

class qrTrackingNode(object):
    def __init__(self, name, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        # ModuleServer
        self._asm = ModuleServer(name)
        # Publisher
        self.init_pose_publisher = rospy.Publisher('/initialpose' , PoseWithCovarianceStamped, queue_size=10)
        self.moving_control_run_pause_pub = rospy.Publisher("/moving_control/run_pause_req", StringStamped, queue_size=5)
        # self.label_tracking_status_publisher = rospy.Publisher('~tracker_status', StringStamped, queue_size=10)
        # Subscriber
        rospy.Subscriber('current_path', Path, self.path_callback)
        rospy.Subscriber("/data_gls621", DataMatrixStamped, self.label_reader_callback)
        rospy.Subscriber('/robot_pose', Pose, self.robot_pose_callback)
        # rospy.Subscriber('~reset_tracker', StringStamped, self.tracker_reset_callback)
        rospy.Subscriber('robot_status', StringStamped, self.robot_status_callback)
        rospy.Subscriber('/mission_manager/reset_error', EmptyStamped, self.reset_error_callback)
        rospy.Subscriber(
            "/mission_manager/module_status",
            StringStamped,
            self.mission_status_cb
        )
        rospy.Subscriber("/disable_check_error_qr_code", Int8Stamped, self.disable_check_error_qr_code_callback)

        #For checking lost qr at crossroad
        rospy.Subscriber('/lost_goal_label', Pose, self.lost_goal_label_callback)

        rospy.Subscriber('/goal_send_to_server', PoseStamped, self.goal_send_to_server_callback)

        # Event
        rospy.on_shutdown(self.shutdown)
        # Loop
        self.loop()

    def init_variable(self, *args, **kwargs):
        self.min_label_distance = rospy.get_param("min_label_distance")
        self.loop_rate = rospy.get_param("loop_rate")

        rospy.loginfo(f'min_label_distance: {self.min_label_distance}')
        rospy.loginfo(f'loop_rate: {self.loop_rate}')

        self.current_qr_label = CartesianPosition() #store current QR code label


        self.label_pose = Pose()
        self.robot_pose = Pose()
        self.in_manual = True

        self._state = MainState.INIT
        self._prev_state = MainState.NONE

        self.reset_req = False
        self.distance = 0.0
        self.lost_qr_at_goal = False # ERROR flag
        self.goal_pose = Pose()
        self.first_pose_in_path = Pose()
        self.last_pose_in_path = Pose()
        self.last_time_read_qr = rospy.get_time()
        self.disable_check_qr = False
        self.need_receive_new_path = True
        self.count_check = 0
        self.mission_status = "WAITING"

    def shutdown(self):
        rospy.loginfo("Shutting down...")

    def mission_status_cb(self, msg):
        data_dict = json.loads(msg.data)
        if "status" in data_dict:
            self.mission_status = data_dict["status"]

    def lost_goal_label_callback(self, msg):
        self.lost_qr_at_goal  = False
        self.goal_pose = msg

    def goal_send_to_server_callback(self, msg):
        self.need_receive_new_path = True

    def reset_error_callback(self, msg):
        rospy.logwarn_once('RESET QR TRACKER ERROR')
        self.reset_req = True

    def robot_status_callback(self, msg):
        data = json.loads(msg.data)
        if "state" in data:
            if data["state"] == "MANUAL":
                self.in_manual = True
            elif data["state"] == "AUTO":
                self.in_manual = False
        if "status" in data:
            if data["status"] == "WAITING":
                self.need_receive_new_path = True

    def path_callback(self,msg):
        if len(msg.poses) == 0:
            rospy.loginfo("No poses in the path.")
            return

        if len(msg.poses) == 1:
            self.first_pose_in_path = msg.poses[0].pose
            self.last_pose_in_path = msg.poses[0].pose
        else:
            self.first_pose_in_path = msg.poses[0].pose
            self.last_pose_in_path = msg.poses[-1].pose

        self.first_pose_in_path.position.x = round(self.first_pose_in_path.position.x, 3)
        self.first_pose_in_path.position.y = round(self.first_pose_in_path.position.y, 3)

        self.last_pose_in_path.position.x = round(self.last_pose_in_path.position.x, 3)
        self.last_pose_in_path.position.y = round(self.last_pose_in_path.position.y, 3)
        self.need_receive_new_path = False

    def disable_check_error_qr_code_callback(self, msg):
        self.disable_check_qr = msg.data

    def label_reader_callback(self, msg):
        self.label_pose = self.label_msg_to_pose(msg)
        self.last_time_read_qr = rospy.get_time()
        # rospy.loginfo(f"CURRENT QR pose:{self.qr_pose_x} : {self.qr_pose_y}")
        if (self.current_qr_label.x != msg.lable.x or self.current_qr_label.y != msg.lable.y):
            # update current qr data
            self.current_qr_label = msg.lable
            # rospy.logwarn(f"CURRENT QR data:{self.current_qr_label}")

    def robot_pose_callback(self, msg):
        self.robot_pose = msg

    # def tracker_reset_callback(self, msg):
    #     if msg.data is not None:
    #         self.reset_req = True

    # TODO: Change to accurate init position
    def label_msg_to_pose(self, qr_msg):
        qr_pose = Pose()
        qr_pose.position.x = (qr_msg.lable.x)/1000
        qr_pose.position.y = (qr_msg.lable.y)/1000
        return qr_pose

    def robot_is_on_label(self, label_pose, current_pose):
        x_dist = abs(current_pose.position.x - label_pose.position.x)
        y_dist = abs(current_pose.position.y - label_pose.position.y)
        # rospy.logerr(f'x_dist: {x_dist} \n y_dist: {y_dist}')
        self.distance = math.sqrt(x_dist * x_dist + y_dist * y_dist)
        if (self.distance > self.min_label_distance or self.point_to_line_distance_pose(self.first_pose_in_path, self.last_pose_in_path, self.robot_pose) >= 0.5):
            # rospy.logerr("robot lost track of qr code")
            self.count_check += 1
            if self.count_check > 2:
                return False
            return True
        else:
            # rospy.loginfo("robot is on qr code")
            self.count_check = 0
            return True

    def pub_initial_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.header.frame_id = 'map'
        initial_pose.pose.pose.position.x = self.label_pose.position.x
        initial_pose.pose.pose.position.y = self.label_pose.position.y
        # TODO: Need to publish orientation info

    def pub_tracker_status(self, status):
        tracker_status_msg = StringStamped()
        tracker_status_msg.stamp = rospy.Time.now()
        tracker_status_msg.data = status.toString()
        self.label_tracking_status_publisher.publish(tracker_status_msg)

    def point_to_line_distance_pose(self, pose_A, pose_B, pose_C):
        if self.disable_check_qr:
            return 0
        if self.mission_status != "RUNNING":
            return 0
        if (pose_A == Pose() or pose_B == Pose()):
            return 0
        if self.need_receive_new_path:
            return 0
        # Extract positions from Pose objects
        x1, y1 = pose_A.position.x, pose_A.position.y
        x2, y2 = pose_B.position.x, pose_B.position.y
        xC, yC = pose_C.position.x, pose_C.position.y

        # Check if A and B are the same point
        if x1 == x2 and y1 == y2:
            # Return Euclidean distance from C to A
            return math.sqrt((xC - x1) ** 2 + (yC - y1) ** 2)

        # Compute the perpendicular distance from C to the line AB
        numerator = abs((y2 - y1) * xC - (x2 - x1) * yC + x2 * y1 - y2 * x1)
        denominator = math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)

        if denominator == 0:
            return float('inf')  # Should not happen after the A == B check

        distance = numerator / denominator
        return distance

    def loop(self):
        rate = rospy.Rate(self.loop_rate)
        status_msg = StringStamped()

        while not rospy.is_shutdown():
            if not self.in_manual and self.mission_status != "WAITING":
            # rospy.loginfo(f"label_pose: {self.label_pose} || robot_pose: {self.robot_pose}")
                # rospy.loginfo(f"current_state: {self._state}")
            #if robot in manual mode disable this loop
                if self._asm.reset_action_req:
                    self._asm.reset_flag()
                    rospy.sleep(1)
                    self.label_pose = self.robot_pose
                    self.first_pose_in_path = self.robot_pose
                    self.last_pose_in_path = self.robot_pose
                    self.lost_qr_at_goal = False
                    self._state = MainState.NORMAL
                if self._state == MainState.INIT:
                    self._asm.module_status = ModuleStatus.WAITING
                    # rospy.loginfo("INTITILIZING LABEL TRACKING NODE...")
                    self._state = MainState.NORMAL

                elif self._state == MainState.NORMAL:
                    self._asm.module_status = ModuleStatus.RUNNING
                    self._asm.error_code = ("")
                    #If robot is near goal and able to read QR code then reset this error message
                    if False and not self.robot_is_on_label(self.label_pose, self.robot_pose):
                        rospy.logerr(f"Error: X: {self.current_qr_label.x} | Y:{self.current_qr_label.y}")
                        rospy.logerr(f"Distance: {self.distance}")
                        rospy.logerr("QR_TRACKER: LOST LABEL IN PATH FROM "
                        f"({self.first_pose_in_path.position.x}, {self.first_pose_in_path.position.y}) "
                        f"to ({self.last_pose_in_path.position.x}, {self.last_pose_in_path.position.y})")
                        self._state = MainState.ERROR
                    if self.lost_qr_at_goal:
                        rospy.logerr(f"QR_TRACKER: LOST LABEL AT START OR END OF PATH FROM "
                        f"({self.first_pose_in_path.position.x}, {self.first_pose_in_path.position.y}) "
                        f"to ({self.last_pose_in_path.position.x}, {self.last_pose_in_path.position.y})")
                        self._state = MainState.ERROR

                elif self._state == MainState.ERROR:
                    self._asm.module_status = ModuleStatus.ERROR
                    if self.lost_qr_at_goal:
                        self._asm.error_code = (f"QR_TRACKER: LOST LABEL AT START OR END OF PATH FROM "
                        f"({self.first_pose_in_path.position.x}, {self.first_pose_in_path.position.y}) "
                        f"to ({self.last_pose_in_path.position.x}, {self.last_pose_in_path.position.y}) --> Need to read QR CODE in this path to RESET ERROR")
                    else:
                        self._asm.error_code = ("QR_TRACKER: LOST LABEL IN PATH FROM "
                        f"({self.first_pose_in_path.position.x}, {self.first_pose_in_path.position.y}) "
                        f"to ({self.last_pose_in_path.position.x}, {self.last_pose_in_path.position.y}) --> Need to read QR CODE in this path to RESET ERROR")

                    # self.moving_control_run_pause_pub.publish(StringStamped(stamp=rospy.Time.now(), data="PAUSE"))
                    # if self._asm.reset_error_request:
                    #     self._asm.reset_flag()
                    #     self._state = MainState.NORMAL
                    if self.reset_req:
                        self.reset_req = False
                        self._asm.reset_flag()
                        self._state = MainState.NORMAL
                        #If robot is near goal and able to read QR code then reset this error message
                        if (self.lost_qr_at_goal
                            and self.point_to_line_distance_pose(self.first_pose_in_path, self.last_pose_in_path, self.label_pose) < 0.5) and (rospy.get_time() - self.last_time_read_qr < 0.2):
                            self.lost_qr_at_goal = False
                # UPDATE STATE
                if self._prev_state != self._state:
                    rospy.logerr(
                    "Main state: {} -> {}".format(
                            self._prev_state.toString(), self._state.toString()
                        )
                    )
                    self._prev_state = self._state
                    # self.pub_tracker_status(self._state)
                    self._asm.module_state = self._state.toString()

            else:
                self._state = MainState.NORMAL
                self._asm.module_state = self._state.toString()
                self._asm.module_status = ModuleStatus.RUNNING
                self._asm.error_code = ("")
                self._asm.reset_flag()
                #If robot is near goal then reset this error message
                if (self.lost_qr_at_goal
                    and self.point_to_line_distance_pose(self.first_pose_in_path, self.last_pose_in_path, self.label_pose) < 0.5) and (rospy.get_time() - self.last_time_read_qr < 0.2):
                    rospy.sleep(1)
                    self.lost_qr_at_goal = False

            #Publish tracker_status
            status_msg.stamp = rospy.Time.now()
            status_msg.data = json.dumps({
                    "status": self._asm.module_status.toString(),
                    "state": self._asm.module_state,
                    "error_code": self._asm.error_code,
            })
            self._asm.module_status_pub.publish(status_msg)
            rate.sleep()
def parse_opts():
    from optparse import OptionParser
    parser = OptionParser()

    parser.add_option(
        "-d",
        "--ros_debug",
        action="store_true",
        dest="log_debug",
        default=False, help="log_level=rospy.DEBUG",)

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)

def main():
    (options, args) = parse_opts()
    log_level = None
    if options.log_debug:
        log_level = rospy.DEBUG
    rospy.init_node('qrTrackingNode', log_level=log_level)
    rospy.loginfo('Init node ' + rospy.get_name())
    qrTrackingNode(rospy.get_name(), **vars(options))

if __name__ == '__main__':
    main()
