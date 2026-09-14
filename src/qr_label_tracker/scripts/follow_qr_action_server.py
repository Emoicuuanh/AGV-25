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
from std_stamped_msgs.msg import StringAction, StringStamped, StringResult, StringFeedback, StringGoal, Int8Stamped, EmptyStamped
from agv_msgs.msg import DataMatrixStamped, CartesianPosition
from geometry_msgs.msg import PoseStamped, Pose

from actionlib_msgs.msg import GoalStatus, GoalID, GoalStatusArray
from safety_msgs.msg import SafetyStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionResult, MoveBaseActionFeedback, MoveBaseGoal, MoveBaseActionGoal

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
    obj_to_dict
)

class MainState(EnumString):
    NONE          = -1
    SEND_GOAL     = 0
    RE_SEND_GOAL  = 1
    MOVING        = 2
    GOAL_REACHED  = 3
    PAUSED        = 4
    ERROR         = 5
    DONE          = 6        
        

class FollowQrAction(object):
    _feedback = StringFeedback()
    _result = StringResult()

    def __init__(self, name, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        
        # Follow_QR Action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, StringAction, execute_cb=self.execute_follow_qr_cb, auto_start = False)
        self._as.start()
        
        # Move_base Action client
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        
        # Publisher
        move_base_cancel_publisher = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        
        
        # Subscriber
        rospy.Subscriber("/data_gls621", DataMatrixStamped, self.qr_data_callback)
        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.goal_callback)
        rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.move_base_feedback_cb)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.move_base_result_cb)
        rospy.Subscriber('/robot_pose', Pose, self.robot_pose_callback)
        # ModuleServer, to control this follow_qr_action node pause, reset and get node status
        self._asm = ModuleServer(name)
        self.last_module_status = self._asm.module_status
        
        # Event
        rospy.on_shutdown(self.shutdown)
        # Loop
        self.loop()

    def init_variable(self, *args, **kwargs):
        # self.config_path = kwargs["config_path"]
        self.previous_qr_label = CartesianPosition() #store previous seen QR code label
        self.current_qr_label = CartesianPosition() #store current QR code label

        self.goal_result = GoalStatus()
        self.seen_qr = False
        self.move_base_received_goal = False
        self.move_base_reached_goal = False
        self.stop_at_qr_goal = False
        
        # self.move_action_status = GoalStatus()
        # self.move_base_result = GoalStatus()
        # self.prev_move_action_status = GoalStatus()
        
        self.qr_pose_x = 0
        self.qr_pose_y = 0
        self.robot_pose_x = 0
        self.robot_pose_y = 0
        # self.previous_qr_label = None
        
    def shutdown(self):
        rospy.loginfo("[{}] Shutting down...".format(self._action_name))

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
    
    # Run when client accepts goal
    # def goal_response_callback(self):
    #     rospy.loginfo('Goal accepted :)')

    # Run when client sends feedback
    def move_base_feedback_cb(self, feedback_msg):
        if feedback_msg.status == GoalStatus.ACTIVE:
            self.move_base_received_goal = True
        if feedback_msg.status == GoalStatus.SUCCEEDED:
            self.move_base_reached_goal = True

    """    
    # Run when client sends final result
    def get_result_callback(self, state, result):
        # Show log and exit node
        rospy.loginfo('Result: {0}'.format(result.is_finished))
        rospy.signal_shutdown("Shutting-down client node")
    """
        
    def qr_data_callback(self, msg):
        # rospy.loginfo(f"QR now is: {msg.lable}")
        seen_qr = True # Set seen_qr flag to True it will be reset if in state MOVING
        self.qr_pose_x, self.qr_pose_y = self.convert_qr_msg_to_pose(msg)
        # rospy.loginfo(f"CURRENT QR pose:{self.qr_pose_x} : {self.qr_pose_y}")
        if (self.current_qr_label.x != msg.lable.x or self.current_qr_label.y != msg.lable.y):
            
            # update current qr data
            self.current_qr_label = msg.lable
            rospy.loginfo(f"CURRENT QR data:{self.current_qr_label}")
    
    def robot_pose_callback(self, msg):
        self.robot_pose_x = msg.position.x
        self.robot_pose_y = msg.position.y
        
        self.stop_at_qr_goal = self.check_curr_pose_at_qr(self.qr_pose_x, self.qr_pose_y, self.robot_pose_x, self.robot_pose_y)
            
    
    def goal_callback(self, msg):
        rospy.loginfo(f"Goal:{msg}") 
    """
    ######## ##     ## ########  ######  ##     ## ######## ########
    ##        ##   ##  ##       ##    ## ##     ##    ##    ##
    ##         ## ##   ##       ##       ##     ##    ##    ##
    ######      ###    ######   ##       ##     ##    ##    ######
    ##         ## ##   ##       ##       ##     ##    ##    ##
    ##        ##   ##  ##       ##    ## ##     ##    ##    ##
    ######## ##     ## ########  ######   #######     ##    ########
    """

    def execute_follow_qr_cb(self, goal):
        # rospy.loginfo(f"received goal: {goal.data.decode('utf-8')}")
        goal_dict = json.loads(goal.data)
        print_debug("Receive goal:\n{}".format(json.dumps(goal_dict, indent=2)))

        # Loop control variables
        rate = 10
        loop_rate = rospy.Rate(rate)
        t = rospy.get_time()    
        diff_time = 1/rate
        
        last_loop_time = rospy.get_time()
        
        success = False
        _state = MainState.SEND_GOAL
        _prev_state = MainState.NONE
        _prev_state_temp = MainState.NONE
        feedback_msg = ""
        _state_when_pause = MainState.NONE
        _state_when_error = MainState.ERROR
        _state_bf_error = MainState.NONE
        self._asm.reset_flag()
        self._asm.action_running = True
        

        while not rospy.is_shutdown():
            self._asm.action_running = True
            
            # Check for loop execute time
            if rospy.get_time() - t < diff_time:
                rospy.sleep(0.0001)
                continue
            else:
                t = rospy.get_time()
            
            # DEBUG: CHECK FOR LOOP DURATION
            now = rospy.get_time()
            duration = now - last_loop_time
            rospy.logdebug(f" LOOPING {duration}")
            if duration > 0.25:
                rospy.logwarn("Loop time: {}".format(round(duration, 2)))
            last_loop_time = rospy.get_time()
            
            if self._as.is_preempt_requested() or self._asm.reset_action_req:
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                self.send_feedback(self._as, 'PREEMPTED')
                break

            if self._asm.module_status != ModuleStatus.ERROR:
                self._asm.error_code = ""
                
            if _state != MainState.PAUSED and self._asm.error_code == "":
                self._asm.module_status = ModuleStatus.RUNNING

            # Update State
            if _prev_state != _state:
                rospy.loginfo('Action state: {} -> {}'.format(_prev_state.toString(), _state.toString()))
                _prev_state = _state
                self._asm.module_state = _state.toString()
            self.send_feedback(self._as, _state.toString())

            # State: SEND_GOAL
            if _state == MainState.SEND_GOAL:
                
                move_base_goal = MoveBaseGoal()
                
                #get parameter from follow_qr/goal
                move_base_goal.target_pose.header.frame_id = goal_dict["target_pose"]["frame_id"]
                # move_base_goal.target_pose.header.stamp = rospy.Time.now()
                move_base_goal.target_pose.pose.position.x = goal_dict["target_pose"]["position"]["x"]
                move_base_goal.target_pose.pose.position.y = goal_dict["target_pose"]["position"]["y"]
                move_base_goal.target_pose.pose.orientation.z = goal_dict["target_pose"]["orientation"]["z"]
                move_base_goal.target_pose.pose.orientation.w = goal_dict["target_pose"]["orientation"]["w"]
                
                self.move_base_client.send_goal(move_base_goal)
                
                #check for feedback from move_base_server
                if self.move_base_received_goal:
                # if True:
                    _state = MainState.MOVING

            # State: MOVING
            if _state == MainState.MOVING:
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    _state_when_pause = _state
                    _state = MainState.PAUSED
                    continue
                
                #TODO: CHECK FOR QR POSITION WHEN ROBOT IS MOVING
                
                if self.move_base_reached_goal:
                # if True:
                    _state = MainState.GOAL_REACHED 
                
            # State: GOAL_REACHED
            elif _state == MainState.GOAL_REACHED:
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    _state_when_pause = _state
                    _state = MainState.PAUSED
                    continue
                
                """
                if False: # ERROR
                    _state_bf_error = MainState.STATE_1
                    _state_when_error = _state
                    _state = MainState.ERROR
                    continue
                """
                #TODO: CHECK FOR QR POSITION AT GOAL:
                if (self.seen_qr 
                    and self.stop_at_qr_goal):
                    rospy.loginfo("QR DETECTED WHEN REACHED GOAL !!!!")
                    _state = MainState.DONE
                    continue
                else: 
                    rospy.logerr("NO QR DETECTED WHEN  REACHED GOAL !!!!")
                    _state = MainState.ERROR
                    continue
                
            # State: PAUSED
            elif _state == MainState.PAUSED:
                self._asm.module_status = ModuleStatus.PAUSED
                if self._asm.resume_req:
                    self._asm.reset_flag()
                    _state = _state_when_pause
                
            # State: ERROR
            elif _state == MainState.ERROR:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = rospy.get_name() + ": {}".format(_state.toString())
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = _state_when_error
                    _state = MainState.DONE
                
                # User will need to push robot to qr and press reset button to continue
                if True:
                    _state = MainState.DONE
                
            # State: DONE
            elif _state == MainState.DONE:
                success = True
                print("STATE == DONE")
                break
            loop_rate.sleep()

        self._asm.action_running = False

        
        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
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

    def move_base_feedback(self, feedback):
        rospy.loginfo("Received feedback: %s", feedback)
        self.move_base_feedback_ = feedback.status
    
    def convert_qr_msg_to_pose(self, qr_msg):
        qr_pose_x = (qr_msg.lable.x + qr_msg.possition.x)/1000
        qr_pose_y = (qr_msg.lable.y + qr_msg.possition.y)/1000
        return qr_pose_x, qr_pose_y
    
    def check_curr_pose_at_qr(self, qr_pose_x, qr_pose_y, current_pose_x, current_pose_y):
        robot_to_qr_max_distance = 0.5 #TODO: add this to config of follow_qr
        if (abs(current_pose_x - qr_pose_x) > robot_to_qr_max_distance 
            or abs(current_pose_y - qr_pose_y) > robot_to_qr_max_distance):
            # rospy.logerr("robot lost track of qr code")
            return False
        else:
            # rospy.loginfo("robot is on qr code")
            return True
        
        

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
    # parser.add_option("-c", "--config_path", dest="config_path",
    #                 default=os.path.join(rospkg.RosPack().get_path('action_template'), 'cfg'))

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)

def main():
    (options, args) = parse_opts()
    log_level = None
    if options.log_debug:
        log_level = rospy.DEBUG
    rospy.init_node('follow_qr', log_level=log_level)
    rospy.loginfo('Init node ' + rospy.get_name())
    FollowQrAction(rospy.get_name(), **vars(options))

if __name__ == '__main__':
    main()