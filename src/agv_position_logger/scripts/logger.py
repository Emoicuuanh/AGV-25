#!/usr/bin/env python

import rospy
from agv_msgs.msg import DataMatrixStamped, ErrorRobotToPath
from std_stamped_msgs.msg import StringStamped
from geometry_msgs.msg import Pose, Twist
import csv
from datetime import datetime
import time
import string

import json

class Logger(object):
    def __init__(self):
        rospy.init_node('agv_postion_logger', anonymous=True)
        rospy.Subscriber("/robot_status", StringStamped, self.robot_status_cb)
        rospy.Subscriber("/data_gls621", DataMatrixStamped, self.gls621_data_cb)
        rospy.Subscriber("/robot_pose", Pose, self.robot_pose_cb)
        rospy.Subscriber('/error_robot_to_path', ErrorRobotToPath, self.error_robot_to_path_cb)
        rospy.Subscriber('/mission_manager/module_status', StringStamped, self.mission_manager_cb)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_cb)

        self.current_robot_pose = Pose()
        self.current_data_gls621 = DataMatrixStamped()
        self.current_robot_status = None
        self.current_action = None
        self.error_position = 0.0
        self.error_angle = 0.0
        self.vel_x = 0.0
        self.rot_z = 0.0


        self.new_gls621 = False
        self.new_robot_status = False
        self.new_robot_pose = False
        self.new_action = False
        self.new_error_to_path = False

        self.rate = rospy.Rate(10)

        self.csv_filename = f"/home/mkac/tmp/ros/agv_pose_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        # self.csv_filename = f"/home/mkac/mav_ws/src/agv_position_logger/log/agv_pose_log.csv"

    def cmd_vel_cb(self, msg):
        self.vel_x = msg.linear.x
        self.rot_z = msg.angular.z

    def mission_manager_cb(self, msg):
        self.new_action = True
        action = json.loads(msg.data)
        if action != None:
            self.current_action = action["current_action_type"]

    def robot_status_cb(self, msg):
        self.new_robot_status = True
        robot_status = json.loads(msg.data)
        if robot_status != None:
            self.current_robot_status = robot_status["mode"]

    def error_robot_to_path_cb(self, msg):
        self.new_error_to_path = True
        self.error_position = msg.error_position
        self.error_angle = msg.error_angle


    def robot_pose_cb(self, msg):
        self.new_robot_pose = True
        self.current_robot_pose = msg

    def gls621_data_cb(self, msg):
        self.new_gls621 = True
        self.current_data_gls621 = msg

    def format_ros_time(self, ros_time):
        """Convert ROS time to formatted string."""
        dt = datetime.fromtimestamp(ros_time.to_sec())
        return dt.strftime("%d/%m/%Y %H:%M:%S")

    def write_log_to_file(self):
        with open(self.csv_filename, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['ROS Time', 'Robot_status', "Current_action_type", 'Code_X', 'Code_Y', 'Code_off_x', 'Code_off_y', 'Code_angle', 'Robot_X', 'Robot_Y', 'Error_pos_to_path', 'Error_ang_to_path', "vel_x", "vel_rotation"])  # Write header

            rate = rospy.Rate(10)  # 10 Hz
            while not rospy.is_shutdown():
                if (self.vel_x > 0.0 or self.rot_z > 0.0):
                    rospy.loginfo("log now")
                    # ROS Time
                    ros_time = rospy.Time.now()
                    formatted_time = self.format_ros_time(ros_time)
                    # seconds = time.time() - self.start_time

                    #Robot status


                    #QR Code
                    code_x = self.current_data_gls621.lable.x
                    code_y = self.current_data_gls621.lable.y
                    code_off_x = self.current_data_gls621.possition.x
                    code_off_y = self.current_data_gls621.possition.y
                    code_angle = self.current_data_gls621.possition.angle

                    #Robot pose
                    robot_x = self.current_robot_pose.position.x
                    robot_y = self.current_robot_pose.position.y

                    # rospy.logerr(f"Time: {formatted_time}")
                    # rospy.loginfo(f"Status: {self.current_robot_status}")
                    # rospy.loginfo(f"Action: {self.current_action}")
                    # rospy.loginfo(f"Code: {code_x} | {code_y} | Code_off: {code_off_x} | {code_off_y} | {code_angle}")
                    # rospy.loginfo(f"Robot_pose: {robot_x} | {robot_y}")
                    # rospy.loginfo(f"Error_to_path: {self.error_position} | {self.error_angle}")
                    csv_writer.writerow([formatted_time, self.current_robot_status, self.current_action, code_x, code_y, code_off_x, code_off_y, code_angle, robot_x, robot_y, self.error_position, self.error_angle, self.vel_x, self.rot_z])
                    csvfile.flush()  # Ensure data is written to file

                rate.sleep()

    def run_log(self):
        while not rospy.is_shutdown():
            if not (self.new_robot_status and
                self.new_gls621 and
                self.new_robot_pose and
                self.new_action ):
                rospy.logwarn_once("Waiting for all msg to be available !")
            else:
                rospy.loginfo_once("New msgs received !")
                self.write_log_to_file()

def main():
    agvLogger = Logger()
    agvLogger.run_log()

if __name__ == '__main__':
    main()
