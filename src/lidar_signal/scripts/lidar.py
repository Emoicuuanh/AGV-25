#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from math import cos, sin
import os
import os
import sys
import rospy
import serial
import DE32xx_20231108_ros as src
import rospkg
import copy
import json
import yaml
import math
from std_stamped_msgs.msg import (

    StringStamped,
    StringAction,
    StringFeedback,
    StringResult,
    StringGoal,
    EmptyStamped,
)
from sensor_msgs.msg import LaserScan
import time
common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)

from common_function import (
    MIN_FLOAT,
    EnumString,
    lockup_pose,
    offset_pose_x,
    offset_pose_yaw,
    distance_two_pose,
    delta_angle,
    dict_to_obj,
    merge_two_dicts,
    print_debug,
    print_info,
    print_warn,
    print_error,
    obj_to_dict,
    offset_pose_xy_theta,
    angle_two_pose,
    pose_dict_template,
)


class LidarSignalClass(object):


    def __init__(self, name, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        self.use_marker = rospy.get_param('/use_marker')

        self.pub = rospy.Publisher(self.topic,LaserScan, queue_size=10)
        self.marker_pub = rospy.Publisher(self.visualization_marker_topic, Marker, queue_size=10)
        if self.use_marker is True:
            rospy.Subscriber(self.topic, LaserScan, self.callback_laserscan1)
        self.publish_laserscan()
        self.init_serial()
        self.loop()


    def init_variable(self, *args, **kwargs):
        self.config_file = kwargs["config_file"]
        self.load_param(self.config_file)
        rospy.loginfo("config_file: %s" % self.config_file)
        # Read YAML file
        
        self.req_frame = bytes.fromhex('525363616ED4EA')
        self.resp_header = bytes.fromhex('525363616E')
        
        self.rate = rospy.Rate(self.Hz)
        

    def load_param(self,path):
        # with open(path) as file:
            # param = yaml.load(file, Loader=yaml.Loader)
        self.node = rospy.get_param('~node')
        self.topic = rospy.get_param('~topic')
        self.visualization_marker_topic= rospy.get_param('~visualization_marker_topic')
        self.baudrate = rospy.get_param('~baudrate')
        self.port = rospy.get_param('~port')
        # _msg = param['laserscan_msg1']
        self.frame_id =  rospy.get_param('~laserscan_msg/header/frame_id')
        self.angle_min = rospy.get_param('~laserscan_msg/angle_min')
        self.angle_min_filter = rospy.get_param('~laserscan_msg/angle_min_filter')
        self.angle_max = rospy.get_param('~laserscan_msg/angle_max')
        self.angle_max_filter = rospy.get_param('~laserscan_msg/angle_max_filter')    
        self.angle_increment = rospy.get_param('~laserscan_msg/angle_increment')
        self.time_increment = rospy.get_param('~laserscan_msg/time_increment')
        self.scan_time = rospy.get_param('~laserscan_msg/scan_time')
        self.range_min = rospy.get_param('~laserscan_msg/range_min')
        self.range_max = rospy.get_param('~laserscan_msg/range_max')
        self.Hz = rospy.get_param('~laserscan_msg/Hz')

    def publish_laserscan(self):
    # Tạo message LaserScan
        self.laserscan_msg = LaserScan()

    # Cấu hình thông tin LaserScan
        self.laserscan_msg.header.frame_id = self.frame_id
        self.laserscan_msg.angle_min = math.radians(self.angle_min)
        self.laserscan_msg.angle_max = math.radians(self.angle_max)
        self.laserscan_msg.angle_increment = math.radians(self.angle_increment)
        self.laserscan_msg.time_increment =  self.time_increment
        self.laserscan_msg.scan_time = self.scan_time
        self.laserscan_msg.range_min = self.range_min
        self.laserscan_msg.range_max = self.range_max

        # Các giá trị dữ liệu LaserScan
        self.laserscan_msg.ranges = []  # Dữ liệu khoảng cách
        self.laserscan_msg.intensities = []  # Dữ liệu cường độ

        



    def init_serial(self):
        try:
            self.serial_port = serial.Serial(
                port=self.port ,
                baudrate=self.baudrate,
                parity="N",
                stopbits=1,
                bytesize=8,
                timeout=1
            )
            if self.serial_port.isOpen():
                rospy.loginfo(
                    "Connected to port: {}".format(self.serial_port.portstr)
                )
        except:
            rospy.logerr("Serial1 connect fail")
        
    def shutdown(self):
        self.serial_port1.close()
        rospy.loginfo("Shuting down")

    """
     ######     ###    ##       ##       ########     ###     ######  ##    ##
    ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
    ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
    ##       ##     ## ##       ##       ########  ##     ## ##       #####
    ##       ######### ##       ##       ##     ## ######### ##       ##  ##
    ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
     ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
    """

    def def_cb(self, msg):
        rospy.loginfo(msg)
    def callback_laserscan(self,msg):
        # Tạo marker để hiển thị dữ liệu LaserScan theo kiểu các tia trên Rviz
        self.marker = Marker()
        self.marker.header = msg.header
        self.marker.ns = "laser_scan1"
        self.marker.id = 0
        self.marker.type = Marker.LINE_LIST
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.05
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0

        # Chuyển đổi dữ liệu LaserScan thành các đỉnh của các tia trong marker
        for i, range_value in enumerate(msg.ranges):
            # Bỏ qua các giá trị vô hạn hoặc không hợp lệ
            if range_value >= msg.range_min and range_value <= msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                # Tọa độ điểm bắt đầu của tia
                start_point = Point()
                start_point.x = 0.0
                start_point.y = 0.0
                start_point.z = 0.0
                self.marker.points.append(start_point)
                # Tọa độ điểm kết thúc của tia
                end_point = Point()
                end_point.x = range_value * cos(angle)
                end_point.y = range_value * sin(angle)
                end_point.z = 0.0
                self.marker.points.append(end_point)

        # Publish marker lên topic "visualization_marker"
        self.marker_pub.publish(self.marker)

  


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
        t_req = rospy.Time.now()
        serial_buff = b''
        while not rospy.is_shutdown():
            # Update the buffer
            if (rospy.Time.now() - t_req).to_sec() >= 0.1:
                if src.req_distance_data(self):
                    # time.sleep(0.005)
                    data = src.get_distance_data(self)  # Dữ liệu khoảng cách
                    if len(data) != 1085 or self.resp_header not in data :
                        rospy.logwarn_throttle(1, f"Incorrect frame: {data[0:10]} - len: {len(data)}")
                    if len(data) == 1085:
                        serial_buff += data
                    
                    #Erase buffer if data len reach maximum size
                    if len(serial_buff) > (1085 * 20):
                        serial_buff = serial_buff[0 : 1085 * 19]
                # rospy.loginfo(f"Update data {serial_buff[0:5]} - len:{len(serial_buff)}")
            
            #Check for header in the buffer
            if self.resp_header in serial_buff:
                # rospy.loginfo("Found header")
                start_index = serial_buff.index(self.resp_header)
                if len(serial_buff) > start_index + 5 + 1080:
                    distance_bytes = serial_buff[start_index + 5: start_index + 5 + 1080]
                    self.laserscan_msg.header.stamp = rospy.Time.now()
                    distance = [int.from_bytes(distance_bytes[i:i+2], "big") / 1000 for i in range(0, len(distance_bytes), 2)]
                    # filter data with angle min and max
                    if self.angle_min_filter != self.angle_min:
                        numble_filter_min = int((self.angle_min_filter - self.angle_min) / (self.angle_max - self.angle_min) * len(distance))
                        distance[:numble_filter_min + 1] = [0] * (numble_filter_min + 1)

                    if self.angle_max_filter != self.angle_max:
                        numble_filter_max = int((self.angle_max - self.angle_max_filter) / (self.angle_max - self.angle_min) * len(distance))
                        distance[-numble_filter_max:] = [0] * numble_filter_max

                        # self.laserscan_msg1.intensities = src.get_intensities_data(self)  # Dữ liệu cường độ
                        # Gửi message LaserScan
                        
                    self.laserscan_msg.ranges = distance
                    self.pub.publish(self.laserscan_msg)
                        
                    #Done processing -> erase data from buffer
                    serial_buff = serial_buff[start_index + 5 + 1080:]
                else:
                    rospy.logwarn("Waiting for data frame to be complete")
            else:
                rospy.logwarn("No header found")
                        
            self.rate.sleep()
                    
            # else:
            #     # rospy.logerr("HISON ERROR")


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
        "--config_file",
        dest="config_file",
        default=os.path.join(
            rospkg.RosPack().get_path("lidar_signal"),
            "cfg",
            "config.yaml",
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
    rospy.init_node("lidar_signal", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())


    LidarSignalClass(rospy.get_name(), **vars(options))



if __name__ == "__main__":
    main()
