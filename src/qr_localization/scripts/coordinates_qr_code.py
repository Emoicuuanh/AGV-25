#! /usr/bin/env python
# -*- coding: utf-8 -*-
import yaml
import os
import sys
import rospy
import rospkg
import numpy as np
import threading
from math import *
from agv_msgs.msg import *
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
import json
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
)
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from datetime import time
from std_stamped_msgs.msg import StringStamped

common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)

from agv_msgs.msg import DataMatrixStamped


class Coordinate_estimate(object):


    def __init__(self, name, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        rospy.Subscriber(
            "/initialpose", PoseWithCovarianceStamped, self.init_pose_cb
        )
        self.initPose_pub = rospy.Publisher(
            "/initialpose",
            PoseWithCovarianceStamped,
            queue_size=10,
        )
        rospy.Subscriber("/robot_status", StringStamped, self.robot_status_cb)
        rospy.Subscriber(
            "/data_gls621", DataMatrixStamped, self.frame_device_cb
        )

    def init_variable(self, *args, **kwargs):
        self.tf_listener = tf.TransformListener()
        self.trans = 0
        self.rot = 0
        self.br_cam_to_qr = tf.TransformBroadcaster()
        self.bt_qr_to_fix_map = tf.TransformBroadcaster()
        self.static_transformStamped = TransformStamped()
        self.listener = tf.TransformListener()
        self.static_br2 =  tf2_ros.StaticTransformBroadcaster()

        self.qr_frame = rospy.get_param("~qr_frame", "qr_code")
        self.qr_cam_frame = rospy.get_param("~qr_cam_frame", "qr_cam")
        self.map_fix_frame = rospy.get_param("~map_fix_frame", "map_fix")

        self.rate = rospy.Rate(30)
        self.map_to_odom_pose_x = 0.0
        self.map_to_odom_pose_y = 0.0
        self.map_to_odom_ori_z = 0.0
        self.map_to_odom_ori_w = 1.0

        self.first_init_pose = False

    def robot_status_cb(self, msg):
        self.robot_status = json.loads(msg.data)
        if self.robot_status["status"] == "WAITING_INIT_POSE":
            self.first_init_pose = True
        else:
            self.first_init_pose = False

    def frame_device_cb(self, msg):
        label_x = msg.lable.x/1000
        label_y = msg.lable.y/1000

        # Vi tri tu tam cua camera toi tam cua QR code
        deviation_origin_x = msg.possition.x / 1000
        deviation_origin_y = msg.possition.y / 1000
        deviation_theta = msg.possition.angle

        self.br_cam_to_qr.sendTransform((deviation_origin_x, deviation_origin_y, 0),
                    tf.transformations.quaternion_from_euler(0, 0, -deviation_theta),
                    rospy.Time.now(),
                    self.qr_frame,
                    self.qr_cam_frame)

        self.bt_qr_to_fix_map.sendTransform((-label_x, -label_y, 0),
                    tf.transformations.quaternion_from_euler(0, 0, 0),
                    rospy.Time.now(),
                    self.map_fix_frame,
                    self.qr_frame)

        if self.first_init_pose:
            init_pose = PoseWithCovarianceStamped()
            init_pose.header.frame_id = "map"
            init_pose.pose.pose.position.x = label_x
            init_pose.pose.pose.position.y = label_y
            init_pose.pose.pose.orientation.z = 0.0
            init_pose.pose.pose.orientation.w = 1.0
            # rospy.loginfo(f"INIT POSE AT X:{label_x} Y:{label_y}")
            self.initPose_pub.publish(init_pose)
            self.first_init_pose = False
    def init_pose_cb(self, msg):
        rospy.logwarn(
            "Received init pose request position: {} /t orientation: {}".format(
                msg.pose.pose.position, msg.pose.pose.orientation
            )
        )
        if not self.get_odom():
            return
        trans_robot_map = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            0,
        ]
        rot_robot_map = [
            0,
            0,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        TTT, RRR = self.getPOSE(
            np.asarray(self.trans),
            np.asarray(self.rot),
            np.asarray(trans_robot_map),
            np.asarray(rot_robot_map),
        )
        self.map_to_odom_pose_x = TTT.item(0)
        self.map_to_odom_pose_y = TTT.item(1)

        self.map_to_odom_ori_z = RRR[2]
        self.map_to_odom_ori_w = RRR[3]

    def getRTfromQuat(self, translation, quat):
        # pose_map = np.array([[-0.22571, 0.063645, 0.20089]])
        # orient_map = np.array([ -0.0027935, 0.70555, -0.0043037, 0.70864])

        # print quat
        [roll1, pitch1, yaw1] = euler_from_quaternion(
            np.squeeze(np.asarray(quat))
        )
        # print roll1, pitch1, yaw1
        Rp1 = np.matrix(
            [
                [cos((pitch1)), 0, sin((pitch1))],
                [0, 1, 0],
                [-sin(pitch1), 0, cos((pitch1))],
            ]
        )

        Ry1 = np.matrix(
            [
                [cos((yaw1)), -sin((yaw1)), 0],
                [sin((yaw1)), cos((yaw1)), 0],
                [0, 0, 1],
            ]
        )

        Rr1 = np.matrix(
            [
                [1, 0, 0],
                [0, cos((roll1)), -sin((roll1))],
                [0, sin((roll1)), cos((roll1))],
            ]
        )

        R1 = Ry1 * Rp1 * Rr1

        R = np.asarray(R1)
        R = np.vstack((R, np.array([0, 0, 0])))

        R = np.hstack(
            (
                R,
                np.array(
                    [
                        [translation.item(0)],
                        [translation.item(1)],
                        [translation.item(2)],
                        [1],
                    ]
                ),
            )
        )
        return R

    def getQuatfromRT(self, RT):
        qw = sqrt(1 + RT[0, 0] + RT[1, 1] + RT[2, 2]) / 2
        qx = (RT[2, 1] - RT[1, 2]) / (4 * qw)
        qy = (RT[0, 2] - RT[2, 0]) / (4 * qw)
        qz = (RT[1, 0] - RT[0, 1]) / (4 * qw)
        return np.array([qx, qy, qz, qw])

    def getPOSE(
        self, poseODOM_ar3, orientODOM_ar3, pose_robot_map, orient_robot_map
    ):
        T_MA = np.asmatrix(self.getRTfromQuat(pose_robot_map, orient_robot_map))
        T_AO = np.asmatrix(self.getRTfromQuat(poseODOM_ar3, orientODOM_ar3))
        T_MO = T_MA * T_AO
        Rr = T_MO[0:3, 0:3]
        position = T_MO[0:3, -1]  # supress the z coordinate
        orientation = self.getQuatfromRT(T_MO)
        return position, orientation

    def get_odom(self):
        try:
            self.tf_listener.waitForTransform(
                "/base_link", "/odom", rospy.Time(0), rospy.Duration(4.0)
            )
            # rospy.loginfo("transform found :)")
            self.trans, self.rot = self.tf_listener.lookupTransform(
                "/base_link", "/odom", rospy.Time(0)
            )
            ############self.rot is in quaternion############
            return True
        except (
            tf.Exception,
            tf.ConnectivityException,
            tf.LookupException,
            KeyboardInterrupt,
        ):
            rospy.logwarn("TF exception")
            return False

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
    # parser.add_option(
    #     "-c",
    #     "--config_file",
    #     dest="config_file",
    #     default=os.path.join(
    #         rospkg.RosPack().get_path("coordinate_rfid"),
    #         "cfg",
    #         "transform.yaml",
    #     ),
    # )
    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)


def main():
    (options, args) = parse_opts()
    log_level = None
    if options.log_debug:
        log_level = rospy.DEBUG
    rospy.init_node("coordinate_qr_code", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    Coordinate_estimate(rospy.get_name(), **vars(options))
    rospy.spin()

if __name__ == "__main__":
    main()
