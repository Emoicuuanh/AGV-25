#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point
import math
# import numpy as np

class agv_offset(object):
    # def __init__(self, waiting_pose, docking_pose, offset_x, offset_y):
    #     self.waiting_pose = waiting_pose
    #     self.docking_pose = docking_pose
    #     self.x_offset = offset_x
    #     self.y_offset = offset_y
    #     self.heading_angle = math.atan2 (self.docking_pose.position.y - self.waiting_pose.position.y,
    #                                     self.docking_pose.position.x - self.waiting_pose.position.x)
    #     self.map_heading_angle = math.atan2 (self.waiting_pose.position.y - self.docking_pose.position.y,
    #                                     self.waiting_pose.position.x - self.docking_pose.position.x)
    #     print(f"Heading angle: {self.heading_angle}")
    #     print(f"Map heading angle: {self.map_heading_angle}")

    def __init__(self, waiting_pose, docking_pose, offset_x, offset_y):
        self.waiting_pose = waiting_pose
        self.docking_pose = docking_pose
        self.x_offset = offset_x
        self.y_offset = offset_y
        self.heading_angle = math.atan2 (self.docking_pose[1] - self.waiting_pose[1],
                                        self.docking_pose[0] - self.waiting_pose[0])
        self.map_heading_angle = math.atan2 (self.waiting_pose[1] - self.docking_pose[1],
                                        self.waiting_pose[0] - self.docking_pose[0])
        
        rospy.logerr(f"Heading angle: {self.heading_angle}")
        rospy.logerr(f"apply offset to path: x: {self.x_offset}, y: {self.y_offset}")
        # print(f"Map heading angle: {self.map_heading_angle}")

    # def apply_rotation(self, point, angle):
    #     new_point = Point()
    #     new_point.x = point.x * math.cos(angle) - point.y * math.sin(angle)
    #     new_point.y = point.x * math.sin(angle) + point.y * math.cos(angle)
    #     return new_point

    def apply_rotation(self, point, angle):
        # new_point = []
        new_point_x = point[0] * math.cos(angle) - point[1] * math.sin(angle)
        new_point_y = point[0] * math.sin(angle) + point[1] * math.cos(angle)
        return [new_point_x, new_point_y]

    # def apply_translation(self, point, x_offset, y_offset):
    #     new_point = Point()
    #     new_point.x = point.x + x_offset
    #     new_point.y = point.y + y_offset
    #     return new_point

    def apply_translation(self, point, x_offset, y_offset):
        # new_point = []
        new_point_x = point[0] + x_offset
        new_point_y = point[1] + y_offset
        return [new_point_x, new_point_y]

    def calculate_offset(self, point):
        # Rotate to robot coordinate
        p1 = self.apply_rotation(point, self.heading_angle)
        # Offset point in robot coordinate
        p2 = self.apply_translation(p1, self.x_offset, self.y_offset)
        # Rotate back to map coordinates
        p3 = self.apply_rotation(p2, - self.heading_angle)
        return p3

"""TESING"""
# pose_1 = [1, 0]

# pose_2 = [2, 0]

# # Offset in robot coordinate system
# offset_x = 0
# offset_y = 1

# offset_agv = agv_offset(pose_1, pose_2, offset_x, offset_y)
# print(f"new_offset_pose_1:\n {offset_agv.calculate_offset(pose_1)}")
# print(f"new_offset_pose_2:\n {offset_agv.calculate_offset(pose_2)}")