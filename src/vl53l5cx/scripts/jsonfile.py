#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

rospy.init_node("line_pub_example")
pub_line_min_dist = rospy.Publisher("~line_min_dist", Marker, queue_size=1)
rospy.loginfo("Publishing example line")
count = 0


def make_marker(frame_id, id, pose, scale, color, frame_locked):
    msg = Marker()
    msg.header.frame_id = frame_id
    msg.header.stamp = rospy.Time.now()
    msg.ns = "pouring_visualization"
    msg.id = id
    msg.action = Marker.ADD
    msg.pose = pose
    msg.scale = scale
    msg.color = color
    msg.frame_locked = frame_locked
    return


def make_cylinder_marker(frame_id, id, pose, scale, color, frame_locked=False):
    msg = make_marker(frame_id, id, pose, scale, color, frame_locked)
    msg.type = Marker.CYLINDER
    return msg


while not rospy.is_shutdown():
    marker = MarkerArray()
    # marker.id = 1
    marker.header.frame_id = "map"
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD

    # marker scale
    marker.scale.x = 0.03
    marker.scale.y = 0.03
    marker.scale.z = 0.03

    # marker color
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    # marker orientaiton
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # marker position
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    # marker line points
    marker.points = []
    # first point
    first_line_point = Point()
    first_line_point.x = 0.0
    first_line_point.y = 0.0
    first_line_point.z = 0.0
    marker.points.append(first_line_point)
    # second point
    second_line_point = Point()
    second_line_point.x = 1.0
    second_line_point.y = 1.0
    second_line_point.z = 0.0
    marker.points.append(second_line_point)

    # Publish the Marker
    pub_line_min_dist.publish(marker)
    rospy.sleep(0.5)
    marker.id = 2
    marker.points = []
    # first point
    first_line_point = Point()
    first_line_point.x = 1.0
    first_line_point.y = 1.0
    first_line_point.z = 0.0
    marker.points.append(first_line_point)
    # second point
    second_line_point = Point()
    second_line_point.x = 2.0
    second_line_point.y = 2.0
    second_line_point.z = 0.0
    marker.points.append(second_line_point)
    pub_line_min_dist.publish(marker)
    rospy.sleep(0.5)
    # marker = Marker()
    # marker.header.frame_id = "map"
    # marker.type = marker.LINE_STRIP
    # marker.id = 2
    # marker.action = Marker.DELETEALL
    pub_line_min_dist.publish(marker)
    print(marker.points)
    # rospy.sleep(0.5)
    # count += 1
    # if count > 2:
    #     break
