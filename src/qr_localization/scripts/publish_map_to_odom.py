#! /usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf

def update_map():
    tf_listener = tf.TransformListener()
    br_map_to_odom = tf.TransformBroadcaster()
    rate = rospy.Rate(30)
    pose_x = 0.0
    pose_y = 0.0
    ori_z = 0.0
    ori_w = 1.0
    
    while not rospy.is_shutdown():
        try:
            tf_listener.waitForTransform(
                "/map_fix", "/odom", rospy.Time(0), rospy.Duration(0.05)
            )
            (trans, rot) = tf_listener.lookupTransform("/map_fix", "/odom", rospy.Time(0))
            pose_x = trans[0]
            pose_y = trans[1]
            ori_z = rot[2]
            ori_w = rot[3]
        except:
            pass
        
        br_map_to_odom.sendTransform((pose_x, pose_y, 0),
            [0,0,ori_z,ori_w],
            rospy.Time.now(),
            "odom",
            "map")

        rate.sleep()
        
if __name__ == "__main__":
    rospy.init_node("publish_map_to_odom")
    update_map()
