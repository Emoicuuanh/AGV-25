#!/usr/bin/env python
import rospy
from safety_msgs.msg import SafetyStatus
from std_msgs.msg import Time, Int16, Int32
from std_stamped_msgs.msg import StringStamped
from vl53l5cx.msg import Vl53l5cxRanges, Safety_Esp, vl53l5cx_safety
import message_filters

def listener():
    left_sub = message_filters.Subscriber('/vl35l5cx_ranges_left', Vl53l5cxRanges)
    right_sub= message_filters.Subscriber('/vl35l5cx_ranges_right', Vl53l5cxRanges) 
    pub_result = rospy.Publisher('/vl53l5_status_node', vl53l5cx_safety, queue_size=10)
    ts = message_filters.TimeSynchronizer([left_sub, right_sub], 10)
    ts.registerCallback(callback)
    rospy.spin()

def callback(vl35l5cx_ranges_left, vl35l5cx_ranges_right):
    # pub = rospy.Publisher('objectsTopic', ObjectPose, queue_size=10)                       
    # msg = ObjectPose()                              
    # msg.objectName = objectsInfo.DetectionInfo.filePaths
    # msg.poseX = objectsPose.PoseStamped.pose.position.x     
    # msg.poseY = objectsPose.PoseStamped.pose.position.y               

    # pub.publish(msg)                    
    print(vl35l5cx_ranges_left.range)  
    print(vl35l5cx_ranges_right.range)        
    rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node("vl53l5cx_node", anonymous=True)             
    try:
        listener()
    except rospy.ROSInterruptException:
        pass