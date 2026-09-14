#!/usr/bin/env python
import rospy
import rosbag
from sensor_msgs.msg import LaserScan

class SaveScan:
    def __init__(self):
        self.sub = rospy.Subscriber("/scan_map3", LaserScan, self.callback, queue_size=1)
        self.bag = rosbag.Bag("/home/thinh/SLAM_Mirror/src/mirror_detect/pcd/scan_map3.bag", "w")
        rospy.loginfo("Waiting for point cloud data on /scan_map3...")

    def callback(self, msg):
        # Save message to bag file
        self.bag.write("scan_map3", msg)
        rospy.loginfo("Saved point cloud to scan_map3.bag")
        
        # Close bag and shutdown
        self.bag.close()
        rospy.signal_shutdown("Saved point cloud")

if __name__ == "__main__":
    rospy.init_node("save_scan_node", anonymous=True)
    save_scan = SaveScan()
    rospy.spin()