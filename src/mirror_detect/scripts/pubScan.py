#!/usr/bin/env python
import rospy
import rosbag
from sensor_msgs.msg import LaserScan

class LoadScan:
    def __init__(self):
        self.pub = rospy.Publisher("/scan_map3", LaserScan, queue_size=1)
        self.rate = rospy.Rate(10)  # Publish at 10 Hz
        rospy.loginfo("Loading and publishing laser scan to /scan_map3...")

        # Load bag file
        self.bag_file = "/home/thinh/SLAM_Mirror/src/mirror_detect/pcd/scan_map3.bag"
        try:
            bag = rosbag.Bag(self.bag_file, "r")
            self.msg = None
            # Read the first message from the bag
            for topic, msg, t in bag.read_messages(topics=["scan_map3"]):
                self.msg = msg
                break
            bag.close()

            if self.msg is None:
                rospy.logerr(f"No messages found in {self.bag_file}")
                rospy.signal_shutdown("Empty bag file")
                return

            rospy.loginfo(f"Loaded laser scan from {self.bag_file}")
            self.publish_loop()

        except Exception as e:
            rospy.logerr(f"Failed to load {self.bag_file}: {str(e)}")
            rospy.signal_shutdown("Bag file error")

    def publish_loop(self):
        while not rospy.is_shutdown():
            self.msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.msg)
            rospy.loginfo_once("Publishing laser scan to /scan_map3 continuously")
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("load_scan_node", anonymous=True)
    try:
        load_scan = LoadScan()
    except rospy.ROSInterruptException:
        pass