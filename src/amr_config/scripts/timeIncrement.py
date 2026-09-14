#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

class ScanMapModifier:
    def __init__(self):
        # Khởi tạo node
        rospy.init_node('scan_map_modifier', anonymous=True)

        # Subscriber cho topic /scan_map
        self.sub = rospy.Subscriber('/scan_map', LaserScan, self.scan_callback)

        # Publisher cho topic /scan_map_sim
        self.pub = rospy.Publisher('/scan_map_sim', LaserScan, queue_size=1)

    def scan_callback(self, msg):
        # Tạo bản sao của message
        modified_scan = LaserScan()
        modified_scan = msg

        # Tính toán time_increment
        if len(msg.ranges) > 0 and msg.scan_time > 0:
            modified_scan.time_increment = msg.scan_time / len(msg.ranges)
        else:
            modified_scan.time_increment = 0.0

        # Publish message đã chỉnh sửa
        self.pub.publish(modified_scan)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        modifier = ScanMapModifier()
        modifier.run()
    except rospy.ROSInterruptException:
        pass