#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_stamped_msgs.msg import StringStamped

def fake_led_status_publisher():
    rospy.init_node('fake_led_status_node', anonymous=True)
    pub = rospy.Publisher('/led_status', StringStamped, queue_size=10)
    rate = rospy.Rate(0.1) # 0.5 Hz = mỗi 2 giây đổi trạng thái

    # Danh sách trạng thái LED lấy từ file YAML bạn đưa
    statuses = [
        'MANUAL',
        'EMG'
        'MAPPING',
        'WAIT_RESPOND',
        'PAUSED',
        'WAITING',
        'BATTERY_LOW',
        'MANUAL_CHARGING',
        'AUTO_CHARGING',
        'CHARGING_READY',
        'IO_BOARD_ERROR',
        'GENERAL_ERROR',
        'CHARGING_ERROR',
        'SAFETY_STOP',
        'MOVING_ERROR',
        'SAFETY_DISABLED',
        'GOING_TO_POS',
        'MOTOR_RELEASED',
        'RUNNING_NORMAL',
        'WAITING_INIT_POSE'
    ]

    i = 0
    while not rospy.is_shutdown():
        msg = StringStamped()
        msg.stamp = rospy.Time.now()
        msg.data = statuses[i % len(statuses)]  # Lặp qua từng trạng thái
        rospy.loginfo("Gửi trạng thái LED giả: %s", msg.data)
        pub.publish(msg)
        i += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        fake_led_status_publisher()
    except rospy.ROSInterruptException:
        pass
