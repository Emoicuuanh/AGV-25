#!/usr/bin/env python
# license removed for brevity
import rospy
import json
from std_msgs.msg import String
from std_stamped_msgs.msg import StringStamped, Float32Stamped, EmptyStamped
from arduino_serial.msg import SetPinStamped, BoolArrayStamped


class ArduinoRosFake(object):
    CODE_ONLY = False

    def __init__(self, name):
        # Load params
        self.emg_button = rospy.get_param("~EMG_PIN", 69)
        # self.emg_2_pin = rospy.get_param('~EMG_2_PIN', 23)
        self.start_1_button = rospy.get_param("~START_1_PIN", 63)
        self.start_2_button = rospy.get_param("~START_2_PIN", 62)
        self.stop_1_button = rospy.get_param("~STOP_1_PIN", 65)
        self.stop_2_button = rospy.get_param("~STOP_2_PIN", 64)
        self.auto_manual_sw = rospy.get_param("~AUTO_MAN_SW", 60)
        self.lift_min_sensor = rospy.get_param("~LIFT_MIN_SENSOR", 66)
        self.lift_max_sensor = rospy.get_param("~LIFT_MAX_SENSOR", 67)
        self.sensor_a14_arduino = rospy.get_param("~detect_vrack", 68)
        self.bumper = rospy.get_param("~bumper", 59)

        self.auto_charging_detect = rospy.get_param("~auto_charging_detect", 47)
        self.man_charging_detect = rospy.get_param("~man_charging_detect", 48)
        self.auto_charging_en_pin = rospy.get_param("~auto_charging_en_pin", 31)

        # Publisher
        self.standard_io_pub = rospy.Publisher(
            "/standard_io", StringStamped, queue_size=5
        )
        self.arduino_blink_pub = rospy.Publisher(
            "/arduino_driver/led_blink", EmptyStamped, queue_size=5
        )

        # Subscriber
        rospy.Subscriber(
            "/arduino_driver/pin_status_array",
            BoolArrayStamped,
            self.pin_status_cb,
        )

        self.sensors_msg_dict = {}
        self.last_msg = None
        self.loop()

    def pin_status_cb(self, msg):
        # if self.last_msg != None:
        #     for i in range(len(msg.data)):
        #         if self.last_msg[i] != msg.data[i]:
        #             print(i)
        # self.last_msg = msg.data

        if self.CODE_ONLY:
            msg = BoolArrayStamped()
        self.sensors_msg_dict["emg_button"] = not msg.data[self.emg_button]
        self.sensors_msg_dict["start_2_button"] = not msg.data[
            self.start_1_button
        ]
        self.sensors_msg_dict["start_1_button"] = not msg.data[
            self.start_2_button
        ]
        self.sensors_msg_dict["stop_2_button"] = not msg.data[
            self.stop_1_button
        ]
        self.sensors_msg_dict["stop_1_button"] = not msg.data[
            self.stop_2_button
        ]
        self.sensors_msg_dict["auto_manual_sw"] = msg.data[self.auto_manual_sw]
        self.sensors_msg_dict["lift_min_sensor"] = msg.data[
            self.lift_min_sensor
        ]
        self.sensors_msg_dict["lift_max_sensor"] = msg.data[
            self.lift_max_sensor
        ]

        self.sensors_msg_dict["detect_vrack"] = not msg.data[
            self.sensor_a14_arduino
        ]

        self.sensors_msg_dict["bumper"] = not msg.data[self.bumper]
        self.sensors_msg_dict["auto_charging_detect"] = not msg.data[
            self.auto_charging_detect
        ]
        self.sensors_msg_dict["manual_charging_detect"] = not msg.data[
            self.man_charging_detect
        ]
        self.sensors_msg_dict["auto_charging_en_pin"] = msg.data[
            self.auto_charging_en_pin
        ]
        std_io_msg = StringStamped()
        std_io_msg.stamp = rospy.Time.now()
        std_io_msg.data = json.dumps(self.sensors_msg_dict, indent=2)
        self.standard_io_pub.publish(std_io_msg)

    def loop(self):
        _val = Float32Stamped()
        _blink = EmptyStamped()
        rate = rospy.Rate(4)  # 10hz
        while not rospy.is_shutdown():

            # Publish arduino blink led
            _blink.stamp = rospy.Time.now()
            self.arduino_blink_pub.publish(_blink)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("arduino_ros_fake")
    rospy.loginfo("Init node: %s" % rospy.get_name())
    ArduinoRosFake(rospy.get_name())
