#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from re import A
import sys
import json
import rospy
import rospkg
import os, time
from rosserial_arduino import SerialClient
from serial import SerialException
from time import sleep
import serial.tools.list_ports

# import json
from std_msgs.msg import String
from std_stamped_msgs.msg import StringStamped, Float32Stamped, EmptyStamped
from arduino_serial.msg import SetPinStamped, BoolArrayStamped


class RosserialConnect:
    def __init__(self, *args, **kwargs):
        self.init_variable(*args, **kwargs)

        rospy.init_node("pointcloud_publisher_node", anonymous=True)
        rospy.loginfo("Init node " + rospy.get_name())
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
            self.stand_io_cb,
        )

        self.initial_parameter()
        self.loop()

    def init_variable(self, *args, **kwargs):
        self.config_path = kwargs["config_path"]

    def initial_parameter(self):
        self.baud = int(rospy.get_param("~baud", "57600"))
        self.port_location = rospy.get_param("~port", "2-6")
        self.port_name = self.find_soft_port(self.port_location)
        self.auto_reset_timeout = int(
            rospy.get_param("~auto_reset_timeout", "0")
        )
        self.fix_pyserial_for_test = rospy.get_param(
            "~fix_pyserial_for_test", False
        )
        self.stand_io_config = rospy.get_param("~sensors", dict({}))
        self.sensors_msg_dict = {}
        self._blink = EmptyStamped()
        # self.parameter_adjust()

    def find_soft_port(self, hardware_port):
        if "dev" in hardware_port:
            return hardware_port
        for port in serial.tools.list_ports.comports():
            if port.location == hardware_port:
                return port.device

    def find_soft_port_from_pid(self, port_pid):
        for p in serial.tools.list_ports.comports():
            if p.pid == port_pid:
                return p.device

    def parameter_adjust(self):
        for sensor, params in self.stand_io_config.items():
            # rospy.logerr("sensor ", self.baud)
            self.sensors_msg_dict[sensor] = params["pin"]
        # rospy.logerr(self.sensors_msg_dict)

    def stand_io_cb(self, msg):
        for sensor, params in self.stand_io_config.items():
            if not params["logic_type"]:
                self.sensors_msg_dict[sensor] = msg.data[params["pin"]]
            else:
                self.sensors_msg_dict[sensor] = not msg.data[params["pin"]]
        # rospy.logerr(self.sensors_msg_dict)

        # msg = BoolArrayStamped()
        std_io_msg = StringStamped()
        std_io_msg.stamp = rospy.Time.now()
        std_io_msg.data = json.dumps(self.sensors_msg_dict, indent=2)
        self.standard_io_pub.publish(std_io_msg)
        self._blink.stamp = rospy.Time.now()
        self.arduino_blink_pub.publish(self._blink)

    def loop(self):
        rate = rospy.Rate(20)

        sys.argv = rospy.myargv(argv=sys.argv)
        if len(sys.argv) >= 2:
            self.port_name = sys.argv[1]
        # self._blink = EmptyStamped()
        while not rospy.is_shutdown():
            rospy.loginfo(
                "Connecting to %s at %d baud" % (self.port_name, self.baud)
            )
            try:
                client = SerialClient(
                    self.port_name,
                    self.baud,
                    fix_pyserial_for_test=self.fix_pyserial_for_test,
                    auto_reset_timeout=self.auto_reset_timeout,
                )
                client.run()
            except KeyboardInterrupt:
                break
            except SerialException:
                sleep(1.0)
                continue
            except OSError:
                sleep(1.0)
                continue


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
    parser.add_option(
        "-p",
        "--config_path",
        dest="config_path",
        # default=os.path.join(
        #     rospkg.RosPack().get_path("........."),
        #     "cfg",
        #     "algorithm.json",
        # ),
    )

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)


def main():
    (options, args) = parse_opts()
    log_level = None
    if options.log_debug:
        log_level = rospy.DEBUG
    RosserialConnect(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
