#!/usr/bin/env python
# -*- coding: utf-8 -*-


import os, time
import sys
import rospy
import rospkg
from rosserial_arduino import SerialClient
from serial import SerialException
from time import sleep
import serial.tools.list_ports


def find_soft_port(hardware_port):
    if "dev" in hardware_port:
        return hardware_port
    for port in serial.tools.list_ports.comports():
        if port.location == hardware_port:
            return port.device


def find_soft_port_from_pid(port_pid):
    for p in serial.tools.list_ports.comports():
        if p.pid == port_pid:
            return p.device
            # print('Port can tim : '+ str(p.location) + str(p.device))


def main():
    port_location = rospy.get_param("~port", "2-3")
    print("Set port: {}".format(port_location))
    port_name = find_soft_port(port_location)
    print(port_name)

    rospy.init_node("arduino_rosserial")
    rospy.loginfo("ROS Serial Python Node")
    baud = int(rospy.get_param("~baud", "57600"))

    # Number of seconds of sync failure after which Arduino is auto-reset.
    # 0 = no timeout, auto-reset disabled
    auto_reset_timeout = int(rospy.get_param("~auto_reset_timeout", "0"))

    # for systems where pyserial yields errors in the fcntl.ioctl(self.fd, TIOCMBIS, \
    # TIOCM_DTR_str) line, which causes an IOError, when using simulated port
    fix_pyserial_for_test = rospy.get_param("~fix_pyserial_for_test", False)

    # TODO: do we really want command line params in addition to parameter server params?
    sys.argv = rospy.myargv(argv=sys.argv)
    if len(sys.argv) >= 2:
        port_name = sys.argv[1]

    while not rospy.is_shutdown():
        rospy.loginfo("Connecting to %s at %d baud" % (port_name, baud))
        try:
            client = SerialClient(
                port_name,
                baud,
                fix_pyserial_for_test=fix_pyserial_for_test,
                auto_reset_timeout=auto_reset_timeout,
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


if __name__ == "__main__":
    main()
