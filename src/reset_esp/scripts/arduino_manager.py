#! /usr/bin/env python
# -*- coding: utf-8 -*-
import time
import os
import subprocess
import sys
import rospy
import rospkg
import copy
import json
import re
import yaml
import subprocess
from vl53l5cx.msg import Vl53l5cxRanges
from std_stamped_msgs.msg import (

    StringStamped,
    StringAction,
    StringFeedback,
    StringResult,
    StringGoal,
    EmptyStamped,
)
from std_msgs.msg import String
from datetime import datetime
common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)

from common_function import (
    EnumString,
    find_soft_port
)


class MainState(EnumString):
    NONE = 0
    CONNECT = 1
    DISCONNECT = 2
    RESTARTING = 3
    ERROR = 4


class Arduino_manager(object):


    def __init__(self, name, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        self.load_config(kwargs["config_file"])

        # Publisher
        self.pub_continue_run = rospy.Publisher("/request_run_stop", StringStamped, queue_size=10)
        self.standard_io_pub = rospy.Publisher("/reset_arduino_status", StringStamped, queue_size=10)

        # Subscriber
        rospy.Subscriber("/standard_io", StringStamped, self.standard_io_cb)
        rospy.Subscriber("/robot_status", StringStamped, self.robot_status_cb)

        # log_dir = os.path.join(
        #     rospkg.RosPack().get_path("reset_esp"), "log")
        home_dir = os.path.expanduser("~")

        # Tạo đường dẫn đầy đủ cho thư mục log
        log_dir = os.path.join(home_dir, "log_esp_arduino")

        # Tạo thư mục nếu nó chưa tồn tại
        os.makedirs(log_dir, exist_ok=True)

        print("Log directory:", log_dir)
        log_file = f"{log_dir}/logfileArduino.txt"  # Đường dẫn và tên của file
        self.file = open(log_file, "a")
        self.file.write("\n-------------------------------------------------------------------------------------------------\n")
        self.file.write(
            "\ntime_start: {}\n".format(
                datetime.now().strftime("%Y-%m-%d-%H:%M:%S.%f")
            )
        )
        port_location = rospy.get_param("~port", "/dev/ttyUSB0")
        print("Set port: {}".format(port_location))
        self.port_name = find_soft_port(port_location)
        rospy.loginfo("Connecting port: {}".format(self.port_name))
        # self.usb_device_path = self.get_usb_device_path(self.port_name)
        self.usb_device_path = port_location
        rospy.loginfo("Find usb_device_path: {}".format(self.usb_device_path))
        # rospy.on_shutdown(self.closeLog)
        rospy.sleep(30.0)
        self.loop()

    def init_variable(self, *args, **kwargs):
        self.last_data_arduino_received = rospy.get_time()

        self.bool_disconnect = False
        self.time_reset_error_success = rospy.get_time()

        self.count_restart = 0
        self.number_error = 0
        self.data_time_out = 5.0  # second
        self.time_wait_restart = 30.0
        self.time_restart = 0.0
        self.old_time_state_restart = rospy.get_time()
        self.status_robot = ''
        self.mode_robot = ''
        self.continue_run = False
        self.data_run = StringStamped()
        self.data_run.data = 'RUN'

    def load_config(self, file_path):
        try:
            with open(file_path) as file:
                self.config_file = yaml.load(file, Loader=yaml.Loader)
                return True
        except Exception as e:
            rospy.logerr("Error loading: {}".format(e))
            return False

    def kill_node_by_name(self, name):
        try:
            # Kill the node by name
            cmd = f"pkill -f {name}"
            subprocess.run(cmd, shell=True, check=True)
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Error killing node: {e}")

        rospy.sleep(1)

        try:
            # Kill the ROS node by name
            cmd = f"rosnode kill {name}"
            subprocess.run(cmd, shell=True, check=True)
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Error killing node: {e}")

        rospy.sleep(3)

        try:
            if self.usb_device_path:
                subprocess.run(f'echo "mkac" |sudo -S sh -c \'echo "0" > /sys/bus/usb/devices/{self.usb_device_path}/authorized\'', shell=True, check=True)
                rospy.loginfo(f"Disabled USB device at {self.usb_device_path}")
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Error disabling USB: {e}")

    def run_node_by_name(self):
        try:
            if self.usb_device_path:
                # Enable the USB device
                subprocess.run(f'echo "mkac" |sudo -S sh -c \'echo "1" > /sys/bus/usb/devices/{self.usb_device_path}/authorized\'', shell=True, check=True)
                rospy.loginfo(f"Enabled USB device at {self.usb_device_path}")
                rospy.sleep(1)

        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Error enabling USB device: {e}")

        try:
            # Run the node again
            self.launch_arduino = subprocess.Popen(
                ["roslaunch", self.config_file["ros_pkg"], self.config_file["launch_file"]]
            )
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Error running node: {e}")

    def get_usb_device_path(self, tty_device):
        try:
            # Run udevadm to get information about the tty device
            result = subprocess.run(
                ['udevadm', 'info', '--name=' + tty_device, '--query=all'],
                capture_output=True, text=True, check=True
            )

            # Print udevadm output for debugging
            # print("udevadm output:")
            # print(result.stdout)

            # Find the DEVPATH
            devpath = None
            for line in result.stdout.splitlines():
                if "DEVPATH=" in line:
                    devpath = line.split('=')[1]
                    break

            if devpath:
                # Extract the USB device name from DEVPATH
                # This assumes the USB device path is before the tty device path
                match = re.search(r'/usb(\d+)/', devpath)  # Look for usbX in the DEVPATH
                if match:
                    usb_device_name = match.group(1)  # Get the number from usbX
                    return f'usb{usb_device_name}'
        except subprocess.CalledProcessError as e:
            print(f"Error: {e}")

        return None

    def writeLog(self, state, number_retry, error_count, time_restart_success):
        if time_restart_success == 0:
            self.file.write(
                        "time: {}, state: {}, number_error_retry: {}, number_error_count: {}\n".format(
                            datetime.now().strftime("%Y-%m-%d-%H:%M:%S.%f"),
                            state,
                            number_retry,
                            error_count,
                        )
                    )
        else:
            self.file.write(
                        "time: {}, state: {}, number_error_retry: {}, number_error_count: {}, time_restart_success: {}\n".format(
                            datetime.now().strftime("%Y-%m-%d-%H:%M:%S.%f"),
                            state,
                            number_retry,
                            error_count,
                            time_restart_success
                        )
                    )

    def closeLog(self):
        self.file.write(
                    "time end: {}".format(
                        datetime.now().strftime("%Y-%m-%d-%H:%M:%S.%f")
                    )
                )
        self.file.close()
        rospy.loginfo("Close log file!")
        # rospy.loginfo("Shuting down")

    """
     ######     ###    ##       ##       ########     ###     ######  ##    ##
    ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
    ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
    ##       ##     ## ##       ##       ########  ##     ## ##       #####
    ##       ######### ##       ##       ##     ## ######### ##       ##  ##
    ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
     ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
    """

    def standard_io_cb(self, msg):
        self.last_data_arduino_received = rospy.get_time()

    def robot_status_cb(self,msg):
        robot_status = json.loads(msg.data)
        if "status" in robot_status:
            self.status_robot = robot_status["status"]
        if "mode" in robot_status:
            self.mode_robot = robot_status["mode"]

    """
    ##        #######   #######  ########
    ##       ##     ## ##     ## ##     ##
    ##       ##     ## ##     ## ##     ##
    ##       ##     ## ##     ## ########
    ##       ##     ## ##     ## ##
    ##       ##     ## ##     ## ##
    ########  #######   #######  ##
    """

    def loop(self):
        _state = MainState.CONNECT
        _pre_state = MainState.NONE
        std_io_msg = StringStamped()
        sensors_msg_dict = {}
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            # check sensor disconnect
            if _state != _pre_state:
                if _pre_state == MainState.RESTARTING and _state == MainState.CONNECT:
                    time_restart = rospy.get_time() - self.time_reset_error_success
                    self.writeLog(_state, self.count_restart, self.number_error, time_restart)
                else:
                    self.writeLog(_state, self.count_restart, self.number_error, 0)

                rospy.logwarn(
                    "Action state: {} -> {}".format(
                        _pre_state.toString(), _state.toString()
                    )
                )
                _pre_state = _state
            if rospy.get_time() - self.last_data_arduino_received > self.data_time_out:
                    self.bool_disconnect = True
            else:
                self.bool_disconnect = False

            if _state == MainState.CONNECT:
                if self.bool_disconnect == True:
                    self.number_error += 1
                    rospy.logwarn("Change state from Connect to Disconnect")
                    self.time_reset_error_success = rospy.get_time()
                    _state = MainState.DISCONNECT
                else:
                    if self.continue_run:
                        if self.status_robot == "PAUSED" and self.mode_robot == "AUTO":
                            for i in range(3):
                                self.pub_continue_run.publish(self.data_run)
                                rospy.sleep(0.1)
                        self.continue_run = False
            elif _state == MainState.DISCONNECT:
                self.kill_node_by_name("/arduino_driver")
                rospy.sleep(2)
                self.run_node_by_name()
                self.old_time_state_restart = rospy.get_time()
                _state = MainState.RESTARTING
                self.time_restart = 0.0
                self.count_restart +=1

            elif _state == MainState.RESTARTING:
                dt = rospy.get_time() - self.old_time_state_restart
                self.old_time_state_restart = rospy.get_time()
                self.time_restart += dt
                # rospy.logerr("Wait restart arduino")
                if self.time_restart > self.time_wait_restart and self.bool_disconnect:
                    _state = MainState.DISCONNECT
                else:
                    if not self.bool_disconnect:
                        self.count_restart = 0
                        _state = MainState.CONNECT
                        self.continue_run = True
            sensors_msg_dict["state"] = _state.toString()
            sensors_msg_dict["count_restart"] = self.count_restart
            sensors_msg_dict["number_error"] = self.number_error
            std_io_msg.stamp = rospy.Time.now()
            std_io_msg.data = json.dumps(sensors_msg_dict, indent=2)
            self.standard_io_pub.publish(std_io_msg)
            rate.sleep()
        self.closeLog()


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
        "-c",
        "--config_file",
        dest="config_file",
        default=os.path.join(
            rospkg.RosPack().get_path("reset_esp"), "cfg", "restart_arduino.yaml"
        ),
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
    rospy.init_node("reset_arduino", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    Arduino_manager(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()

