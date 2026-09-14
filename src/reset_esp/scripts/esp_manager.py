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


class Esp_manager(object):


    def __init__(self, name, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        self.load_config(kwargs["config_file"])

        # Publisher
        self.pub_continue_run = rospy.Publisher("/request_run_stop", StringStamped, queue_size=10)
        self.standard_io_pub = rospy.Publisher("/reset_esp_status", StringStamped, queue_size=10)

        # Subscriber
        rospy.Subscriber("/vl53l5cx_r1", Vl53l5cxRanges, self.vl53l5cx_r1_status_cb)
        rospy.Subscriber("/vl53l5cx_r2", Vl53l5cxRanges, self.vl53l5cx_r2_status_cb)
        rospy.Subscriber("/vl53l5cx_r3", Vl53l5cxRanges, self.vl53l5cx_r3_status_cb)
        rospy.Subscriber("/vl53l5cx_r4", Vl53l5cxRanges, self.vl53l5cx_r4_status_cb)
        rospy.Subscriber("/robot_status", StringStamped, self.robot_status_cb)

        # log_dir = os.path.join(
        #     rospkg.RosPack().get_path("reset_esp"), "log")
        home_dir = os.path.expanduser("~")

        # Tạo đường dẫn đầy đủ cho thư mục log
        log_dir = os.path.join(home_dir, "log_esp_arduino")

        # Tạo thư mục nếu nó chưa tồn tại
        os.makedirs(log_dir, exist_ok=True)

        print("Log directory:", log_dir)

        log_file = f"{log_dir}/logfileESP.txt"  # Đường dẫn và tên của file
        self.file = open(log_file, "a")
        self.file.write("\n-------------------------------------------------------------------------------------------------\n")
        self.file.write(
            "\ntime_start: {}\n".format(
                datetime.now().strftime("%Y-%m-%d-%H:%M:%S.%f")
            )
        )
        # rospy.on_shutdown(self.writelog)
        self.port_location_esp_forward = rospy.get_param("~port_esp_forward", "/dev/ttyUSB0")
        self.port_location_esp_backward = rospy.get_param("~port_esp_backward", "/dev/ttyUSB0")
        print("Set port esp forward: {}".format(self.port_location_esp_forward))
        print("Set port esp backward: {}".format(self.port_location_esp_backward))
        self.port_name_esp_forward = find_soft_port(self.port_location_esp_forward)
        self.port_name_esp_backward = find_soft_port(self.port_location_esp_backward)
        rospy.loginfo("Connecting port esp forward : {}".format(self.port_name_esp_forward))
        rospy.loginfo("Connecting port esp backward : {}".format(self.port_name_esp_backward))
        # usb_device_path = self.get_usb_device_path(self.port_name)
        rospy.loginfo("Find usb_device_path esp forward: {}".format(self.port_location_esp_forward))
        rospy.loginfo("Find usb_device_path esp backward: {}".format(self.port_location_esp_backward))
        rospy.sleep(30.0)
        self.loop()

    def init_variable(self, *args, **kwargs):
        self.last_data_sensor1_received = rospy.get_time()
        self.last_data_sensor2_received = rospy.get_time()
        self.last_data_sensor3_received = rospy.get_time()
        self.last_data_sensor4_received = rospy.get_time()
        self.time_reset_error_success = rospy.get_time()

        self.bool_disconnect = False

        self.number_error = 0
        self.count_restart = 0
        self.data_time_out = 5.0  # second
        self.time_wait_restart = 40.0
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

    def kill_node_by_name(self, name, usb_device_path):
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

        rospy.sleep(1)

        # try:
        #     if usb_device_path:
        #         subprocess.run(f'echo "mkac" |sudo -S sh -c \'echo "0" > /sys/bus/usb/devices/{usb_device_path}/authorized\'', shell=True, check=True)
        #         rospy.loginfo(f"Disabled USB device at {usb_device_path}")
        # except subprocess.CalledProcessError as e:
        #     rospy.logerr(f"Error disabling USB: {e}")

    def run_node_by_name(self, usb_device_path_1, usb_device_path_2):
        # try:
        #     if usb_device_path_1:
        #         # Enable the USB device
        #         subprocess.run(f'echo "mkac" |sudo -S sh -c \'echo "1" > /sys/bus/usb/devices/{usb_device_path_1}/authorized\'', shell=True, check=True)
        #         rospy.loginfo(f"Enabled USB device at {usb_device_path_1}")

        # except subprocess.CalledProcessError as e:
        #     rospy.logerr(f"Error enabling USB device: {e}")

        # rospy.sleep(1)
        # try:
        #     if usb_device_path_2:
        #         # Enable the USB device
        #         subprocess.run(f'echo "mkac" |sudo -S sh -c \'echo "1" > /sys/bus/usb/devices/{usb_device_path_2}/authorized\'', shell=True, check=True)
        #         rospy.loginfo(f"Enabled USB device at {usb_device_path_2}")

        # except subprocess.CalledProcessError as e:
        #     rospy.logerr(f"Error enabling USB device: {e}")
        rospy.sleep(1)
        try:
            # Run the node again
            self.launch_esp = subprocess.Popen(
                ["roslaunch", self.config_file["ros_pkg"], self.config_file["launch_file"]]
            )
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Error running node: {e}")


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

    def vl53l5cx_r1_status_cb(self, msg):
        self.last_data_sensor1_received = rospy.get_time()

    def vl53l5cx_r2_status_cb(self, msg):
        self.last_data_sensor2_received = rospy.get_time()

    def vl53l5cx_r3_status_cb(self, msg):
        self.last_data_sensor3_received = rospy.get_time()

    def vl53l5cx_r4_status_cb(self, msg):
        self.last_data_sensor4_received= rospy.get_time()

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
            if rospy.get_time() - self.last_data_sensor1_received > self.data_time_out or \
                rospy.get_time() - self.last_data_sensor2_received > self.data_time_out or \
                rospy.get_time() - self.last_data_sensor3_received > self.data_time_out or \
                rospy.get_time() - self.last_data_sensor4_received > self.data_time_out:
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
                if self.count_restart < 5:
                    self.kill_node_by_name("/backward_esp_serial", self.port_location_esp_backward)
                    rospy.sleep(1)
                    self.kill_node_by_name("/forward_esp_serial", self.port_location_esp_forward)
                    rospy.sleep(5)
                    self.run_node_by_name(self.port_location_esp_forward, self.port_location_esp_backward)
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
            rospkg.RosPack().get_path("reset_esp"), "cfg", "restart_esp.yaml"
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
    rospy.init_node("reset_esp", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    Esp_manager(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
