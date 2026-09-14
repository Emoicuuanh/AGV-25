#!/usr/bin/env python
# -*- coding: utf-8 -*-
from pyclbr import Function
import yaml
import os
import sys
import rospy
import rospkg
import numpy as np
from math import *
from nav_msgs.msg import Odometry
from agv_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
import threading
import copy
import json
import queue
import tkinter as tk
from tkinter.scrolledtext import ScrolledText
from sensor_msgs.msg import LaserScan
from std_stamped_msgs.msg import (
    StringAction,
    StringStamped,
    StringResult,
    StringFeedback,
    StringGoal,
    Int8Stamped,
    EmptyStamped,
    Float32Stamped,
    Int16MultiArrayStamped,
)
from cognex_qr_code.srv import *
from std_msgs.msg import Bool, Int16, String, Int16MultiArray, Int8
from agv_msgs.msg import ArduinoIO, DigitalSensor, FollowLineSensor, LedControl
from agv_msgs.msg import (
    FollowLineSensor,
    DiffDriverMotorSpeed,
    EncoderDifferential,
    DataMatrixStamped,
)
from datetime import datetime
import csv

# Restore common_func_dir
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
    lockup_pose,
    dict_to_obj,
    merge_two_dicts,
    print_debug,
    print_info,
    print_warn,
    print_error,
    obj_to_dict,
    angle_robot_vs_robot_to_goal,
    distance_two_pose,
    YamlDumper,
    make_transform_stamped,
    angle_two_pose,
)

SENSOR_DEACTIVATE = 0
SENSOR_ACTIVATE = 1
LIFT_UP = 0
LIFT_DOWN = 1

class MainState(EnumString):
    NONE = -1
    DONE = 0
    CHECK_FOLLOWLINE = 1
    CHECK_CARD_ID = 2
    CHECK_MOTOR = 3
    CHECK_QR_CODE = 4
    CHECK_FRONT_START_FIRST = 5
    CHECK_FRONT_START_SECOND = 6
    CHECK_FRONT_STOP_FIRST = 7
    CHECK_FRONT_STOP_SECOND = 8
    CHECK_EMG_FISRT = 9
    CHECK_EMG_RELEASE = 10
    CHECK_BACK_START_FIRST = 11
    CHECK_BACK_START_SECOND = 13
    CHECK_BACK_STOP_FIRST = 14
    CHECK_BACK_STOP_SECOND = 15
    CHECK_BUMPER_FIRST = 16
    CHECK_BUMPER_SECOND = 17
    CHECK_LIFT_UP = 18
    CHECK_LIFT_DOWN = 19
    CHECK_DETECT_VRACK_SENSOR = 20
    CHECK_EMG_SECOND = 21
    CHECK_FRONT_LIDAR = 22
    CHECK_BACK_LIDAR = 23
    CHECK_RELEASE_STATE = 24
    CHECK_MANUAL_STATE = 25
    CHECK_AUTO_STATE = 26
    CHECK_MOTOR_OFF_SW = 27
    CHECK_MOTOR_ON_SW = 28
    CHECK_LED_TURN = 29
    CHECK_FASTTECH_INPUT = 30
    CHECK_HUB_QR_CODE = 31

class Button(EnumString):
    BUTTON_FRONT_START = 1
    BUTTON_FRONT_STOP = 2
    BUTTON_BACK_START = 3
    BUTTON_BACK_STOP = 4
    EMG = 5
    BUMPER = 6

class LedTurn(EnumString):
    FRONT_RIGHT = 1
    FRONT_LEFT = 2
    BACK_LEFT = 3
    BACK_RIGHT = 4
    FRONT = 5
    BACK = 6
    ALL = 7
    OFF_FRONT_RIGHT = 8
    OFF_BACK_LEFT = 9
    OFF_BACK_RIGHT = 10
    OFF_FRONT = 11
    OFF_BACK = 12
    OFF_ALL = 13
    RIGHT = 14
    LEFT = 15
    OFF_RIGHT = 16
    OFF_LEFT = 17

class LogWindow:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Kiem tra phan cung AGV")
        self.root.geometry("928x696")
        # Log frame
        self.log_frame = tk.Frame(self.root)
        self.log_frame.pack(side=tk.LEFT, padx=10, pady=10, fill=tk.BOTH, expand=True)
        self.text_area = ScrolledText(self.log_frame, wrap=tk.WORD, height=20, width=50, state="disabled")
        self.text_area.pack(fill=tk.BOTH, expand=True)
        # Button frame
        self.button_frame = tk.Frame(self.log_frame)
        self.button_frame.pack(pady=5)
        self.clear_button = tk.Button(self.button_frame, text="Clear Logs", command=self.clear_logs)
        self.clear_button.pack(side=tk.LEFT, padx=5)
        self.pass_button = tk.Button(self.button_frame, text="Pass", command=self.pass_clicked)
        self.pass_button.pack(side=tk.LEFT, padx=5)
        self.fail_button = tk.Button(self.button_frame, text="Fail", command=self.fail_clicked)
        self.fail_button.pack(side=tk.LEFT, padx=5)
        self.export_button = tk.Button(self.button_frame, text="Export", command=self.export_results, state="disabled")
        self.export_button.pack(side=tk.LEFT, padx=5)
        self.quit_button = tk.Button(self.button_frame, text="Quit", command=self.quit_clicked)
        self.quit_button.pack(side=tk.LEFT, padx=5)
        # Scan canvas
        self.canvas_frame = tk.Frame(self.root)
        self.canvas_frame.pack(side=tk.RIGHT, padx=10, pady=10)
        self.canvas = tk.Canvas(self.canvas_frame, width=300, height=300, bg='white')
        self.canvas.pack()
        self.canvas.create_oval(50, 50, 250, 250, outline='black')
        self.scan_title = self.canvas.create_text(150, 20, text="LIDAR Scan", font=("Arial", 12))
        # Queues and state
        self.queue = queue.Queue(maxsize=100)  # For logs and scans
        self.result_queue = queue.Queue(maxsize=1)  # For pass/fail results
        self.state_queue = queue.Queue(maxsize=1)  # For state changes
        self._logged_once = set()
        self.current_state = MainState.NONE
        self.last_scan_update = 0.0
        self.state_results = {}  # To store results from TEST_IO_AGV
        # Schedule periodic update
        self.root.after(100, self._process_queue)

    def _process_queue(self):
        try:
            while True:
                item = self.queue.get_nowait()
                if item['type'] == 'log':
                    self.text_area.configure(state="normal")
                    self.text_area.insert(tk.END, item['message'] + "\n")
                    self.text_area.configure(state="disabled")
                    self.text_area.yview(tk.END)
                elif item['type'] == 'scan':
                    current_time = rospy.get_time()
                    if current_time - self.last_scan_update < 0.2:  # Limit to 5 Hz
                        continue
                    self.current_state = item['state']
                    scan_data = item['data']
                    self.canvas.delete("scan")
                    if self.current_state == MainState.CHECK_FRONT_LIDAR:
                        self.canvas.itemconfig(self.scan_title, text="Front LIDAR")
                        ranges = scan_data.ranges
                        angle_increment = scan_data.angle_increment
                        angle_min = scan_data.angle_min
                        max_range = 10.0
                        scale = 100.0 / max_range
                        center_x, center_y = 150, 150
                        for i, r in enumerate(ranges):
                            if r < max_range:
                                angle = angle_min + i * angle_increment
                                x = center_x + r * scale * cos(angle)
                                y = center_y + r * scale * sin(angle)
                                self.canvas.create_oval(x-2, y-2, x+2, y+2, fill='blue', tags="scan")
                        self.last_scan_update = current_time
                    elif self.current_state == MainState.CHECK_BACK_LIDAR:
                        self.canvas.itemconfig(self.scan_title, text="Back LIDAR")
                        ranges = scan_data.ranges
                        angle_increment = scan_data.angle_increment
                        angle_min = scan_data.angle_min
                        max_range = 10.0
                        scale = 100.0 / max_range
                        center_x, center_y = 150, 150
                        for i, r in enumerate(ranges):
                            if r < max_range:
                                angle = angle_min + i * angle_increment
                                x = center_x + r * scale * cos(angle)
                                y = center_y + r * scale * sin(angle)
                                self.canvas.create_oval(x-2, y-2, x+2, y+2, fill='blue', tags="scan")
                        self.last_scan_update = current_time
                    else:
                        self.canvas.delete("scan")
                        self.canvas.itemconfig(self.scan_title, text="LIDAR Scan")
                elif item['type'] == 'state':
                    self.current_state = item['state']
                    self.state_results = item['results']
                    if self.current_state == MainState.DONE:
                        self.export_button.config(state="normal")
                    else:
                        self.export_button.config(state="disabled")
        except queue.Empty:
            pass
        except Exception as e:
            rospy.logerr(f"GUI queue processing error: {e}, Thread: {threading.current_thread().name}")
        self.root.after(100, self._process_queue)

    def log(self, message):
        try:
            self.queue.put_nowait({'type': 'log', 'message': message, 'thread': threading.current_thread().name})
        except queue.Full:
            rospy.logwarn("GUI queue full, dropping log message")

    def log_once(self, message):
        if message not in self._logged_once:
            self._logged_once.add(message)
            self.log(message)

    def update_scan(self, scan_data, state):
        try:
            self.queue.put_nowait({'type': 'scan', 'data': scan_data, 'state': state, 'thread': threading.current_thread().name})
        except queue.Full:
            rospy.logwarn("GUI queue full, dropping scan update")

    def update_state(self, state, results):
        try:
            self.queue.put_nowait({'type': 'state', 'state': state, 'results': results, 'thread': threading.current_thread().name})
        except queue.Full:
            rospy.logwarn("State queue full, dropping state update")

    def clear_logs(self):
        try:
            self.text_area.configure(state="normal")
            self.text_area.delete(1.0, tk.END)
            self.text_area.configure(state="disabled")
            self._logged_once.clear()
        except Exception as e:
            rospy.logerr(f"Clear logs error: {e}, Thread: {threading.current_thread().name}")

    def pass_clicked(self):
        try:
            self.result_queue.put_nowait({'result': True, 'thread': threading.current_thread().name})
            self.log("Pass button clicked")
        except queue.Full:
            rospy.logwarn("Result queue full, dropping pass result")

    def fail_clicked(self):
        try:
            self.result_queue.put_nowait({'result': False, 'thread': threading.current_thread().name})
            self.log("Fail button clicked")
        except queue.Full:
            rospy.logwarn("Result queue full, dropping fail result")
            
    def quit_clicked(self):
        try:
            self.log("Quit button clicked")
            rospy.signal_shutdown("Quit button pressed")
            self.root.quit()
        except Exception as e:
            rospy.logerr(f"Quit button error: {e}, Thread: {threading.current_thread().name}")

    def export_results(self):
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_dir = os.path.join(rospkg.RosPack().get_path("test_io_agv"), "results")
            os.makedirs(output_dir, exist_ok=True)
            output_file = os.path.join(output_dir, f"test_results_{timestamp}.csv")
            with open(output_file, "w", newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["State", "Result"])  # Header
                for state, result in self.state_results.items():
                    if result is not None:
                        writer.writerow([state.toString(), "Pass" if result else "Fail"])
            self.log(f"Results exported to {output_file}")
            rospy.loginfo(f"Results exported to {output_file}")
        except Exception as e:
            self.log(f"Export error: {e}")
            rospy.logerr(f"Export error: {e}, Thread: {threading.current_thread().name}")

    def get_result(self):
        try:
            return self.result_queue.get_nowait()
        except queue.Empty:
            return None

    def start(self):
        rospy.loginfo(f"Starting Tkinter mainloop in thread: {threading.current_thread().name}")
        self.root.mainloop()

class TEST_IO_AGV(object):
    def __init__(self, name, *args, **kwargs):
        self.rate = rospy.Rate(10)
        self.current_state = MainState.NONE
        self.state_start_time = rospy.get_time()
        self.gui = LogWindow()
        self.get_qr_code = rospy.ServiceProxy("ReadQrCode", QrCode)
        self.init_subscribers()
        self.init_publishers()
        self.init_variable(*args, **kwargs)
        self.ros_thread = threading.Thread(target=self.ros_loop, daemon=True)
        self.ros_thread.start()

    def init_subscribers(self):
        try:
            rospy.Subscriber("/odom", Odometry, self.odom_cb)
            rospy.Subscriber("/card_id", String, self.card_id_cb)
            rospy.Subscriber("/standard_io", StringStamped, self.standard_io_cb)
            rospy.Subscriber("/followline_sensor", FollowLineSensor, self.followline_cb)
            rospy.Subscriber("/motor_encoder", EncoderDifferential, self.motor_encoder_cb)
            rospy.Subscriber("/scan_1", LaserScan, self.scan_1_cb)
            rospy.Subscriber("/scan_2", LaserScan, self.scan_2_cb)
            rospy.Subscriber("/fastech_input", Int16MultiArray, self.optical_sensor_cb)
            rospy.Subscriber("/fastech_output", Int16MultiArray, self.read_fastech_output_cb)
            rospy.Subscriber("/data_gls621", DataMatrixStamped, self.qr_update_cb)
            # rospy.Subscriber("/qr_sensor", StringStamped, self.qr_hub_cb)
        except Exception as e:
            rospy.logerr(f"Subscriber initialization error: {e}")

    def init_publishers(self):
        try:
            self.pub_lift_cmd = rospy.Publisher("/lift_cart", Int8Stamped, queue_size=10)
            self.pub_led_turn = rospy.Publisher("/arduino_driver/led_turn", Int16MultiArrayStamped, queue_size=10)
            self.fastech_control_pub = rospy.Publisher("/fastech_control_multiarray", Int16MultiArray, queue_size=10)
            self.pub_led_status = rospy.Publisher("/led_status", StringStamped, queue_size=5, latch=True)
        except Exception as e:
            rospy.logerr(f"Publisher initialization error: {e}")
            
    def init_services(self):
        try:
            self.get_qr_code = rospy.ServiceProxy("ReadQrCode", QrCode)
        except Exception as e:
            rospy.logerr(f"Service init error: {e}")

    def yaml_load(self, filepath):
        if not os.path.isfile(filepath):
            rospy.logerr(f"YAML file not found: {filepath}")
            return None
        try:
            with open(filepath, "r") as read_file:
                data = yaml.load(read_file, Loader=yaml.FullLoader)
            return data
        except Exception as e:
            rospy.logerr(f"Load yaml file error: {e}")
            return None

    def init_variable(self, *args, **kwargs):
        config_file_led = kwargs["config_file_led"]
        rospy.loginfo(f"Loading config_file_led: {config_file_led}")
        led_effect_config = self.yaml_load(config_file_led)
        if led_effect_config is None or "led_effect" not in led_effect_config:
            rospy.logerr("Failed to load led_effect from YAML or invalid format")
            self.led_effect_list = []
        else:
            self.led_effect_list = led_effect_config["led_effect"]
        self.led_turn = [0, 0, 0, 0]
        self.led_turn_flag = False
        self.liftup_finish = False
        self.liftdown_finish = False
        self.emg_status = False
        self.start_1 = False
        self.start_2 = False
        self.stop_1 = False
        self.stop_2 = False
        self.detect_vrack = False
        self.bumper = False
        self.motor_enable_sw = False
        self.auto_manual_sw = False
        self.pre_liftup_finish = False
        self.pre_liftdown_finish = False
        self.pre_emg_status = False
        self.pre_start_1 = False
        self.pre_start_2 = False
        self.pre_stop_1 = False
        self.pre_stop_2 = False
        self.pre_detect_vrack = False
        self.pre_bumper = False
        self.pre_motor_enable_sw = False
        self.pre_auto_manual_sw = False
        self.led_number = 0
        self.led_turn_msg = Int16MultiArrayStamped()
        self.led_msg = StringStamped()
        self.optical_msg = Int16MultiArray()
        self.lift_msg = Int8Stamped()
        self.followline = ""
        self.card_id = ""
        self.motor = ""
        self.pose_odom2robot = Pose()
        self.qr_code = ""
        self.qr_code_gls621 = None
        self.scan_1 = None
        self.scan_2 = None
        self.control_lift_up = False
        self.control_lift_down = False
        self.emg_1_done = False
        self.emg_2_done = False
        self.gui_once = False
        self.switch_ok_logged = False  # Track if "OK" message logged
        self.thread_optical = threading.Thread(name="thread_publish", target=self.optical_sensor)
        self.thread_optical.daemon = True
        self.thread_optical.start()
        self._logged_once = set()
        self.state_results = {state: None for state in MainState}  # Initialize results dictionary

    def shutdown(self):
        rospy.loginfo("Shutting down")
        try:
            # Log final results
            self.gui_log("Final State Results:")
            for state, result in self.state_results.items():
                if result is not None:
                    self.gui_log(f"{state.toString()}: {'Pass' if result else 'Fail'}")
            self.gui.root.quit()
        except Exception as e:
            rospy.logerr(f"Shutdown GUI error: {e}")

    def ros_loop(self):
        self.loop()
        rospy.signal_shutdown("ROS loop terminated")

    def check_result(self):
        result = self.gui.get_result()
        if result is not None:
            self.state_results[self.current_state] = result['result']
            self.gui_log(f"State {self.current_state.toString()} marked as {'Pass' if result['result'] else 'Fail'}")
            return result['result']
        return None

    def loop(self):
        _state = MainState.CHECK_AUTO_STATE
        _prev_state = MainState.NONE
        self.led_turn_control(LedTurn.OFF_ALL)
        pre_odom_x = self.pose_odom2robot.position.x
        rospy.sleep(1)
        while not rospy.is_shutdown():
            try:
                if _prev_state != _state:
                    self.gui_log(f"CHECK MODULE: {_state.toString()}")
                    self.current_state = _state
                    self.gui.update_state(_state, self.state_results)
                    self.state_start_time = rospy.get_time()
                    self.gui_once = False
                    self.switch_ok_logged = False
                    _prev_state = _state
                # Check for timeout
                if rospy.get_time() - self.state_start_time > 120.0:
                    self.gui_log(f"Timeout in state {_state.toString()}, marking as Fail")
                    rospy.logwarn(f"Timeout in state {_state.toString()}")
                    self.state_results[_state] = False
                    # Advance to next state based on original logic (without condition check, but for branches, use current variables)
                    if _state == MainState.CHECK_AUTO_STATE:
                        _state = MainState.CHECK_RELEASE_STATE
                    elif _state == MainState.CHECK_RELEASE_STATE:
                        _state = MainState.CHECK_MANUAL_STATE
                    elif _state == MainState.CHECK_MANUAL_STATE:
                        _state = MainState.CHECK_MOTOR_OFF_SW
                    elif _state == MainState.CHECK_MOTOR_OFF_SW:
                        _state = MainState.CHECK_MOTOR_ON_SW
                    elif _state == MainState.CHECK_MOTOR_ON_SW:
                        _state = MainState.CHECK_MOTOR
                    elif _state == MainState.CHECK_MOTOR:
                        _state = MainState.CHECK_FRONT_START_FIRST
                    elif _state == MainState.CHECK_FRONT_START_FIRST:
                        _state = MainState.CHECK_FRONT_STOP_FIRST
                    elif _state == MainState.CHECK_FRONT_STOP_FIRST:
                        _state = MainState.CHECK_BACK_STOP_FIRST
                    elif _state == MainState.CHECK_BACK_STOP_FIRST:
                        _state = MainState.CHECK_BACK_START_FIRST
                    elif _state == MainState.CHECK_BACK_START_FIRST:
                        _state = MainState.CHECK_EMG_FISRT
                    elif _state == MainState.CHECK_EMG_FISRT:
                        _state = MainState.CHECK_EMG_RELEASE
                    elif _state == MainState.CHECK_EMG_SECOND:
                        _state = MainState.CHECK_EMG_RELEASE
                    elif _state == MainState.CHECK_EMG_RELEASE:
                        if self.emg_1_done and not self.emg_2_done:
                            _state = MainState.CHECK_EMG_SECOND
                        else:
                            _state = MainState.CHECK_BUMPER_FIRST
                    elif _state == MainState.CHECK_BUMPER_FIRST:
                        _state = MainState.CHECK_BUMPER_SECOND
                    elif _state == MainState.CHECK_BUMPER_SECOND:
                        _state = MainState.CHECK_LED_TURN
                    elif _state == MainState.CHECK_LED_TURN:
                        _state = MainState.CHECK_QR_CODE
                    elif _state == MainState.CHECK_QR_CODE:
                        _state = MainState.CHECK_FRONT_LIDAR
                    elif _state == MainState.CHECK_FRONT_LIDAR:
                        _state = MainState.CHECK_BACK_LIDAR
                    elif _state == MainState.CHECK_BACK_LIDAR:
                        _state = MainState.CHECK_LIFT_UP
                    elif _state == MainState.CHECK_LIFT_UP:
                        _state = MainState.CHECK_DETECT_VRACK_SENSOR
                    elif _state == MainState.CHECK_DETECT_VRACK_SENSOR:
                        _state = MainState.CHECK_LIFT_DOWN
                    elif _state == MainState.CHECK_LIFT_DOWN:
                        _state = MainState.CHECK_FASTTECH_INPUT
                    elif _state == MainState.CHECK_FASTTECH_INPUT:
                        _state = MainState.CHECK_HUB_QR_CODE
                    elif _state == MainState.CHECK_HUB_QR_CODE:
                        _state = MainState.DONE
                    continue
                # Check for pass/fail button press
                result = self.check_result()
                if result is not None:
                    # Advance to next state based on original logic
                    if _state == MainState.CHECK_AUTO_STATE:
                        _state = MainState.CHECK_RELEASE_STATE
                    elif _state == MainState.CHECK_RELEASE_STATE:
                        _state = MainState.CHECK_MANUAL_STATE
                    elif _state == MainState.CHECK_MANUAL_STATE:
                        _state = MainState.CHECK_MOTOR_OFF_SW
                    elif _state == MainState.CHECK_MOTOR_OFF_SW:
                        _state = MainState.CHECK_MOTOR_ON_SW
                    elif _state == MainState.CHECK_MOTOR_ON_SW:
                        _state = MainState.CHECK_MOTOR
                    elif _state == MainState.CHECK_MOTOR:
                        _state = MainState.CHECK_FRONT_START_FIRST
                    elif _state == MainState.CHECK_FRONT_START_FIRST:
                        _state = MainState.CHECK_FRONT_STOP_FIRST
                    elif _state == MainState.CHECK_FRONT_STOP_FIRST:
                        _state = MainState.CHECK_BACK_STOP_FIRST
                    elif _state == MainState.CHECK_BACK_STOP_FIRST:
                        _state = MainState.CHECK_BACK_START_FIRST
                    elif _state == MainState.CHECK_BACK_START_FIRST:
                        _state = MainState.CHECK_EMG_FISRT
                    elif _state == MainState.CHECK_EMG_FISRT:
                        _state = MainState.CHECK_EMG_RELEASE
                    elif _state == MainState.CHECK_EMG_SECOND:
                        _state = MainState.CHECK_EMG_RELEASE
                    elif _state == MainState.CHECK_EMG_RELEASE:
                        if self.emg_1_done and not self.emg_2_done:
                            _state = MainState.CHECK_EMG_SECOND
                        else:
                            _state = MainState.CHECK_BUMPER_FIRST
                    elif _state == MainState.CHECK_BUMPER_FIRST:
                        _state = MainState.CHECK_BUMPER_SECOND
                    elif _state == MainState.CHECK_BUMPER_SECOND:
                        _state = MainState.CHECK_LED_TURN
                    elif _state == MainState.CHECK_LED_TURN:
                        _state = MainState.CHECK_QR_CODE
                    elif _state == MainState.CHECK_QR_CODE:
                        _state = MainState.CHECK_FRONT_LIDAR
                    elif _state == MainState.CHECK_FRONT_LIDAR:
                        _state = MainState.CHECK_BACK_LIDAR
                    elif _state == MainState.CHECK_BACK_LIDAR:
                        _state = MainState.CHECK_LIFT_UP
                    elif _state == MainState.CHECK_LIFT_UP:
                        _state = MainState.CHECK_DETECT_VRACK_SENSOR
                    elif _state == MainState.CHECK_DETECT_VRACK_SENSOR:
                        _state = MainState.CHECK_LIFT_DOWN
                    elif _state == MainState.CHECK_LIFT_DOWN:
                        _state = MainState.CHECK_FASTTECH_INPUT
                    elif _state == MainState.CHECK_FASTTECH_INPUT:
                        _state = MainState.CHECK_HUB_QR_CODE
                    elif _state == MainState.CHECK_HUB_QR_CODE:
                        _state = MainState.DONE
                    continue 
                # State machine changing
                if _state == MainState.CHECK_AUTO_STATE:
                    if not self.gui_once:
                        self.gui_log("xoay công tắc AUTO")
                        self.gui_once = True
                    if self.auto_manual_sw == SENSOR_ACTIVATE and not self.switch_ok_logged:
                        self.gui_log("Đang ở chế độ AUTO")
                        self.switch_ok_logged = True
                elif _state == MainState.CHECK_RELEASE_STATE:
                    if not self.gui_once:
                        self.gui_log("xoay công tắc RELEASE")
                        self.gui_once = True
                    if self.motor_enable_sw == SENSOR_DEACTIVATE and self.auto_manual_sw == SENSOR_DEACTIVATE and not self.switch_ok_logged:
                        self.gui_log("Đang ở chế độ RELEASE")
                        self.switch_ok_logged = True
                elif _state == MainState.CHECK_MANUAL_STATE:
                    if not self.gui_once:
                        self.gui_log("xoay công tắc MANUAL")
                        self.gui_once = True
                    if self.auto_manual_sw == SENSOR_DEACTIVATE and self.motor_enable_sw == SENSOR_ACTIVATE and not self.switch_ok_logged:
                        self.gui_log("Đang ở chế độ MANUAL")
                        self.switch_ok_logged = True
                elif _state == MainState.CHECK_MOTOR_OFF_SW:
                    if not self.gui_once:
                        self.gui_log("xoay công tắc MOTOR_OFF")
                        self.gui_once = True
                    if self.motor_enable_sw == SENSOR_DEACTIVATE and not self.switch_ok_logged:
                        self.gui_log("Đang ở chế độ MOTOR OFF")
                        self.switch_ok_logged = True
                elif _state == MainState.CHECK_MOTOR_ON_SW:
                    if not self.gui_once:
                        self.gui_log("xoay công tắc MOTOR_ON")
                        self.gui_once = True
                    if self.motor_enable_sw == SENSOR_ACTIVATE and not self.switch_ok_logged:
                        self.gui_log("Đang ở chế độ MOTOR ON")
                        self.switch_ok_logged = True
                elif _state == MainState.CHECK_MOTOR:
                    if not self.gui_once:
                        self.gui_log("Di chuyển bằng joystick")
                        self.gui_once = True
                    if abs(self.pose_odom2robot.position.x - pre_odom_x) > 0.1 and not self.switch_ok_logged:
                        self.gui_log("MOTOR nhận được lệnh từ JOYSTICK")
                        self.switch_ok_logged = True
                elif _state == MainState.CHECK_FRONT_START_FIRST:
                    if not self.gui_once:
                        self.gui_log("nhấn nút RUN phía trước")
                        self.gui_once = True
                    if self.check_click_button(Button.BUTTON_FRONT_START) and not self.switch_ok_logged:
                        self.gui_log("đã nhận tín hiệu nút RUN phía trước")
                        self.switch_ok_logged = True
                elif _state == MainState.CHECK_FRONT_STOP_FIRST:
                    if not self.gui_once:
                        self.gui_log("nhấn nút PAUSE phía trước")
                        self.gui_once = True
                    if self.check_click_button(Button.BUTTON_FRONT_STOP) and not self.switch_ok_logged:
                        self.gui_log("đã nhận tín hiệu nút PAUSE phía trước")
                        self.switch_ok_logged = True
                elif _state == MainState.CHECK_BACK_STOP_FIRST:
                    if not self.gui_once:
                        self.gui_log("nhấn nút PAUSE phía sau")
                        self.gui_once = True
                    if self.check_click_button(Button.BUTTON_BACK_STOP) and not self.switch_ok_logged:
                        self.gui_log("đã nhận tín hiệu nút PAUSE phía sau")
                        self.switch_ok_logged = True
                elif _state == MainState.CHECK_BACK_START_FIRST:
                    if not self.gui_once:
                        self.gui_log("nhấn nút RUN phía sau")
                        self.gui_once = True
                    if self.check_click_button(Button.BUTTON_BACK_START) and not self.switch_ok_logged:
                        self.gui_log("đã nhận tín hiệu nút RUN phía sau")
                        self.switch_ok_logged = True
                elif _state == MainState.CHECK_EMG_FISRT:
                    if not self.gui_once:
                        self.gui_log("nhấn nút EMG phía trước")
                        self.gui_once = True
                    if not self.emg_status and not self.switch_ok_logged:
                        self.gui_log("EMG 1 ON OK")
                        self.emg_1_done = True
                        self.switch_ok_logged = True
                elif _state == MainState.CHECK_EMG_SECOND:
                    if not self.gui_once:
                        self.gui_log("nhấn nút EMG phía sau")
                        self.gui_once = True
                    if not self.emg_status and not self.switch_ok_logged:
                        self.gui_log("EMG 2 ON OK")
                        self.emg_2_done = True
                        self.led_turn_control(LedTurn.FRONT)
                        self.switch_ok_logged = True
                elif _state == MainState.CHECK_EMG_RELEASE:
                    if not self.gui_once:
                        self.gui_log("thả nút EMG")
                        self.gui_once = True
                    if self.emg_status and not self.switch_ok_logged:
                        self.gui_log("EMG OFF OK")
                        self.led_turn_control(LedTurn.OFF_ALL)
                        self.switch_ok_logged = True
                elif _state == MainState.CHECK_BUMPER_FIRST:
                    if not self.gui_once:
                        self.gui_log("nhấn BUMPER phía trước")
                        self.gui_once = True
                    if self.check_click_button(Button.BUMPER) and not self.switch_ok_logged:
                        self.gui_log("BUMPER 1st OK")
                        self.switch_ok_logged = True
                elif _state == MainState.CHECK_BUMPER_SECOND:
                    if not self.gui_once:
                        self.gui_log("nhấn BUMPER phía sau")
                        self.gui_once = True
                    if self.check_click_button(Button.BUMPER) and not self.switch_ok_logged:
                        self.gui_log("BUMPER 2nd OK")
                        self.switch_ok_logged = True
                elif _state == MainState.CHECK_LED_TURN:
                    if not self.gui_once:
                        self.gui_log("Nhấn RUN/PAUSE trên AGV để bật tắt LED xi nhan, chọn pass/fail để đánh giá và chuyển qua bước tiếp theo")
                        self.gui_once = True
                    if self.check_click_button(Button.BUTTON_BACK_START) or self.check_click_button(Button.BUTTON_FRONT_START):
                        self.led_turn_flag = True
                    if self.led_turn_flag:
                        self.led_turn_control(LedTurn.ALL)
                    if self.check_click_button(Button.BUTTON_BACK_STOP) or self.check_click_button(Button.BUTTON_FRONT_STOP) and not self.switch_ok_logged:
                        self.led_turn_control(LedTurn.OFF_ALL)
                        self.gui_log("LED TURN OK")
                        self.switch_ok_logged = True
                        self.led_turn_flag = False
                elif _state == MainState.CHECK_QR_CODE:
                    if not self.gui_once:
                        self.gui_log("Di chuyển AGV đến QR trên mặt sàn sao cho 4 xi nhan đều sáng, nhấn RUN trên AGV để đọc giá trị QR")
                        self.gui_once = True
                    if self.check_click_button(Button.BUTTON_BACK_START) or self.check_click_button(Button.BUTTON_FRONT_START):
                        if self.qr_code_gls621 is not None:
                            self.gui_log(f"QR CODE OK: \n\
                                Sai lệch phương X CAM->QR: {self.qr_code_gls621.possition.x} \n\
                                Sai lệch phương Y CAM->QR: {self.qr_code_gls621.possition.y} \n\
                                Sai lệch góc CAM->QR: {self.qr_code_gls621.possition.angle} \n")
                        else:
                            self.gui_log("Không đọc được / tìm thấy QR code")
                elif _state == MainState.CHECK_FRONT_LIDAR:
                    if not self.gui_once:
                        scan_len_1 = len(self.scan_1) if self.scan_1 is not None else 0
                        if scan_len_1 != 0:
                            self.gui_log(f"Kiểm tra FRONT LIDAR")
                        else:
                            self.gui_log("Không thấy dữ liệu FRONT LIDAR")
                        self.gui_once = True
                elif _state == MainState.CHECK_BACK_LIDAR:
                    if not self.gui_once:
                        scan_len_2 = len(self.scan_2) if self.scan_2 is not None else 0
                        if scan_len_2 != 0:
                            self.gui_log(f"Kiểm tra BACK LIDAR")
                        else:
                            self.gui_log("Không thấy dữ liệu BACK LIDAR")
                        self.gui_once = True
                elif _state == MainState.CHECK_LIFT_UP:
                    if not self.gui_once:
                        self.gui_log("Giữ nút RUN để nâng bàn nâng AGV")
                        self.gui_once = True
                    # if self.check_click_button(Button.BUTTON_FRONT_START):
                    #     self.control_lift_up = True
                    if self.control_lift_up:
                        self.lift_msg.stamp = rospy.Time.now()
                        self.lift_msg.data = LIFT_UP
                        self.pub_lift_cmd.publish(self.lift_msg)
                    if self.liftup_finish and not self.switch_ok_logged:
                        self.gui_log("LIFT UP OK")
                        self.led_test("MANUAL")
                        self.switch_ok_logged = True
                        self.control_lift_up = False
                elif _state == MainState.CHECK_DETECT_VRACK_SENSOR:
                    if not self.gui_once:
                        self.gui_log("DETECT VRACK SENSOR")
                        self.gui_once = True
                    if self.detect_vrack and not self.switch_ok_logged:
                        rospy.sleep(3)
                        self.gui_log("DETECT VRACK OK")
                        self.led_test("OFF")
                        self.switch_ok_logged = True
                elif _state == MainState.CHECK_LIFT_DOWN:
                    if not self.gui_once:
                        self.gui_log("Giữ nút PAUSE để hạ bàn nâng")
                        self.gui_once = True
                    # if self.check_click_button(Button.BUTTON_FRONT_STOP):
                    #     self.control_lift_down = True
                    if self.control_lift_down:
                        self.lift_msg.stamp = rospy.Time.now()
                        self.lift_msg.data = LIFT_DOWN
                        self.pub_lift_cmd.publish(self.lift_msg)
                    if self.liftdown_finish and not self.switch_ok_logged:
                        self.gui_log("LIFT DOWN OK")
                        self.switch_ok_logged = True
                        self.control_lift_down = False
                elif _state == MainState.CHECK_FASTTECH_INPUT:
                    if not self.gui_once:
                        self.gui_log("Kiểm tra các đầu input của fastech sáng theo thứ tự 1->7")
                        self.gui_once = True
                    if not self.switch_ok_logged:
                        # self.gui_log("FASTECH OK")
                        self.switch_ok_logged = True
                elif _state == MainState.CHECK_HUB_QR_CODE:
                    if not self.gui_once:
                        self.gui_log("nhấn RUN trên AGV để đọc giá trị QR HUB")
                        self.gui_once = True
                    if self.check_click_button(Button.BUTTON_BACK_START) or self.check_click_button(Button.BUTTON_FRONT_START):
                        resp = self.get_qr_code(2)
                        self.qr_code = resp.Res
                        self.gui_log(f"{self.qr_code}")
                    if not self.switch_ok_logged:
                        # 
                        self.switch_ok_logged = True
                elif _state == MainState.DONE:
                    self.led_test("MANUAL")
                    self.led_turn_control(LedTurn.ALL)
                    self.gui_log("Hoàn thành test. Chọn export để xuất kết quả")
                    sys.exit()
                self.pre_start_1 = self.start_1
                self.pre_start_2 = self.start_2
                self.pre_stop_1 = self.stop_1
                self.pre_stop_2 = self.stop_2
                self.pre_emg_status = self.emg_status
                self.pre_bumper = self.bumper
                self.pre_motor_enable_sw = self.motor_enable_sw
                self.pre_auto_manual_sw = self.auto_manual_sw
                self.rate.sleep()
            except Exception as e:
                rospy.logerr(f"Loop error: {e}, Thread: {threading.current_thread().name}")

    def led_turn_control(self, led_type):
        try:
            if led_type == LedTurn.FRONT_RIGHT:
                self.led_turn_msg.data = [0, 1, 0, 0]
            elif led_type == LedTurn.FRONT_LEFT:
                self.led_turn_msg.data = [1, 0, 0, 0]
            elif led_type == LedTurn.BACK_RIGHT:
                self.led_turn_msg.data = [0, 0, 0, 1]
            elif led_type == LedTurn.BACK_LEFT:
                self.led_turn_msg.data = [0, 0, 1, 0]
            elif led_type == LedTurn.FRONT:
                self.led_turn_msg.data = [1, 1, 0, 0]
            elif led_type == LedTurn.BACK:
                self.led_turn_msg.data = [0, 0, 1, 1]
            elif led_type == LedTurn.RIGHT:
                self.led_turn_msg.data = [0, 1, 1, 0]
            elif led_type == LedTurn.LEFT:
                self.led_turn_msg.data = [1, 0, 0, 1]
            elif led_type == LedTurn.ALL:
                self.led_turn_msg.data = [1, 1, 1, 1]
            else:
                self.led_turn_msg.data = [0, 0, 0, 0]
            self.led_turn_msg.stamp = rospy.Time.now()
            self.pub_led_turn.publish(self.led_turn_msg)
            rospy.sleep(0.1)
        except Exception as e:
            rospy.logerr(f"LED turn control error: {e}, Thread: {threading.current_thread().name}")

    def check_click_button(self, type_button):
        try:
            if type_button == Button.BUTTON_FRONT_START:
                return self.start_1 == SENSOR_ACTIVATE and self.pre_start_1 == SENSOR_DEACTIVATE
            elif type_button == Button.BUTTON_FRONT_STOP:
                return self.stop_1 == SENSOR_ACTIVATE and self.pre_stop_1 == SENSOR_DEACTIVATE
            elif type_button == Button.BUTTON_BACK_STOP:
                return self.stop_2 == SENSOR_ACTIVATE and self.pre_stop_2 == SENSOR_DEACTIVATE
            elif type_button == Button.BUTTON_BACK_START:
                return self.start_2 == SENSOR_ACTIVATE and self.pre_start_2 == SENSOR_DEACTIVATE
            elif type_button == Button.EMG:
                return self.emg_status == SENSOR_DEACTIVATE and self.pre_emg_status == SENSOR_ACTIVATE
            elif type_button == Button.BUMPER:
                return self.bumper == SENSOR_DEACTIVATE and self.pre_bumper == SENSOR_ACTIVATE
            return False
        except Exception as e:
            rospy.logerr(f"Check button error: {e}, Thread: {threading.current_thread().name}")
            return False

    def gui_log(self, msg):
        try:
            self.gui.log(f"{msg}")
        except Exception as e:
            rospy.logerr(f"GUI log error: {e}, Thread: {threading.current_thread().name}")

    def gui_log_once(self, msg):
        try:
            self.gui.log_once(f"[{rospy.get_time():.2f}] {msg}")
        except Exception as e:
            rospy.logerr(f"GUI log once error: {e}, Thread: {threading.current_thread().name}")

    def optical_sensor(self):
        while not rospy.is_shutdown():
            try:
                for i in range(8):
                    list = [0, 0, 0, 0, 0, 0, 0, 0]
                    list[i] = 1
                    self.optical_msg.data = list
                    self.fastech_control_pub.publish(self.optical_msg)
                    rospy.sleep(1)
            except Exception as e:
                rospy.logerr(f"Optical sensor publish error: {e}, Thread: {threading.current_thread().name}")

    def led_test(self, led_name):
        try:
            self.led_msg.stamp = rospy.Time.now()
            self.led_msg.data = led_name
            self.pub_led_status.publish(self.led_msg)
            rospy.sleep(0.1)
        except Exception as e:
            rospy.logerr(f"Control led error: {e}, Thread: {threading.current_thread().name}")

    def odom_cb(self, msg):
        try:
            rospy.logdebug(f"Received odom data, Thread: {threading.current_thread().name}")
            self.pose_odom2robot = msg.pose.pose
        except Exception as e:
            rospy.logerr(f"Odom callback error: {e}, Thread: {threading.current_thread().name}")

    def read_fastech_output_cb(self, msg):
        try:
            rospy.logdebug(f"Received fastech_output data, Thread: {threading.current_thread().name}")
            self.read_fastech_value_fb = list(msg.data)
        except Exception as e:
            rospy.logerr(f"Fastech output callback error: {e}, Thread: {threading.current_thread().name}")

    def optical_sensor_cb(self, msg):
        try:
            rospy.logdebug(f"Received fastech_input data, Thread: {threading.current_thread().name}")
            self.digial_input = msg
        except Exception as e:
            rospy.logerr(f"Optical sensor callback error: {e}, Thread: {threading.current_thread().name}")

    def standard_io_cb(self, msg):
        try:
            rospy.logdebug(f"Received standard_io data, Thread: {threading.current_thread().name}")
            data = json.loads(msg.data)
            if "lift_max_sensor" in data: self.liftup_finish = data["lift_max_sensor"]
            if "lift_min_sensor" in data: self.liftdown_finish = data["lift_min_sensor"]
            if "emg_button" in data: self.emg_status = data["emg_button"]
            if "start_1_button" in data: self.start_1 = data["start_1_button"]
            if "start_2_button" in data: self.start_2 = data["start_2_button"]
            if "stop_1_button" in data: self.stop_1 = data["stop_1_button"]
            if "stop_2_button" in data: self.stop_2 = data["stop_2_button"]
            if "detect_vrack" in data: self.detect_vrack = data["detect_vrack"]
            if "bumper" in data: self.bumper = data["bumper"]
            if "motor_enable_sw" in data: self.motor_enable_sw = data["motor_enable_sw"]
            if "auto_manual_sw" in data: self.auto_manual_sw = data["auto_manual_sw"]
        except Exception as e:
            rospy.logerr(f"Standard IO callback error: {e}, Thread: {threading.current_thread().name}")

    def motor_encoder_cb(self, msg):
        try:
            rospy.logdebug(f"Received motor_encoder data, Thread: {threading.current_thread().name}")
            self.motor = msg.left
        except Exception as e:
            rospy.logerr(f"Motor encoder callback error: {e}, Thread: {threading.current_thread().name}")

    def followline_cb(self, msg):
        try:
            rospy.logdebug(f"Received followline_sensor data, Thread: {threading.current_thread().name}")
            FollowLineCounter = 0
            for i in range(16):
                FollowLineCounter += msg.data[i]
            if FollowLineCounter != 0:
                self.followline = msg.data
        except Exception as e:
            rospy.logerr(f"Followline callback error: {e}, Thread: {threading.current_thread().name}")

    def card_id_cb(self, msg):
        try:
            rospy.logdebug(f"Received card_id data, Thread: {threading.current_thread().name}")
            self.card_id = msg.data
        except Exception as e:
            rospy.logerr(f"Card ID callback error: {e}, Thread: {threading.current_thread().name}")

    def scan_1_cb(self, msg):
        try:
            rospy.logdebug(f"Received scan_1 data, Thread: {threading.current_thread().name}")
            self.scan_1 = msg.ranges
            if self.current_state == MainState.CHECK_FRONT_LIDAR:
                self.gui.update_scan(msg, self.current_state)
        except Exception as e:
            rospy.logerr(f"Scan 1 callback error: {e}, Thread: {threading.current_thread().name}")

    def scan_2_cb(self, msg):
        try:
            rospy.logdebug(f"Received scan_2 data, Thread: {threading.current_thread().name}")
            self.scan_2 = msg.ranges
            if self.current_state == MainState.CHECK_BACK_LIDAR:
                self.gui.update_scan(msg, self.current_state)
        except Exception as e:
            rospy.logerr(f"Scan 2 callback error: {e}, Thread: {threading.current_thread().name}")

    def qr_update_cb(self, msg):
        try:
            rospy.logdebug(f"Received data_gls621 data, Thread: {threading.current_thread().name}")
            self.qr_code_gls621 = msg
        except Exception as e:
            rospy.logerr(f"QR update callback error: {e}, Thread: {threading.current_thread().name}")

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
        "--config_file_led",
        dest="config_file_led",
        default=os.path.join(
            rospkg.RosPack().get_path("amr_config"), "cfg", "led_effect.yaml"
        ),
    )
    (options, args) = parser.parse_args()
    rospy.loginfo(f"Options: {options}")
    rospy.loginfo(f"Args: {args}")
    return (options, args)

def main():
    (options, args) = parse_opts()
    log_level = rospy.DEBUG if options.log_debug else rospy.INFO
    rospy.init_node("test_io_agv", log_level=log_level)
    rospy.loginfo(f"Init node {rospy.get_name()} in thread: {threading.current_thread().name}")
    try:
        node = TEST_IO_AGV(rospy.get_name(), **vars(options))
        node.gui.start()  # Run Tkinter mainloop in main thread
    except Exception as e:
        rospy.logerr(f"Node initialization failed: {e}, Thread: {threading.current_thread().name}")
        sys.exit(1)

if __name__ == "__main__":
    main()