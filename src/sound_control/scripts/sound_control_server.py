#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from copy import deepcopy
from enum import Enum
import os
import sys
import rospy
import rospkg
import time
import actionlib
import json
import yaml
from pathlib import Path
import vlc
from std_stamped_msgs.msg import (
    StringStamped,
    StringAction,
    StringFeedback,
    StringResult,
)
from os.path import expanduser

HOME = expanduser("~")

common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)

agv_mongodb_dir = os.path.join(
    rospkg.RosPack().get_path("agv_mongodb"), "scripts"
)
if not os.path.isdir(agv_mongodb_dir):
    agv_mongodb_dir = os.path.join(
        rospkg.RosPack().get_path("agv_mongodb"), "release"
    )
sys.path.insert(0, agv_mongodb_dir)

from mongodb import mongodb
from sound_control.msg import (
    SoundControlResult,
    SoundControlFeedback,
    SoundControlAction,
    SoundControlGoal,
)

from common_function import (
    EnumString,
    lockup_pose,
    offset_pose_x,
    offset_pose_yaw,
    distance_two_pose,
    delta_angle,
    dict_to_obj,
    merge_two_dicts,
    print_debug,
    print_warn,
    print_error,
    print_info,
    get_line_info,
    obj_to_dict,
    offset_pose_xy_theta,
)


class MainState(EnumString):
    NONE = -1
    INIT = 0
    WAIT_REQUEST = 1
    CHECK_TIME = 2
    PLAY = 3
    DONE = 4
    ERROR = 5


class SoundStatus(EnumString):
    WAITING = 0
    RUNNING = 1
    ERROR = 2


class SoundControl(object):
    _feedback = SoundControlFeedback()
    _result = SoundControlResult()

    def __init__(self, name, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        # Action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            SoundControlAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()
        self.client = actionlib.SimpleActionClient(
            self._action_name, SoundControlAction
        )
        # Publisher
        self.sound_status_pub = rospy.Publisher(
            "/sound_status", StringStamped, queue_size=5
        )
        # Subscriber
        rospy.Subscriber("/request_sound", StringStamped, self.request_sound_cb)
        # listening for goals.
        self.client.wait_for_server()

    def init_variable(self, *args, **kwargs):
        self.audio = None
        self.audio_player = vlc.MediaPlayer()
        db_address = rospy.get_param("/mongodb_address")
        self.db = mongodb(db_address)
        self.loop_sound = False
        self.play_sound = False
        self.sound_file = ""
        self.last_sound_request = ""
        self.prev_file_name = ""
        self.sound_remap_file = kwargs["config_file"]
        self.sound_remap_config = ""
        self.load_sound_remap()

    def shutdown(self):
        pass

    def send_feedback(self, msg):
        self._feedback.Feedback = msg
        self._as.publish_feedback(self._feedback)

    def download_file(self, file_name):
        abs_file = os.path.join(
            HOME + "/tmp/ros/sounds/", "{}.mp3".format(file_name)
        )
        begin_download = rospy.get_time()
        # Only download if file not exist
        while (
            not Path(abs_file).exists()
            and rospy.get_time() - begin_download < 1.0
        ):
            # Download from db
            self.db.downloadFile(file_name, abs_file, "sound")
            rospy.loginfo("Download file: {}".format(file_name))
            rospy.Rate(5.0).sleep()
        if not Path(abs_file).exists():
            rospy.logerr(
                "Downloaded sound file not exist: {}".format(file_name)
            )
            return False
        return abs_file
    def load_sound_remap(self):
        try:
            with open(self.sound_remap_file) as file:
                self.sound_remap_config = yaml.load(file)['sound_status']
                rospy.logerr(self.sound_remap_config["EMG"])
        except Exception as e:
            rospy.logerr('Load file remap sound error: {}'.format(e))

    """
     ######     ###    ##       ##       ########     ###     ######  ##    ##
    ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
    ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
    ##       ##     ## ##       ##       ########  ##     ## ##       #####
    ##       ######### ##       ##       ##     ## ######### ##       ##  ##
    ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
     ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
    """

    def request_sound_cb(self, msg):
        if msg.data == "":
            self.client.cancel_all_goals()
        if msg.data in self.sound_remap_config:
            sound = self.sound_remap_config[msg.data]
        else:
            sound = msg.data
        if sound != self.last_sound_request:
            rospy.loginfo("Request sound: {}".format(sound))
            # Creates a goal to send to the action server.
            goal = SoundControlGoal()
            goal.SoundName = sound
            goal.SoundLoop = True
            # Sends the goal to the action server.
            self.client.send_goal(goal)
        self.last_sound_request = msg.data

    """
       ###     ######  ######## ####  #######  ##    ##    ######## ##     ## ########
      ## ##   ##    ##    ##     ##  ##     ## ###   ##    ##        ##   ##  ##
     ##   ##  ##          ##     ##  ##     ## ####  ##    ##         ## ##   ##
    ##     ## ##          ##     ##  ##     ## ## ## ##    ######      ###    ######
    ######### ##          ##     ##  ##     ## ##  ####    ##         ## ##   ##
    ##     ## ##    ##    ##     ##  ##     ## ##   ###    ##        ##   ##  ##
    ##     ##  ######     ##    ####  #######  ##    ##    ######## ##     ## ########
    """

    def execute_cb(self, goal):
        success = True
        get_sound_success = False
        # get file name sound
        try:
            self.loop_sound = goal.SoundLoop
            self.play_sound = True
            sound_name = goal.SoundName
            file_name = self.download_file(sound_name)
            if not file_name:
                raise Exception("Sound file not exist")
            if self.play_sound:
                self.sound_file = file_name
            else:
                self.sound_file = ""
            get_sound_success = True
        except Exception as e:
            self._as.set_aborted(text="Sound action goal error")
            rospy.logerr("Sound control goal error: {}".format(e))
            return

        if get_sound_success:
            self.send_feedback("Get sound name OK")
            rospy.loginfo("Get sound file succeeded %s " % self.sound_file)
            # play sound loop
            success = self.loop()

            if success:
                self._result.Result = "Action Success"
                rospy.loginfo("%s: Succeeded" % self._action_name)
                self._as.set_succeeded(self._result)

    def loop(self):
        _state = MainState.WAIT_REQUEST
        _prev_state = MainState.NONE
        duration = 0.0
        start_time = None
        sleep_time = 0.1
        status_msg = SoundStatus.WAITING
        last_stt_pub = rospy.get_time()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                # if _state != _prev_state:
                #     rospy.loginfo('State: {} -> {}'.format(_prev_state.toString(), _state.toString()))
                #     _prev_state = _state
                status_msg = SoundStatus.WAITING
                # Transaction
                if _state == MainState.INIT:
                    _state = MainState.WAIT_REQUEST
                elif _state == MainState.WAIT_REQUEST:
                    if self.sound_file != "":
                        audio = vlc.Media(self.sound_file)
                        self.audio_player.set_media(audio)
                        self.audio_player.play()
                        _state = MainState.CHECK_TIME
                    else:
                        return False
                elif _state == MainState.CHECK_TIME:
                    duration = self.audio_player.get_length() / 1000.0
                    # print('Duration: {} s'.format(duration))
                    start_time = rospy.get_time()
                    _state = MainState.PLAY
                elif _state == MainState.PLAY:
                    status_msg = SoundStatus.RUNNING
                    running_time = rospy.get_time() - start_time
                    # print('Running time: {} s'.format(running_time))
                    if running_time >= duration - 3 * sleep_time:
                        if self.loop_sound:
                            _state = MainState.WAIT_REQUEST
                        else:
                            self.audio_player.stop()
                            _state = MainState.WAIT_REQUEST
                            return True
                # self.prev_file_name = self.sound_file

                if rospy.get_time() - last_stt_pub >= 1.0:
                    last_stt_pub = rospy.get_time()
                    msg = StringStamped()
                    msg.stamp = rospy.Time.now()
                    msg.data = status_msg.toString()
                    self.sound_status_pub.publish(msg)

                if self._as.is_preempt_requested():
                    self.audio_player.stop()
                    rospy.loginfo("%s: Preempted" % self._action_name)
                    self._as.set_preempted()
                    return False
                r.sleep()
            except KeyboardInterrupt:
                rospy.logwarn("Keyboard Interrupt action")
                break
        return True


def parse_opts():
    from optparse import OptionParser

    parser = OptionParser()
    parser.add_option(
        "-s",
        "--simulation",
        action="store_true",
        dest="simulation",
        default=False,
        help='type "-s" if simulation',
    )
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
            rospkg.RosPack().get_path("sound_control"),
            "cfg",
            "sound_control.yaml",
        ),
    )

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)


def main():
    os.system("mkdir -p {}/tmp/ros/sounds/".format(HOME))
    (options, args) = parse_opts()
    log_level = None
    if options.log_debug:
        log_level = rospy.DEBUG
    rospy.init_node("sound_control_server", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    SoundControl(rospy.get_name(), **vars(options))
    rospy.spin()


if __name__ == "__main__":
    main()
