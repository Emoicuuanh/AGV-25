#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
from std_stamped_msgs.msg import Int8Stamped, StringStamped
from std_stamped_msgs.srv import StringService, StringServiceResponse
import json
from sensor_msgs.msg import Joy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from os.path import expanduser
import rosnode
import rosgraph
import subprocess
import time
import datetime
import threading
import re

HOME = expanduser("~")


class BagManager:
    def __init__(self):
        self.bag_process = None
        self.play_process = None
        self.current_bag_path = None
        self.current_bag_start_time = None
        self.bag_status_pub = rospy.Publisher('/bag_status', StringStamped, queue_size=10)
        self.init_ros()
        self.poll()

    def init_ros(self):
        rospy.init_node("bag_manager")
        rospy.loginfo("Init node: " + rospy.get_name())
        self.srv = rospy.Service('/record_bag', StringService, self.handle_record_bag)
        self.srv = rospy.Service('/stop_record_bag', StringService, self.handle_stop_record_bag)
        self.srv = rospy.Service('/get_list_bag', StringService, self.handle_get_list_bag)
        self.srv = rospy.Service('/get_topic_list', StringService, self.handle_get_topic_list)
        self.srv = rospy.Service('/delete_bag', StringService, self.handle_delete_bag)
        self.srv = rospy.Service('/play_bag', StringService, self.handle_play_bag)
        self.srv = rospy.Service('/play_bag_control', StringService, self.handle_play_bag_control)

    def handle_play_bag_control(self, req):
        rospy.loginfo(f"Received /play_bag_control: {req.request}")
        try:
            data = json.loads(req.request)
            command = data.get("command", "")
            if command not in ["pause", "resume", "stop"]:
                return StringServiceResponse("Invalid command")

            if not self.play_process or self.play_process.poll() is not None:
                return StringServiceResponse("No active play process")

            if command == "pause":
                rospy.loginfo("Pausing bag playback")
                self.play_process.send_signal(subprocess.signal.SIGSTOP)
            elif command == "resume":
                rospy.loginfo("Resuming bag playback")
                self.play_process.send_signal(subprocess.signal.SIGCONT)
            elif command == "stop":
                rospy.loginfo("Stopping bag playback")
                self.play_process.terminate()
                self.play_process.wait()
                self.play_process = None

            return StringServiceResponse(f"{command} command sent to play_bag")
        except Exception as e:
            rospy.logerr(f"Failed play_bag_control: {e}")
            return StringServiceResponse(f"Error: {str(e)}")

    def handle_play_bag(self, req):
        rospy.loginfo(f"Received /play_bag request: {req.request}")
        try:
            data = json.loads(req.request)
            bag_name = data.get("bag_name", "")
            if not bag_name:
                return StringServiceResponse("Missing bag_name")

            bag_dir = "{}/tmp/ros/bags/".format(HOME)
            bag_path = os.path.join(bag_dir, bag_name + ".bag")

            if not os.path.exists(bag_path):
                return StringServiceResponse(f"Bag file {bag_name} not found")

            # Nếu đã có process đang play thì dừng
            if self.play_process and self.play_process.poll() is None:
                self.play_process.terminate()
                self.play_process.wait()
                rospy.sleep(1)

            # Bắt đầu phát lại
            cmd = ['rosbag', 'play', bag_path]
            self.play_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True
            )
            threading.Thread(target=self._monitor_play_process_output, daemon=True).start()
            rospy.loginfo(f"Started playing bag: {bag_path}")
            return StringServiceResponse(f"Started playing {bag_name}")
        except Exception as e:
            rospy.logerr(f"Failed to play bag: {e}")
            return StringServiceResponse(f"Error: {str(e)}")

    def _monitor_play_process_output(self):
        if not self.play_process:
            return

        try:
            for line in self.play_process.stdout:
                if "Bag Time:" in line and "Duration:" in line:
                    try:
                        parts = line.strip().split("Duration:")
                        bag_time = parts[0].replace("Bag Time:", "").strip()
                        clean_bag_time = re.sub(r'\[.*?\]', '', bag_time).strip()
                        progress_str = parts[1].strip()
                        current, total = progress_str.split(" / ")
                        progress_percent = min(100.0, (float(current) / float(total)) * 100.0)

                        status_msg = {
                            "status": "PLAYING",
                            "bag_time": clean_bag_time,
                            "duration_current": float(current),
                            "duration_total": float(total),
                            "progress_percent": round(progress_percent, 2),
                            "bag_name": os.path.basename(self.current_bag_path) if self.current_bag_path else "",
                        }

                        msg = StringStamped()
                        msg.stamp = rospy.Time.now()
                        msg.data = json.dumps(status_msg)
                        self.bag_status_pub.publish(msg)
                    except Exception as e:
                        rospy.logwarn(f"Error parsing rosbag play output: {e}")

            # ✅ Khi tiến trình kết thúc
            self.play_process.wait()
            rospy.loginfo("Bag play completed.")
            self.play_process = None

            # Optional: gửi trạng thái hoàn thành
            done_msg = StringStamped()
            done_msg.stamp = rospy.Time.now()
            done_msg.data = json.dumps({
                "bag_name": os.path.basename(self.current_bag_path) if self.current_bag_path else "",
                "status": "PLAY_COMPLETED"
            })
            self.bag_status_pub.publish(done_msg)

        except Exception as e:
            rospy.logerr(f"Error monitoring rosbag play output: {e}")
            self.play_process = None


    def handle_record_bag(self, req):
        rospy.loginfo(f"Received /record_bag request: {req.request}")
        data = json.loads(req.request)
        if not data:
            rospy.logwarn("Invalid request format")
            return StringServiceResponse("Invalid request format")
        if 'bag_name' not in data:
            rospy.logwarn("Bag name not provided in request")
            return StringServiceResponse("Bag name not provided in request")
        if not isinstance(data['bag_name'], str):
            rospy.logwarn("Bag name must be a string")
            return StringServiceResponse("Bag name must be a string")
        new_file = data['bag_name']
        topics = data.get('chosen_topics', [])
        if not isinstance(topics, list):
            rospy.logwarn("Topics must be a list")
            return StringServiceResponse("Topics must be a list")

        rospy.loginfo(f"Bag name: {new_file}, Topics: {topics}")
        bag_dir = "{}/tmp/ros/bags/".format(HOME)
        os.makedirs(bag_dir, exist_ok=True)
        bag_path = os.path.join(bag_dir, new_file + ".bag")

        # Stop existing bag recording
        if self.bag_process is not None and self.bag_process.poll() is None:
            self.bag_process.terminate()
            rospy.sleep(1)

        try:
            cmd = ['rosbag', 'record', '-O', bag_path] + topics
            self.bag_process = subprocess.Popen(cmd)
            self.current_bag_path = bag_path
            self.current_bag_start_time = time.time()
            rospy.loginfo(f"Started recording bag: {bag_path}")
            response_str = f"Started recording: {bag_path}"
        except Exception as e:
            rospy.logerr(f"Failed to start recording: {e}")
            response_str = f"Failed to start recording: {str(e)}"

        return StringServiceResponse(response_str)

    def handle_get_topic_list(self, req):
        rospy.loginfo(f"Received /get_topic_list request: {req.request}")
        master = rosgraph.Master('/rostopic')
        try:
            topics = master.getTopicTypes()
            topic_names = [t[0] for t in topics]
            response_str = json.dumps(topic_names)
        except Exception as e:
            response_str = json.dumps({"error": str(e)})
        return StringServiceResponse(response_str)

    def handle_stop_record_bag(self, req):
        rospy.loginfo(f"Received /stop_record_bag request: {req.request}")
        if self.bag_process is not None and self.bag_process.poll() is None:
            self.bag_process.terminate()
            self.bag_process.wait()
            rospy.loginfo("Stopped bag recording.")
            self.bag_process = None
            self.current_bag_path = None
            self.current_bag_start_time = None
            response_str = "Stopped bag recording"
        else:
            response_str = "No active bag recording"
        return StringServiceResponse(response_str)

    def handle_delete_bag(self, req):
        rospy.loginfo(f"Received /delete_bag request: {req.request}")
        try:
            data = json.loads(req.request)
            bag_name = data.get("bag_name", "")
            if not bag_name:
                return StringServiceResponse("Missing bag_name")

            bag_dir = "{}/tmp/ros/bags/".format(HOME)
            bag_path = os.path.join(bag_dir, bag_name + ".bag")

            if not os.path.exists(bag_path):
                return StringServiceResponse(f"Bag file {bag_name} not found")

            os.remove(bag_path)
            rospy.loginfo(f"Deleted bag file: {bag_path}")
            return StringServiceResponse("Delete success")
        except Exception as e:
            rospy.logerr(f"Error deleting bag file: {e}")
            return StringServiceResponse(f"Error: {str(e)}")


    def handle_get_list_bag(self, req):
        rospy.loginfo(f"Received /get_list_bag request: {req.request}")
        bag_dir = "{}/tmp/ros/bags/".format(HOME)
        os.makedirs(bag_dir, exist_ok=True)

        bag_files = []
        for f in os.listdir(bag_dir):
            if f.endswith('.bag'):
                try:
                    full_path = os.path.join(bag_dir, f)
                    size_bytes = os.path.getsize(full_path)
                    size_mb = round(size_bytes / (1024 * 1024), 2)
                    created_at = datetime.datetime.fromtimestamp(
                        os.path.getctime(full_path)
                    ).isoformat()
                    modified_at = datetime.datetime.fromtimestamp(
                        os.path.getmtime(full_path)
                    ).isoformat()

                    bag_files.append({
                        "name": os.path.splitext(f)[0],
                        "size_mb": size_mb,
                        "created_at": created_at,
                        "modified_at": modified_at
                    })
                except Exception as e:
                    rospy.logwarn(f"Error reading file {f}: {e}")

        response_str = json.dumps(bag_files)
        return StringServiceResponse(response_str)

    def publish_bag_status(self):
        if self.bag_process and self.bag_process.poll() is None and self.current_bag_path:
            try:
                size_bytes = os.path.getsize(self.current_bag_path + ".active")
                size_mb = round(size_bytes / (1024 * 1024), 2)
                duration = round(time.time() - self.current_bag_start_time, 2)
                data = {
                    "status": "RECORDING",
                    "bag_name": os.path.basename(self.current_bag_path),
                    "bag_path": self.current_bag_path,
                    "size_mb": size_mb,
                    "duration_sec": duration
                }
                msg = StringStamped()
                msg.stamp = rospy.Time.now()
                msg.data = json.dumps(data)
                self.bag_status_pub.publish(msg)
            except Exception as e:
                rospy.logwarn(f"Error reading bag file info: {e}")

    def update_play_status_from_stdout(self):
        if self.play_process and self.play_process.poll() is not None:
            rospy.loginfo("Bag play process has ended.")
            self.play_process = None  # Reset process handle

    def poll(self):
        rate = rospy.Rate(1.0)  # 1 Hz
        while not rospy.is_shutdown():
            self.publish_bag_status()
            # self.update_play_status_from_stdout()
            rate.sleep()


def main():
    BagManager()


if __name__ == "__main__":
    main()
