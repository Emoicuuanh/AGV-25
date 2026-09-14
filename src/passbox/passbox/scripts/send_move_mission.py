#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Script gửi mission MOVE tới 1 điểm qua /moving_control action server.
Chạy: rosrun moving_control send_move_mission.py
"""

import rospy
import actionlib
import json
from std_stamped_msgs.msg import StringAction, StringGoal


# ============================================================
# CẤU HÌNH MISSION - Chỉnh sửa tại đây
# ============================================================

# Tên pose đích (phải khớp với waypoints[0]["name"])
TARGET_POSITION_NAME = "pose_17594868261"

# Waypoints: chỉ cần 1 phần tử = điểm đích
# Lưu ý: data.next / data.road_type là metadata bản đồ, không dùng khi chỉ có 1 điểm
WAYPOINTS = [
    {
        "name": "pose_17594868261",
        "map": "MKHC_3F",
        "type": "position",
        "data": {
            "radius": 0.10340787852145206
        },
        "position": {
            "orientation": {
                "w": 1,
                "x": 0,
                "y": 0,
                "z": 0
            },
            "position": {
                "x": 21.349,
                "y": 65.318
            }
        }
    }
]

# ============================================================

def build_mission(target_name, waypoints, next_action_type="hub"):
    """Tạo dict mission theo format của moving_control_server."""
    mission = {
        "name": "Move to position {}".format(target_name),
        "next_action_type": next_action_type,
        "param_test": {
            "need_set_vel_before_move": True,
            "need_to_wait_receive_new_path": True
        },
        "params": {
            "position": target_name
        },
        "type": "move",
        "waypoints": waypoints
    }
    return mission


def send_mission():
    rospy.init_node("send_move_mission_node", anonymous=True)

    action_server = "/moving_control"
    rospy.loginfo("Waiting for action server: {}".format(action_server))

    client = actionlib.SimpleActionClient(action_server, StringAction)
    connected = client.wait_for_server(timeout=rospy.Duration(10.0))
    if not connected:
        rospy.logerr("Cannot connect to action server: {}".format(action_server))
        return

    rospy.loginfo("Connected to action server.")

    # Build mission
    mission_dict = build_mission(TARGET_POSITION_NAME, WAYPOINTS)
    rospy.loginfo("Mission:\n{}".format(json.dumps(mission_dict, indent=2, ensure_ascii=False)))

    # Send goal
    goal = StringGoal()
    goal.data = json.dumps(mission_dict, ensure_ascii=False)

    rospy.loginfo("Sending goal to {}...".format(action_server))
    client.send_goal(goal)

    # Wait for result
    finished = client.wait_for_result(timeout=rospy.Duration(120.0))
    if finished:
        state = client.get_state()
        result = client.get_result()
        rospy.loginfo("Action finished. State: {}  Result: {}".format(state, result))
    else:
        rospy.logwarn("Action did not finish within timeout. Cancelling...")
        client.cancel_goal()


if __name__ == "__main__":
    try:
        send_mission()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted.")

