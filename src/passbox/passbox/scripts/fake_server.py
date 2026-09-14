#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Giả lập server gửi nhiệm vụ passbox xuống mission_manager.
Topic: /mission_from_server  (std_stamped_msgs/StringStamped)

Chạy:
    rosrun mission_manager fake_server_passbox.py
"""

import rospy
import json
from std_stamped_msgs.msg import StringStamped

def main():
    rospy.init_node("fake_server_passbox", anonymous=True)
    pub = rospy.Publisher("/mission_from_server", StringStamped, queue_size=1)

    rospy.sleep(1.0)  # Chờ publisher kết nối

    # ===================== DỮ LIỆU NHIỆM VỤ =====================
    mission = {
        "mission_queue_id": "fake_mission_001",
        "name": "Test passbox mission",
        "result": {
            "name": "Test passbox mission",
            "mission_queue_id": "fake_mission_001",
            "actions": [
                {
                    "name": "Take cart 1 from hub HA2005 at ceil 0",
                    "next_action_type": "move",
                    "type": "passbox_equal",
                    "params": {
                        "cart": "1",
                        "cell": 0,
                        "invert": True,
                        "lot": "",
                        "name": "HA2005",
                        "pick_or_place": True,       # True = lấy hàng, False = đặt hàng
                        "floor_equal": True,         # Passbox bằng sàn
                        "position": {
                            "ceil": 0,
                            "name": "passbox",
                            "x": 61.157922341209925,
                            "y": 69.89591496109745
                        },
                        "properties": {
                            "disable_check_cart": True,
                            "use_mirror": True,
                            "offset_x_lift" : -0.18,
                            "offset_y_lift" : 0.07
                        },
                       "waiting_position": {
                            "x": 0.20251828205611463,
                            "y": -0.018224944834614565
                        },
                        "inside_position": {
                                    "name": "3",
                                    "x": 61.12522224155607,
                                    "y": 68.31541014449412
                        },
                        "lift_position": {
                            "x": -1.40442354778104,
                            "y": 0.023622936706833353
                        }
                    }
                }
            ]
        }
    }
    # =============================================================

    msg = StringStamped()
    msg.stamp = rospy.Time.now()
    msg.data = json.dumps(mission)

    rospy.loginfo("Sending fake passbox mission...")
    rospy.loginfo("Data:\n{}".format(json.dumps(mission, indent=2, ensure_ascii=False)))

    pub.publish(msg)
    rospy.loginfo("Mission sent!")
    rospy.sleep(0.5)

if __name__ == "__main__":
    main()
