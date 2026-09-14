#!/usr/bin/env python3

import rospy
import yaml
import os
from nav_msgs.msg import Odometry
from tuw_multi_robot_msgs.msg import RobotInfo, RobotGoalsArray, RobotGoals
from geometry_msgs.msg import PoseWithCovariance

class RobotInfoPublisher:
    def __init__(self):
        rospy.init_node('robot_info_publisher', anonymous=True)

        # Load config
        self.robot_mapping = self.load_config()
        if not self.robot_mapping:
            rospy.logerr("No robot mapping found in config file. Shutting down node.")
            rospy.signal_shutdown("No robot mapping configuration")
            return

        # Publishers
        self.robot_info_pub = rospy.Publisher('/robot_info_receive_from_robot', RobotInfo, queue_size=10)
        self.goal_pub = rospy.Publisher('/goal_from_robot', RobotGoals, queue_size=10)

        # Create subscribers only for robots defined in config
        self.subscribers = {}
        self.setup_subscribers()

        # Subscribe to goals
        self.goals_sub = rospy.Subscriber('/goals', RobotGoalsArray, self.goals_callback)

        rospy.loginfo(f"Initialized with robots: {list(self.robot_mapping.keys())}")

    def load_config(self):
        # Get config file path from ROS param
        config_file = rospy.get_param('~config_file', 'config/robot_config.yaml')

        # If path is relative, make it absolute using package path
        if not os.path.isabs(config_file):
            package_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
            config_file = os.path.join(package_path, config_file)

        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                robot_mapping = config.get('robot_mapping', {})
                if robot_mapping:
                    rospy.loginfo(f"Loaded robot mapping from {config_file}: {robot_mapping}")
                else:
                    rospy.logwarn(f"No robot mapping found in {config_file}")
                return robot_mapping
        except Exception as e:
            rospy.logerr(f"Failed to load config file: {e}")
            return {}

    def setup_subscribers(self):
        """Setup subscribers for each robot defined in the config"""
        for robot_name in self.robot_mapping.keys():
            topic = f'/{robot_name}/odom'
            self.subscribers[robot_name] = rospy.Subscriber(
                topic,
                Odometry,
                self.odom_callback,
                callback_args=robot_name
            )
            rospy.loginfo(f"Subscribed to {topic}")

    def get_robot_number(self, robot_name):
        return self.robot_mapping.get(robot_name, '')

    def get_mapped_robot_name(self, original_name):
        # Extract robot number (assuming format like 'robot_1')
        for robot_key, mapped_number in self.robot_mapping.items():
            if robot_key in original_name:
                return f"AGV300_QR_{mapped_number}"
        return original_name

    def create_robot_info_msg(self, odom_msg, robot_name):
        robot_info = RobotInfo()

        # Header
        robot_info.header = odom_msg.header
        robot_info.header.frame_id = "map"

        # Robot name using mapping from config
        mapped_number = self.get_robot_number(robot_name)
        robot_info.robot_name = f"AGV300_QR_{mapped_number}"

        # Pose
        robot_info.pose = odom_msg.pose

        # Default values
        robot_info.shape = RobotInfo.SHAPE_RECTANGLE  # 1 for rectangle
        robot_info.layout_map = 0
        robot_info.shape_variables = [0.86, 0.63]  # Default size

        # Sync info
        robot_info.sync.robot_id = robot_info.robot_name
        robot_info.sync.current_route_segment = -1
        robot_info.sync.current_segment_id = 0

        # Status
        robot_info.mode = "AUTO"
        robot_info.status = "RUNNING"
        robot_info.detail_status = "GOING_TO_POS"
        robot_info.good_id = -2
        robot_info.order_id = 0
        robot_info.order_status = 0

        # Flags
        robot_info.auto_mode = True
        robot_info.pause = False
        robot_info.init_pose = True

        robot_info.direction_move = "FORWARD"
        robot_info.state_move = "ROTATING_CW"


        return robot_info

    def odom_callback(self, odom_msg, robot_name):
        robot_info = self.create_robot_info_msg(odom_msg, robot_name)
        self.robot_info_pub.publish(robot_info)
        rospy.logdebug(f"Published robot info for {robot_info.robot_name}")

    def goals_callback(self, goals_array):
        """Handle incoming RobotGoalsArray messages"""
        for robot_goals in goals_array.robots:
            original_name = robot_goals.robot_name

            # Check if this robot is in our config mapping
            is_robot_in_config = any(robot_key in original_name for robot_key in self.robot_mapping.keys())

            if is_robot_in_config:
                # Create a new RobotGoals message
                mapped_goals = RobotGoals()

                # Map the robot name
                mapped_goals.robot_name = self.get_mapped_robot_name(original_name)

                # Copy other fields
                mapped_goals.id = robot_goals.id
                mapped_goals.destinations = robot_goals.destinations

                # Publish the mapped goals
                self.goal_pub.publish(mapped_goals)
                rospy.loginfo(f"Remapped and published goals for {original_name} -> {mapped_goals.robot_name}")
            else:
                rospy.logdebug(f"Skipping goals for {original_name} - not in config")


    def shutdown(self):
        """Clean shutdown of the node"""
        for sub in self.subscribers.values():
            sub.unregister()
        rospy.loginfo("Shutting down robot_info_publisher node")

if __name__ == '__main__':
    try:
        robot_info_publisher = RobotInfoPublisher()
        rospy.on_shutdown(robot_info_publisher.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
