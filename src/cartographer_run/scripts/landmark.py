#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseArray, PointStamped, Pose
from visualization_msgs.msg import MarkerArray
from cartographer_ros_msgs.msg import LandmarkEntry, LandmarkList
import numpy as np

class LandmarkProcessor:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('landmark_processor', anonymous=True)
        
        # Parameters
        self.landmark_topic = rospy.get_param('~landmark_topic', '/mirror_point')
        self.carto_landmark_topic = rospy.get_param('~carto_landmark_topic', '/landmark_poses_list')
        self.output_topic = rospy.get_param('~output_topic', '/landmark')
        self.laser_frame = rospy.get_param('~laser_frame', 'laser')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.tolerance = rospy.get_param('~tolerance', 0.2)  # Tolerance for landmark uniqueness (meters)
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.landmarkListPub=LandmarkList()
        # Store known landmarks (from both Cartographer and local processing)
        self.known_landmarks = {}  # {id: (x, y)}
        self.next_id = 0  # For generating new IDs
        
        # Publishers and Subscribers
        self.pub = rospy.Publisher(self.output_topic, LandmarkList, queue_size=10)
        rospy.Subscriber(self.landmark_topic, PoseArray, self.landmark_callback)
        # rospy.Subscriber(self.carto_landmark_topic, MarkerArray, self.carto_landmark_callback)
        
        rospy.loginfo("Landmark Processor Node started")

    def carto_landmark_callback(self, msg):
        # Update known landmarks from Cartographer's MarkerArray
        for marker in msg.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            landmark_id = str(marker.id)
            # Only update if landmark_id is new or position differs significantly
            if landmark_id not in self.known_landmarks or \
               np.sqrt((x - self.known_landmarks[landmark_id][0])**2 + 
                       (y - self.known_landmarks[landmark_id][1])**2) > self.tolerance:
                self.known_landmarks[landmark_id] = (x, y)
            
            # Update next_id to avoid conflicts
            try:
                num_id = int(landmark_id)
                self.next_id = max(self.next_id, num_id + 1)
            except ValueError:
                pass  # Non-numeric IDs are ignored for next_id
        
        # rospy.loginfo(f"Updated {len(self.known_landmarks)} known landmarks from Cartographer")

    def landmark_callback(self, msg):
        try:
            # rospy.loginfo("CB")
            # Transform each pose from laser frame to map frame
            transform = self.tf_buffer.lookup_transform(self.map_frame, 
                                                      self.laser_frame, 
                                                      msg.header.stamp, 
                                                      rospy.Duration(1.0))
            
            # Create LandmarkList for publishing
            landmark_list = LandmarkList()
            landmark_list.header = msg.header
            landmark_list.header.frame_id = self.map_frame
            
            # Process each pose in PoseArray
            for pose in msg.poses:
                # Convert pose to PointStamped for transformation
                point_stamped = PointStamped()
                point_stamped.header = msg.header
                point_stamped.point.x = pose.position.x
                point_stamped.point.y = pose.position.y
                point_stamped.point.z = pose.position.z
                
                # Transform to map frame
                point_map = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
                
                # Extract position
                x, y = point_map.point.x, point_map.point.y
                
                # Get or assign landmark ID
                landmark_id = self.get_landmark_id(x, y)
                
                # Create LandmarkEntry
                landmark_entry = LandmarkEntry()
                landmark_entry.id = landmark_id
                # Set tracking_from_landmark_transform (Pose)
                landmark_entry.tracking_from_landmark_transform = Pose()
                landmark_entry.tracking_from_landmark_transform.position.x = x
                landmark_entry.tracking_from_landmark_transform.position.y = y
                landmark_entry.tracking_from_landmark_transform.position.z = 0.0
                # Set default orientation (no rotation)
                landmark_entry.tracking_from_landmark_transform.orientation.w = 1.0
                landmark_entry.translation_weight = 10000.0
                landmark_entry.rotation_weight = 10000.0  # Ignoring orientation for now
                
                # Add to LandmarkList
                landmark_list.landmarks.append(landmark_entry)
                
                # Store landmark locally to maintain ID consistency
                if landmark_id not in self.known_landmarks:
                    self.known_landmarks[landmark_id] = (x, y)
                
                # rospy.loginfo(f"Processed landmark ID {landmark_id} at ({x:.2f}, {y:.2f}) in laser frame")
            self.landmarkListPub=landmark_list
            # Publish to Cartographer
            # if landmark_list.landmarks:
            #     self.pub.publish(landmark_list)
            #     rospy.loginfo(f"Published {len(landmark_list.landmarks)} landmarks")
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF transform error: {e}")
            return

    def get_landmark_id(self, x, y):
        # Check if the position matches a known landmark
        for lm_id, pos in self.known_landmarks.items():
            px, py = pos
            if np.sqrt((x - px)**2 + (y - py)**2) < self.tolerance:
                return lm_id
        
        # New landmark, assign new ID
        new_id = str(self.next_id)
        self.next_id += 1
        return new_id

    def run(self):
        rate_=rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.landmarkListPub.landmarks:
                self.pub.publish(self.landmarkListPub)
                rospy.loginfo(f"total id:{self.next_id}")
                # rospy.loginfo(f"Published {len(self.landmarkListPub.landmarks)} landmarks")
            rate_.sleep()

            # rospy.spin()

if __name__ == '__main__':
    try:
        processor = LandmarkProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        pass