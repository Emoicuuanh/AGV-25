#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from cartographer_ros_msgs.srv import FinishTrajectory, StartTrajectory
from cartographer_ros_msgs.msg import SubmapList
import tf

class CartographerInitialPoseSetter:
    def __init__(self):
        rospy.init_node('cartographer_initialpose_setter')

        # Parameters
        self.config_dir = rospy.get_param('~configuration_directory', '/home/your_user/catkin_ws/src/your_cartographer_config/config')
        self.config_basename = rospy.get_param('~configuration_basename', 'your_config.lua')

        # Subscribers
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initialpose_callback)

        self.current_trajectory_id = None

        rospy.loginfo("Ready. Please set initial pose using RViz '2D Pose Estimate'.")
        rospy.spin()

    @staticmethod
    def pose_to_mat44(p: Pose):
        """Pose → ma trận biến đổi 4×4 (list of lists)."""
        trans = (p.position.x, p.position.y, p.position.z)
        quat  = (p.orientation.x, p.orientation.y,
                p.orientation.z, p.orientation.w)
        T = tf.transformations.quaternion_matrix(quat)        # 4×4
        T[0:3, 3] = trans
        return T

    @staticmethod
    def mat44_to_pose(T):
        """Ma trận 4×4 → Pose."""
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = T[0:3, 3]
        quat = tf.transformations.quaternion_from_matrix(T)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
        return pose

    def get_current_trajectory_id(self):
        try:
            # Wait for message on /submap_list topic
            msg = rospy.wait_for_message('/submap_list', SubmapList, timeout=5.0)
            trajectory_ids = [s.trajectory_id for s in msg.submap]

            if not trajectory_ids:
                rospy.logwarn("No active trajectory found. Defaulting to 0.")
                return 0

            latest_id = max(trajectory_ids)
            rospy.loginfo("Current active trajectory_id: %d", latest_id)
            return latest_id
        except rospy.ROSException as e:
            rospy.logerr("Failed to get /submap_list message: %s", str(e))
            return 0

    def get_data_transform_map(self):
        try:
            # Wait for message on /submap_list topic
            msg = rospy.wait_for_message('/submap_list', SubmapList, timeout=5.0)
            candidates = [s for s in msg.submap if s.trajectory_id == 0]
            if not candidates:
                return

            # Lấy submap_index nhỏ nhất
            first = min(candidates, key=lambda s: s.submap_index)
            self.submap_pose = first.pose  # geometry_msgs/Pose
            return self.pose_to_mat44(self.submap_pose)
        except:
            pass

    def initialpose_callback(self, msg):
        rospy.loginfo("Received initial pose from RViz.")

        T_submap = self.get_data_transform_map()
        T_init = self.pose_to_mat44(msg.pose.pose)

        # Biến đổi init sang hệ tọa độ submap:  T_rel = T_submap⁻¹ * T_init
        T_rel = tf.transformations.inverse_matrix(T_submap).dot(T_init)
        rel_pose = self.mat44_to_pose(T_rel)

        x = rel_pose.position.x
        y = rel_pose.position.y

        # Extract yaw angle from quaternion
        orientation_q = rel_pose.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]

        yaw_deg = yaw * 180.0 / 3.14159265

        rospy.loginfo("Initial pose: x=%.3f, y=%.3f, yaw=%.2f degrees", x, y, yaw_deg)

        # Step 1: Get current trajectory id
        self.current_trajectory_id = self.get_current_trajectory_id()

        # Step 2: Finish current trajectory
        self.finish_trajectory(self.current_trajectory_id)

        # Step 3: Start new trajectory with initial pose
        self.start_trajectory_with_initial_pose(x, y, yaw)

    def finish_trajectory(self, trajectory_id):
        rospy.wait_for_service('/finish_trajectory')
        try:
            finish_srv = rospy.ServiceProxy('/finish_trajectory', FinishTrajectory)
            resp = finish_srv(trajectory_id=trajectory_id)
            rospy.loginfo("Finished trajectory %d successfully.", trajectory_id)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call finish_trajectory: %s", str(e))

    def start_trajectory_with_initial_pose(self, x, y, yaw):
        rospy.wait_for_service('/start_trajectory')
        try:
            start_srv = rospy.ServiceProxy('/start_trajectory', StartTrajectory)

            from cartographer_ros_msgs.srv import StartTrajectoryRequest
            req = StartTrajectoryRequest()

            req.configuration_directory = self.config_dir
            req.configuration_basename = self.config_basename

            req.use_initial_pose = True
            req.relative_to_trajectory_id = 0

            quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)

            req.initial_pose.position.x = x
            req.initial_pose.position.y = y
            req.initial_pose.position.z = 0.0
            req.initial_pose.orientation.x = quaternion[0]
            req.initial_pose.orientation.y = quaternion[1]
            req.initial_pose.orientation.z = quaternion[2]
            req.initial_pose.orientation.w = quaternion[3]

            resp = start_srv(req)
            rospy.loginfo("Started new trajectory successfully.")
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call start_trajectory: %s", str(e))

if __name__ == '__main__':
    try:
        CartographerInitialPoseSetter()
    except rospy.ROSInterruptException:
        pass
