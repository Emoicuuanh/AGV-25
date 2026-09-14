#ifndef HEADERS_H
#define HEADERS_H

#include <mirror_detect/TFHelpers.h>
#include <mirror_detect/Helpers.h>
#include <mirror_detect/PCLHelpers.h>
#include <mirror_detect/PoseHelpers.h>
// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.

// mirror_detect ROS Messages
#include <mirror_detect/BoundingBox.h>
#include <mirror_detect/Cluster.h>
#include <mirror_detect/ClusterArray.h>
#include <mirror_detect/Dock.h>
#include <mirror_detect/Line.h>
#include <mirror_detect/LineArray.h>
#include <mirror_detect/MinMaxPoint.h>
#include <mirror_detect/Plan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
// #include <point_cloud2_iterator.h>
// mirror_detect ROS Actions
// #include <mirror_detect/mirror_detectAction.h>

// mirror_detect Classes
#include <mirror_detect/ICP.h>
#include <mirror_detect/Clustering.h>
#include <mirror_detect/PoseEstimation.h>
#include <mirror_detect/LineDetection.h>
#include <mirror_detect/ransac_circle.h>
// ROS Messages
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2/impl/utils.h>
#include <tf2/impl/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/cloud_iterator.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/pcl_base.h>
#include <pcl/segmentation/extract_clusters.h>
// #include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>

#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Dense>
#include <chrono>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>
#include <angles/angles.h>
#include <math.h>
#include <limits>
#include <sensor_msgs/LaserScan.h>
#endif // HEADERS_H
