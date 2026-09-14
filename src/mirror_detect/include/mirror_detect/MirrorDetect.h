// -*- mode: c++ -*-

#ifndef MIRRORDETECT_H_
#define MIRRORDETECT_H_

#include "mirror_detect/HubService.h"
#include <geometry_msgs/PoseStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <mirror_detect/Headers.h>
#include <mirror_detect/MirrorDetectNodeConfig.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>
#define LINE_ 0
#define CIRCLE_ 1
typedef pcl::PointCloud<pcl::PointXYZRGB> POINT;

// namespace mirror_detect {
struct LaserPoint {
  float range;
  float intensity;
  int original_index;
};
struct BoundaryPoints {
  double range;
  int index;
};
struct PoseData {
  geometry_msgs::PoseStamped pose;
  int index;
};
struct LaserCoord {
  float x;
  float y;
  int index;
};
bool compareByIndex(const LaserCoord &a, const LaserCoord &b) {
  return a.index < b.index;
}
double distance(const Point2D &p1, const Point2D &p2) {
  return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                   (p1.y - p2.y) * (p1.y - p2.y));
}

class MirrorDetect {
  ///////////////// BEGIN VARIABLES /////////////////
public:
  MirrorDetect(ros::NodeHandle nh) : nh_(nh), tfListener_(tfBuffer_) {
    startDynamicReconfigureServer();
    loadParams();
    service_ = nh_.advertiseService("hub_service",
                                    &MirrorDetect::handleHubService, this);
    startPub();
    initGlobals();
    // initDockParams();
    ros::Duration(1).sleep(); // Wait for pointcloud_to_laserscan node to init
                              // and publish cloud topic
    startCloudSub(cloud_topic_);
  }
  ~MirrorDetect() {}
  //=============================================================================================================================================================================================================================================================================================================================================================
  //
  //  ##   ##    ###    #####    ##    ###    #####   ##      #####
  //  ##   ##   ## ##   ##  ##   ##   ## ##   ##  ##  ##      ##
  //  ##   ##  ##   ##  #####    ##  ##   ##  #####   ##      #####
  //   ## ##   #######  ##  ##   ##  #######  ##  ##  ##      ##
  //    ###    ##   ##  ##   ##  ##  ##   ##  #####   ######  #####
  //
  //=============================================================================================================================================================================================================================================================================================================================================================
  ros::Publisher circle_marker_pub;
  ros::Publisher scan_mirror_pub;
  ros::Publisher mirror_point_pub;
  ros::Publisher center_point_pub;
  ros::Publisher circle_marker_arr_pub;
  ros::Publisher status_pub_;
  ros::Publisher clusters_cloud_pub_;
  ros::Publisher clusters_pub_;
  ros::Publisher lines_cloud_pub_;
  ros::Publisher lines_pub_;
  ros::Publisher line_marker_pub_;
  ros::Publisher line_segment_pub_;
  ros::Publisher dock_marker_pub_;
  ros::Publisher dock_pose_marker_pub_;
  ros::Publisher dock_pose_pub_;
  ros::Publisher bbox_pub_;       // Bounding box publisher
  ros::Publisher debug_pub_;      // debug publisher
  ros::Publisher icp_in_pub_;     // ICP Input Cloud publisher
  ros::Publisher icp_target_pub_; // ICP Target Cloud publisher
  ros::Publisher icp_out_pub_;    // ICP Output Cloud publisher
  ros::Subscriber cloudSub_;
  ros::Subscriber scanSub_;
  ros::Subscriber scanSub_2;
  ros::Subscriber scanSub_3;
  ros::Publisher marker_pub_intensity;
  ros::Publisher status_detect_mirror_pub_;
  ros::Subscriber selectTypeSub;
  ros::Publisher myPoint_min;
  ros::Publisher myPoint_max;
  ros::Publisher cluster_marker_pub;
  ros::Publisher cloud_pub;
  ros::Publisher scan_filter_pub;
  ros::Publisher scan_neighbour_pub;
  ros::Publisher expected_hub_pub;
  ros::NodeHandle nh_;

  ros::Time after_time123;
  std::vector<double> distance_range;
  std::vector<float> radius_tolerance;
  std::vector<int> intensity_range;
  YAML::Node config;
  std::string yaml_file_path_;
  ros::ServiceServer service_;
  sensor_msgs::LaserScan scan_msg_intensity;
  laser_geometry::LaserProjection projector_;
  sensor_msgs::LaserScan scan_msg_raw;
  //! Dynamic reconfigure server.
  dynamic_reconfigure::Server<mirror_detect::MirrorDetectNodeConfig> dr_srv_;
  sensor_msgs::PointCloud2 cloud_raw;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2_ros::TransformBroadcaster tfBroadcaster_;

  //! Target Dock PCL Cloud Pointer
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr dockTargetPCLPtr_;

  //! Global list of line markers for publisher
  visualization_msgs::Marker lines_marker_;
  //! Global list of line msgs for publisher
  mirror_detect::LineArray lines_;
  //! Global list of cluster msgs for publisher
  mirror_detect::LineArray::Ptr linesPtr_;
  //! Global list of cluster msgs for publisher
  mirror_detect::ClusterArray clusters_;
  //! Global list of cluster msgs for publisher
  mirror_detect::ClusterArray::Ptr clustersPtr_;
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::MarkerArray mirror_detect_arr;
  geometry_msgs::TransformStamped transformStamped;
  //! RANSAC Maximum Iterations

  int RS_max_iter_;
  bool publish_scan;
  double scan_resolution;
  //! RANSAC Minimum Inliers
  int RS_min_inliers_;
  //! RANSAC Distance Threshold
  double RS_dist_thresh_;
  //! Perform RANSAC after Clustering Points
  bool RANSAC_on_clusters_;
  int d_min = 99999;
  int RANSAC_iterations;
  //! EuclideanCluster Tolerance (m)
  double EC_cluster_tolerance_;
  //! EuclideanCluster Min Cluster Size
  int EC_min_size_;
  //! EuclideanCluster Max Cluster Size
  int EC_max_size_;
  int icp_type;
  double BAR_LENGTH_CONFIG;
  double BAR_length;
  double BAR_length_tolerance;
  double BAR_distance;
  double BAR_distance_tolerance;
  double length_tolerance;
  double Leg_length;
  double Leg_service;
  double Leg_distance;
  double Leg_tolerance;
  double Leg_offset;
  double laser_resolution;
  // Expected hub orientation in base_link frame (from service request)
  double expected_hub_orientation_ = std::numeric_limits<double>::quiet_NaN();
  // Sai so goc cho phep (rad). Nap tu param server / dynamic_reconfigure
  // qua key "orientation_tolerance" (don vi deg). Mac dinh 10 deg.
  double orientation_tolerance_ = M_PI / 18.0;

  ///
  double max_cluster_size;
  double max_intensity_time;
  double intensity_times;
  double radius_detect;
  int line_discard;
  int point_discard;
  int mirror_type;
  int detect_type_;
  double length_mirror;
  bool use_laser;
  std::vector<LaserPoint> minIntensityList;
  //! Bool of dock search status
  std_msgs::Bool found_dock_;
  //! Bool of detection node status
  std_msgs::Bool perform_detection_;
  //! String for select marker type
  std_msgs::String selectType;
  //! Bool of having printed detection node status
  bool printed_deactivation_status_;
  int mirror_point_publish;
  //! Dock Wing Length
  double dock_wing_length;
  bool use_diff_laser;
  //! Dock target template filepath
  std::string dockFilePath_;
  bool dock_template_loaded_;
  //! ICP Fitness Score Threshold
  double ICP_min_score_;
  //! ICP The maximum distance threshold between two correspondent points
  double ICP_max_correspondence_distance_;
  //! ICP Max iterations
  double ICP_max_iterations_;
  //! ICP Max transformation translation epsilon (diff) btwn previous & current
  //! estimated
  double ICP_max_transformation_eps_;
  //! ICP Max transformation rotation epsilon (diff) btwn previous & current
  //! estimated
  double ICP_max_transformation_rotation_eps_;
  //! ICP Max Euclidean squared errors
  double ICP_max_euclidean_fitness_eps_;

  //! Leaf Size for Voxel Grid
  double Voxel_leaf_size_;

  //! Name of world frame
  std::string world_frame_;
  //! Name of robot frame
  std::string robot_frame_;
  //! Name of cloud frame
  std::string cloud_frame_;
  //! Name of cloud topic
  std::string cloud_topic_;
  //! Name of laser frame
  std::string laser_frame_;
  //! Name of laser topic
  std::string laser_topic_;
  std::string scan_mirror_topic_;
  std_msgs::Header header_;

  geometry_msgs::TransformStamped
      base_link_to_leap_motion; // My frames are named "base_link" and
                                // "leap_motion"
  geometry_msgs::TransformStamped laser_to_base;
  geometry_msgs::TransformStamped base_to_laser;
  geometry_msgs::TransformStamped laser_to_base2;
  geometry_msgs::TransformStamped base_to_laser2;
  geometry_msgs::TransformStamped laser_to_base3;
  geometry_msgs::TransformStamped base_to_laser3;
  geometry_msgs::Pose poseHub;
  bool isGotTf = false;
  bool isGotTf2 = false;
  bool isGotTf3 = false;
  bool use_scan_merge = false;
  bool use_laser_1 = true;
  bool enable_detect = false;
  bool mirror_detected = false; // Track if mirror is detected

  ///////////////// END VARIABLES /////////////////

  //=========================================================================================================================================================================================================================================================================================================================
  //
  //  ##  ##     ##  ##  ######
  //  ##  ####   ##  ##    ##
  //  ##  ##  ## ##  ##    ##
  //  ##  ##    ###  ##    ##
  //  ##  ##     ##  ##    ##
  //
  //=========================================================================================================================================================================================================================================================================================================================

  void calcTf() {
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener(tf_buffer);
    base_link_to_leap_motion = tf_buffer.lookupTransform(
        robot_frame_, laser_frame_, ros::Time(0), ros::Duration(1.0));
    isGotTf = true;
  }

  // Declaration and Definition
  void startDynamicReconfigureServer() {
    // Set up a dynamic reconfigure server.
    // Do this before parameter server, else some of the parameter server values
    // can be overwritten.
    ROS_INFO_STREAM(
        "startDynamicReconfigureServer: STARTING DYNAMIC RECONFIGURE SERVER");
    dynamic_reconfigure::Server<
        mirror_detect::MirrorDetectNodeConfig>::CallbackType cb;
    cb = boost::bind(&MirrorDetect::configCallback, this, _1, _2);
    dr_srv_.setCallback(cb);
  }

  void initLineMarker() {
    lines_marker_.type = visualization_msgs::Marker::LINE_LIST;
    lines_marker_.header = header_;
    lines_marker_.ns = "mirror_detect";
    lines_marker_.id = 0;
  }

  void initDockParams() {
    //    std::string
    //    dockFilePath("/home/rwbot/dock_ws/src/mirror_detect/pcd/dock_cloud_perfect.ply");
    //    std::string dockFilePath("UNSPECIFIED");
    ROS_INFO_STREAM("initDockParams: INITIALIZING TARGET CLOUD FILE PATH");

    // bool dockFilepathExists = nh_.hasParam("dock_filepath");
    bool dockFilepathExists = dockFilePath_ == "" ? false : true;
  }

  void initGlobals() {
    found_dock_.data = false;
    mirror_detect::ClusterArray temp1 = mirror_detect::ClusterArray();
    mirror_detect::ClusterArray::Ptr tempPtr(new mirror_detect::ClusterArray());
    clustersPtr_ = tempPtr;
    *clustersPtr_ = temp1;

    mirror_detect::LineArray temp2 = mirror_detect::LineArray();
    mirror_detect::LineArray::Ptr tempPtr2(new mirror_detect::LineArray());
    linesPtr_ = tempPtr2;
    *linesPtr_ = temp2;
  }

  void startPub() {
    status_pub_ = nh_.advertise<std_msgs::Bool>("mirror_detect/found_dock", 1);

    myPoint_min = nh_.advertise<geometry_msgs::PointStamped>(
        "mirror_detect/point_min", 1);
    myPoint_max = nh_.advertise<geometry_msgs::PointStamped>(
        "mirror_detect/point_max", 1);

    clusters_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "mirror_detect/clustersCloud", 1);
    clusters_pub_ =
        nh_.advertise<mirror_detect::ClusterArray>("mirror_detect/clusters", 1);

    lines_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("mirror_detect/linesCloud", 1);
    lines_pub_ =
        nh_.advertise<mirror_detect::LineArray>("mirror_detect/lines", 1);
    line_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "mirror_detect/lines_marker", 1);

    dock_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "mirror_detect/dock_marker", 1);
    circle_marker_arr_pub = nh_.advertise<visualization_msgs::MarkerArray>(
        "mirror_detect/circle_arr", 1);
    circle_marker_pub =
        nh_.advertise<visualization_msgs::Marker>("mirror_detect/circle", 1);
    dock_pose_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "mirror_detect/dock_pose_marker", 1);
    dock_pose_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>("mirror_detect/dock_pose", 1);
    mirror_point_pub =
        nh_.advertise<geometry_msgs::PoseArray>("mirror_point", 1);
    center_point_pub =
        nh_.advertise<geometry_msgs::PoseArray>("center_point", 1);
    bbox_pub_ =
        nh_.advertise<mirror_detect::BoundingBox>("mirror_detect/bbox", 1);
    expected_hub_pub =
        nh_.advertise<geometry_msgs::PoseArray>("expected_hub_pose", 1);

    debug_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("mirror_detect/debugCloud", 1);
    scan_mirror_pub =
        nh_.advertise<sensor_msgs::PointCloud2>(scan_mirror_topic_, 1);
    icp_in_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
        "mirror_detect/icp_in_pub", 1);
    icp_target_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
        "mirror_detect/icp_target_pub", 1);
    icp_out_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
        "mirror_detect/icp_out_pub", 1);
    cluster_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>(
        "mirror_detect/marker_cluster", 0);
    scan_filter_pub = nh_.advertise<sensor_msgs::LaserScan>("scan_filter", 1);
    scan_neighbour_pub =
        nh_.advertise<sensor_msgs::LaserScan>("scan_neighbour", 1);
    marker_pub_intensity =
        nh_.advertise<visualization_msgs::Marker>("intensity_debug_marker", 10);
    status_detect_mirror_pub_ =
        nh_.advertise<std_msgs::String>("status_detect_mirror", 1);
    // cloud_pub =
    // nh_.advertise<sensor_msgs::PointCloud2>("merged_cluster_pointcloud", 1);
  }

  void startCloudSub(std::string cloud_topic) {
    //  std::string ns_cloud_topic = "/" + cloud_topic;
    // if(!checkTopicExists(cloud_topic)){
    //   // ROS_WARN_STREAM("Namespaced Topic " + ns_cloud_topic + " does not
    //   exist"); ROS_WARN_STREAM("Topic " + cloud_topic + " does not exist");
    //   ROS_WARN_STREAM("Check to see if the topic is correct or if it is
    //   namespaced to continue"); return;
    // }
    ROS_INFO_STREAM("Subscribing to new cloud topic " + cloud_topic_);
    if (!use_laser)
      cloudSub_ =
          nh_.subscribe(cloud_topic, 1, &MirrorDetect::cloudCallback, this);
    else {
      scanSub_ =
          nh_.subscribe(cloud_topic, 1, &MirrorDetect::scanCallback, this);
      scanSub_2 =
          nh_.subscribe("scan_2", 1, &MirrorDetect::scanCallback2, this);
      scanSub_3 =
          nh_.subscribe("scan_map", 1, &MirrorDetect::scanCallback3, this);
      config = YAML::LoadFile(yaml_file_path_);
      for (std::size_t i = 0; i < config["points"].size(); ++i) {
        if (!config["points"][i]["distance"] ||
            !config["points"][i]["intensity_discard"]) {
          ROS_WARN("Point %zu dont have 'distance' or 'intensity_discard'. in "
                   "point:",
                   i);
          continue;
        }

        double distance = config["points"][i]["distance"].as<double>();
        int intensity_discard_i =
            config["points"][i]["intensity_discard"].as<int>();
        float radius_tolerance_i =
            config["points"][i]["radius_tolerance"].as<float>();
        // ROS_INFO("dis %d :%f, inten %d
        // :%d",i,distance,i,intensity_discard_i);
        distance_range.push_back(distance);
        intensity_range.push_back(intensity_discard_i);
        radius_tolerance.push_back(radius_tolerance_i);
      }
    }
    if (cloudSub_) {
      ROS_INFO_STREAM("SUCCESSFULLY subscribed to new cloud topic " +
                      cloud_topic);
    } else {
      ROS_WARN_STREAM("FAILED to subscribe to new cloud topic " + cloud_topic);
    }
  }

  bool checkTopicExists(std::string &topic) {
    ROS_INFO_STREAM("Checking existence of new cloud topic " + topic);
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);
    for (int i = 0; i < topic_infos.size(); i++) {
      ROS_INFO_STREAM("Check topic #" << i << "  " << topic_infos.at(i).name);
      // if(topic == topic_infos.at(i).name)
      if (topic_infos.at(i).name.find(topic) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  void clearGlobals() {
    lines_.lines.clear();
    lines_marker_.points.clear();
    lines_marker_.colors.clear();
    clustersPtr_->clusters.clear();
    linesPtr_->lines.clear();
    marker_array.markers.clear();

    //    linesPtr_->segments.clear();
  }
  bool getLaserTobase2(std::string frame_id) // tf from laser to base
  {
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener(tf_buffer);
    try {
      laser_to_base2 = tf_buffer.lookupTransform(
          "base_link", frame_id, ros::Time(0), ros::Duration(10.0));
      tf2::Transform tf2_laser_to_base;
      tf2::fromMsg(laser_to_base2.transform, tf2_laser_to_base);

      // Invert the transform to get base_to_laser
      tf2::Transform tf2_base_to_laser = tf2_laser_to_base.inverse();

      // Convert back to geometry_msgs::TransformStamped
      base_to_laser2.header = laser_to_base2.header;
      base_to_laser2.header.frame_id =
          laser_to_base2.child_frame_id; // Now laser is the parent
      base_to_laser2.child_frame_id = laser_to_base2.header.frame_id;
      base_to_laser2.transform = tf2::toMsg(tf2_base_to_laser);
      ROS_INFO("LASER_TO_BASE2 COORD X:%f y:%f",
               laser_to_base2.transform.translation.x,
               laser_to_base2.transform.translation.y);
    } catch (const tf2::TransformException &ex) {
      ROS_WARN("Could not get initial transform from base to laser2 frame, %s",
               ex.what());
      ROS_WARN("base_frame: base_link -- frame_laser:%s", frame_id.c_str());
      ROS_ERROR("TRANSFORM ERROR SHUTDOWN");
      return false;
    }
    isGotTf2 = true;
    return true;
  }
  bool getLaserTobase(
      std::string frame_id, geometry_msgs::TransformStamped &B_TO_L,
      geometry_msgs::TransformStamped &L_TO_B) // tf from laser to base
  {
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener(tf_buffer);
    try {
      L_TO_B = tf_buffer.lookupTransform("base_link", frame_id, ros::Time(0),
                                         ros::Duration(10.0));
      tf2::Transform tf2_laser_to_base;
      tf2::fromMsg(L_TO_B.transform, tf2_laser_to_base);

      // Invert the transform to get base_to_laser
      tf2::Transform tf2_base_to_laser = tf2_laser_to_base.inverse();

      // Convert back to geometry_msgs::TransformStamped
      B_TO_L.header = L_TO_B.header;
      B_TO_L.header.frame_id = L_TO_B.child_frame_id; // Now laser is the parent
      B_TO_L.child_frame_id = L_TO_B.header.frame_id;
      B_TO_L.transform = tf2::toMsg(tf2_base_to_laser);
      ROS_INFO("LASER_TO_BASE COORD X:%f y:%f", L_TO_B.transform.translation.x,
               L_TO_B.transform.translation.y);
    } catch (const tf2::TransformException &ex) {
      ROS_WARN("Could not get initial transform from base to laser frame, %s",
               ex.what());
      ROS_WARN("base_frame: base_link -- frame_laser:%s", frame_id.c_str());
      ROS_ERROR("TRANSFORM ERROR SHUTDOWN");
      return false;
    }
    // isGotTF = true;
    return true;
  }
  geometry_msgs::PoseStamped getBaseLinkCoord(geometry_msgs::Pose pose_laser) {
    geometry_msgs::PoseStamped center_pose_in;
    center_pose_in.header.frame_id = laser_frame_;
    center_pose_in.header.stamp = ros::Time::now();
    center_pose_in.pose = pose_laser;
    geometry_msgs::PoseStamped center_pose_out;
    try {
      if (use_scan_merge)
        tf2::doTransform(center_pose_in, center_pose_out, laser_to_base3);
      else {
        if (use_laser_1)
          tf2::doTransform(center_pose_in, center_pose_out, laser_to_base);
        else
          tf2::doTransform(center_pose_in, center_pose_out, laser_to_base2);
      }
      // pose_arr.poses.push_back(center_pose_out.pose);
    } catch (tf2::TransformException &ex) {
      ROS_ERROR("Transform failed to base_link: %s", ex.what());
      // return;
    }
    return center_pose_out;
  }
  // Log tam hub (center_hub) trong frame odom, de doi chieu voi phia
  // hub_server/matehan_server (von lookup odom -> center_hub)
  void logHubPoseInOdom(const geometry_msgs::Pose &pose_base) {
    geometry_msgs::PoseStamped pose_in_base;
    pose_in_base.header.frame_id = "base_link";
    pose_in_base.header.stamp = ros::Time(0);
    pose_in_base.pose = pose_base;
    try {
      geometry_msgs::PoseStamped pose_in_odom =
          tfBuffer_.transform(pose_in_base, "odom", ros::Duration(0.2));
      double yaw = 2.0 * std::atan2(pose_in_odom.pose.orientation.z,
                                    pose_in_odom.pose.orientation.w);
      ROS_INFO("HUB(odom) x:%f y:%f yaw:%f deg", pose_in_odom.pose.position.x,
               pose_in_odom.pose.position.y, yaw * (180.0 / M_PI));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Cannot log center_hub in odom: %s", ex.what());
    }
  }
  geometry_msgs::PoseStamped baseToLaser(geometry_msgs::Pose pose_base) {
    geometry_msgs::PoseStamped center_pose_in;
    center_pose_in.header.frame_id = "base_link";
    center_pose_in.header.stamp = ros::Time::now();
    center_pose_in.pose = pose_base;
    geometry_msgs::PoseStamped center_pose_out;
    try {
      if (use_scan_merge)
        tf2::doTransform(center_pose_in, center_pose_out, base_to_laser3);
      else {
        if (use_laser_1)
          tf2::doTransform(center_pose_in, center_pose_out, base_to_laser);
        else
          tf2::doTransform(center_pose_in, center_pose_out, base_to_laser2);
      }
      // pose_arr.poses.push_back(center_pose_out.pose);
    } catch (tf2::TransformException &ex) {
      ROS_ERROR("Transform failed to base_link: %s", ex.what());
      // return;
    }
    return center_pose_out;
  }
  //==============================================================================================================================================================================================================================================================================================================================================================
  //
  //
  //
  //
  //
  //
  //   ####    ###    ##      ##      #####     ###     ####  ##  ##
  //  ##      ## ##   ##      ##      ##  ##   ## ##   ##     ## ##
  //  ##     ##   ##  ##      ##      #####   ##   ##  ##     ####
  //  ##     #######  ##      ##      ##  ##  #######  ##     ## ##
  //   ####  ##   ##  ######  ######  #####   ##   ##   ####  ##  ##
  //
  //
  //
  //
  //
  //
  //==============================================================================================================================================================================================================================================================================================================================================================

  bool handleHubService(mirror_detect::HubService::Request &req,
                        mirror_detect::HubService::Response &res) {
    // Process the incoming pose_target
    ROS_INFO("Received pose_target:");
    ROS_INFO("Position: x=%f, y=%f, z=%f", req.pose_target.position.x,
             req.pose_target.position.y, req.pose_target.position.z);
    ROS_INFO("Orientation: x=%f, y=%f, z=%f, w=%f",
             req.pose_target.orientation.x, req.pose_target.orientation.y,
             req.pose_target.orientation.z, req.pose_target.orientation.w);
    ROS_INFO("Enable Detect: %s", req.enable_detect ? "true" : "false");
    ROS_INFO("Orientation (map frame): %f deg",
             req.orientation * (180.0 / M_PI));
    // Convert expected orientation from map frame to base_link frame
    try {
      geometry_msgs::TransformStamped tf_map_base = tfBuffer_.lookupTransform(
          "map", "base_link", ros::Time(0), ros::Duration(1.0));
      double robot_yaw = 2.0 * std::atan2(tf_map_base.transform.rotation.z,
                                          tf_map_base.transform.rotation.w);
      expected_hub_orientation_ = req.orientation - robot_yaw;
      // Normalize to [-pi, pi]
      while (expected_hub_orientation_ > M_PI)
        expected_hub_orientation_ -= 2 * M_PI;
      while (expected_hub_orientation_ < -M_PI)
        expected_hub_orientation_ += 2 * M_PI;
      ROS_INFO("Expected hub orientation base_link: %f deg (robot_yaw=%f deg)",
               expected_hub_orientation_ * (180.0 / M_PI),
               robot_yaw * (180.0 / M_PI));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Cannot get robot yaw for orientation filter: %s", ex.what());
      expected_hub_orientation_ = std::numeric_limits<double>::quiet_NaN();
    }

    geometry_msgs::PoseStamped pose_in_map;
    pose_in_map.header.frame_id = "map";
    pose_in_map.header.stamp =
        ros::Time::now();               // Use current time for TF lookup
    pose_in_map.pose = req.pose_target; // Assign the input Pose
    geometry_msgs::PoseStamped pose_in_base_link;
    try {
      // Transform the pose from "map" to "base_link"
      pose_in_base_link =
          tfBuffer_.transform(pose_in_map, "base_link", ros::Duration(1.0));
      poseHub = pose_in_base_link.pose;
      // Log the transformed pose
      ROS_INFO("Position in base_link: x=%f, y=%f, z=%f", poseHub.position.x,
               poseHub.position.y, poseHub.position.z);
    } catch (tf2::TransformException &ex) {
      enable_detect = false;
      ROS_WARN("Failed to transform pose from map to base_link: %s", ex.what());
      return false; // Handle the error as needed
    }
    enable_detect = req.enable_detect;
    Leg_length = req.length;
    use_scan_merge = req.use_scan_merge;
    static bool first_service = false;
    if (!first_service) {
      BAR_LENGTH_CONFIG = BAR_length;
      first_service = true;
    }
    if (Leg_length > 1.0) {
      BAR_length = BAR_LENGTH_CONFIG * 1.67;
    } else {
      BAR_length = BAR_LENGTH_CONFIG;
    }
    // Publish expected pose to RViz for debugging
    geometry_msgs::PoseArray expected_arr;
    expected_arr.header.frame_id = "base_link";
    expected_arr.header.stamp = ros::Time::now();

    geometry_msgs::Pose p = poseHub;
    if (!std::isnan(expected_hub_orientation_)) {
      p.orientation.x = 0.0;
      p.orientation.y = 0.0;
      p.orientation.z = std::sin(expected_hub_orientation_ / 2.0);
      p.orientation.w = std::cos(expected_hub_orientation_ / 2.0);
    }
    expected_arr.poses.push_back(p);
    expected_hub_pub.publish(expected_arr);

    // Set response
    res.success = true;
    return true;
  }
  //==================================================================================================================================================================================================================================================================================================================================================================================================================================================
  //
  //    ###     ####  ######  ##  ##   ##    ###    ######  ##   #####   ## ##
  //    ####    ###    ##      ##      #####     ###     ####  ##  ##
  //   ## ##   ##       ##    ##  ##   ##   ## ##     ##    ##  ##   ##  #### ##
  //   ##      ## ##   ##      ##      ##  ##   ## ##   ##     ## ##
  //  ##   ##  ##       ##    ##  ##   ##  ##   ##    ##    ##  ##   ##  ##  ##
  //  ##        ##     ##   ##  ##      ##      #####   ##   ##  ##     ####
  //  #######  ##       ##    ##   ## ##   #######    ##    ##  ##   ##  ## ###
  //  ##     #######  ##      ##      ##  ##  #######  ##     ## ##
  //  ##   ##   ####    ##    ##    ###    ##   ##    ##    ##   #####   ## ##
  //  ####  ##   ##  ######  ######  #####   ##   ##   ####  ##  ##
  //
  //==================================================================================================================================================================================================================================================================================================================================================================================================================================================

  //=======================================================================================================================================================================================================================================================================================================================================================================================================================================================
  //
  //   ####  #####  ##      #####   ####  ######        ######  ##    ##  #####
  //   #####         ####    ###    ##      ##      #####     ###     ####  ##
  //   ##
  //  ##     ##     ##      ##     ##       ##            ##     ##  ##   ##  ##
  //  ##           ##      ## ##   ##      ##      ##  ##   ## ##   ##     ## ##
  //   ###   #####  ##      #####  ##       ##            ##      ####    #####
  //   #####        ##     ##   ##  ##      ##      #####   ##   ##  ##     ####
  //     ##  ##     ##      ##     ##       ##            ##       ##     ## ##
  //     ##     #######  ##      ##      ##  ##  #######  ##     ## ##
  //  ####   #####  ######  #####   ####    ##            ##       ##     ##
  //  #####         ####  ##   ##  ######  ######  #####   ##   ##   ####  ## ##
  //
  //=======================================================================================================================================================================================================================================================================================================================================================================================================================================================

  //===================================================================================================================================================================================================================================================================================================================================================================================================================
  //
  //   ####   #####   ##     ##  #####  ##   ####           ####    ###    ## ##
  //   #####     ###     ####  ##  ##
  //  ##     ##   ##  ####   ##  ##     ##  ##             ##      ## ##   ## ##
  //  ##  ##   ## ##   ##     ## ##
  //  ##     ##   ##  ##  ## ##  #####  ##  ##  ###        ##     ##   ##  ## ##
  //  #####   ##   ##  ##     ####
  //  ##     ##   ##  ##    ###  ##     ##  ##   ##        ##     #######  ## ##
  //  ##  ##  #######  ##     ## ##
  //   ####   #####   ##     ##  ##     ##   ####           ####  ##   ## ######
  //   ######  #####   ##   ##   ####  ##  ##
  //
  //===================================================================================================================================================================================================================================================================================================================================================================================================================

  //! Callback function for dynamic reconfigure server.

  void configCallback(mirror_detect::MirrorDetectNodeConfig &config,
                      uint32_t level __attribute__((unused))) {
    ROS_INFO_STREAM("Config");

    world_frame_ = config.world_frame;
    robot_frame_ = config.robot_frame;
    cloud_frame_ = config.cloud_frame;
    cloud_topic_ = config.cloud_topic;
    laser_frame_ = config.laser_frame;
    laser_topic_ = config.laser_topic;

    RS_max_iter_ = config.RS_max_iter;
    RS_min_inliers_ = config.RS_min_inliers;
    RS_dist_thresh_ = config.RS_dist_thresh;
    RANSAC_on_clusters_ = config.RANSAC_on_clusters;
    Voxel_leaf_size_ = config.Voxel_leaf_size;
    EC_cluster_tolerance_ = config.EC_cluster_tolerance;
    EC_min_size_ = config.EC_min_size;
    EC_max_size_ = config.EC_max_size;

    ICP_min_score_ = config.ICP_min_score;
    ICP_max_correspondence_distance_ = config.ICP_max_corres_dist;
    ICP_max_iterations_ = config.ICP_max_iter;
    ICP_max_transformation_eps_ = config.ICP_max_transl_eps;
    ICP_max_transformation_rotation_eps_ = config.ICP_max_rot_eps;
    ICP_max_euclidean_fitness_eps_ = config.ICP_max_eucl_fit_eps;

    BAR_length = config.BAR_length;
    BAR_length_tolerance = config.BAR_length_tolerance;
    BAR_distance = config.BAR_distance;
    BAR_distance_tolerance = config.BAR_distance_tolerance;

    detect_type_ = config.detect_type;
    radius_detect = config.radius_detect;
    intensity_times = config.intensity_times;
    point_discard = config.point_discard;
    Leg_length = config.Leg_length;
    Leg_distance = config.Leg_distance;
    Leg_offset = config.Leg_offset;
    Leg_tolerance = config.Leg_tolerance;
    orientation_tolerance_ = config.orientation_tolerance * (M_PI / 180.0);
    line_discard = config.line_discard;
    publish_scan = config.publish_scan;
    scan_mirror_topic_ = config.scan_mirror_topic_;
    mirror_type = config.mirror_type;
    length_mirror = config.length_mirror;
    length_tolerance = config.length_tolerance;
    icp_type = config.icp_type;
    use_laser = config.use_laser;
    yaml_file_path_ = config.intensity_range;
    mirror_point_publish = config.mirror_point_publish;
    max_intensity_time = config.max_intensity_time;
    max_cluster_size = config.max_cluster_size;
    use_diff_laser = config.use_diff_laser;
    laser_resolution = config.laser_resolution;
    if ((config.cloud_topic != "") && (config.cloud_topic != cloud_topic_)) {
      cloud_topic_ = config.cloud_topic;
      ROS_INFO_STREAM("configCallback: New Input Cloud Topic");
      startCloudSub(cloud_topic_);
    }
  }

  void loadParams() {
    std::string tempFilePath;
    ros::NodeHandle nhh("~");

    nhh.param("use_laser", use_laser);
    nhh.param("intensity_range", yaml_file_path_);
    nhh.param("mirror_point_publish", mirror_point_publish);
    nhh.param("max_cluster_size", max_cluster_size);
    nhh.param("max_intensity_time", max_intensity_time);
    nhh.param("use_diff_laser", use_diff_laser);
    nhh.param("laser_resolution", laser_resolution);
    // Sai so goc (deg) tu param server -> doi sang rad
    double orientation_tolerance_deg = orientation_tolerance_ * (180.0 / M_PI);
    nhh.param("orientation_tolerance", orientation_tolerance_deg,
              orientation_tolerance_deg);
    orientation_tolerance_ = orientation_tolerance_deg * (M_PI / 180.0);
    ROS_INFO("use_diff_laser:%d yaml_file:%s mirror_point:%d "
             "max_intensity_time:%f, laser_resolution:%f max_cluster_size:%f "
             "orientation_tolerance:%f deg",
             use_diff_laser, yaml_file_path_.c_str(), mirror_point_publish,
             max_intensity_time, laser_resolution, max_cluster_size,
             orientation_tolerance_deg);
  }

  void scanCallback(const sensor_msgs::LaserScanConstPtr &msg) {
    if (!use_laser_1 || use_scan_merge)
      return;
    sensor_msgs::PointCloud2Ptr scan_cloud;
    sensor_msgs::LaserScan scan_msg_new = *msg;
    scan_msg_raw = *msg;
    scan_resolution = scan_msg_new.angle_increment;
    // ROS_INFO("min_angle:%f
    // max_angle:%f",scan_msg_new.angle_min,scan_msg_new.angle_max);
    std_msgs::Header scan_header = scan_msg_new.header;
    laser_frame_ = scan_msg_new.header.frame_id;
    scan_msg_new.ranges.assign(scan_msg_new.ranges.size(),
                               std::numeric_limits<float>::quiet_NaN());
    if (!isGotTf) {
      if (getLaserTobase(laser_frame_, base_to_laser, laser_to_base)) {
        isGotTf = true;
      } else {
        ROS_ERROR("DONT HAVE TF FROM BASE TO LASER_FRAME:%s",
                  laser_frame_.c_str());
        return;
      }
    }
    if (!enable_detect && icp_type == 2) {
      ROS_WARN_THROTTLE(
          2.0, "[scanCallback] enable_detect=false, skipping detection");
      return;
    }
    for (size_t i = 0; i < scan_msg_new.intensities.size(); ++i) {

      double dis_now = scan_msg_raw.ranges[i];
      float intensity_now = scan_msg_new.intensities[i];
      float angle_now = scan_msg_new.angle_min + i * scan_resolution;
      if (use_diff_laser) {
        if (angle_now < 1.5713)
          scan_msg_new.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        continue;
      }
      for (int s = 1; s < distance_range.size(); s++) {

        if (dis_now > distance_range[s])
          continue;
        if (dis_now < distance_range[s] && dis_now > distance_range[s - 1]) {
          // if(scan_msg_new.intensities[i]>maxInteninRange[s])maxInteninRange[s]=scan_msg_new.intensities[i];
          // else
          // if(scan_msg_new.intensities[i]<minInteninRange[s])minInteninRange[s]=scan_msg_new.intensities[i];
          // if(maxInteninRange[s] !=0 && maxInteninRange[s]
          // !=9999)intensity_range[s]=(maxInteninRange[s]+minInteninRange[s])/2.0;
          // if(scan_msg_new.ranges[i]<10)scan_msg_new.intensities[i]=scan_msg_new.intensities[i]*(scan_msg_new.ranges[i]*scan_msg_new.ranges[i]);
          if (intensity_now > intensity_range[s - 1]) {
            scan_msg_new.ranges[i] = scan_msg_raw.ranges[i];
            break;
          }
        }
      }
    }

    int start_clus_index = 0;
    float initial_intensity = 0;
    float slide_dis = 0;
    bool in_high_intensity = false;
    int high_intensity_count = 0;
    std::vector<int> intensity_clus;
    std::vector<int> intensity_clus_debug;
    int size_last = scan_msg_new.intensities.size();
    int size_count_erase = 0;

    scan_filter_pub.publish(scan_msg_new);
    // scan_msg_intensity=scan_msg_new;

    scan_cloud.reset(new sensor_msgs::PointCloud2);

    projector_.projectLaser(scan_msg_new, *scan_cloud);
    // projector_.projectLaser(scan_msg_raw, cloud_raw);
    // scan_mirror_pub.publish(cloud_raw);
    cloudMirror(scan_cloud, scan_header);
    // cloudCallback(scan_cloud);
  }
  void scanCallback2(const sensor_msgs::LaserScanConstPtr &msg) {
    if (use_laser_1 || use_scan_merge)
      return;
    sensor_msgs::PointCloud2Ptr scan_cloud;
    sensor_msgs::LaserScan scan_msg_new = *msg;
    scan_msg_raw = *msg;
    scan_resolution = scan_msg_new.angle_increment;
    // ROS_INFO("min_angle:%f
    // max_angle:%f",scan_msg_new.angle_min,scan_msg_new.angle_max);
    std_msgs::Header scan_header = scan_msg_new.header;
    laser_frame_ = scan_msg_new.header.frame_id;
    scan_msg_new.ranges.assign(scan_msg_new.ranges.size(),
                               std::numeric_limits<float>::quiet_NaN());
    if (!isGotTf2) {
      if (getLaserTobase(laser_frame_, base_to_laser2, laser_to_base2)) {
        isGotTf2 = true;
      } else {
        ROS_ERROR("DONT HAVE TF FROM BASE TO LASER_FRAME:%s",
                  laser_frame_.c_str());
        return;
      }
    }
    if (!enable_detect && icp_type == 2) {
      ROS_WARN_THROTTLE(
          2.0, "[scanCallback2] enable_detect=false, skipping detection");
      return;
    }
    for (size_t i = 0; i < scan_msg_new.intensities.size(); ++i) {

      double dis_now = scan_msg_raw.ranges[i];
      float intensity_now = scan_msg_new.intensities[i];
      float angle_now = scan_msg_new.angle_min + i * scan_resolution;
      if (use_diff_laser) {
        if (angle_now < 1.5713)
          scan_msg_new.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        continue;
      }
      for (int s = 1; s < distance_range.size(); s++) {

        if (dis_now > distance_range[s])
          continue;
        if (dis_now < distance_range[s] && dis_now > distance_range[s - 1]) {
          // if(scan_msg_new.intensities[i]>maxInteninRange[s])maxInteninRange[s]=scan_msg_new.intensities[i];
          // else
          // if(scan_msg_new.intensities[i]<minInteninRange[s])minInteninRange[s]=scan_msg_new.intensities[i];
          // if(maxInteninRange[s] !=0 && maxInteninRange[s]
          // !=9999)intensity_range[s]=(maxInteninRange[s]+minInteninRange[s])/2.0;
          // if(scan_msg_new.ranges[i]<10)scan_msg_new.intensities[i]=scan_msg_new.intensities[i]*(scan_msg_new.ranges[i]*scan_msg_new.ranges[i]);
          if (intensity_now > intensity_range[s - 1]) {
            scan_msg_new.ranges[i] = scan_msg_raw.ranges[i];
            break;
          }
        }
      }
    }

    int start_clus_index = 0;
    float initial_intensity = 0;
    float slide_dis = 0;
    bool in_high_intensity = false;
    int high_intensity_count = 0;
    std::vector<int> intensity_clus;
    std::vector<int> intensity_clus_debug;
    int size_last = scan_msg_new.intensities.size();
    int size_count_erase = 0;

    scan_filter_pub.publish(scan_msg_new);
    // scan_msg_intensity=scan_msg_new;

    scan_cloud.reset(new sensor_msgs::PointCloud2);

    projector_.projectLaser(scan_msg_new, *scan_cloud);
    // projector_.projectLaser(scan_msg_raw, cloud_raw);
    // scan_mirror_pub.publish(cloud_raw);
    cloudMirror(scan_cloud, scan_header);
    // cloudCallback(scan_cloud);
  }
  sensor_msgs::LaserScan
  ClusterToScan(std::vector<mirror_detect::Cluster> clusters) {
    sensor_msgs::LaserScan scan_clusters = scan_msg_raw;
    scan_clusters.ranges.assign(scan_clusters.ranges.size(),
                                std::numeric_limits<float>::quiet_NaN());
    scan_clusters.intensities.assign(scan_clusters.intensities.size(),
                                     std::numeric_limits<float>::quiet_NaN());
    std::vector<BoundaryPoints> boundList;
    for (const auto &cluster : clusters) {
      if (cluster.points.indices.empty()) {
        continue;
      }
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(cluster.cloud, cloud);
      for (const auto &point : cloud) {

        double range_sq = pow(point.y, 2) + pow(point.x, 2);
        float angle = std::atan2(point.y, point.x);
        int bin_index = static_cast<int>((angle - scan_clusters.angle_min) /
                                         scan_clusters.angle_increment);
        BoundaryPoints newPoint;
        newPoint.index = bin_index;
        newPoint.range = range_sq;
        boundList.push_back(newPoint);
        // if (bin_index >= 0 && bin_index < scan_clusters.ranges.size())
        // {
        //   scan_clusters.ranges[bin_index] = sqrt(range_sq);
        // }
      }
    }
    std::sort(boundList.begin(), boundList.end(),
              [](const BoundaryPoints &a, const BoundaryPoints &b) {
                return a.index < b.index;
              });
    scan_clusters.ranges[boundList[0].index] = sqrt(boundList[0].range);
    scan_clusters.ranges[boundList[boundList.size() - 1].index] =
        sqrt(boundList[boundList.size() - 1].range);
    for (int i = 0; i < boundList.size() - 1; i++) {
      if (abs(boundList[i].index - boundList[i + 1].index) > 2) {
        scan_clusters.ranges[boundList[i].index] = sqrt(boundList[i].range);
        scan_clusters.ranges[boundList[i + 1].index] =
            sqrt(boundList[i + 1].range);
      }
    }
    return scan_clusters;
  }
  void scanCallback3(const sensor_msgs::LaserScanConstPtr &msg) {
    if (!use_scan_merge)
      return;
    sensor_msgs::PointCloud2Ptr scan_cloud;
    sensor_msgs::LaserScan scan_msg_new = *msg;
    scan_msg_raw = *msg;
    scan_resolution = scan_msg_new.angle_increment;
    // ROS_INFO("min_angle:%f
    // max_angle:%f",scan_msg_new.angle_min,scan_msg_new.angle_max);
    std_msgs::Header scan_header = scan_msg_new.header;
    laser_frame_ = scan_msg_new.header.frame_id;
    scan_msg_new.ranges.assign(scan_msg_new.ranges.size(),
                               std::numeric_limits<float>::quiet_NaN());
    if (!isGotTf3) {
      if (getLaserTobase(laser_frame_, base_to_laser3, laser_to_base3)) {
        isGotTf3 = true;
      } else {
        ROS_ERROR("DONT HAVE TF FROM BASE TO LASER_FRAME:%s",
                  laser_frame_.c_str());
        return;
      }
    }
    if (!enable_detect && icp_type == 2) {
      ROS_WARN_THROTTLE(
          2.0, "[scanCallback3] enable_detect=false, skipping detection");
      return;
    }
    for (size_t i = 0; i < scan_msg_new.intensities.size(); ++i) {

      double dis_now = scan_msg_raw.ranges[i];
      float intensity_now = scan_msg_new.intensities[i];
      float angle_now = scan_msg_new.angle_min + i * scan_resolution;
      if (use_diff_laser) {
        if (angle_now < 1.5713)
          scan_msg_new.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        continue;
      }
      for (int s = 1; s < distance_range.size(); s++) {

        if (dis_now > distance_range[s])
          continue;
        if (dis_now < distance_range[s] && dis_now > distance_range[s - 1]) {
          // if(scan_msg_new.intensities[i]>maxInteninRange[s])maxInteninRange[s]=scan_msg_new.intensities[i];
          // else
          // if(scan_msg_new.intensities[i]<minInteninRange[s])minInteninRange[s]=scan_msg_new.intensities[i];
          // if(maxInteninRange[s] !=0 && maxInteninRange[s]
          // !=9999)intensity_range[s]=(maxInteninRange[s]+minInteninRange[s])/2.0;
          // if(scan_msg_new.ranges[i]<10)scan_msg_new.intensities[i]=scan_msg_new.intensities[i]*(scan_msg_new.ranges[i]*scan_msg_new.ranges[i]);
          if (intensity_now > intensity_range[s - 1]) {
            scan_msg_new.ranges[i] = scan_msg_raw.ranges[i];
            break;
          }
        }
      }
    }

    int start_clus_index = 0;
    float initial_intensity = 0;
    float slide_dis = 0;
    bool in_high_intensity = false;
    int high_intensity_count = 0;
    std::vector<int> intensity_clus;
    std::vector<int> intensity_clus_debug;
    int size_last = scan_msg_new.intensities.size();
    int size_count_erase = 0;

    scan_filter_pub.publish(scan_msg_new);
    // scan_msg_intensity=scan_msg_new;

    scan_cloud.reset(new sensor_msgs::PointCloud2);

    projector_.projectLaser(scan_msg_new, *scan_cloud);
    // projector_.projectLaser(scan_msg_raw, cloud_raw);
    // scan_mirror_pub.publish(cloud_raw);

    // Reset mirror_detected before detection
    mirror_detected = false;

    cloudMirror(scan_cloud, scan_header);

    // Publish detection status if enabled
    if (enable_detect) {
      std_msgs::String status_msg;
      if (mirror_detected) {
        status_msg.data = "success";
        ROS_INFO_THROTTLE(1.0, "Mirror Detection: SUCCESS");
      } else {
        status_msg.data = "fail";
        ROS_WARN_THROTTLE(1.0, "Mirror Detection: FAIL");
      }
      status_detect_mirror_pub_.publish(status_msg);
    }
    // cloudCallback(scan_cloud);
  }

  void getNeighBourCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCLPtr,
                           mirror_detect::ClusterArray::Ptr clustersPtr) {
    const float neighbor_distance = 0.0065;
    std::set<int> included_indices;
    sensor_msgs::LaserScan scan_near = ClusterToScan(clustersPtr->clusters);
    scan_neighbour_pub.publish(scan_near);
  }
  //=============================================================================================================================================================================================================================================================================================================================================================================================================
  //
  //   ####  ##       #####   ##   ##  ####           ####    ###    ##      ##
  //   #####     ###     ####  ##  ##
  //  ##     ##      ##   ##  ##   ##  ##  ##        ##      ## ##   ##      ##
  //  ##  ##   ## ##   ##     ## ##
  //  ##     ##      ##   ##  ##   ##  ##  ##        ##     ##   ##  ##      ##
  //  #####   ##   ##  ##     ####
  //  ##     ##      ##   ##  ##   ##  ##  ##        ##     #######  ##      ##
  //  ##  ##  #######  ##     ## ##
  //   ####  ######   #####    #####   ####           ####  ##   ##  ######
  //   ######  #####   ##   ##   ####  ##  ##
  //
  //=============================================================================================================================================================================================================================================================================================================================================================================================================
  void cloudMirror(const sensor_msgs::PointCloud2ConstPtr &msg,
                   const std_msgs::Header header_all) {
    clearGlobals();
    // ROS_INFO("HEADER:%s", header_all.frame_id.c_str());
    ros::Time beginCallback = ros::Time::now();
    header_ = header_all;
    linesPtr_->header = clustersPtr_->header = clusters_.header =
        lines_.header = lines_marker_.header = header_;
    // ROS_INFO("TIME:%f", (beginCallback.toSec() - after_time123.toSec()));
    after_time123 = beginCallback;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCLPtr(
        new pcl::PointCloud<pcl::PointXYZI>(cloud));
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(cloudPCLPtr);
    sor.setLeafSize(Voxel_leaf_size_, Voxel_leaf_size_, Voxel_leaf_size_);
    sor.filter(*cloudPCLPtr);
    Clustering clustering;
    clustering.setHeader(header_);
    clustering.ClusterPointsIntensity(cloudPCLPtr, clustersPtr_,
                                      EC_cluster_tolerance_, EC_min_size_,
                                      EC_max_size_);
    static uint8_t count_change = 0;
    if (clustersPtr_->clusters.size() == 0) {
      if (!use_scan_merge)
        count_change++;
      if (count_change >= 4) {
        count_change = 0;
        use_laser_1 = !use_laser_1;
        ROS_WARN_STREAM("CHANGE LASER");
      }
      ROS_WARN_STREAM("CALLBACK: NO CLUSTERS FOUND");

      return;
    }
    // count_change = 0;
    getNeighBourCluster(cloudPCLPtr, clustersPtr_);
    LineDetection lineDetection;
    lineDetection.setHeader(header_);
    lineDetection.setParams(RS_max_iter_, RS_min_inliers_, RS_dist_thresh_);
    /* ========================================
     * RANSAC LINES FROM CLUSTERS
     * ========================================*/
    if (clustersPtr_->clusters.size() > 0) {
      count_change = 0;
      lineDetection.getRansacLinesOnClusterInten(clustersPtr_, linesPtr_);
      lines_cloud_pub_.publish(linesPtr_->combinedCloud);
      sensor_msgs::PointCloud2 output_msg;
      clustersPtr_->header = header_all;
      clusters_cloud_pub_.publish(clustersPtr_->combinedCloud);
      DetectBarMarker(linesPtr_);
      // DetectLegMarker(clustersPtr_);
    } else {
      if (!use_scan_merge)
        count_change++;
      if (count_change >= 4) {
        count_change = 0;
        use_laser_1 = !use_laser_1;
        ROS_WARN_STREAM("CHANGE LASER");
      }
    }

    //  ROS_INFO("cluster size:%ld",clustersPtr_->clusters.size());
    // DetectMirrorIntensity(clustersPtr_);
    // pcl::toROSMsg(clustersPtr_->combinedCloud, output_msg);

    // scan_mirror_pub.publish(clustersPtr_->combinedCloud);
  }

  void DetectBarMarker(mirror_detect::LineArray::Ptr linesPtr_) {
    int i, j = 0;
    int index = 0;
    int index_2 = 0;
    int left_point = 0;
    int right_point = 1;
    // std::vector<int> FirstLegInd;
    // std::vector<int> SecondLegInd;
    std::vector<PoseData> valid_poses;
    geometry_msgs::PoseArray centers;

    pcl::PointCloud<pcl::PointXYZ> centroidPointsPtr;
    ROS_INFO("line_Total %d", linesPtr_->lines.size());
    static uint8_t count_change = 0;
    if (linesPtr_->lines.size() < 2) {
      ROS_WARN("NOT ENOUGHT 2 LINE");
      if (!use_scan_merge)
        count_change++;
      if (count_change >= 4) {
        count_change = 0;
        use_laser_1 = !use_laser_1;
        ROS_WARN_STREAM("CHANGE LASER");
      }
      return;
    }
    // else count_change = 0;

    geometry_msgs::PoseArray pose_arr;
    pose_arr.header.frame_id = "base_link";
    pose_arr.header.stamp = ros::Time::now();
    centers.header = pose_arr.header;
    for (int i = 0; i < linesPtr_->lines.size(); i++) {
      double len_test = linesPtr_->lines[i].length.data;
      ROS_INFO("[DebugLeg] Line %d: length=%.3f (target=%.3f, tol=%.3f)", i,
               len_test, BAR_length, BAR_length_tolerance);
      if (((len_test < (BAR_length - BAR_length_tolerance)) ||
           (len_test > (BAR_length + BAR_length_tolerance)))) {
        ROS_WARN("[DebugLeg] Line %d REJECTED: Length out of tolerance", i);
        continue;
      }
      geometry_msgs::PoseStamped center_pose_out =
          getBaseLinkCoord(linesPtr_->lines[i].centroid);
      // ROS_INFO("%d x_distance :%f y:%f", i, center_pose_out.pose.position.x,
      // center_pose_out.pose.position.y);
      bool firstLeg = center_pose_out.pose.position.x < BAR_distance;

      if (firstLeg) {
        ROS_INFO("[DebugLeg] Line %d ACCEPTED as valid leg candidate", i);
        // index += 1;
        PoseData pose_data;
        pose_data.pose = center_pose_out;
        pose_data.index = index;
        valid_poses.push_back(pose_data);
        index += 1;
        // if (pose_data.pose.pose.position.y < 0)
        //     left_point = pose_data.index;
        //   else
        //     right_point = pose_data.index;
        // Use RANSAC orientation in base_link frame
        double yaw = 2.0 * std::atan2(center_pose_out.pose.orientation.z, center_pose_out.pose.orientation.w);
        
        center_pose_out.pose.orientation.x = 0.0;
        center_pose_out.pose.orientation.y = 0.0;
        center_pose_out.pose.orientation.z = std::sin(yaw / 2.0);
        center_pose_out.pose.orientation.w = std::cos(yaw / 2.0);

        pose_arr.poses.push_back(center_pose_out.pose);
      } else {
        ROS_WARN(
            "[DebugLeg] Line %d REJECTED: Position X=%.3f > BAR_distance=%.3f",
            i, center_pose_out.pose.position.x, BAR_distance);
      }
    }
    ROS_INFO("INDEX:%d ", index);
    int countHubNear = 0;
    if (index >= 2) {
      std::vector<int> indexPose;
      std::vector<std::pair<int, int>> indexValid;
      for (int k = 0; k < index; k++) {
        bool exist = false;
        for (int j = 0; j < indexPose.size(); j++) {
          if (k == indexPose[j]) {
            exist = true;
            break;
          }
        }
        if (exist)
          continue;
        for (int m = k + 1; m < index; m++) {

          double dx = valid_poses[k].pose.pose.position.x -
                      valid_poses[m].pose.pose.position.x;
          double dy = valid_poses[k].pose.pose.position.y -
                      valid_poses[m].pose.pose.position.y;
          double lengthDis = getRangeDis(dx, dy);
          double err = std::abs(lengthDis - Leg_length);
          ROS_INFO("[DebugPair] Checking pair (%d, %d): dist=%.3f "
                   "(target=%.3f, err=%.3f, tol=%.3f)",
                   valid_poses[k].index, valid_poses[m].index, lengthDis,
                   Leg_length, err, Leg_tolerance);

          if (lengthDis < (Leg_length + Leg_tolerance) &&
              lengthDis > (Leg_length - Leg_tolerance)) {
            ROS_INFO("[DebugPair] Pair (%d, %d) ACCEPTED", valid_poses[k].index,
                     valid_poses[m].index);
            indexPose.push_back(k);
            indexPose.push_back(m);
            indexValid.push_back({valid_poses[k].index, valid_poses[m].index});
          } else {
            ROS_WARN("[DebugPair] Pair (%d, %d) REJECTED: Distance error %.3f "
                     "> tol %.3f",
                     valid_poses[k].index, valid_poses[m].index, err,
                     Leg_tolerance);
          }
        }
      }
      ROS_INFO("Center pose size:%ld", indexValid.size());
      geometry_msgs::Pose HubPoseDetect;

      for (const auto &pair : indexValid) {
        int idx1 = pair.first;
        int idx2 = pair.second;
        geometry_msgs::Pose centerPose;
        // ROS_INFO("INDEX:%d  %d", idx1, idx2);
        centerPose.position.x = (valid_poses[idx1].pose.pose.position.x +
                                 valid_poses[idx2].pose.pose.position.x) /
                                2.0;
        centerPose.position.y = (valid_poses[idx1].pose.pose.position.y +
                                 valid_poses[idx2].pose.pose.position.y) /
                                2.0;

        double dx = valid_poses[idx1].pose.pose.position.x -
                    valid_poses[idx2].pose.pose.position.x;
        double dy = valid_poses[idx1].pose.pose.position.y -
                    valid_poses[idx2].pose.pose.position.y;

        // Compute orientation (same as your original code)
        double perp_dx = dy;
        double perp_dy = -dx;
        double angle = std::atan2(perp_dy, perp_dx);
        double angle_2 = std::atan2(-perp_dy, -perp_dx);
        double angle_to_robot =
            std::atan2(centerPose.position.y, centerPose.position.x);
        double centerAngle;
        double diffAng1 = fabs(angle_to_robot - angle);
        double diffAng2 = fabs(angle_to_robot - angle_2);
        centerAngle = (diffAng1 < diffAng2) ? angle_2 : angle;
        if (diffAng1 * 57.29 > 270) {
          centerAngle = angle_2;
          // ROS_INFO("DIFF ANG1");
        } else if (diffAng2 * 57.29 > 270) {
          centerAngle = angle;
          // ROS_INFO("DIFF ANG2");
        }

        centerPose.orientation.x = 0.0;
        centerPose.orientation.y = 0.0;
        centerPose.orientation.z = std::sin(centerAngle / 2.0);
        centerPose.orientation.w = std::cos(centerAngle / 2.0);
        // Filter by orientation: reject pairs whose facing angle doesn't match
        // hub->waiting direction
        if (!std::isnan(expected_hub_orientation_)) {
          double angle_diff = centerAngle - expected_hub_orientation_;
          // Normalize to [-pi, pi]
          while (angle_diff > M_PI)
            angle_diff -= 2 * M_PI;
          while (angle_diff < -M_PI)
            angle_diff += 2 * M_PI;
          double angle_diff_abs = std::fabs(angle_diff);
          ROS_INFO("Pair (%d,%d) orientation check: centerAngle=%f expected=%f "
                   "diff=%f (tol=%f)",
                   idx1, idx2, centerAngle * (180.0 / M_PI),
                   expected_hub_orientation_ * (180.0 / M_PI),
                   angle_diff_abs * (180.0 / M_PI),
                   orientation_tolerance_ * (180.0 / M_PI));
          if (angle_diff_abs > orientation_tolerance_) {
            ROS_WARN("Pair (%d,%d) REJECTED: orientation mismatch diff=%.1f "
                     "deg > tol=%.1f deg",
                     idx1, idx2, angle_diff_abs * (180.0 / M_PI),
                     orientation_tolerance_ * (180.0 / M_PI));
            continue; // Skip this pair
          }
        }

        if (getRangeDis(centerPose.position.x - poseHub.position.x,
                        centerPose.position.y - poseHub.position.y) <
            radius_detect) {
          countHubNear += 1;
          HubPoseDetect = centerPose; // laser
        }
        // double dxy = getRangeDis(dx, dy);
        ROS_INFO("LEG x:%f y:%f angle:%f angle_2:%f angle_to_robot:%f "
                 "last_angle:%f length:%f",
                 centerPose.position.x, centerPose.position.y, angle * 57.29,
                 angle_2 * 57.29, angle_to_robot * 57.29, centerAngle * 57.29);

        centers.poses.push_back(baseToLaser(centerPose).pose);

        // std::cout << "Index: (" << pair.first << ", " << pair.second << ")"
        // << std::endl;
      }
      if (countHubNear == 0) {
        if (!use_scan_merge)
          count_change++;
        if (count_change >= 20) {
          count_change = 0;
          use_laser_1 = !use_laser_1;
          ROS_WARN_STREAM("CHANGE LASER");
        }
        ROS_WARN("NO POSE NEARBY DETECT x:%f,y:%f", poseHub.position.x,
                 poseHub.position.y);
        mirror_detected = false; // No mirror detected
      } else if (countHubNear == 1) {
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "base_link";
        transformStamped.child_frame_id = "center_hub";
        transformStamped.transform.translation.x = HubPoseDetect.position.x;
        transformStamped.transform.translation.y = HubPoseDetect.position.y;
        transformStamped.transform.translation.z = HubPoseDetect.position.z;
        transformStamped.transform.rotation = HubPoseDetect.orientation;
        ROS_INFO("HUB x:%f y:%f", HubPoseDetect.position.x,
                 HubPoseDetect.position.y);
        tfBroadcaster_.sendTransform(transformStamped);
        logHubPoseInOdom(HubPoseDetect);
        mirror_detected = true; // Mirror detected successfully
      } else {
        ROS_WARN("TOO MUCH COUNT HUB NEARBY POSE IGNORE:%d", countHubNear);
        mirror_detected = false; // Too many detections, consider as fail
      }

      // geometry_msgs::PoseArray result_poses;
      // Step 3: Process each cluster
      // std::vector<pcl::PointIndices> cluster_indices = ClusterLeg(pose_arr,
      // pose_indices); processCluster(pose_indices, cluster_indices, pose_arr,
      // valid_poses);
    } else {
      ROS_WARN("[DetectBar] FAILED: only %d valid leg(s) found, need >= 2 to "
               "form a pair",
               index);
      if (!use_scan_merge)
        count_change++;
      if (count_change >= 8) {
        count_change = 0;
        use_laser_1 = !use_laser_1;
        ROS_WARN_STREAM("CHANGE LASER");
      }
      mirror_detected = false; // Not enough valid poses
    }
    mirror_point_pub.publish(pose_arr);
    center_point_pub.publish(centers);
  }
  std::vector<pcl::PointIndices> ClusterLeg(geometry_msgs::PoseArray pose_arr,
                                            std::vector<int> &pose_indices) {
    // Step 1: Convert pose_arr.poses to a point cloud for clustering
    pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < pose_arr.poses.size(); ++i) {
      pcl::PointXYZ point;
      point.x = pose_arr.poses[i].position.x;
      point.y = pose_arr.poses[i].position.y;
      point.z = 0.0; // Set z = 0 for 2D clustering
      centroid_cloud->push_back(point);
      pose_indices.push_back(i);
    }
    // Step 2: Apply Euclidean Cluster Extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(max_cluster_size);
    ec.setMinClusterSize(2);
    ec.setMaxClusterSize(10);
    ec.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(
        new pcl::search::KdTree<pcl::PointXYZ>));
    ec.setInputCloud(centroid_cloud);
    ec.extract(cluster_indices);
    return cluster_indices;
  }
  void processCluster(std::vector<int> pose_indices,
                      std::vector<pcl::PointIndices> cluster_indices,
                      geometry_msgs::PoseArray &pose_arr,
                      std::vector<PoseData> valid_poses) {
    ROS_INFO("CLUSTER SIZE:%ld", cluster_indices.size());
    geometry_msgs::PoseArray centers;
    centers.header = pose_arr.header;
    for (const auto &cluster : cluster_indices) {
      if (cluster.indices.size() ==
          2) { // Process clusters with exactly 2 points
        int idx1 = pose_indices[cluster.indices[0]];
        int idx2 = pose_indices[cluster.indices[1]];

        geometry_msgs::Pose centerPose;
        centerPose.position.x = (valid_poses[idx1].pose.pose.position.x +
                                 valid_poses[idx2].pose.pose.position.x) /
                                2.0;
        centerPose.position.y = (valid_poses[idx1].pose.pose.position.y +
                                 valid_poses[idx2].pose.pose.position.y) /
                                2.0;

        double dx = valid_poses[idx1].pose.pose.position.x -
                    valid_poses[idx2].pose.pose.position.x;
        double dy = valid_poses[idx1].pose.pose.position.y -
                    valid_poses[idx2].pose.pose.position.y;

        // Compute orientation (same as your original code)
        double perp_dx = dy;
        double perp_dy = -dx;
        double angle = std::atan2(perp_dy, perp_dx);
        double angle_2 = std::atan2(-perp_dy, -perp_dx);
        double angle_to_robot =
            std::atan2(centerPose.position.y, centerPose.position.x);
        double centerAngle;
        double diffAng1 = fabs(angle_to_robot - angle);
        double diffAng2 = fabs(angle_to_robot - angle_2);
        centerAngle = (diffAng1 < diffAng2) ? angle_2 : angle;
        if (diffAng1 * 57.29 > 270) {
          centerAngle = angle_2;
          // ROS_INFO("DIFF ANG1");
        } else if (diffAng2 * 57.29 > 270) {
          centerAngle = angle;
          // ROS_INFO("DIFF ANG2");
        }

        centerPose.orientation.x = 0.0;
        centerPose.orientation.y = 0.0;
        centerPose.orientation.z = std::sin(centerAngle / 2.0);
        centerPose.orientation.w = std::cos(centerAngle / 2.0);

        double dxy = getRangeDis(dx, dy);
        ROS_INFO("LEG x:%f y:%f angle:%f angle_2:%f angle_to_robot:%f "
                 "last_angle:%f length:%f",
                 centerPose.position.x, centerPose.position.y, angle * 57.29,
                 angle_2 * 57.29, angle_to_robot * 57.29, centerAngle * 57.29,
                 dxy);

        // Add to result poses
        // result_poses.poses.push_back(baseToLaser(centerPose).pose);
        // pose_arr.poses.push_back(baseToLaser(centerPose).pose);
        centers.poses.push_back(baseToLaser(centerPose).pose);
        center_point_pub.publish(centers);
      } else {
        ROS_WARN("Cluster with %zu points ignored (need exactly 2)",
                 cluster.indices.size());
      }
    }
  }
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {

    // Only perform detection if activated
    // ROS

    ros::Time beginCallback = ros::Time::now();

    // ROS_INFO("TIME:%f",(beginCallback.toSec()-after_time123.toSec()));
    after_time123 = beginCallback;
    // ROS_INFO_STREAM("CLOUD CALLBACK: CALLBACK CALLED ");

    if (msg->width == 0 || msg->row_step == 0) {
      ROS_WARN_STREAM("CALLBACK: POINT CLOUD MSG EMPTY ");
      return;
    }

    clearGlobals();

    header_ = msg->header;
    linesPtr_->header = clustersPtr_->header = clusters_.header =
        lines_.header = lines_marker_.header = header_;

    static tf2_ros::TransformBroadcaster tfbr;

    // Container for original & filtered data
    /*
     * CONVERT POINTCLOUD ROS->PCL
     */
    pcl::PointCloud<pcl::PointXYZRGB> cloudPCL;
    pcl::fromROSMsg(*msg, cloudPCL);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPCLPtr(
        new pcl::PointCloud<pcl::PointXYZRGB>(cloudPCL));
    if (cloudPCLPtr->size() == 0) {
      ROS_WARN_STREAM("CALLBACK: PCL POINT CLOUD EMPTY ");
      return;
    }

    /* ========================================
     * VOXEL DOWNSAMPLING
     * ========================================*/
    VoxelGrid(cloudPCLPtr, Voxel_leaf_size_, cloudPCLPtr);

    /* ========================================
     * CLUSTERING
     * ========================================*/
    Clustering clustering;
    clustering.setHeader(header_);
    clustering.ClusterPoints(cloudPCLPtr, clustersPtr_, EC_cluster_tolerance_,
                             EC_min_size_, EC_max_size_);
    if (clustersPtr_->clusters.size() == 0) {
      ROS_WARN_STREAM("CALLBACK: NO CLUSTERS FOUND ");
      return;
    }

    /* ========================================
     * RANSAC LINES
     * ========================================*/
    LineDetection lineDetection;
    lineDetection.setHeader(header_);
    lineDetection.setParams(RS_max_iter_, RS_min_inliers_, RS_dist_thresh_);
    /* ========================================
     * RANSAC LINES FROM CLUSTERS
     * ========================================*/
    if (clustersPtr_->clusters.size() > 0) {
      lineDetection.getRansacLinesOnCluster(clustersPtr_, linesPtr_);
    }
    /* ========================================
     * PUBLISH CLOUD
     * ========================================*/

    if (clustersPtr_->clusters.size() > 0) {
      clusters_cloud_pub_.publish(clustersPtr_->combinedCloud);
      clusters_pub_.publish(clustersPtr_);
      lines_cloud_pub_.publish(linesPtr_->combinedCloud);
      lines_pub_.publish(linesPtr_);
    }
  }

  double getRangeDis(double x, double y) { return sqrt(x * x + y * y); }

  void addPointsToCloud(sensor_msgs::PointCloud2 &cloud_msg, double cx,
                        double cy) {
    // Initialize PointCloud2Modifier with XYZ fields
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);

    // Set the point cloud type and resize it to add one more point
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(modifier.size() + 1); // Add one more point to the cloud

    // Create iterators to fill in the point cloud
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    // Move the iterators to the end of the current data to add new points
    iter_x += cloud_msg.width * cloud_msg.height - 1;
    iter_y += cloud_msg.width * cloud_msg.height - 1;
    iter_z += cloud_msg.width * cloud_msg.height - 1;

    // Set the new point's values
    *iter_x = cx;
    *iter_y = cy;
    *iter_z = 0.1; // Assuming Z is 0, you can set it to your desired value
  }

  double getPCLEuclideanDistance(pcl::PointXYZ p1, pcl::PointXYZ p2) {
    double length, x1, x2, y1, y2;
    x1 = p1.x;
    x2 = p2.x;
    y1 = p1.y;
    y2 = p2.y;

    length = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    return length;
  }
  bool isNearDistance(double ref, double inp, double err) {
    if (ref > (inp - err) && ref < (inp + err)) {
      return true;
    }
    return false;
  }
  pcl::PointXYZ
  getAverPoint(typename pcl::PointCloud<pcl::PointXYZ>::Ptr inCloudPtr) {
    pcl::PointXYZ averPointPCL;
    double sumX = 0, sumY = 0;
    for (int i = 0; i < inCloudPtr->size(); i++) {
      sumX = sumX + inCloudPtr->at(i).x;
      sumY = sumY + inCloudPtr->at(i).y;
    }
    averPointPCL.x = sumX / inCloudPtr->size();
    averPointPCL.y = sumY / inCloudPtr->size();
    averPointPCL.z = 0;
    return averPointPCL;
  }
  ///////////////// BEGIN VOXEL GRID /////////////////
  /// \brief VoxelGrid
  /// \param inCloudPtr
  /// \param leafSize
  /// \return
  ///
  void VoxelGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloudPtr,
                 float leafSize,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloudPtr) {
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(inCloudPtr);
    voxel.setLeafSize(leafSize, leafSize, leafSize);
    voxel.filter(*filteredCloudPtr);
  }
};

//} // namespace mirror_detect

#endif /*"MIRRORDETECT_H_"*/

