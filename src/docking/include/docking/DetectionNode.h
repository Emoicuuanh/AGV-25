// -*- mode: c++ -*-

#ifndef DETECTIONNODE_H_
#define DETECTIONNODE_H_

#include "docking/DockService.h"
#include <docking/DetectionNodeConfig.h>
#include <docking/Headers.h>
#include <geometry_msgs/PoseStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <ros/package.h>
#include <ros/service.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> POINT;

class DetectionNode
{
  ///////////////// BEGIN VARIABLES /////////////////
public:
  DetectionNode(ros::NodeHandle nh) : nh_(nh), tfListener_(tfBuffer_)
  {
    startDynamicReconfigureServer();
    startPub();
    activationSub();
    initGlobals();
    initDockParams();
    ros::Duration(1).sleep();
    startCloudSubs();  // Start all 3 cloud subscribers
    startDockService();
  }
  ~DetectionNode() {}

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
  ros::Publisher bbox_pub_;
  ros::Publisher debug_pub_;
  ros::Publisher icp_in_pub_;
  ros::Publisher icp_target_pub_;
  ros::Publisher icp_out_pub_;
  ros::Publisher myPoint_min;
  ros::Publisher myPoint_max;

  // Three cloud subscribers for fallback strategy
  ros::Subscriber cloudSub1_;
  ros::Subscriber cloudSub2_;
  ros::Subscriber cloudSubMerger_;

  ros::Subscriber activationSub_;
  ros::Subscriber selectTypeSub;
  ros::ServiceServer dock_service_;

  ros::NodeHandle nh_;
  dynamic_reconfigure::Server<docking::DetectionNodeConfig> dr_srv_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2_ros::TransformBroadcaster tfBroadcaster_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr dockTargetPCLPtr_;

  visualization_msgs::Marker lines_marker_;
  docking::LineArray lines_;
  docking::LineArray::Ptr linesPtr_;
  docking::ClusterArray clusters_;
  docking::ClusterArray::Ptr clustersPtr_;

  int RS_max_iter_;
  int RS_min_inliers_;
  double RS_dist_thresh_;
  bool RANSAC_on_clusters_;

  double EC_cluster_tolerance_;
  int EC_min_size_;
  int EC_max_size_;

  int detect_type_;
  std_msgs::Bool found_dock_;
  std_msgs::Bool perform_detection_;
  std_msgs::String selectType;
  bool printed_deactivation_status_;

  double dock_wing_length;

  std::string dockFilePath_;
  bool dock_template_loaded_;
  double ICP_min_score_;
  double ICP_max_correspondence_distance_;
  double ICP_max_iterations_;
  double ICP_max_transformation_eps_;
  double ICP_max_transformation_rotation_eps_;
  double ICP_max_euclidean_fitness_eps_;

  double Voxel_leaf_size_;

  std::string world_frame_;
  std::string robot_frame_;
  std::string cloud1_frame_;
  std::string cloud1_topic_;
  std::string cloud2_frame_;
  std::string cloud2_topic_;
  std::string cloud_merger_frame_;
  std::string cloud_merger_topic_;

  std_msgs::Header header_;

  geometry_msgs::TransformStamped base_link_to_cloud1;
  geometry_msgs::TransformStamped base_link_to_cloud2;
  geometry_msgs::TransformStamped base_link_to_cloud_merger;
  bool isGotTf1 = false;
  bool isGotTf2 = false;
  bool isGotTfMerger = false;

  double max_dock_distance_;
  double dock_offset_x_;
  double dock_offset_y_;
  double dock_offset_yaw_;

  geometry_msgs::PoseStamped last_target_pose_;

  // Fallback cloud strategy variables
  int current_cloud_source_;         // 0=cloud1, 1=cloud2, 2=cloud_merger
  std::vector<int> cloud_sequence_;  // Sequence of clouds to try
  int current_sequence_index_;       // Index in cloud_sequence_
  int detection_attempts_;           // Count attempts for current cloud
  const int MAX_ATTEMPTS_PER_CLOUD = 5;
  bool dock_detected_in_sequence_;  // Track if dock found during sequence

  ///////////////// END VARIABLES /////////////////

  void calcTf()
  {
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener(tf_buffer);

    try
    {
      base_link_to_cloud1 = tf_buffer.lookupTransform(
          robot_frame_, cloud1_frame_, ros::Time(0), ros::Duration(1.0));
      isGotTf1 = true;
      ROS_INFO_STREAM("calcTf: Got transform base_link -> cloud1");
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_STREAM(
          "calcTf: Failed to get transform base_link -> cloud1: " << ex.what());
    }

    try
    {
      base_link_to_cloud2 = tf_buffer.lookupTransform(
          robot_frame_, cloud2_frame_, ros::Time(0), ros::Duration(1.0));
      isGotTf2 = true;
      ROS_INFO_STREAM("calcTf: Got transform base_link -> cloud2");
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_STREAM(
          "calcTf: Failed to get transform base_link -> cloud2: " << ex.what());
    }

    try
    {
      base_link_to_cloud_merger = tf_buffer.lookupTransform(
          robot_frame_, cloud_merger_frame_, ros::Time(0), ros::Duration(1.0));
      isGotTfMerger = true;
      ROS_INFO_STREAM("calcTf: Got transform base_link -> cloud_merger");
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_STREAM(
          "calcTf: Failed to get transform base_link -> cloud_merger: "
          << ex.what());
    }
  }

  void startDynamicReconfigureServer()
  {
    ROS_INFO_STREAM(
        "startDynamicReconfigureServer: STARTING DYNAMIC RECONFIGURE SERVER");
    dynamic_reconfigure::Server<docking::DetectionNodeConfig>::CallbackType cb;
    cb = boost::bind(&DetectionNode::configCallback, this, _1, _2);
    dr_srv_.setCallback(cb);
  }

  void startDockService()
  {
    dock_service_ = nh_.advertiseService(
        "dock_service", &DetectionNode::dockServiceCallback, this);
    ROS_INFO_STREAM("startDockService: STARTING DOCK SERVICE");
  }

  void initLineMarker()
  {
    lines_marker_.type = visualization_msgs::Marker::LINE_LIST;
    lines_marker_.header = header_;
    lines_marker_.ns = "docking";
    lines_marker_.id = 0;
  }

  void initDockParams()
  {
    ROS_INFO_STREAM("initDockParams: INITIALIZING TARGET CLOUD FILE PATH");
    bool dockFilepathExists = dockFilePath_ == "" ? false : true;
    if (dockFilepathExists)
    {
      ROS_INFO_STREAM("initDockParams: PARAM SERVER TARGET CLOUD FILE PATH: "
                      << dockFilePath_);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr dockTargetPCLPtr(
          new pcl::PointCloud<pcl::PointXYZRGB>);
      ROS_INFO_STREAM("initDockParams: LOADING DOCK TARGET CLOUD FILE"
                      << dockFilePath_);
      if (readPointCloudFile(dockFilePath_, dockTargetPCLPtr) == false)
      {
        ROS_ERROR_STREAM("initDockParams: FAILED TO LOAD TARGET DOCK FILE");
      }
      else
      {
        ROS_INFO_STREAM("initDockParams: SUCCESSFULLY LOADED TARGET DOCK FILE");
        dockTargetPCLPtr_ = dockTargetPCLPtr;
      }
    }
    else
    {
      ROS_ERROR_STREAM("initDockParams: NO PARAMETER dock_filepath EXISTS FOR "
                       "TARGET CLOUD FILE PATH");
    }
  }

  void initGlobals()
  {
    found_dock_.data = false;
    docking::ClusterArray temp1 = docking::ClusterArray();
    docking::ClusterArray::Ptr tempPtr(new docking::ClusterArray());
    clustersPtr_ = tempPtr;
    *clustersPtr_ = temp1;

    docking::LineArray temp2 = docking::LineArray();
    docking::LineArray::Ptr tempPtr2(new docking::LineArray());
    linesPtr_ = tempPtr2;
    *linesPtr_ = temp2;

    current_cloud_source_ = 0;

    // Initialize fallback strategy variables
    cloud_sequence_.clear();
    current_sequence_index_ = 0;
    detection_attempts_ = 0;
    dock_detected_in_sequence_ = false;
  }

  void startPub()
  {
    status_pub_ = nh_.advertise<std_msgs::Bool>("docking/found_dock", 1);
    myPoint_min =
        nh_.advertise<geometry_msgs::PointStamped>("docking/point_min", 1);
    myPoint_max =
        nh_.advertise<geometry_msgs::PointStamped>("docking/point_max", 1);
    clusters_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("docking/clustersCloud", 1);
    clusters_pub_ = nh_.advertise<docking::ClusterArray>("docking/clusters", 1);
    lines_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("docking/linesCloud", 1);
    lines_pub_ = nh_.advertise<docking::LineArray>("docking/lines", 1);
    line_marker_pub_ =
        nh_.advertise<visualization_msgs::Marker>("docking/lines_marker", 1);
    dock_marker_pub_ =
        nh_.advertise<visualization_msgs::Marker>("docking/dock_marker", 1);
    dock_pose_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "docking/dock_pose_marker", 1);
    dock_pose_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>("docking/dock_pose", 1);
    bbox_pub_ = nh_.advertise<docking::BoundingBox>("docking/bbox", 1);
    debug_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("docking/debugCloud", 1);
    icp_in_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
        "docking/icp_in_pub", 1);
    icp_target_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
        "docking/icp_target_pub", 1);
    icp_out_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
        "docking/icp_out_pub", 1);
  }

  void startCloudSubs()
  {
    ROS_INFO_STREAM("Starting all cloud subscribers");
    cloudSub1_ =
        nh_.subscribe(cloud1_topic_, 1, &DetectionNode::cloudCallback1, this);
    cloudSub2_ =
        nh_.subscribe(cloud2_topic_, 1, &DetectionNode::cloudCallback2, this);
    cloudSubMerger_ = nh_.subscribe(cloud_merger_topic_, 1,
                                    &DetectionNode::cloudCallbackMerger, this);
    ROS_INFO_STREAM("Subscribed to: " << cloud1_topic_ << ", " << cloud2_topic_
                                      << ", " << cloud_merger_topic_);
  }

  void cloudCallback1(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (current_cloud_source_ != 0)
    {
      ROS_DEBUG_THROTTLE(5.0,
                         "cloudCallback1: Ignoring - current_cloud_source_=%d",
                         current_cloud_source_);
      return;
    }

    ROS_INFO_THROTTLE(2.0, "cloudCallback1: Processing cloud from %s",
                      cloud1_topic_.c_str());
    processCloud(msg);
  }

  void cloudCallback2(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (current_cloud_source_ != 1)
    {
      ROS_DEBUG_THROTTLE(5.0,
                         "cloudCallback2: Ignoring - current_cloud_source_=%d",
                         current_cloud_source_);
      return;
    }

    ROS_INFO_THROTTLE(2.0, "cloudCallback2: Processing cloud from %s",
                      cloud2_topic_.c_str());
    processCloud(msg);
  }

  void cloudCallbackMerger(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (current_cloud_source_ != 2)
    {
      ROS_DEBUG_THROTTLE(
          5.0, "cloudCallbackMerger: Ignoring - current_cloud_source_=%d",
          current_cloud_source_);
      return;
    }

    ROS_INFO_THROTTLE(2.0, "cloudCallbackMerger: Processing cloud from %s",
                      cloud_merger_topic_.c_str());
    processCloud(msg);
  }

  void activationSub()
  {
    activationSub_ = nh_.subscribe("docking/perform_detection", 1,
                                   &DetectionNode::activationCallback, this);
    selectTypeSub = nh_.subscribe("docking/select_type", 1,
                                  &DetectionNode::selectTypeCallback, this);
  }

  void clearGlobals()
  {
    lines_.lines.clear();
    lines_marker_.points.clear();
    lines_marker_.colors.clear();
    clustersPtr_->clusters.clear();
    linesPtr_->lines.clear();
  }

  void initializeCloudSequence(bool use_scan_merge)
  {
    cloud_sequence_.clear();
    detection_attempts_ = 0;
    dock_detected_in_sequence_ = false;
    current_sequence_index_ = 0;

    if (use_scan_merge)
    {
      // Try: cloud_merger (2) → cloud1 (0) → cloud2 (1)
      cloud_sequence_.push_back(2);
      cloud_sequence_.push_back(0);
      cloud_sequence_.push_back(1);
      ROS_INFO_STREAM("dockServiceCallback: Initialized sequence - "
                      "cloud_merger -> cloud1 -> cloud2");
    }
    else
    {
      // Try: cloud1 (0) → cloud2 (1)
      cloud_sequence_.push_back(0);
      cloud_sequence_.push_back(1);
      ROS_INFO_STREAM(
          "dockServiceCallback: Initialized sequence - cloud1 -> cloud2");
    }

    // Set to first cloud in sequence
    current_cloud_source_ = cloud_sequence_[current_sequence_index_];
    ROS_INFO_STREAM("dockServiceCallback: Starting with cloud source "
                    << current_cloud_source_);
  }

  void moveToNextCloud()
  {
    ROS_WARN_STREAM(">>> moveToNextCloud CALLED <<<");
    ROS_INFO_STREAM(
        "  dock_detected_in_sequence_: " << dock_detected_in_sequence_);
    ROS_INFO_STREAM("  detection_attempts_: " << detection_attempts_);
    ROS_INFO_STREAM("  MAX_ATTEMPTS_PER_CLOUD: " << MAX_ATTEMPTS_PER_CLOUD);
    ROS_INFO_STREAM("  current_sequence_index_: " << current_sequence_index_);
    ROS_INFO_STREAM("  cloud_sequence_.size(): " << cloud_sequence_.size());

    if (dock_detected_in_sequence_)
    {
      ROS_INFO_STREAM(
          "moveToNextCloud: Dock already detected, staying on current cloud");
      return;
    }

    detection_attempts_++;
    ROS_INFO_STREAM(
        "  detection_attempts_ incremented to: " << detection_attempts_);

    if (detection_attempts_ >= MAX_ATTEMPTS_PER_CLOUD)
    {
      ROS_WARN_STREAM("moveToNextCloud: Reached max attempts ("
                      << MAX_ATTEMPTS_PER_CLOUD << ") for cloud source "
                      << current_cloud_source_);

      if (current_sequence_index_ < cloud_sequence_.size() - 1)
      {
        current_sequence_index_++;
        current_cloud_source_ = cloud_sequence_[current_sequence_index_];
        detection_attempts_ = 0;
        ROS_WARN_STREAM("!!! SWITCHING TO CLOUD SOURCE "
                        << current_cloud_source_ << " !!!");
      }
      else
      {
        ROS_ERROR_STREAM(
            "moveToNextCloud: All cloud sources exhausted, no dock detected");
        ROS_ERROR_STREAM("  Sequence was: ");
        for (size_t i = 0; i < cloud_sequence_.size(); i++)
        {
          ROS_ERROR_STREAM("    [" << i
                                   << "] cloud_source=" << cloud_sequence_[i]);
        }
      }
    }
    else
    {
      ROS_INFO_STREAM("  Still have "
                      << (MAX_ATTEMPTS_PER_CLOUD - detection_attempts_)
                      << " attempts left on cloud source "
                      << current_cloud_source_);
    }
  }

  bool dockServiceCallback(docking::DockService::Request& req,
                           docking::DockService::Response& res)
  {
    ROS_INFO_STREAM("dockServiceCallback: Received request");
    ROS_INFO("Position: x=%f, y=%f, z=%f", req.pose_target.position.x,
             req.pose_target.position.y, req.pose_target.position.z);
    ROS_INFO("Orientation: x=%f, y=%f, z=%f, w=%f",
             req.pose_target.orientation.x, req.pose_target.orientation.y,
             req.pose_target.orientation.z, req.pose_target.orientation.w);
    ROS_INFO("Enable Detect: %s", req.enable_detect ? "true" : "false");
    ROS_INFO("Use Scan Merge: %s", req.use_scan_merge ? "true" : "false");
    res.success = true;

    perform_detection_.data = req.enable_detect;
    printed_deactivation_status_ = false;

    // Initialize cloud sequence based on use_scan_merge
    initializeCloudSequence(req.use_scan_merge);

    if (req.enable_detect)
    {
      geometry_msgs::PoseStamped input_pose;
      input_pose.header.frame_id = "map";
      input_pose.header.stamp = ros::Time::now();
      input_pose.pose = req.pose_target;

      // Transform to the appropriate cloud frame based on current cloud source
      std::string target_frame;
      if (current_cloud_source_ == 0)
      {
        target_frame = cloud1_frame_;
      }
      else if (current_cloud_source_ == 1)
      {
        target_frame = cloud2_frame_;
      }
      else if (current_cloud_source_ == 2)
      {
        target_frame = cloud_merger_frame_;
      }

      try
      {
        tfBuffer_.transform(input_pose, last_target_pose_, target_frame,
                            ros::Duration(1.0));
        ROS_INFO_STREAM(
            "dockServiceCallback: Transformed pose to frame: " << target_frame);
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN_STREAM("dockServiceCallback: Transform failed: " << ex.what());
        return true;
      }
    }

    return true;
  }

  void detectDockNearPose(const geometry_msgs::PoseStamped& target_pose,
                          docking::ClusterArray::Ptr clustersPtr_)
  {
    ROS_INFO_STREAM("=== detectDockNearPose START === Cloud source: "
                    << current_cloud_source_
                    << " | Attempt: " << detection_attempts_ + 1 << "/"
                    << MAX_ATTEMPTS_PER_CLOUD);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ICPInputCloudPtr(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ICPOutCloudPtr(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    docking::Cluster::Ptr dockClusterPtr(new docking::Cluster());
    PoseEstimation poseEstimation;
    poseEstimation.setHeader(header_);
    poseEstimation.setICPScore(ICP_min_score_);
    poseEstimation.setMaxIterations(ICP_max_iterations_);
    poseEstimation.setMaxCorrespondenceDistance(
        ICP_max_correspondence_distance_);
    poseEstimation.setMaxTransformationEps(ICP_max_transformation_eps_);
    poseEstimation.setMaxEuclideanFitnessEps(ICP_max_euclidean_fitness_eps_);

    bool icpSuccess = poseEstimation.clusterArrayICP(
        clustersPtr_, dockTargetPCLPtr_, dockClusterPtr);
    ROS_WARN_STREAM("CLOUD CALLBACK: ICP "
                    << (icpSuccess ? "SUCCESS" : "FAILED")
                    << " on cloud source " << current_cloud_source_);
    ICPInputCloudPtr->header.frame_id = header_.frame_id;
    dockTargetPCLPtr_->header.frame_id = header_.frame_id;
    ICPOutCloudPtr->header.frame_id = header_.frame_id;

    if (icpSuccess)
    {
      ROS_WARN_STREAM("--------------------------------------------------------"
                      "---------------------------------");
      found_dock_.data = true;
      dock_detected_in_sequence_ = true;

      icp_in_pub_.publish(dockClusterPtr->cloud);
      icp_out_pub_.publish(dockClusterPtr->icpCombinedCloud);
      if (!isGotTf1 && !isGotTf2 && !isGotTfMerger)
      {
        calcTf();
      }

      geometry_msgs::TransformStamped current_base_link_to_cloud;
      if (current_cloud_source_ == 0)
        current_base_link_to_cloud = base_link_to_cloud1;
      else if (current_cloud_source_ == 1)
        current_base_link_to_cloud = base_link_to_cloud2;
      else if (current_cloud_source_ == 2)
        current_base_link_to_cloud = base_link_to_cloud_merger;

      geometry_msgs::PoseStamped icpPose;
      tf2::doTransform(dockClusterPtr->icp.poseStamped, icpPose,
                       current_base_link_to_cloud);

      dock_pose_pub_.publish(icpPose);
      dock_pose_marker_pub_.publish(dockClusterPtr->icp.poseTextMarker);
      dock_marker_pub_.publish(dockClusterPtr->bbox.marker);
      bbox_pub_.publish(dockClusterPtr->bbox);

      dockClusterPtr->icp.transformStamped.header.stamp = ros::Time::now();
      dockClusterPtr->icp.transformStamped.header.frame_id = robot_frame_;
      dockClusterPtr->icp.transformStamped.child_frame_id = "dock";

      // === ✅ compute centroid from original detected cluster (before ICP) ===
      pcl::PointCloud<pcl::PointXYZRGB> originalCloudPCL;
      pcl::fromROSMsg(dockClusterPtr->cloud, originalCloudPCL);

      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(originalCloudPCL, centroid);

      // === ✅ Transform centroid from cloud frame to robot frame ===
      geometry_msgs::PointStamped centroid_in_cloud;
      centroid_in_cloud.header.frame_id = header_.frame_id;
      centroid_in_cloud.header.stamp = ros::Time::now();
      centroid_in_cloud.point.x = centroid[0];
      centroid_in_cloud.point.y = centroid[1];
      centroid_in_cloud.point.z = centroid[2];

      geometry_msgs::PointStamped centroid_in_robot;
      tf2::doTransform(centroid_in_cloud, centroid_in_robot,
                       current_base_link_to_cloud);

      // === ✅ Create TF with centroid position and ICP orientation ===
      tf2::Transform tf_icp;
      tf2::fromMsg(dockClusterPtr->icp.transformStamped.transform, tf_icp);

      tf2::Transform tf_cloud_to_base;
      tf2::fromMsg(current_base_link_to_cloud.transform, tf_cloud_to_base);
      tf_cloud_to_base = tf_cloud_to_base.inverse();

      tf2::Transform tf_base_to_dock = tf_cloud_to_base * tf_icp;

      // Override position with centroid, keep ICP orientation
      tf_base_to_dock.setOrigin(tf2::Vector3(centroid_in_robot.point.x,
                                             centroid_in_robot.point.y,
                                             centroid_in_robot.point.z));

      // === ✅ Apply dock offset ===
      tf2::Transform dock_offset;
      dock_offset.setOrigin(tf2::Vector3(dock_offset_x_, dock_offset_y_, 0.0));
      tf2::Quaternion q;
      q.setRPY(0, 0, dock_offset_yaw_);
      dock_offset.setRotation(q);

      tf2::Transform tf_with_offset = tf_base_to_dock * dock_offset;

      geometry_msgs::TransformStamped dock_tf_with_offset =
          dockClusterPtr->icp.transformStamped;
      dock_tf_with_offset.transform = tf2::toMsg(tf_with_offset);

      // === ✅ Ghi đè roll = 0, giữ nguyên pitch và yaw ===
      tf2::Quaternion current_quat;
      tf2::fromMsg(dock_tf_with_offset.transform.rotation, current_quat);
      tf2::Matrix3x3 rotation_matrix(current_quat);
      double roll, pitch, yaw;
      rotation_matrix.getRPY(roll, pitch, yaw);

      // Tạo quaternion mới với roll = 0
      tf2::Quaternion fixed_quat;
      fixed_quat.setRPY(0.0, pitch, yaw);  // roll = 0, giữ pitch và yaw
      dock_tf_with_offset.transform.rotation = tf2::toMsg(fixed_quat);

      // distance check now based on centroid in cloud frame (target_pose is in cloud frame)
      double distance = sqrt(pow(centroid[0] - target_pose.pose.position.x, 2) +
                             pow(centroid[1] - target_pose.pose.position.y, 2) +
                             pow(centroid[2] - target_pose.pose.position.z, 2));

      if (distance <= max_dock_distance_)
      {
        tfBroadcaster_.sendTransform(dock_tf_with_offset);
        ROS_INFO_STREAM("detectDockNearPose: Dock centroid TF broadcast with roll=0.");
      }
      else
      {
        ROS_INFO_STREAM("detectDockNearPose: ICP ok but dock too far.");
        found_dock_.data = false;
        dock_detected_in_sequence_ = false;
        moveToNextCloud();
      }
    }
    else
    {
      ROS_WARN_STREAM("detectDockNearPose: ICP failed on cloud source "
                      << current_cloud_source_);
      found_dock_.data = false;
      moveToNextCloud();
    }

    status_pub_.publish(found_dock_);
    ROS_INFO_STREAM(
        "=== detectDockNearPose END === found_dock: " << found_dock_.data);
  }

  void activationCallback(const std_msgs::BoolConstPtr& msg)
  {
    if (perform_detection_.data == msg->data)
    {
      return;
    }
    perform_detection_ = *msg;
    ROS_WARN_STREAM("SETTING DETECTION ACTIVATION STATUS TO  "
                    << perform_detection_);
    printed_deactivation_status_ = false;
  }

  void selectTypeCallback(const std_msgs::StringConstPtr& msg)
  {
    ROS_INFO_STREAM("Current type: " + selectType.data +
                    ", Set type: " + msg->data);
    if (selectType.data == msg->data)
      return;
    selectType.data = msg->data;
    ROS_INFO_STREAM("Current type: " + selectType.data);
    std::string filePathStr = selectType.data;
    boost::filesystem::path cfg_file_path =
        boost::filesystem::path(filePathStr);
    namespace fsys = boost::filesystem;
    if (!fsys::exists(cfg_file_path) ||
        !(fsys::is_regular_file(cfg_file_path) ||
          fsys::is_symlink(cfg_file_path)))
    {
      ROS_WARN_STREAM("Could not open marker param file "
                      << cfg_file_path.string());
    }
    else
    {
      std::string command = "rosparam load " + cfg_file_path.string() + " " +
                            ros::this_node::getName();
      int result = std::system(command.c_str());
      if (result != 0)
      {
        ROS_WARN_STREAM("Could not set config file "
                        << cfg_file_path.string()
                        << " to the parameter server.");
      }
      else
      {
        ROS_WARN_STREAM("Load param");
        loadParams();
      }
    }
  }

  void configCallback(docking::DetectionNodeConfig& config,
                      uint32_t level __attribute__((unused)))
  {
    ROS_INFO_STREAM("Config");
    if (dockFilePath_ != config.dock_filepath)
    {
      dockFilePath_ = config.dock_filepath;
      initDockParams();
    }
    world_frame_ = config.world_frame;
    robot_frame_ = config.robot_frame;
    cloud1_frame_ = config.cloud1_frame;
    cloud1_topic_ = config.cloud1_topic;
    cloud2_frame_ = config.cloud2_frame;
    cloud2_topic_ = config.cloud2_topic;
    cloud_merger_frame_ = config.cloud_merger_frame;
    cloud_merger_topic_ = config.cloud_merger_topic;
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
    detect_type_ = config.detect_type;
    max_dock_distance_ = config.max_dock_distance;
    dock_offset_x_ = config.dock_offset_x;
    dock_offset_y_ = config.dock_offset_y;
    dock_offset_yaw_ = config.dock_offset_yaw;
  }

  void loadParams()
  {
    std::string tempFilePath;
    ros::NodeHandle nhh("~");
    bool result = nhh.param("dock_filepath", tempFilePath, dockFilePath_);
    if (dockFilePath_ != tempFilePath)
    {
      dockFilePath_ = tempFilePath;
      initDockParams();
    }
    result = nhh.param("world_frame", world_frame_, world_frame_);
    nhh.param("robot_frame", robot_frame_, robot_frame_);
    nhh.param("cloud1_frame", cloud1_frame_, cloud1_frame_);
    nhh.param("cloud1_topic", cloud1_topic_, cloud1_topic_);
    nhh.param("cloud2_frame", cloud2_frame_, cloud2_frame_);
    nhh.param("cloud2_topic", cloud2_topic_, cloud2_topic_);
    nhh.param("cloud_merger_frame", cloud_merger_frame_, cloud_merger_frame_);
    nhh.param("cloud_merger_topic", cloud_merger_topic_, cloud_merger_topic_);
    nhh.param("RS_max_iter", RS_max_iter_, RS_max_iter_);
    nhh.param("RS_min_inliers", RS_min_inliers_, RS_min_inliers_);
    nhh.param("RS_dist_thresh", RS_dist_thresh_, RS_dist_thresh_);
    nhh.param("RANSAC_on_clusters", RANSAC_on_clusters_, RANSAC_on_clusters_);
    nhh.param("Voxel_leaf_size", Voxel_leaf_size_, Voxel_leaf_size_);
    nhh.param("EC_cluster_tolerance", EC_cluster_tolerance_,
              EC_cluster_tolerance_);
    nhh.param("EC_min_size", EC_min_size_, EC_min_size_);
    nhh.param("EC_max_size", EC_max_size_, EC_max_size_);
    nhh.param("ICP_min_score", ICP_min_score_, ICP_min_score_);
    nhh.param("ICP_max_corres_dist", ICP_max_correspondence_distance_,
              ICP_max_correspondence_distance_);
    nhh.param("ICP_max_iter", ICP_max_iterations_, ICP_max_iterations_);
    nhh.param("ICP_max_transl_eps", ICP_max_transformation_eps_,
              ICP_max_transformation_eps_);
    nhh.param("ICP_max_rot_eps", ICP_max_transformation_rotation_eps_,
              ICP_max_transformation_rotation_eps_);
    nhh.param("ICP_max_eucl_fit_eps", ICP_max_euclidean_fitness_eps_,
              ICP_max_euclidean_fitness_eps_);
    nhh.param("detect_type", detect_type_, detect_type_);
    nhh.param<double>("dock_offset_x", dock_offset_x_, dock_offset_x_);
    nhh.param<double>("dock_offset_y", dock_offset_y_, dock_offset_y_);
    nhh.param<double>("dock_offset_yaw", dock_offset_yaw_, dock_offset_yaw_);
  }

  void processCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (perform_detection_.data == false)
    {
      if (!printed_deactivation_status_)
      {
        ROS_WARN_STREAM("processCloud: DETECTION NOT ACTIVATED");
        printed_deactivation_status_ = true;
      }
      return;
    }

    ros::Time beginCallback = ros::Time::now();
    ROS_INFO_STREAM("processCloud: Processing cloud from source "
                    << current_cloud_source_ << " (topic: "
                    << (current_cloud_source_ == 0   ? cloud1_topic_
                        : current_cloud_source_ == 1 ? cloud2_topic_
                                                     : cloud_merger_topic_)
                    << ")");
    if (msg->width == 0 || msg->row_step == 0)
    {
      ROS_WARN_STREAM("CALLBACK: POINT CLOUD MSG EMPTY");
      detection_attempts_++;
      moveToNextCloud();
      return;
    }

    clearGlobals();
    header_ = msg->header;
    linesPtr_->header = clustersPtr_->header = clusters_.header =
        lines_.header = lines_marker_.header = header_;

    static tf2_ros::TransformBroadcaster tfbr;

    pcl::PointCloud<pcl::PointXYZRGB> cloudPCL;
    pcl::fromROSMsg(*msg, cloudPCL);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPCLPtr(
        new pcl::PointCloud<pcl::PointXYZRGB>(cloudPCL));
    ROS_INFO_STREAM("processCloud: Received cloud with " << cloudPCLPtr->size()
                                                         << " points");

    // Apply z-offset of -0.25m for cloud1 and cloud2
    ROS_WARN_STREAM("____________-----------------____________");
    if (current_cloud_source_ == 0 || current_cloud_source_ == 1)
    {
      for (auto& point : cloudPCLPtr->points)
      {
        point.z += 0.0f;  // Offset z by -0.25m
      }
      ROS_WARN_STREAM("processCloud: Applied +0.0m z-offset to cloud source "
                      << current_cloud_source_);
    }

    // Log z-range
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    for (const auto& point : cloudPCLPtr->points)
    {
      min_z = std::min(min_z, point.z);
      max_z = std::max(max_z, point.z);
    }
    ROS_WARN_STREAM("processCloud: Cloud z-range: min=" << min_z
                                                        << ", max=" << max_z);

    if (cloudPCLPtr->size() == 0)
    {
      ROS_WARN_STREAM("CALLBACK: PCL POINT CLOUD EMPTY");
      detection_attempts_++;
      moveToNextCloud();
      return;
    }

    VoxelGrid(cloudPCLPtr, Voxel_leaf_size_, cloudPCLPtr);
    ROS_INFO_STREAM("processCloud: After VoxelGrid, cloud has "
                    << cloudPCLPtr->size() << " points");

    Clustering clustering;
    clustering.setHeader(header_);
    clustering.ClusterPoints(cloudPCLPtr, clustersPtr_, EC_cluster_tolerance_,
                             EC_min_size_, EC_max_size_);
    ROS_INFO_STREAM("processCloud: Found " << clustersPtr_->clusters.size()
                                           << " clusters");
    if (clustersPtr_->clusters.size() == 0)
    {
      ROS_WARN_STREAM("CALLBACK: NO CLUSTERS FOUND");
      detection_attempts_++;
      moveToNextCloud();
      return;
    }

    LineDetection lineDetection;
    lineDetection.setHeader(header_);
    lineDetection.setParams(RS_max_iter_, RS_min_inliers_, RS_dist_thresh_);
    if (clustersPtr_->clusters.size() > 0)
    {
      lineDetection.getRansacLinesOnCluster(clustersPtr_, linesPtr_);
      ROS_INFO_STREAM("processCloud: Found " << linesPtr_->lines.size()
                                             << " lines");
    }

    if (clustersPtr_->clusters.size() > 0)
    {
      clusters_cloud_pub_.publish(clustersPtr_->combinedCloud);
      clusters_pub_.publish(clustersPtr_);
      lines_cloud_pub_.publish(linesPtr_->combinedCloud);
      lines_pub_.publish(linesPtr_);
      ROS_INFO_STREAM("processCloud: Calling detectDockNearPose");
      detectDockNearPose(last_target_pose_, clustersPtr_);
    }
  }

  double getPCLEuclideanDistance(pcl::PointXYZ p1, pcl::PointXYZ p2)
  {
    double length, x1, x2, y1, y2;
    x1 = p1.x;
    x2 = p2.x;
    y1 = p1.y;
    y2 = p2.y;
    length = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    return length;
  }

  bool isNearDistance(double ref, double inp, double err)
  {
    if (ref > (inp - err) && ref < (inp + err))
    {
      return true;
    }
    return false;
  }

  pcl::PointXYZ
  getAverPoint(typename pcl::PointCloud<pcl::PointXYZ>::Ptr inCloudPtr)
  {
    pcl::PointXYZ averPointPCL;
    double sumX = 0, sumY = 0;
    for (int i = 0; i < inCloudPtr->size(); i++)
    {
      sumX = sumX + inCloudPtr->at(i).x;
      sumY = sumY + inCloudPtr->at(i).y;
    }
    averPointPCL.x = sumX / inCloudPtr->size();
    averPointPCL.y = sumY / inCloudPtr->size();
    averPointPCL.z = 0;
    return averPointPCL;
  }

  void VoxelGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloudPtr,
                 float leafSize,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloudPtr)
  {
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(inCloudPtr);
    voxel.setLeafSize(leafSize, leafSize, leafSize);
    voxel.filter(*filteredCloudPtr);
  }

  void printDebugCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr debugCloudPtr)
  {
    sensor_msgs::PointCloud2 debugCloudMsg;
    pcl::toROSMsg(*debugCloudPtr, debugCloudMsg);
    debugCloudMsg.header = header_;
    debug_pub_.publish(debugCloudMsg);
  }

  void printDebugCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr debugCloudPtr)
  {
    sensor_msgs::PointCloud2 debugCloudMsg;
    pcl::toROSMsg(*debugCloudPtr, debugCloudMsg);
    debugCloudMsg.header = header_;
    debug_pub_.publish(debugCloudMsg);
  }

  bool readPointCloudFile(const std::string& filePath,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCLPtr)
  {
    if (filePath.find(".ply") != std::string::npos)
    {
      ROS_INFO_STREAM("readPointCloudFile: PLY file found");
      if (pcl::io::loadPLYFile(filePath, *PCLPtr) != 0)
      {
        return false;
      }
      ROS_INFO_STREAM("readPointCloudFile: PLY file loaded");
    }
    else if (filePath.find(".pcd") != std::string::npos)
    {
      ROS_INFO_STREAM("readPointCloudFile: PCD file found");
      if (pcl::io::loadPCDFile(filePath, *PCLPtr) != 0)
      {
        return false;
      }
      ROS_INFO_STREAM("readPointCloudFile: PCD file loaded");
    }
    else
    {
      ROS_ERROR_STREAM("readPointCloudFile: Data format not supported.");
      std::cout << "readPointCloudFile: FAILED CLOUD FILE PATH: " << filePath
                << std::endl;
      return false;
    }
    ROS_INFO_STREAM("readPointCloudFile: SUCCESSFULLY Loaded point cloud with "
                    << PCLPtr->height * PCLPtr->width << " points.");
    return true;
  }
};

#endif /*"DETECTIONNODE_H_"*/
