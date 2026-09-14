// -*- mode: c++ -*-

#ifndef DETECTIONNODE_H_
#define DETECTIONNODE_H_

#include <docking/Headers.h>
#include <docking/DetectionNodeConfig.h>
#include "docking/DockService.h" // Assumes docking/DockService.srv: geometry_msgs/Pose pose_target, bool enable_detect, bool use_scan_merge --- bool success
#include <ros/package.h>
#include <ros/service.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> POINT;

// namespace docking {

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
    ros::Duration(1).sleep(); // Wait for pointcloud_to_laserscan node to init and publish cloud topic
    startCloudSub(cloud_topic_);
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
  ros::Publisher bbox_pub_;       // Bounding box publisher
  ros::Publisher debug_pub_;      // debug publisher
  ros::Publisher icp_in_pub_;     // ICP Input Cloud publisher
  ros::Publisher icp_target_pub_; // ICP Target Cloud publisher
  ros::Publisher icp_out_pub_;    // ICP Output Cloud publisher
  ros::Subscriber cloudSub_;
  ros::Subscriber activationSub_;
  ros::Subscriber selectTypeSub;
  ros::Subscriber scanSub1_;      // Laser 1 scan subscriber
  ros::Subscriber scanSub2_;      // Laser 2 scan subscriber
  ros::Subscriber scanSubMerger_; // Laser merger scan subscriber
  ros::Publisher myPoint_min;
  ros::Publisher myPoint_max;
  ros::ServiceServer dock_service_; // Service for dock detection

  ros::NodeHandle nh_;
  //! Dynamic reconfigure server.
  dynamic_reconfigure::Server<docking::DetectionNodeConfig> dr_srv_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2_ros::TransformBroadcaster tfBroadcaster_;

  //! Target Dock PCL Cloud Pointer
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr dockTargetPCLPtr_;

  //! Global list of line markers for publisher
  visualization_msgs::Marker lines_marker_;
  //! Global list of line msgs for publisher
  docking::LineArray lines_;
  //! Global list of cluster msgs for publisher
  docking::LineArray::Ptr linesPtr_;
  //! Global list of cluster msgs for publisher
  docking::ClusterArray clusters_;
  //! Global list of cluster msgs for publisher
  docking::ClusterArray::Ptr clustersPtr_;

  //! RANSAC Maximum Iterations
  int RS_max_iter_;
  //! RANSAC Minimum Inliers
  int RS_min_inliers_;
  //! RANSAC Distance Threshold
  double RS_dist_thresh_;
  //! Perform RANSAC after Clustering Points
  bool RANSAC_on_clusters_;

  //! EuclideanCluster Tolerance (m)
  double EC_cluster_tolerance_;
  //! EuclideanCluster Min Cluster Size
  int EC_min_size_;
  //! EuclideanCluster Max Cluster Size
  int EC_max_size_;

  int detect_type_;
  //! Bool of dock search status
  std_msgs::Bool found_dock_;
  //! Bool of detection node status
  std_msgs::Bool perform_detection_;
  //! String for select marker type
  std_msgs::String selectType;
  //! Bool of having printed detection node status
  bool printed_deactivation_status_;

  //! Dock Wing Length
  double dock_wing_length;

  //! Dock target template filepath
  std::string dockFilePath_;
  bool dock_template_loaded_;
  //! ICP Fitness Score Threshold
  double ICP_min_score_;
  //! ICP The maximum distance threshold between two correspondent points
  double ICP_max_correspondence_distance_;
  //! ICP Max iterations
  double ICP_max_iterations_;
  //! ICP Max transformation translation epsilon (diff) btwn previous & current estimated
  double ICP_max_transformation_eps_;
  //! ICP Max transformation rotation epsilon (diff) btwn previous & current estimated
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
  //! Name of laser 1 frame
  std::string laser1_frame_;
  //! Name of laser 1 topic
  std::string laser1_topic_;
  //! Name of laser 2 frame
  std::string laser2_frame_;
  //! Name of laser 2 topic
  std::string laser2_topic_;
  //! Name of laser merger frame
  std::string laser_merger_frame_;
  //! Name of laser merger topic
  std::string laser_merger_topic_;

  std_msgs::Header header_;

  geometry_msgs::TransformStamped base_link_to_laser1; // Transform from base_link to laser1
  geometry_msgs::TransformStamped base_link_to_laser2; // Transform from base_link to laser2
  geometry_msgs::TransformStamped base_link_to_laser_merger; // Transform from base_link to laser_merger
  bool isGotTf1 = false;
  bool isGotTf2 = false;
  bool isGotTfMerger = false;

  double max_dock_distance_; // Maximum distance to consider dock as nearby (0.5m)
  double dock_offset_x_; // Dock X offset (5 cm)
  double dock_offset_y_; // Dock Y offset (5 cm)
  double dock_offset_yaw_; // Dock Yaw offset (8 degrees in radians)

  geometry_msgs::PoseStamped last_target_pose_; // Store last target pose from service call
  
  //! Current laser source: 0 = laser1, 1 = laser2, 2 = laser_merger
  int current_laser_source_;
  
  //! Latest cloud data from different sources
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud_;

  ///////////////// END VARIABLES /////////////////

  void calcTf()
  {
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener(tf_buffer);
    
    try
    {
      //get the transform from base_link -> laser
      base_link_to_laser1 = tf_buffer.lookupTransform(robot_frame_, laser1_frame_, ros::Time(0), ros::Duration(1.0));
      isGotTf1 = true;
      ROS_INFO_STREAM("calcTf: Got transform base_link -> laser1");
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN_STREAM("calcTf: Failed to get transform base_link -> laser1: " << ex.what());
    }
    
    try
    {
      base_link_to_laser2 = tf_buffer.lookupTransform(robot_frame_, laser2_frame_, ros::Time(0), ros::Duration(1.0));
      isGotTf2 = true;
      ROS_INFO_STREAM("calcTf: Got transform base_link -> laser2");
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN_STREAM("calcTf: Failed to get transform base_link -> laser2: " << ex.what());
    }
    
    try
    {
      base_link_to_laser_merger = tf_buffer.lookupTransform(robot_frame_, laser_merger_frame_, ros::Time(0), ros::Duration(1.0));
      isGotTfMerger = true;
      ROS_INFO_STREAM("calcTf: Got transform base_link -> laser_merger");
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN_STREAM("calcTf: Failed to get transform base_link -> laser_merger: " << ex.what());
    }
  }

  void startDynamicReconfigureServer()
  {
    ROS_INFO_STREAM("startDynamicReconfigureServer: STARTING DYNAMIC RECONFIGURE SERVER");
    dynamic_reconfigure::Server<docking::DetectionNodeConfig>::CallbackType cb;
    cb = boost::bind(&DetectionNode::configCallback, this, _1, _2);
    dr_srv_.setCallback(cb);
  }

  void startDockService()
  {
    dock_service_ = nh_.advertiseService("dock_service", &DetectionNode::dockServiceCallback, this);
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
      ROS_INFO_STREAM("initDockParams: PARAM SERVER TARGET CLOUD FILE PATH: " << dockFilePath_);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr dockTargetPCLPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
      ROS_INFO_STREAM("initDockParams: LOADING DOCK TARGET CLOUD FILE" << dockFilePath_);
      if (readPointCloudFile(dockFilePath_, dockTargetPCLPtr) == false) //read the template cloud file
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
      ROS_ERROR_STREAM("initDockParams: NO PARAMETER dock_filepath EXISTS FOR TARGET CLOUD FILE PATH");
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

    current_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    current_laser_source_ = 0; // Default to laser1
  }

  void startPub()
  {
    status_pub_ = nh_.advertise<std_msgs::Bool>("docking/found_dock", 1);
    myPoint_min = nh_.advertise<geometry_msgs::PointStamped>("docking/point_min", 1);
    myPoint_max = nh_.advertise<geometry_msgs::PointStamped>("docking/point_max", 1);
    clusters_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("docking/clustersCloud", 1);
    clusters_pub_ = nh_.advertise<docking::ClusterArray>("docking/clusters", 1);
    lines_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("docking/linesCloud", 1);
    lines_pub_ = nh_.advertise<docking::LineArray>("docking/lines", 1);
    line_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("docking/lines_marker", 1);
    dock_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("docking/dock_marker", 1);
    dock_pose_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("docking/dock_pose_marker", 1);
    dock_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("docking/dock_pose", 1);
    bbox_pub_ = nh_.advertise<docking::BoundingBox>("docking/bbox", 1);
    debug_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("docking/debugCloud", 1);
    icp_in_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("docking/icp_in_pub", 1);
    icp_target_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("docking/icp_target_pub", 1);
    icp_out_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("docking/icp_out_pub", 1);
  }

  void startCloudSub(std::string cloud_topic)
  {
    ROS_INFO_STREAM("Subscribing to new cloud topic " + cloud_topic_);
    cloudSub_ = nh_.subscribe(cloud_topic, 1, &DetectionNode::cloudCallback, this);
    if (cloudSub_)
    {
      ROS_INFO_STREAM("SUCCESSFULLY subscribed to new cloud topic " + cloud_topic);
    }
    else
    {
      ROS_WARN_STREAM("FAILED to subscribe to new cloud topic " + cloud_topic);
    }
  }

  void startLaserSubs()
  {
    ROS_INFO_STREAM("Starting laser subscribers");
    scanSub1_ = nh_.subscribe(laser1_topic_, 1, &DetectionNode::scanCallback1, this);
    scanSub2_ = nh_.subscribe(laser2_topic_, 1, &DetectionNode::scanCallback2, this);
    scanSubMerger_ = nh_.subscribe(laser_merger_topic_, 1, &DetectionNode::scanCallbackMerger, this);
  }

  void scanCallback1(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    if (current_laser_source_ != 0)
      return;
    
    sensor_msgs::PointCloud2Ptr scan_cloud(new sensor_msgs::PointCloud2);
    static laser_geometry::LaserProjection projector;
    projector.projectLaser(*msg, *scan_cloud);
    
    pcl::PointCloud<pcl::PointXYZRGB> cloudPCL;
    pcl::fromROSMsg(*scan_cloud, cloudPCL);
    *current_cloud_ = cloudPCL;
    
    // Process the cloud
    cloudCallback(scan_cloud);
  }

  void scanCallback2(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    if (current_laser_source_ != 1)
      return;
    
    sensor_msgs::PointCloud2Ptr scan_cloud(new sensor_msgs::PointCloud2);
    static laser_geometry::LaserProjection projector;
    projector.projectLaser(*msg, *scan_cloud);
    
    pcl::PointCloud<pcl::PointXYZRGB> cloudPCL;
    pcl::fromROSMsg(*scan_cloud, cloudPCL);
    *current_cloud_ = cloudPCL;
    
    // Process the cloud
    cloudCallback(scan_cloud);
  }

  void scanCallbackMerger(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    if (current_laser_source_ != 2)
      return;
    
    sensor_msgs::PointCloud2Ptr scan_cloud(new sensor_msgs::PointCloud2);
    static laser_geometry::LaserProjection projector;
    projector.projectLaser(*msg, *scan_cloud);
    
    pcl::PointCloud<pcl::PointXYZRGB> cloudPCL;
    pcl::fromROSMsg(*scan_cloud, cloudPCL);
    *current_cloud_ = cloudPCL;
    
    // Process the cloud
    cloudCallback(scan_cloud);
  }

  bool checkTopicExists(std::string &topic)
  {
    ROS_INFO_STREAM("Checking existence of new cloud topic " + topic);
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);
    for (int i = 0; i < topic_infos.size(); i++)
    {
      ROS_INFO_STREAM("Check topic #" << i << "  " << topic_infos.at(i).name);
      if (topic_infos.at(i).name.find(topic) != std::string::npos)
      {
        return true;
      }
    }
    return false;
  }

  void activationSub()
  {
    activationSub_ = nh_.subscribe("docking/perform_detection", 1, &DetectionNode::activationCallback, this);
    selectTypeSub = nh_.subscribe("docking/select_type", 1, &DetectionNode::selectTypeCallback, this);
  }

  void clearGlobals()
  {
    lines_.lines.clear();
    lines_marker_.points.clear();
    lines_marker_.colors.clear();
    clustersPtr_->clusters.clear();
    linesPtr_->lines.clear();
  }

  bool dockServiceCallback(docking::DockService::Request &req, docking::DockService::Response &res)
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

    // Set laser source based on use_scan_merge flag
    if (req.use_scan_merge)
    {
      current_laser_source_ = 2; // Use laser_merger
      ROS_INFO_STREAM("dockServiceCallback: Using laser merger (scan_map)");
    }
    else
    {
      current_laser_source_ = 0; // Default to laser1
      ROS_INFO_STREAM("dockServiceCallback: Using laser 1");
    }

    if (req.enable_detect)
    {
      // Transform pose_target to laser1_frame_
      geometry_msgs::PoseStamped input_pose;
      input_pose.header.frame_id = "map";
      input_pose.header.stamp = ros::Time::now();
      input_pose.pose = req.pose_target;

      try
      {
        // Transform input pose (map) -> last target pose (laser1_frame)
        tfBuffer_.transform(input_pose, last_target_pose_, laser1_frame_, ros::Duration(1.0));
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN_STREAM("dockServiceCallback: Transform failed: " << ex.what());
        return true;
      }
    }

    return true;
  }

  void detectDockNearPose(const geometry_msgs::PoseStamped &target_pose, docking::ClusterArray::Ptr clustersPtr_)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ICPInputCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ICPOutCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    docking::Cluster::Ptr dockClusterPtr(new docking::Cluster());
    PoseEstimation poseEstimation;
    poseEstimation.setHeader(header_);
    poseEstimation.setICPScore(ICP_min_score_);
    poseEstimation.setMaxIterations(ICP_max_iterations_);
    poseEstimation.setMaxCorrespondenceDistance(ICP_max_correspondence_distance_);
    poseEstimation.setMaxTransformationEps(ICP_max_transformation_eps_);
    poseEstimation.setMaxEuclideanFitnessEps(ICP_max_euclidean_fitness_eps_);

    bool icpSuccess = poseEstimation.clusterArrayICP(clustersPtr_, dockTargetPCLPtr_, dockClusterPtr);
    ROS_WARN_STREAM("CLOUD CALLBACK: ICP " << icpSuccess);
    ICPInputCloudPtr->header.frame_id = header_.frame_id;
    dockTargetPCLPtr_->header.frame_id = header_.frame_id;
    ICPOutCloudPtr->header.frame_id = header_.frame_id;

    if (icpSuccess)
    {
      ROS_WARN_STREAM("-----------------------------------------------------------------------------------------");
      found_dock_.data = true;

      icp_in_pub_.publish(dockClusterPtr->cloud);
      icp_out_pub_.publish(dockClusterPtr->icpCombinedCloud);
      if (!isGotTf1 && !isGotTf2 && !isGotTfMerger)
      {
        calcTf();
      }

      // Select the appropriate transform based on current laser source
      geometry_msgs::TransformStamped current_base_link_to_laser;
      if (current_laser_source_ == 0)
      {
        current_base_link_to_laser = base_link_to_laser1;
      }
      else if (current_laser_source_ == 1)
      {
        current_base_link_to_laser = base_link_to_laser2;
      }
      else if (current_laser_source_ == 2)
      {
        current_base_link_to_laser = base_link_to_laser_merger;
      }

      // Transform dock pose from laser frame to base_link
      geometry_msgs::PoseStamped icpPose;
      tf2::doTransform(
          dockClusterPtr->icp.poseStamped, icpPose,
          current_base_link_to_laser);
      
      dock_pose_pub_.publish(icpPose);
      dock_pose_marker_pub_.publish(dockClusterPtr->icp.poseTextMarker);
      dock_marker_pub_.publish(dockClusterPtr->bbox.marker);
      bbox_pub_.publish(dockClusterPtr->bbox);

      // Create transform from base_link to dock (instead of laser_frame to dock)
      dockClusterPtr->icp.transformStamped.header.stamp = ros::Time::now();
      dockClusterPtr->icp.transformStamped.header.frame_id = robot_frame_; // base_link frame
      dockClusterPtr->icp.transformStamped.child_frame_id = "dock";

      // Apply offset
      tf2::Transform offset;
      ROS_WARN_STREAM("offset: " << dock_offset_x_ << " and " << dock_offset_y_);
      offset.setOrigin(tf2::Vector3(dock_offset_x_, dock_offset_y_, 0.0));
      tf2::Quaternion q;
      q.setRPY(0, 0, dock_offset_yaw_);
      offset.setRotation(q);

      // Get the ICP transform in laser_frame
      tf2::Transform tf_icp;
      tf2::fromMsg(dockClusterPtr->icp.transformStamped.transform, tf_icp);

      // Transform from laser frame to base_link
      tf2::Transform tf_laser_to_base;
      tf2::fromMsg(current_base_link_to_laser.transform, tf_laser_to_base);
      tf_laser_to_base = tf_laser_to_base.inverse(); // Invert to get laser frame to base_link

      // Combine transforms: base_link -> dock = base_link -> laser_frame * laser_frame -> dock
      tf2::Transform tf_base_to_dock = tf_laser_to_base * tf_icp;
      tf2::Transform tf_with_offset = tf_base_to_dock * offset;

      geometry_msgs::TransformStamped dock_tf_with_offset =
          dockClusterPtr->icp.transformStamped;
      dock_tf_with_offset.transform = tf2::toMsg(tf_with_offset);

      // Calculate Euclidean distance between new transform and target pose
      double distance = sqrt(
          pow(dock_tf_with_offset.transform.translation.x - target_pose.pose.position.x, 2) +
          pow(dock_tf_with_offset.transform.translation.y - target_pose.pose.position.y, 2) +
          pow(dock_tf_with_offset.transform.translation.z - target_pose.pose.position.z, 2));

      if (distance <= max_dock_distance_)
      {
        // Broadcast the modified transform only if within max_dock_distance_
        tfBroadcaster_.sendTransform(dock_tf_with_offset);
      }
      else
      {
        ROS_INFO_STREAM("detectDockNearPose: ICP successful but dock too far: " << distance << "m");
        found_dock_.data = false;
      }
    }
    else
    {
      ROS_WARN_STREAM("detectDockNearPose: ICP failed");
      found_dock_.data = false;
    }

    status_pub_.publish(found_dock_);
  }

  void activationCallback(const std_msgs::BoolConstPtr &msg)
  {
    if (perform_detection_.data == msg->data)
    {
      return;
    }
    perform_detection_ = *msg;
    ROS_WARN_STREAM("SETTING DETECTION ACTIVATION STATUS TO  " << perform_detection_);
    printed_deactivation_status_ = false;
  }

  void selectTypeCallback(const std_msgs::StringConstPtr &msg)
  {
    ROS_INFO_STREAM("Current type: " + selectType.data + ", Set type: " + msg->data);
    if (selectType.data == msg->data)
      return;
    selectType.data = msg->data;
    ROS_INFO_STREAM("Current type: " + selectType.data);
    std::string filePathStr = selectType.data;
    boost::filesystem::path cfg_file_path = boost::filesystem::path(filePathStr);
    namespace fsys = boost::filesystem;
    if (!fsys::exists(cfg_file_path) ||
        !(fsys::is_regular_file(cfg_file_path) || fsys::is_symlink(cfg_file_path)))
    {
      ROS_WARN_STREAM("Could not open marker param file " << cfg_file_path.string());
    }
    else
    {
      std::string command = "rosparam load " + cfg_file_path.string() + " " + ros::this_node::getName();
      int result = std::system(command.c_str());
      if (result != 0)
      {
        ROS_WARN_STREAM("Could not set config file " << cfg_file_path.string()
                                                     << " to the parameter server.");
      }
      else
      {
        ROS_WARN_STREAM("Load param");
        loadParams();
      }
    }
  }

  void configCallback(docking::DetectionNodeConfig &config, uint32_t level __attribute__((unused)))
  {
    ROS_INFO_STREAM("Config");
    if (dockFilePath_ != config.dock_filepath)
    {
      dockFilePath_ = config.dock_filepath;
      initDockParams();
    }
    world_frame_ = config.world_frame;
    robot_frame_ = config.robot_frame;
    cloud_frame_ = config.cloud_frame;
    cloud_topic_ = config.cloud_topic;
    laser1_frame_ = config.laser1_frame;
    laser1_topic_ = config.laser1_topic;
    laser2_frame_ = config.laser2_frame;
    laser2_topic_ = config.laser2_topic;
    laser_merger_frame_ = config.laser_merger_frame;
    laser_merger_topic_ = config.laser_merger_topic;
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
    if ((config.cloud_topic != "") && (config.cloud_topic != cloud_topic_))
    {
      cloud_topic_ = config.cloud_topic;
      ROS_INFO_STREAM("configCallback: New Input Cloud Topic");
      startCloudSub(cloud_topic_);
    }
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
    nhh.param("cloud_frame", cloud_frame_, cloud_frame_);
    nhh.param("cloud_topic", cloud_topic_, cloud_topic_);
    nhh.param("laser1_frame", laser1_frame_, laser1_frame_);
    nhh.param("laser1_topic", laser1_topic_, laser1_topic_);
    nhh.param("laser2_frame", laser2_frame_, laser2_frame_);
    nhh.param("laser2_topic", laser2_topic_, laser2_topic_);
    nhh.param("laser_merger_frame", laser_merger_frame_, laser_merger_frame_);
    nhh.param("laser_merger_topic", laser_merger_topic_, laser_merger_topic_);
    nhh.param("RS_max_iter", RS_max_iter_, RS_max_iter_);
    nhh.param("RS_min_inliers", RS_min_inliers_, RS_min_inliers_);
    nhh.param("RS_dist_thresh", RS_dist_thresh_, RS_dist_thresh_);
    nhh.param("RANSAC_on_clusters", RANSAC_on_clusters_, RANSAC_on_clusters_);
    nhh.param("Voxel_leaf_size", Voxel_leaf_size_, Voxel_leaf_size_);
    nhh.param("EC_cluster_tolerance", EC_cluster_tolerance_, EC_cluster_tolerance_);
    nhh.param("EC_min_size", EC_min_size_, EC_min_size_);
    nhh.param("EC_max_size", EC_max_size_, EC_max_size_);
    nhh.param("ICP_min_score", ICP_min_score_, ICP_min_score_);
    nhh.param("ICP_max_corres_dist", ICP_max_correspondence_distance_, ICP_max_correspondence_distance_);
    nhh.param("ICP_max_iter", ICP_max_iterations_, ICP_max_iterations_);
    nhh.param("ICP_max_transl_eps", ICP_max_transformation_eps_, ICP_max_transformation_eps_);
    nhh.param("ICP_max_rot_eps", ICP_max_transformation_rotation_eps_, ICP_max_transformation_rotation_eps_);
    nhh.param("ICP_max_eucl_fit_eps", ICP_max_euclidean_fitness_eps_, ICP_max_euclidean_fitness_eps_);
    nhh.param("detect_type", detect_type_, detect_type_);
    nhh.param<double>("dock_offset_x", dock_offset_x_, dock_offset_x_);
    nhh.param<double>("dock_offset_y", dock_offset_y_, dock_offset_y_);
    nhh.param<double>("dock_offset_yaw", dock_offset_yaw_, dock_offset_yaw_);
    std::string tempCloud;
    nhh.param("cloud_topic", tempCloud, cloud_topic_);
    if ((tempCloud != "") && (tempCloud != cloud_topic_))
    {
      cloud_topic_ = tempCloud;
      ROS_INFO_STREAM("loadParams: New Input Cloud Topic");
      startCloudSub(cloud_topic_);
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    if (perform_detection_.data == false)
    {
      if (!printed_deactivation_status_)
      {
        ROS_WARN_STREAM("cloudCallback: DETECTION NOT ACTIVATED");
        printed_deactivation_status_ = true;
      }
      return;
    }

    ros::Time beginCallback = ros::Time::now();
    if (msg->width == 0 || msg->row_step == 0)
    {
      ROS_WARN_STREAM("CALLBACK: POINT CLOUD MSG EMPTY ");
      return;
    }

    clearGlobals();
    header_ = msg->header;
    linesPtr_->header = clustersPtr_->header = clusters_.header = lines_.header = lines_marker_.header = header_;

    static tf2_ros::TransformBroadcaster tfbr;

    pcl::PointCloud<pcl::PointXYZRGB> cloudPCL;
    pcl::fromROSMsg(*msg, cloudPCL);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPCLPtr(new pcl::PointCloud<pcl::PointXYZRGB>(cloudPCL));
    if (cloudPCLPtr->size() == 0)
    {
      ROS_WARN_STREAM("CALLBACK: PCL POINT CLOUD EMPTY ");
      return;
    }

    VoxelGrid(cloudPCLPtr, Voxel_leaf_size_, cloudPCLPtr);

    Clustering clustering;
    clustering.setHeader(header_);
    clustering.ClusterPoints(cloudPCLPtr, clustersPtr_, EC_cluster_tolerance_, EC_min_size_, EC_max_size_);
    if (clustersPtr_->clusters.size() == 0)
    {
      ROS_WARN_STREAM("CALLBACK: NO CLUSTERS FOUND ");
      return;
    }

    LineDetection lineDetection;
    lineDetection.setHeader(header_);
    lineDetection.setParams(RS_max_iter_, RS_min_inliers_, RS_dist_thresh_);
    if (clustersPtr_->clusters.size() > 0)
    {
      lineDetection.getRansacLinesOnCluster(clustersPtr_, linesPtr_);
    }

    if (clustersPtr_->clusters.size() > 0)
    {
      clusters_cloud_pub_.publish(clustersPtr_->combinedCloud);
      clusters_pub_.publish(clustersPtr_);
      lines_cloud_pub_.publish(linesPtr_->combinedCloud);
      lines_pub_.publish(linesPtr_);
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

  pcl::PointXYZ getAverPoint(typename pcl::PointCloud<pcl::PointXYZ>::Ptr inCloudPtr)
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

  void VoxelGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloudPtr, float leafSize,
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

  bool readPointCloudFile(const std::string &filePath, pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCLPtr)
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
      std::cout << "readPointCloudFile: FAILED CLOUD FILE PATH: " << filePath << std::endl;
      return false;
    }
    ROS_INFO_STREAM("readPointCloudFile: SUCCESSFULLY Loaded point cloud with " << PCLPtr->height * PCLPtr->width << " points.");
    return true;
  }
};

//} // namespace docking

#endif /*"DETECTIONNODE_H_"*/