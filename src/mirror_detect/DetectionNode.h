// -*- mode: c++ -*-

#ifndef DETECTIONNODE_H_
#define DETECTIONNODE_H_

#include <docking/Headers.h>
#include <docking/DetectionNodeConfig.h>
#include <ros/package.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>

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
  }
  ~DetectionNode() {}
  //=============================================================================================================================================================================================================================================================================================================================================================
  //
  //  ##   ##    ###    #####    ##    ###    #####   ##      #####
  //  ##   ##   ## ##   ##  ##   ##   ## ##   ##  ##  ##      ##
  //  ##   ##  ##   ##  #####    ##  ##   ##  #####   ##      #####
  //   ## ##   #######  ##  ##   ##  #######  ##  ##  ##      ##
  //    ###    ##   ##  ##   ##  ##  ##   ##  #####   ######  #####
  //
  //=============================================================================================================================================================================================================================================================================================================================================================

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
  ros::Publisher myPoint_min;
  ros::Publisher myPoint_max;

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

  double BAR_length;
  double BAR_length_tolerance;
  double BAR_distance;
  double BAR_distance_tolerance;

  double Leg_length;
  double Leg_distance;
  double Leg_tolerance;
  double Leg_offset;

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
  //! Name of laser frame
  std::string laser_frame_;
  //! Name of laser topic
  std::string laser_topic_;

  std_msgs::Header header_;

  geometry_msgs::TransformStamped base_link_to_leap_motion; // My frames are named "base_link" and "leap_motion"
  bool isGotTf = false;
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

  void calcTf()
  {
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener(tf_buffer);
    base_link_to_leap_motion = tf_buffer.lookupTransform(robot_frame_, laser_frame_, ros::Time(0), ros::Duration(1.0));
    isGotTf = true;
  }

  // Declaration and Definition
  void startDynamicReconfigureServer()
  {
    // Set up a dynamic reconfigure server.
    // Do this before parameter server, else some of the parameter server values
    // can be overwritten.
    ROS_INFO_STREAM("startDynamicReconfigureServer: STARTING DYNAMIC RECONFIGURE SERVER");
    dynamic_reconfigure::Server<docking::DetectionNodeConfig>::CallbackType cb;
    cb = boost::bind(&DetectionNode::configCallback, this, _1, _2);
    dr_srv_.setCallback(cb);
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
    //    std::string dockFilePath("/home/rwbot/dock_ws/src/docking/pcd/dock_cloud_perfect.ply");
    //    std::string dockFilePath("UNSPECIFIED");
    ROS_INFO_STREAM("initDockParams: INITIALIZING TARGET CLOUD FILE PATH");

    // bool dockFilepathExists = nh_.hasParam("dock_filepath");
    bool dockFilepathExists = dockFilePath_ == "" ? false : true;

    if (dockFilepathExists)
    {
      ROS_INFO_STREAM("initDockParams: PARAM SERVER TARGET CLOUD FILE PATH: " << dockFilePath_);

      //    ROS_INFO_STREAM("initDockParams: ASSIGNING POINT CLOUD POINTER");
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr dockTargetPCLPtr(new pcl::PointCloud<pcl::PointXYZRGB>);

      ROS_INFO_STREAM("initDockParams: LOADING DOCK TARGET CLOUD FILE" << dockFilePath_);
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
    //  std::string ns_cloud_topic = "/" + cloud_topic;
    // if(!checkTopicExists(cloud_topic)){
    //   // ROS_WARN_STREAM("Namespaced Topic " + ns_cloud_topic + " does not exist");
    //   ROS_WARN_STREAM("Topic " + cloud_topic + " does not exist");
    //   ROS_WARN_STREAM("Check to see if the topic is correct or if it is namespaced to continue");
    //   return;
    // }
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

  bool checkTopicExists(std::string &topic)
  {
    ROS_INFO_STREAM("Checking existence of new cloud topic " + topic);
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);
    for (int i = 0; i < topic_infos.size(); i++)
    {
      ROS_INFO_STREAM("Check topic #" << i << "  " << topic_infos.at(i).name);
      // if(topic == topic_infos.at(i).name)
      if (topic_infos.at(i).name.find(topic) != std::string::npos)
      {
        return true;
      }
    }
    return false;
  }

  void activationSub()
  {
    // activationSub_ = nh_.subscribe("docking/perform_detection", 1,
    // &DetectionNode::activationCallback, this);
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
    //    linesPtr_->segments.clear();
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

  //==================================================================================================================================================================================================================================================================================================================================================================================================================================================
  //
  //    ###     ####  ######  ##  ##   ##    ###    ######  ##   #####   ##     ##         ####    ###    ##      ##      #####     ###     ####  ##  ##
  //   ## ##   ##       ##    ##  ##   ##   ## ##     ##    ##  ##   ##  ####   ##        ##      ## ##   ##      ##      ##  ##   ## ##   ##     ## ##
  //  ##   ##  ##       ##    ##  ##   ##  ##   ##    ##    ##  ##   ##  ##  ## ##        ##     ##   ##  ##      ##      #####   ##   ##  ##     ####
  //  #######  ##       ##    ##   ## ##   #######    ##    ##  ##   ##  ##    ###        ##     #######  ##      ##      ##  ##  #######  ##     ## ##
  //  ##   ##   ####    ##    ##    ###    ##   ##    ##    ##   #####   ##     ##         ####  ##   ##  ######  ######  #####   ##   ##   ####  ##  ##
  //
  //==================================================================================================================================================================================================================================================================================================================================================================================================================================================

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

  //=======================================================================================================================================================================================================================================================================================================================================================================================================================================================
  //
  //   ####  #####  ##      #####   ####  ######        ######  ##    ##  #####   #####         ####    ###    ##      ##      #####     ###     ####  ##  ##
  //  ##     ##     ##      ##     ##       ##            ##     ##  ##   ##  ##  ##           ##      ## ##   ##      ##      ##  ##   ## ##   ##     ## ##
  //   ###   #####  ##      #####  ##       ##            ##      ####    #####   #####        ##     ##   ##  ##      ##      #####   ##   ##  ##     ####
  //     ##  ##     ##      ##     ##       ##            ##       ##     ##      ##           ##     #######  ##      ##      ##  ##  #######  ##     ## ##
  //  ####   #####  ######  #####   ####    ##            ##       ##     ##      #####         ####  ##   ##  ######  ######  #####   ##   ##   ####  ##  ##
  //
  //=======================================================================================================================================================================================================================================================================================================================================================================================================================================================

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
  //===================================================================================================================================================================================================================================================================================================================================================================================================================
  //
  //   ####   #####   ##     ##  #####  ##   ####           ####    ###    ##      ##      #####     ###     ####  ##  ##
  //  ##     ##   ##  ####   ##  ##     ##  ##             ##      ## ##   ##      ##      ##  ##   ## ##   ##     ## ##
  //  ##     ##   ##  ##  ## ##  #####  ##  ##  ###        ##     ##   ##  ##      ##      #####   ##   ##  ##     ####
  //  ##     ##   ##  ##    ###  ##     ##  ##   ##        ##     #######  ##      ##      ##  ##  #######  ##     ## ##
  //   ####   #####   ##     ##  ##     ##   ####           ####  ##   ##  ######  ######  #####   ##   ##   ####  ##  ##
  //
  //===================================================================================================================================================================================================================================================================================================================================================================================================================

  //! Callback function for dynamic reconfigure server.
  void configCallback(docking::DetectionNodeConfig &config,
                      uint32_t level __attribute__((unused)))
  {
    ROS_INFO_STREAM("Config");

    if (dockFilePath_ != config.dock_filepath)
    {
      dockFilePath_ = config.dock_filepath;
      initDockParams();
    };
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

    Leg_length = config.Leg_length;
    Leg_distance = config.Leg_distance;
    Leg_offset = config.Leg_offset;
    Leg_tolerance = config.Leg_tolerance;

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
    };
    result = nhh.param("world_frame", world_frame_, world_frame_);
    nhh.param("robot_frame", robot_frame_, robot_frame_);
    nhh.param("cloud_frame", cloud_frame_, cloud_frame_);
    nhh.param("cloud_topic", cloud_topic_, cloud_topic_);
    nhh.param("laser_frame", laser_frame_, laser_frame_);
    nhh.param("laser_topic", laser_topic_, laser_topic_);
    nhh.param("world_frame", world_frame_, world_frame_);
    nhh.param("robot_frame", robot_frame_, robot_frame_);
    nhh.param("cloud_frame", cloud_frame_, cloud_frame_);
    nhh.param("cloud_topic", cloud_topic_, cloud_topic_);
    nhh.param("laser_frame", laser_frame_, laser_frame_);
    nhh.param("laser_topic", laser_topic_, laser_topic_);
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
    nhh.param("BAR_length", BAR_length, BAR_length);
    nhh.param("BAR_length_tolerance", BAR_length_tolerance, BAR_length_tolerance);
    nhh.param("BAR_distance", BAR_distance, BAR_distance);
    nhh.param("BAR_distance_tolerance", BAR_distance_tolerance, BAR_distance_tolerance);
    nhh.param("detect_type", detect_type_, detect_type_);
    std::string tempCloud;
    nhh.param("cloud_topic", tempCloud, cloud_topic_);
    if ((tempCloud != "") && (tempCloud != cloud_topic_))
    {
      cloud_topic_ = tempCloud;
      ROS_INFO_STREAM("configCallback: New Input Cloud Topic");
      startCloudSub(cloud_topic_);
    }                           
  }                                                     
  //=============================================================================================================================================================================================================================================================================================================================================================================================================
  //
  //   ####  ##       #####   ##   ##  ####           ####    ###    ##      ##      #####     ###     ####  ##  ##
  //  ##     ##      ##   ##  ##   ##  ##  ##        ##      ## ##   ##      ##      ##  ##   ## ##   ##     ## ##
  //  ##     ##      ##   ##  ##   ##  ##  ##        ##     ##   ##  ##      ##      #####   ##   ##  ##     ####
  //  ##     ##      ##   ##  ##   ##  ##  ##        ##     #######  ##      ##      ##  ##  #######  ##     ## ##
  //   ####  ######   #####    #####   ####           ####  ##   ##  ######  ######  #####   ##   ##   ####  ##  ##
  //
  //=============================================================================================================================================================================================================================================================================================================================================================================================================

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {

    // Only perform detection if activated
    // if (perform_detection_.data == false)
    // {
    //   if (!printed_deactivation_status_)
    //   {
    //     ROS_WARN_STREAM("cloudCallback: DETECTION NOT ACTIVATED");
    //     printed_deactivation_status_ = true;
    //   }
    //   return;
    // }

    ros::Time beginCallback = ros::Time::now();

    // ROS_INFO_STREAM("CLOUD CALLBACK: CALLBACK CALLED ");

    if (msg->width == 0 || msg->row_step == 0)
    {
      ROS_WARN_STREAM("CALLBACK: POINT CLOUD MSG EMPTY ");
      return;
    }

    clearGlobals();

    header_ = msg->header;
    linesPtr_->header = clustersPtr_->header = clusters_.header = lines_.header = lines_marker_.header = header_;

    static tf2_ros::TransformBroadcaster tfbr;

    // Container for original & filtered data
    /*
     * CONVERT POINTCLOUD ROS->PCL
     */
    pcl::PointCloud<pcl::PointXYZRGB> cloudPCL;
    pcl::fromROSMsg(*msg, cloudPCL);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPCLPtr(new pcl::PointCloud<pcl::PointXYZRGB>(cloudPCL));
    if (cloudPCLPtr->size() == 0)
    {
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
    clustering.ClusterPoints(cloudPCLPtr, clustersPtr_, EC_cluster_tolerance_, EC_min_size_, EC_max_size_);
    if (clustersPtr_->clusters.size() == 0)
    {
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
    if (clustersPtr_->clusters.size() > 0)
    {
      lineDetection.getRansacLinesOnCluster(clustersPtr_, linesPtr_);
    }
    /* ========================================
     * PUBLISH CLOUD
     * ========================================*/

    if (clustersPtr_->clusters.size() > 0)
    {
      clusters_cloud_pub_.publish(clustersPtr_->combinedCloud);
      clusters_pub_.publish(clustersPtr_);
      lines_cloud_pub_.publish(linesPtr_->combinedCloud);
      lines_pub_.publish(linesPtr_);
      switch (detect_type_)
      {
      case 1: // Bar marker type
        DetectBarMarker(linesPtr_);
        break;
      case 2:
        DetectLegMarker(clustersPtr_);
        break;
      default:
        DetectICPMarker(clustersPtr_, msg->header.frame_id);
        break;
      }
    }
  }
  ////////////////// BEGIN DETECT BAR MARKING//////////////////////
  //====================================================================================================================================================================================================================================================================================================================================================================
  //
  //  ####    #####  ######  #####   ####  ######        ##   ####  #####
  //  ##  ##  ##       ##    ##     ##       ##          ##  ##     ##  ##
  //  ##  ##  #####    ##    #####  ##       ##          ##  ##     #####
  //  ##  ##  ##       ##    ##     ##       ##          ##  ##     ##
  //  ####    #####    ##    #####   ####    ##          ##   ####  ##
  //
  //====================================================================================================================================================================================================================================================================================================================================================================

  void DetectICPMarker(docking::ClusterArray::Ptr clustersPtr_, std::string frame_id)
  {
    int i = 0;
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

    ICPInputCloudPtr->header.frame_id = header_.frame_id;
    dockTargetPCLPtr_->header.frame_id = header_.frame_id;
    ICPOutCloudPtr->header.frame_id = header_.frame_id;

    if (icpSuccess)
    {
      // ROS_INFO_STREAM("CLOUD CALLBACK: ICP SUCCESSFUL ");
      found_dock_.data = true;

      icp_in_pub_.publish(dockClusterPtr->cloud);
      icp_out_pub_.publish(dockClusterPtr->icpCombinedCloud);
      if ((!isGotTf))
      {
        calcTf();
      }
      geometry_msgs::PoseStamped icpPose;
      tf2::doTransform(dockClusterPtr->icp.poseStamped, icpPose, base_link_to_leap_motion); // robot_pose is the PoseStamped I want to transform

      dock_pose_pub_.publish(icpPose);
      dock_pose_marker_pub_.publish(dockClusterPtr->icp.poseTextMarker);
      //      dock_marker_pub_.publish(dockClusterPtr->bbox.marker);
      dock_marker_pub_.publish(dockClusterPtr->bbox.marker);
      bbox_pub_.publish(dockClusterPtr->bbox);

      dockClusterPtr->icp.transformStamped.header.stamp = ros::Time::now();
      dockClusterPtr->icp.transformStamped.header.frame_id = frame_id;
      dockClusterPtr->icp.transformStamped.child_frame_id = "dock";
      //      tfbr.sendTransform(dockClusterPtr->icp.transformStamped);
      tfBroadcaster_.sendTransform(dockClusterPtr->icp.transformStamped);
      icp_target_pub_.publish(dockTargetPCLPtr_);
    }
    else
    {
      ROS_WARN_STREAM("CLOUD CALLBACK: ICP FAILED ");
      found_dock_.data = false;
    }

    status_pub_.publish(found_dock_);

    std::cout << std::endl;
  }

  //=======================================================================================================================================================================================================================================================================================================================================================================================================================================
  //
  //  ####    #####  ######  #####   ####  ######        #####     ###    #####          ###    ###    ###    #####    ##  ##  #####  #####
  //  ##  ##  ##       ##    ##     ##       ##          ##  ##   ## ##   ##  ##         ## #  # ##   ## ##   ##  ##   ## ##   ##     ##  ##
  //  ##  ##  #####    ##    #####  ##       ##          #####   ##   ##  #####          ##  ##  ##  ##   ##  #####    ####    #####  #####
  //  ##  ##  ##       ##    ##     ##       ##          ##  ##  #######  ##  ##         ##      ##  #######  ##  ##   ## ##   ##     ##  ##
  //  ####    #####    ##    #####   ####    ##          #####   ##   ##  ##   ##        ##      ##  ##   ##  ##   ##  ##  ##  #####  ##   ##
  //
  //=======================================================================================================================================================================================================================================================================================================================================================================================================================================

  void DetectBarMarker(docking::LineArray::Ptr linesPtr_)
  {
    int i, j = 0;
    for (int i = 0; i < linesPtr_->lines.size() - 1; i++)
    {
      sensor_msgs::PointCloud2 _pointCloud2_a = linesPtr_->lines[i].cloud;
      pcl::PCLPointCloud2 _pclPointCloud2_a;
      pcl_conversions::toPCL(_pointCloud2_a, _pclPointCloud2_a);
      pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudXYZ_a(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(_pclPointCloud2_a, *pointCloudXYZ_a);
      docking::MinMaxPoint minmax_a = getMinMaxPointMsg(pointCloudXYZ_a);
      double len_a = getEuclideanDistance(minmax_a.min, minmax_a.max);
      if ((len_a < (BAR_length - BAR_length_tolerance)) || (len_a > (BAR_length + BAR_length_tolerance)))
        continue;
      for (int j = i + 1; j < linesPtr_->lines.size(); j++)
      {
        sensor_msgs::PointCloud2 _pointCloud2_b = linesPtr_->lines[j].cloud;
        pcl::PCLPointCloud2 _pclPointCloud2_b;
        pcl_conversions::toPCL(_pointCloud2_b, _pclPointCloud2_b);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudXYZ_b(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(_pclPointCloud2_b, *pointCloudXYZ_b);
        docking::MinMaxPoint minmax_b = getMinMaxPointMsg(pointCloudXYZ_b);
        double len_b = getEuclideanDistance(minmax_b.min, minmax_b.max);
        if ((len_b < (BAR_length - BAR_length_tolerance)) || (len_b > (BAR_length + BAR_length_tolerance)))
          continue;
        double ang = getAngle(linesPtr_->lines[i], linesPtr_->lines[j]);
        // ROS_INFO_STREAM(ang);
        if (ang > 5)
          continue;
        double barDis = getDistanceToLine(minmax_a.min, minmax_b.min, minmax_b.max);
        // ROS_INFO_STREAM(barDis);
        if ((barDis < (BAR_distance - BAR_distance_tolerance)) || (barDis > (BAR_distance + BAR_distance_tolerance)))
          continue;
        double angVec = getVectorAngle(minmax_a.min, minmax_a.max, minmax_b.min);
        if (angVec < 45)
          continue;
        geometry_msgs::Point detectPoint;
        detectPoint.x = (minmax_a.min.x + minmax_a.max.x + minmax_b.min.x + minmax_b.max.x) / 4;
        detectPoint.y = (minmax_a.min.y + minmax_a.max.y + minmax_b.min.y + minmax_b.max.y) / 4;
        detectPoint.z = (minmax_a.min.z + minmax_a.max.z + minmax_b.min.z + minmax_b.max.z) / 4;

        double yaw;
        yaw = getVectorAngle(minmax_a.min, minmax_a.max);
        geometry_msgs::PoseStamped detectedPoseStamped;
        detectedPoseStamped.header.stamp = ros::Time::now();
        detectedPoseStamped.header.frame_id = laser_frame_;
        detectedPoseStamped.pose.position.x = detectPoint.x;
        detectedPoseStamped.pose.position.y = detectPoint.y;
        detectedPoseStamped.pose.position.z = detectPoint.z;
        geometry_msgs::Quaternion tempQuater = tf::createQuaternionMsgFromYaw(yaw);
        detectedPoseStamped.pose.orientation.x = tempQuater.x;
        detectedPoseStamped.pose.orientation.y = tempQuater.y;
        detectedPoseStamped.pose.orientation.z = tempQuater.z;
        detectedPoseStamped.pose.orientation.w = tempQuater.w;
        // barMarkerPose.publish(detectedPoseStamped);
        dock_pose_pub_.publish(detectedPoseStamped);

        geometry_msgs::PointStamped ps;
        ps.header.stamp = ros::Time::now();
        ps.header.frame_id = laser_frame_;

        ps.point.x = minmax_a.min.x;
        ps.point.y = minmax_a.min.y;
        ps.point.z = minmax_a.min.z;
        myPoint_min.publish(ps);
        ps.point.x = minmax_b.min.x;
        ps.point.y = minmax_b.min.y;
        ps.point.z = minmax_b.min.z;
        myPoint_min.publish(ps);

        ps.point.x = minmax_a.max.x;
        ps.point.y = minmax_a.max.y;
        ps.point.z = minmax_a.max.z;
        myPoint_max.publish(ps);
        ps.point.x = minmax_b.max.x;
        ps.point.y = minmax_b.max.y;
        ps.point.z = minmax_b.max.z;
        myPoint_max.publish(ps);
        ROS_INFO_STREAM("Publish point with length, distance and angle is: " << len_a << ":" << barDis << ":" << ang);
      }
    }
  }
  ////////////////// END DETECT BAR MARKING///////////////////////

  //=====================================================================================================================================================================================================================================================================================================================================================================================================================================
  //
  //  ####    #####  ######  #####   ####  ######        ##      #####   ####          ###    ###    ###    #####    ##  ##  #####  #####
  //  ##  ##  ##       ##    ##     ##       ##          ##      ##     ##             ## #  # ##   ## ##   ##  ##   ## ##   ##     ##  ##
  //  ##  ##  #####    ##    #####  ##       ##          ##      #####  ##  ###        ##  ##  ##  ##   ##  #####    ####    #####  #####
  //  ##  ##  ##       ##    ##     ##       ##          ##      ##     ##   ##        ##      ##  #######  ##  ##   ## ##   ##     ##  ##
  //  ####    #####    ##    #####   ####    ##          ######  #####   ####          ##      ##  ##   ##  ##   ##  ##  ##  #####  ##   ##
  //
  //=====================================================================================================================================================================================================================================================================================================================================================================================================================================

  void DetectLegMarker(docking::ClusterArray::Ptr clustersPtr_)
  {
    pcl::PointCloud<pcl::PointXYZ> centroidPointsPtr;
  geometry_msgs::PoseArray pose_arr;
    pose_arr.header.frame_id = laser_frame_;
    pose_arr.header.stamp = ros::Time::now();
    for (int i = 0; i < clustersPtr_->clusters.size(); i++)
    {
      sensor_msgs::PointCloud2 _pointCloud2 = clustersPtr_->clusters[i].cloud;
      pcl::PCLPointCloud2 _pclPointCloud2;
      pcl_conversions::toPCL(_pointCloud2, _pclPointCloud2);
      pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(_pclPointCloud2, *pointCloudXYZ);
      pcl::PointXYZ _centroid = getAverPoint(pointCloudXYZ);
       geometry_msgs::Pose centerPose;
      centerPose.position.x=_centroid.x;
      centerPose.position.y=_centroid.y;
      pose_arr.poses.push_back(centerPose);
      centroidPointsPtr.push_back(_centroid);
    }
    // /*
    //                       2
    //               1

    //                   c   b

    //                       3
    //               4   a
    // */
    // double a, b, c, d, err;
    // a = sqrt(Leg_distance * Leg_distance + Leg_offset * Leg_offset);
    // b = Leg_length;
    // c = sqrt(Leg_distance * Leg_distance + (Leg_offset + Leg_length) * (Leg_offset + Leg_length));
    // d = sqrt(Leg_distance * Leg_distance + (Leg_length - Leg_offset) * (Leg_length - Leg_offset));
    // err = Leg_tolerance;
    // pcl::PointXYZ p1, p2, p3, p4;
    // int finalPoint = 0;
    // bool isFirstDetect = false;
    // for (int i = 0; i < centroidPointsPtr.size(); i++)
    // {
    //   // set as P1
    //   p1.x = centroidPointsPtr.at(i).x;
    //   p1.y = centroidPointsPtr.at(i).y;
    //   for (int j = 0; j < centroidPointsPtr.size(); j++)
    //   {
    //     if (j == i)
    //       continue;
    //     if (centroidPointsPtr.at(j).y > p1.y)
    //       continue;
    //     double distance = getPCLEuclideanDistance(p1, centroidPointsPtr.at(j));
    //     if (isNearDistance(a, distance, err)) // check distance from P2 to P1
    //     {
    //       // set as P2
    //       p2.x = centroidPointsPtr.at(j).x;
    //       p2.y = centroidPointsPtr.at(j).y;
    //       for (int k = 0; k < centroidPointsPtr.size(); k++)
    //       {
    //         if ((k == i) || (k == j))
    //           continue;
    //         if (centroidPointsPtr.at(k).x > p1.x || centroidPointsPtr.at(k).x > p2.x)
    //           continue;
    //         double firstDistance = getPCLEuclideanDistance(p1, centroidPointsPtr.at(k));  // check distance to P1
    //         double secondDistance = getPCLEuclideanDistance(p2, centroidPointsPtr.at(k)); // check distance to P2

    //         if (isNearDistance(d, firstDistance, err))
    //         {
    //           if (!isNearDistance(b, secondDistance, err))
    //             continue;
    //           p3.x = centroidPointsPtr.at(k).x;
    //           p3.y = centroidPointsPtr.at(k).y;
    //           finalPoint = 3;
    //           isFirstDetect = true;
    //           break;
    //         }
    //         else
    //         {
    //           if (isNearDistance(b, firstDistance, err))
    //           {
    //             if (!isNearDistance(c, secondDistance, err))
    //               continue;
    //             p4.x = centroidPointsPtr.at(k).x;
    //             p4.y = centroidPointsPtr.at(k).y;
    //             finalPoint = 4;
    //             isFirstDetect = true;
    //             break;
    //           }
    //         };
    //       }
    //       if (finalPoint != 0)
    //         break;
    //     }
    //   }
    //   if (finalPoint != 0)
    //     break;
    // }
    // if (finalPoint == 0)
    // {
    //   for (int i = 0; i < centroidPointsPtr.size(); i++)
    //   {
    //     // set as P1
    //     p1.x = centroidPointsPtr.at(i).x;
    //     p1.y = centroidPointsPtr.at(i).y;
    //     for (int j = 0; j < centroidPointsPtr.size(); j++)
    //     {
    //       if (j == i)
    //         continue;
    //       if (centroidPointsPtr.at(j).y > p1.y)
    //         continue;
    //       double distance = getPCLEuclideanDistance(p1, centroidPointsPtr.at(j));
    //       if (isNearDistance(a, distance, err)) // check distance from P2 to P1
    //       {
    //         // set as P2
    //         p2.x = centroidPointsPtr.at(j).x;
    //         p2.y = centroidPointsPtr.at(j).y;
    //         for (int k = 0; k < centroidPointsPtr.size(); k++)
    //         {
    //           if ((k == i) || (k == j))
    //             continue;
    //           if (centroidPointsPtr.at(k).x < p1.x || centroidPointsPtr.at(k).x < p2.x)
    //             continue;
    //           double firstDistance = getPCLEuclideanDistance(p1, centroidPointsPtr.at(k));  // check distance to P1
    //           double secondDistance = getPCLEuclideanDistance(p2, centroidPointsPtr.at(k)); // check distance to P2

    //           if (isNearDistance(c, firstDistance, err))
    //           {
    //             if (!isNearDistance(b, secondDistance, err))
    //               continue;
    //             p3.x = centroidPointsPtr.at(k).x;
    //             p3.y = centroidPointsPtr.at(k).y;
    //             finalPoint = 3;
    //             break;
    //           }
    //           else
    //           {
    //             if (isNearDistance(b, firstDistance, err))
    //             {
    //               if (!isNearDistance(d, secondDistance, err))
    //                 continue;
    //               p4.x = centroidPointsPtr.at(k).x;
    //               p4.y = centroidPointsPtr.at(k).y;
    //               finalPoint = 4;
    //               break;
    //             }
    //           };
    //         }
    //         if (finalPoint != 0)
    //           break;
    //       }
    //     }
    //     if (finalPoint != 0)
    //       break;
    //   }
    // }
    // if (finalPoint != 0)
    // {
    //   ROS_INFO_STREAM("Detected");
    //   geometry_msgs::Point detectPoint;
    //   double yaw;
    //   if (finalPoint == 3)
    //   {
    //     detectPoint.x = (p1.x + p3.x) / 2;
    //     detectPoint.y = (p1.y + p3.y) / 2;
    //     detectPoint.z = 0;
    //     if (!isFirstDetect)
    //     {
    //       yaw = getVectorAngle(p2, p3);
    //     }
    //     else
    //     {
    //       yaw = getVectorAngle(p3, p2);
    //     }
    //   }
    //   else
    //   {
    //     detectPoint.x = (p2.x + p4.x) / 2;
    //     detectPoint.y = (p2.y + p4.y) / 2;
    //     detectPoint.z = 0;
    //     if (!isFirstDetect)
    //     {
    //       yaw = getVectorAngle(p1, p4);
    //     }
    //     else
    //     {
    //       yaw = getVectorAngle(p4, p1);
    //     }
    //   }
    //   geometry_msgs::PoseStamped detectedPoseStamped;
    //   detectedPoseStamped.header.stamp = ros::Time::now();
    //   detectedPoseStamped.header.frame_id = laser_frame_;
    //   detectedPoseStamped.pose.position.x = detectPoint.x;
    //   detectedPoseStamped.pose.position.y = detectPoint.y;
    //   detectedPoseStamped.pose.position.z = detectPoint.z;
    //   geometry_msgs::Quaternion tempQuater = tf::createQuaternionMsgFromYaw(yaw);
    //   detectedPoseStamped.pose.orientation.x = tempQuater.x;
    //   detectedPoseStamped.pose.orientation.y = tempQuater.y;
    //   detectedPoseStamped.pose.orientation.z = tempQuater.z;
    //   detectedPoseStamped.pose.orientation.w = tempQuater.w;
    //   // barMarkerPose.publish(detectedPoseStamped);
    //   dock_pose_pub_.publish(detectedPoseStamped);
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
  ///////////////// BEGIN VOXEL GRID /////////////////
  /// \brief VoxelGrid
  /// \param inCloudPtr
  /// \param leafSize
  /// \return
  ///
  void VoxelGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloudPtr, float leafSize,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloudPtr)
  {
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(inCloudPtr);
    voxel.setLeafSize(leafSize, leafSize, leafSize);
    voxel.filter(*filteredCloudPtr);
  }

  ///////////////// END VOXEL GRID /////////////////

  //====================================================================================================================================================================================================================================================================================================================================================================================================================================
  //
  //  #####   #####    ##  ##     ##  ######        ####    #####  #####   ##   ##   ####           ####  ##       #####   ##   ##  ####
  //  ##  ##  ##  ##   ##  ####   ##    ##          ##  ##  ##     ##  ##  ##   ##  ##             ##     ##      ##   ##  ##   ##  ##  ##
  //  #####   #####    ##  ##  ## ##    ##          ##  ##  #####  #####   ##   ##  ##  ###        ##     ##      ##   ##  ##   ##  ##  ##
  //  ##      ##  ##   ##  ##    ###    ##          ##  ##  ##     ##  ##  ##   ##  ##   ##        ##     ##      ##   ##  ##   ##  ##  ##
  //  ##      ##   ##  ##  ##     ##    ##          ####    #####  #####    #####    ####           ####  ######   #####    #####   ####
  //
  //====================================================================================================================================================================================================================================================================================================================================================================================================================================

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
  //===============================================================================================================================================================================================================================================================================================================================================================================================
  //
  //  #####    #####    ###    ####          #####   ##      ##    ##        #####  ##  ##      #####
  //  ##  ##   ##      ## ##   ##  ##        ##  ##  ##       ##  ##         ##     ##  ##      ##
  //  #####    #####  ##   ##  ##  ##        #####   ##        ####          #####  ##  ##      #####
  //  ##  ##   ##     #######  ##  ##        ##      ##         ##           ##     ##  ##      ##
  //  ##   ##  #####  ##   ##  ####          ##      ######     ##           ##     ##  ######  #####
  //
  //===============================================================================================================================================================================================================================================================================================================================================================================================
  bool readPointCloudFile(const std::string &filePath, pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCLPtr)
  {

    if (filePath.find(".ply") != std::string::npos)
    {
      ROS_INFO_STREAM("readPointCloudFile: PLY file found");
      // Load .ply file.
      if (pcl::io::loadPLYFile(filePath, *PCLPtr) != 0)
      {
        return false;
      }
      ROS_INFO_STREAM("readPointCloudFile: PLY file loaded");
    }
    else if (filePath.find(".pcd") != std::string::npos)
    {
      ROS_INFO_STREAM("readPointCloudFile: PCD file found");
      // Load .pcd file.

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
