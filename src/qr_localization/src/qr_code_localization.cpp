#include <qr_localization/qr_code_localization.h>

namespace qr_code_localization
{
using json = nlohmann::json;
QrLocalization::QrLocalization(ros::NodeHandle nh) : nh_(nh)
{
  sub_odom_ = nh_.subscribe("/odom", 1, &QrLocalization::sub_odom_cb, this);
  sub_qr_code_ =
      nh_.subscribe("/data_gls621", 1, &QrLocalization::sub_qr_code_cb, this);
  sub_robot_stt_ = nh_.subscribe("/robot_status", 1,
                                 &QrLocalization::sub_robot_stt_cb, this);
  sub_cmd_vel_ =
      nh_.subscribe("/cmd_vel", 1, &QrLocalization::cmdVelCallback, this);
  robot_pose =
      nh_.subscribe("/robot_pose", 1, &QrLocalization::robot_pose_cb, this);
  sub_control_tf_ =
      nh_.subscribe("/control_tf", 1, &QrLocalization::controlTFCallback, this);
  sub_lidar_info_ =
      nh_.subscribe("/lidar_info", 1, &QrLocalization::lidar_info_cb, this);
  sub_current_control_tf_ =
      nh_.subscribe("/current_control_tf", 1, &QrLocalization::current_control_tf_cb, this);
  sub_status_detect_mirror_ =
      nh_.subscribe("/status_detect_mirror", 1, &QrLocalization::status_detect_mirror_cb, this);

  pub_current_control_tf_ =
      nh_.advertise<std_msgs::Int8>("/current_control_tf", 1);
  pub_initpose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "/initialpose", 1);
  m_led_turn_pub_ = nh_.advertise<std_stamped_msgs::Int16MultiArrayStamped>(
      "/arduino_driver/led_turn", 1);

  angleRotateCam_ = nh.param<double>("angle_cam_qr", 0.05);
  transXCam_ = nh.param<double>("transform_x_cam_qr", 0.0);
  transYCam_ = nh.param<double>("transform_y_cam_qr", 0.0);
  newDataQR_ = false;
  newDataOdom_ = false;
  firstInitPose_ = true;
  led_turn_msg.data.resize(4);
  cmdAngularZ_ = 0.0;
  msg_type_control_tf_.data = typeQRTF;
  
  // Initialize lidar info variables
  has_lidar_info_ = false;
  lidar_scale_ = 1.0;
  lidar_offset_x_ = 0.0;
  lidar_offset_y_ = 0.0;
  lidar_angle_ = 0.0;
  
  // Initialize robot pose variables
  pose_x = 0.0;
  pose_y = 0.0;
  pose_yaw = 0.0;
  current_control_tf_ = 0;
  
  // Initialize last published pose
  has_published_pose_ = false;
  last_published_x_ = 0.0;
  last_published_y_ = 0.0;
  last_published_yaw_ = 0.0;
  
  // Initialize timing
  start_time_ = ros::Time::now().toSec();
  last_pose_check_time_ = ros::Time::now().toSec();
  time_update_QR_ = 0.0;
}

QrLocalization::~QrLocalization() {}

void QrLocalization::cmdVelCallback(const geometry_msgs::Twist::ConstPtr msg)
{
  timeGetCmd_ = ros::Time::now().toSec();
  cmdAngularZ_ = msg->angular.z;
  cmdLinearX_ = msg->linear.x;
}

void QrLocalization::sub_odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  odomPose_ = msg->pose.pose;
}

void QrLocalization::sub_qr_code_cb(
    const agv_msgs::DataMatrixStamped::ConstPtr& msg)
{
  dataGls621_ = *msg;
  newDataQR_ = true;
}

void QrLocalization::controlTFCallback(const std_msgs::Int8::ConstPtr& msg)
{
  typePublishTF = msg->data;
}

void QrLocalization::sub_robot_stt_cb(
    const std_stamped_msgs::StringStamped::ConstPtr& msg)
{
  std::string data = msg->data;
  try
  {
    json jsonData = json::parse(data);
    statusRobot_ = jsonData["status"];
    modeRobot_ = jsonData["mode"];
    if (statusRobot_ == "WAITING_INIT_POSE")
    {
      firstInitPose_ = true;
      // Reset start time when robot enters WAITING_INIT_POSE
      // start_time_ = ros::Time::now().toSec();
    }
  }
  catch (...)
  {
    std::cerr << "Lỗi chuyển đổi JSON: " << std::endl;
  }
}

void QrLocalization::robot_pose_cb(const geometry_msgs::Pose::ConstPtr& msg)
{
  pose_x = msg->position.x;
  pose_y = msg->position.y;
  pose_yaw = quaternionToRPY(msg->orientation.x, msg->orientation.y, 
                             msg->orientation.z, msg->orientation.w);
}

void QrLocalization::lidar_info_cb(const std_stamped_msgs::StringStamped::ConstPtr& msg)
{
  std::string data = msg->data;
  try
  {
    json lidar_data = json::parse(data);
    ROS_INFO_STREAM("Received lidar info: " << lidar_data.dump());
    
    // Kiểm tra có đủ thông tin cần thiết không
    if (lidar_data.contains("scale") && 
        lidar_data.contains("offset") && 
        lidar_data["offset"].contains("x") && 
        lidar_data["offset"].contains("y") &&
        lidar_data.contains("angle"))
    {
      lidar_scale_ = lidar_data["scale"].get<double>();
      lidar_offset_x_ = lidar_data["offset"]["x"].get<double>();
      lidar_offset_y_ = lidar_data["offset"]["y"].get<double>();
      lidar_angle_ = lidar_data["angle"].get<double>();  // Giá trị angle theo radian
      has_lidar_info_ = true;
      lidar_info_ = lidar_data;
      
      // ROS_INFO("Updated lidar conversion params: scale=%f, offset=(%f, %f), angle=%f",
      //          lidar_scale_, lidar_offset_x_, lidar_offset_y_, lidar_angle_);
    }
    else
    {
      ROS_WARN("Incomplete lidar info received, missing required fields");
      has_lidar_info_ = false;
    }
  }
  catch (const json::parse_error& e)
  {
    ROS_ERROR_STREAM("Error parsing lidar info JSON: " << e.what());
    has_lidar_info_ = false;
  }
  catch (const json::type_error& e)
  {
    ROS_ERROR_STREAM("Error parsing lidar info type: " << e.what());
    has_lidar_info_ = false;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Error parsing lidar info: " << e.what());
    has_lidar_info_ = false;
  }
}

void QrLocalization::current_control_tf_cb(const std_msgs::Int8::ConstPtr& msg)
{
  current_control_tf_ = msg->data;
  ROS_DEBUG("Received current_control_tf: %d", current_control_tf_);
}

void QrLocalization::status_detect_mirror_cb(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "success")
  {
    time_update_QR_ = ros::Time::now().toSec();
    // ROS_INFO("Mirror detection success - updated time_update_QR");
  }
  // else
  // {
  //   ROS_DEBUG("Mirror detection status: %s", msg->data.c_str());
  // }
}

double QrLocalization::quaternionToRPY(double qx, double qy, double qz,
                                       double qw)
{
  // The incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  tf2::Quaternion quat;
  quat.setX(qx);
  quat.setY(qy);
  quat.setZ(qz);
  quat.setW(qw);

  // The tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return yaw;
}

void QrLocalization::computeMaptoOdom()
{
  double label_x, label_y, deviation_origin_x, deviation_origin_y,
      deviation_theta, yawOdomtoBaselink, yawMaptoOdom;
  bool pass_update = false;

  // set Rate for loop
  ros::Rate rate(50);

  // Tạo ma trận biến đổi từ transform
  // transOdomtoBaselink : odom -> base_link
  // transBaselinktoQR: base_link -> QR
  // transQRtoMap: QR -> Map
  // transOdomtoMap: odom -> Map
  Eigen::Isometry2d transOdomtoBaselink = Eigen::Isometry2d::Identity();
  Eigen::Isometry2d transBaselinktoQrCam = Eigen::Isometry2d::Identity();
  Eigen::Isometry2d qrCamtoQR = Eigen::Isometry2d::Identity();
  Eigen::Isometry2d transQRtoMap = Eigen::Isometry2d::Identity();
  Eigen::Isometry2d transOdomtoMap = Eigen::Isometry2d::Identity();
  Eigen::Isometry2d transMaptoOdom = Eigen::Isometry2d::Identity();
  Eigen::Isometry2d transBaselinktoMap = Eigen::Isometry2d::Identity();
  Eigen::Isometry2d transMaptoBaselink = Eigen::Isometry2d::Identity();

  Eigen::Vector2d translation;
  Eigen::Rotation2Dd rotation;

  // Chuyển đổi ma trận Eigen thành TransformStamped ROS
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.transform.translation.x = 0;
  transformStamped.transform.translation.y = 0;
  transformStamped.transform.rotation.z = 0;
  transformStamped.transform.rotation.w = 1;

  tf2::Quaternion quaternion;
  tf2_ros::TransformBroadcaster br;

  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "odom";

  while (ros::ok())
  {
    double time = ros::Time::now().toSec();
    ros::spinOnce();
    if (time - timeGetCmd_ > 0.5)
    {
      cmdAngularZ_ = 0.0;
      cmdLinearX_ = 0.0;
    }
    if (newDataQR_)
    {
      double current_time = ros::Time::now().toSec();
      //data of Gls621
      label_x = double(dataGls621_.lable.x) / 1000;
      label_y = double(dataGls621_.lable.y) / 1000;
      deviation_origin_x = double(dataGls621_.possition.x) / 1000;
      deviation_origin_y = double(dataGls621_.possition.y) / 1000;
      deviation_theta = dataGls621_.possition.angle;

      if (statusRobot_ == "RUNNING" && modeRobot_ == "AUTO" &&
          abs(cmdLinearX_) > 0.2 && label_x == oldLabelX_ &&
          label_y == oldLabelY_)
      {
        pass_update = true;
      }
      else
      {
        pass_update = false;
        oldLabelX_ = label_x;
        oldLabelY_ = label_y;
      }
      if (!pass_update)
      {
        // odom -> base_link
        transOdomtoBaselink.translation() << odomPose_.position.x,
            odomPose_.position.y;
        yawOdomtoBaselink =
            quaternionToRPY(odomPose_.orientation.x, odomPose_.orientation.y,
                            odomPose_.orientation.z, odomPose_.orientation.w);
        transOdomtoBaselink.linear() =
            Eigen::Rotation2Dd(yawOdomtoBaselink).toRotationMatrix();

        // base_link -> QR Cam
        transBaselinktoQrCam.translation() << transXCam_, transYCam_;
        transBaselinktoQrCam.linear() =
            Eigen::Rotation2Dd(angleRotateCam_).toRotationMatrix();

        // QRCam -> QR
        qrCamtoQR.translation() << deviation_origin_x, deviation_origin_y;
        qrCamtoQR.linear() =
            Eigen::Rotation2Dd(-deviation_theta).toRotationMatrix();

        // QR -> Map
        transQRtoMap.translation() << -label_x, -label_y;

        // Nhân ma trận để tạo ma trận biến đổi cho odom to Map
        transOdomtoMap = transOdomtoBaselink * transBaselinktoQrCam *
                         qrCamtoQR * transQRtoMap;

        // Tinh ma tran Map chuyen map -> odom
        transMaptoOdom = transOdomtoMap.inverse();

        // Lấy giá trị translation và quaternion từ ma trận Eigen
        translation = transMaptoOdom.translation();
        rotation = Eigen::Rotation2Dd(transMaptoOdom.linear());
        yawMaptoOdom = rotation.angle();
        // send transform map -> odom
        transformStamped.transform.translation.x = translation.x();
        transformStamped.transform.translation.y = translation.y();
        transformStamped.transform.translation.z = 0.0;
        quaternion.setRPY(0, 0, yawMaptoOdom);

        if (abs(cmdLinearX_) > 0.03 || abs(cmdAngularZ_) > 0.03)
        {
          // ros::Time(0);
          // Get current time
          std::time_t t = std::time(nullptr);
          std::tm* now = std::localtime(&t);
          oss << (now->tm_year + 1900) << '-' << (now->tm_mon + 1) << '-'
              << now->tm_mday << ' ' << now->tm_hour << ':' << now->tm_min
              << ':' << now->tm_sec;
          ROS_INFO("TIME: %f", current_time);
          ROS_INFO_STREAM("Custom timestamp: " << oss.str());
          // std::cout << oss.str();
          ROS_INFO("label_x: %f", label_x);
          ROS_INFO("label_y: %f", label_y);
          ROS_INFO("deviation_origin_x: %f", deviation_origin_x);
          ROS_INFO("deviation_origin_y: %f", deviation_origin_y);
          ROS_INFO("deviation_theta: %f", deviation_theta);
          ROS_INFO("robot_x %f", pose_x);
          ROS_INFO("robot_y %f", pose_y);

          // Clear the stream
          oss.str("");
          oss.clear();
        }

        oldUpdateMapX_ = translation.x();
        oldUpdateMapY_ = translation.y();
        oldUpdateMapYaw_ = yawMaptoOdom;

        transformStamped.transform.rotation = tf2::toMsg(quaternion);
        
        // Check timing constraints
        bool has_waited_30s = (current_time - start_time_) >= 30.0;
        bool check_time_elapsed = (current_time - last_pose_check_time_) >= 10.0; // 0.1Hz = 10 seconds
        if (check_time_elapsed)
        {
          // Update last pose check time regardless of whether pose was published
          last_pose_check_time_ = current_time;
        }
        // Check if we need to publish initial pose
        bool condition_met = (firstInitPose_ || 
                             statusRobot_ == "WAITING_INIT_POSE" || 
                             statusRobot_ == "WAITING" || 
                             modeRobot_ == "MANUAL");
        bool should_publish_pose = condition_met && has_waited_30s && check_time_elapsed;
        
        if (condition_met && !has_waited_30s)
        {
          ROS_DEBUG("Waiting for 30s timeout: %.1f/30.0s", current_time - start_time_);
        }
        else if (condition_met && !check_time_elapsed)
        {
          ROS_DEBUG("Waiting for pose check interval: %.1f/10.0s", current_time - last_pose_check_time_);
        }
        // ROS_INFO("Pose conditions - condition_met: %s, has_waited_30s: %s (%.1f/30.0s), check_time_elapsed: %s (%.1f/10.0s), should_publish_pose: %s",
        //          condition_met ? "TRUE" : "FALSE",
        //          has_waited_30s ? "TRUE" : "FALSE", current_time - start_time_,
        //          check_time_elapsed ? "TRUE" : "FALSE", current_time - last_pose_check_time_,
        //          should_publish_pose ? "TRUE" : "FALSE");
        
        if (should_publish_pose)
        {
          transBaselinktoMap = transBaselinktoQrCam * qrCamtoQR * transQRtoMap;
          transMaptoBaselink = transBaselinktoMap.inverse();
          Eigen::Vector2d trans = transMaptoBaselink.translation();
          Eigen::Rotation2Dd rot = Eigen::Rotation2Dd(transMaptoBaselink.linear());
          
          // Convert to lidar coordinate if lidar info is available and current_control_tf == 1
          double final_x = trans.x();
          double final_y = trans.y();
          double final_yaw = rot.angle();
          
          if (has_lidar_info_ && current_control_tf_ == 1)
          {
            std::pair<double, double> converted_pos = convertQRToLidarCoordinate(trans.x(), trans.y());
            final_x = converted_pos.first;
            final_y = converted_pos.second;
            final_yaw = convertQRToLidarAngle(rot.angle());
            
            ROS_INFO("Original pose: (%.3f, %.3f, %.3f deg)", 
                     trans.x(), trans.y(), rot.angle() * 180.0 / M_PI);
            ROS_INFO("Converted pose: (%.3f, %.3f, %.3f deg)", 
                     final_x, final_y, final_yaw * 180.0 / M_PI);
          }
          else if (has_lidar_info_ && current_control_tf_ != 1)
          {
            ROS_INFO("Lidar info available but current_control_tf=%d, using original QR pose", current_control_tf_);
            ROS_INFO("Using original pose: (%.3f, %.3f, %.3f deg)", 
                     final_x, final_y, final_yaw * 180.0 / M_PI);
          }
          
          // Check if pose needs to be updated
          if (shouldUpdatePose(final_x, final_y, final_yaw))
          {
            tf2::Quaternion quat;
            quat.setRPY(0, 0, final_yaw);

            poseInit_.header.frame_id = "map";
            poseInit_.pose.pose.position.x = final_x;
            poseInit_.pose.pose.position.y = final_y;
            poseInit_.pose.pose.orientation.z = quat.getZ();
            poseInit_.pose.pose.orientation.w = quat.getW();
            pub_initpose_.publish(poseInit_);
            
            // Update last published pose
            last_published_x_ = final_x;
            last_published_y_ = final_y;
            last_published_yaw_ = final_yaw;
            has_published_pose_ = true;
            
            ROS_INFO("Published initial pose: (%.3f, %.3f, %.3f deg)", 
                     final_x, final_y, final_yaw * 180.0 / M_PI);
            
            firstInitPose_ = false;
          }
        }
        time_update_QR_ = ros::Time::now().toSec();
      }
      newDataQR_ = false;
    }
    newDataOdom_ = false;
    transformStamped.header.stamp = ros::Time::now();
    if (typePublishTF == typeQRTF)
    {
      pub_current_control_tf_.publish(msg_type_control_tf_);
      br.sendTransform(transformStamped);
    }
    if ((ros::Time::now().toSec() - time_update_QR_) > 0.5)
    {
      led_turn_msg.data[0] = 0;
      led_turn_msg.data[1] = 0;
      led_turn_msg.data[2] = 0;
      led_turn_msg.data[3] = 0;
      m_led_turn_pub_.publish(led_turn_msg);
    }
    else
    {
      led_turn_msg.data[0] = 1;
      led_turn_msg.data[1] = 1;
      led_turn_msg.data[2] = 1;
      led_turn_msg.data[3] = 1;
      m_led_turn_pub_.publish(led_turn_msg);
    }
    rate.sleep();
  }
}

std::pair<double, double> QrLocalization::convertQRToLidarCoordinate(double qr_x, double qr_y)
{
  if (!has_lidar_info_)
  {
    ROS_WARN("No lidar info available for coordinate conversion");
    return std::make_pair(0.0, 0.0);
  }
  
  // Convert angle from degrees to radians (assuming lidar_angle_ is in degrees)
  double angle_rad = lidar_angle_ * M_PI / 180.0;
  double cos_angle = cos(angle_rad);
  double sin_angle = sin(angle_rad);
  
  double lidar_x = (qr_x * lidar_scale_ * cos_angle) - (qr_y * lidar_scale_ * sin_angle) + lidar_offset_x_;
  double lidar_y = (qr_x * lidar_scale_ * sin_angle) + (qr_y * lidar_scale_ * cos_angle) + lidar_offset_y_;
  
  ROS_DEBUG("Convert QR(%.3f, %.3f) -> Lidar(%.3f, %.3f)", qr_x, qr_y, lidar_x, lidar_y);
  
  return std::make_pair(lidar_x, lidar_y);
}

double QrLocalization::convertQRToLidarAngle(double qr_angle)
{
  if (!has_lidar_info_)
  {
    ROS_WARN("No lidar info available for angle conversion");
    return qr_angle;
  }
  
  // Convert lidar_angle_ from degrees to radians and add to qr_angle
  double angle_rad = lidar_angle_ * M_PI / 180.0;
  double converted_angle = qr_angle + angle_rad;
  
  // Normalize angle to [-pi, pi]
  while (converted_angle > M_PI) converted_angle -= 2 * M_PI;
  while (converted_angle < -M_PI) converted_angle += 2 * M_PI;
  
  ROS_DEBUG("Convert QR angle %.3f -> Lidar angle %.3f", qr_angle, converted_angle);
  
  return converted_angle;
}

bool QrLocalization::shouldUpdatePose(double lidar_x, double lidar_y, double lidar_angle)
{
  // Always publish if it's first time
  if (firstInitPose_)
  {
    ROS_INFO("First init pose - will publish");
    return true;
  }
  
  // Check if we have valid robot pose data (not all zeros)
  // if (abs(pose_x) < 0.001 && abs(pose_y) < 0.001 && abs(pose_yaw) < 0.001)
  // {
  //   ROS_WARN("Robot pose data seems invalid (all zeros), will publish anyway");
  //   return true;
  // }
  
  // Calculate position difference with current robot pose from /robot_pose topic
  double position_diff = sqrt(pow(lidar_x - pose_x, 2) + pow(lidar_y - pose_y, 2));
  
  // Calculate angle difference with current robot pose from /robot_pose topic
  double angle_diff = abs(lidar_angle - pose_yaw);
  while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
  angle_diff = abs(angle_diff);
  
  // Convert 10 degrees to radians
  double angle_threshold = 10.0 * M_PI / 180.0;
  
  bool should_update = (position_diff > 0.5) || (angle_diff > angle_threshold);
  
  if (should_update)
  {
    ROS_INFO("Pose update needed: pos_diff=%.3f (current: %.3f, %.3f -> new: %.3f, %.3f), angle_diff=%.3f deg (current: %.3f -> new: %.3f)", 
             position_diff, pose_x, pose_y, lidar_x, lidar_y,
             angle_diff * 180.0 / M_PI, pose_yaw * 180.0 / M_PI, lidar_angle * 180.0 / M_PI);
  }
  
  return should_update;
}
}  // namespace qr_code_localization

int main(int argc, char** argv)
{
  ros::init(argc, argv, "qr_localization");
  ros::NodeHandle nh("~");
  qr_code_localization::QrLocalization qr(nh);
  qr.computeMaptoOdom();
}
