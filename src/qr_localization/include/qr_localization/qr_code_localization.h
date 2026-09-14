#ifndef QR_LOCALIZATION_H_
#define QR_LOCALIZATION_H_

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include "json.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Geometry>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_stamped_msgs/StringStamped.h>
#include <std_stamped_msgs/Int16MultiArrayStamped.h>
#include <nav_msgs/Odometry.h>
#include <agv_msgs/DataMatrixStamped.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include <iostream>
#include <sstream>
#include <ctime>
#include <cmath>

using json = nlohmann::json;
namespace qr_code_localization
{
    class QrLocalization
    {
        public:
            QrLocalization(ros::NodeHandle nh);

            ~QrLocalization();

            void sub_odom_cb(const nav_msgs::Odometry::ConstPtr &msg);

            void sub_qr_code_cb(const agv_msgs::DataMatrixStamped::ConstPtr &msg);

            void sub_robot_stt_cb(const std_stamped_msgs::StringStamped::ConstPtr &msg);

            void controlTFCallback(const std_msgs::Int8::ConstPtr &msg);

            void cmdVelCallback(const geometry_msgs::Twist::ConstPtr msg);

            void lidar_info_cb(const std_stamped_msgs::StringStamped::ConstPtr &msg);
            
            void current_control_tf_cb(const std_msgs::Int8::ConstPtr &msg);
            
            void status_detect_mirror_cb(const std_msgs::String::ConstPtr &msg);

            double quaternionToRPY(double qx, double qy, double qz, double qw);

            void computeMaptoOdom();
            void robot_pose_cb(const geometry_msgs::Pose::ConstPtr &msg);
            
            // Convert QR coordinate to lidar coordinate
            std::pair<double, double> convertQRToLidarCoordinate(double qr_x, double qr_y);
            
            // Convert QR angle to lidar angle
            double convertQRToLidarAngle(double qr_angle);
            
            // Check if pose needs to be updated
            bool shouldUpdatePose(double lidar_x, double lidar_y, double lidar_angle);

        private:
            ros::NodeHandle nh_;

            ros::Subscriber init_pose_;
            ros::Subscriber sub_odom_;
            ros::Subscriber sub_qr_code_;
            ros::Subscriber sub_robot_stt_;
            ros::Subscriber sub_cmd_vel_;
            ros::Subscriber sub_control_tf_;
            ros::Subscriber robot_pose;
            ros::Subscriber sub_lidar_info_;
            ros::Subscriber sub_current_control_tf_;
            ros::Subscriber sub_status_detect_mirror_;

            ros::Publisher pub_initpose_;
            ros::Publisher m_led_turn_pub_;
            ros::Publisher pub_current_control_tf_;

            geometry_msgs::Pose odomPose_;
            geometry_msgs::PoseWithCovarianceStamped poseInit_;
            agv_msgs::DataMatrixStamped dataGls621_;
            std_stamped_msgs::Int16MultiArrayStamped led_turn_msg;
            std_msgs::Int8 msg_type_control_tf_;

            std::string statusRobot_;
            std::string modeRobot_;

            int8_t typeQRTF = 0;
            int8_t typePublishTF = 1;

            bool newDataQR_;
            bool newDataOdom_;
            bool firstInitPose_;

            double angleRotateCam_;
            double transXCam_;
            double transYCam_;
            double timeGetCmd_;
            double cmdAngularZ_;
            double cmdLinearX_;
            double oldLabelX_;
            double oldLabelY_;
            double oldUpdateMapX_;
            double oldUpdateMapY_;
            double oldUpdateMapYaw_;
            double pose_x, pose_y, pose_yaw;
            int current_control_tf_;
            
            // Lidar info variables
            bool has_lidar_info_;
            double lidar_scale_;
            double lidar_offset_x_;
            double lidar_offset_y_;
            double lidar_angle_;
            json lidar_info_;
            
            // Last published pose for comparison
            double last_published_x_, last_published_y_, last_published_yaw_;
            bool has_published_pose_;
            
            // Timing for initial pose publishing
            double start_time_;
            double last_pose_check_time_;
            double time_update_QR_;
            
            std::ostringstream oss;
    };
};

#endif
