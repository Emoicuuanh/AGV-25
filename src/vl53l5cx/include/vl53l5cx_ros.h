#ifndef VL53L5CX_ROS_H_
#define VL53L5CX_ROS_H_

#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "sensor_msgs/PointCloud2.h"
#include <csignal>
#include <cstdio>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <vl53l5cx/Vl53l5cxRanges.h>

const double VL53L5_Zone_Pitch8x8[64] = {
    59.00, 64.00, 67.50, 70.00, 70.00, 67.50, 64.00, 59.00, 64.00, 70.00, 72.90,
    74.90, 74.90, 72.90, 70.00, 64.00, 67.50, 72.90, 77.40, 80.50, 80.50, 77.40,
    72.90, 67.50, 70.00, 74.90, 80.50, 85.75, 85.75, 80.50, 74.90, 70.00, 70.00,
    74.90, 80.50, 85.75, 85.75, 80.50, 74.90, 70.00, 67.50, 72.90, 77.40, 80.50,
    80.50, 77.40, 72.90, 67.50, 64.00, 70.00, 72.90, 74.90, 74.90, 72.90, 70.00,
    64.00, 59.00, 64.00, 67.50, 70.00, 70.00, 67.50, 64.00, 59.00};

const double VL53L5_Zone_Yaw8x8[64] = {
    135.00, 125.40, 113.20, 98.13,  81.87,  66.80,  54.60,  45.00,
    144.60, 135.00, 120.96, 101.31, 78.69,  59.04,  45.00,  35.40,
    156.80, 149.04, 135.00, 108.45, 71.55,  45.00,  30.96,  23.20,
    171.87, 168.69, 161.55, 135.00, 45.00,  18.45,  11.31,  8.13,
    188.13, 191.31, 198.45, 225.00, 315.00, 341.55, 348.69, 351.87,
    203.20, 210.96, 225.00, 251.55, 288.45, 315.00, 329.04, 336.80,
    203.20, 225.00, 239.04, 258.69, 281.31, 300.96, 315.00, 324.60,
    225.00, 234.60, 246.80, 261.87, 278.13, 293.20, 305.40, 315.00};

// typedef struct
// {
//   /* Measured distance in mm */
//   uint16_t distance_mm[64];

// } VL53L5CX_ResultsData;

typedef struct
{
  float Xpos[64];
  float Ypos[64];
  float Zpos[64];
} XYZ_ZoneCoordinates_t;

class Vl53l5cxROS
{

private:
  ros::NodeHandle node_;

  // Subscriber
  ros::Subscriber sub1, sub2, sub3, sub4;

  // Publisher
  ros::Publisher laser_cloud_pub1_, laser_cloud_pub2_, laser_cloud_pub3_,
      laser_cloud_pub4_;

  std::string frame_id_;
  std::string arr_in_topic1_, arr_in_topic2_, arr_in_topic3_, arr_in_topic4_;
  std::string pcl_out_topic1_, pcl_out_topic2_, pcl_out_topic3_,
      pcl_out_topic4_;

  // VL53L5CX_ResultsData data;
  XYZ_ZoneCoordinates_t point_data;
  int socket_fd_;
  double SinOfPitch[64], CosOfPitch[64], SinOfYaw[64], CosOfYaw[64];

public:
  Vl53l5cxROS();
  ~Vl53l5cxROS();

  // Subcriber callback function
  void sub_ss1_cb(const vl53l5cx::Vl53l5cxRanges::ConstPtr& msg);
  void sub_ss2_cb(const vl53l5cx::Vl53l5cxRanges::ConstPtr& msg);
  void sub_ss3_cb(const vl53l5cx::Vl53l5cxRanges::ConstPtr& msg);
  void sub_ss4_cb(const vl53l5cx::Vl53l5cxRanges::ConstPtr& msg);

  // laser message test
  bool MsgDataTest();

  // Compute Sin Cos
  uint8_t ComputeSinCosTables();

  // Convert to distance
  uint8_t ConvertDist2XYZCoords8x8(const vl53l5cx::Vl53l5cxRanges::ConstPtr,
                                   XYZ_ZoneCoordinates_t*);

  void publishMsg(ros::Publisher& laser_cloud_pub_);
};

#endif