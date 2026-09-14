#include <cstdio>
#include <iostream>
#include <vl53l5cx/Vl53l5cxRanges.h>

#include "vl53l5cx_ros.h"
#include "console_bridge/console.h"
#include <string>
#include <cmath>
#include <thread>
#include <chrono>

/**
 * @brief Construct a new object
 * Get param, register pub, sub
 * Calculate cos, sin table for roll, pitch
 */
Vl53l5cxROS::Vl53l5cxROS()
{
  // parameters

  ros::NodeHandle node_("~");
  //   node_.param<std::string>("frame", frame_id_, "laser");
  node_.param<std::string>("arr_in_topic1", arr_in_topic1_, "vl53l5cx_r1");
  node_.param<std::string>("arr_in_topic2", arr_in_topic2_, "vl53l5cx_r2");
  node_.param<std::string>("arr_in_topic3", arr_in_topic3_, "vl53l5cx_r3");
  node_.param<std::string>("arr_in_topic4", arr_in_topic4_, "vl53l5cx_r4");
  node_.param<std::string>("pcl_out_topic1", pcl_out_topic1_, "pointcloud_lidar1");
  node_.param<std::string>("pcl_out_topic2", pcl_out_topic2_, "pointcloud_lidar2");
  node_.param<std::string>("pcl_out_topic3", pcl_out_topic3_, "pointcloud_lidar3");
  node_.param<std::string>("pcl_out_topic4", pcl_out_topic4_, "pointcloud_lidar4");
  //   ROS_INFO_STREAM("frame: " << frame_id_ << "\n pcl output topic: "<< pcl_out_topic_);

  // subscribers
  sub1 = node_.subscribe(arr_in_topic1_, 1, &Vl53l5cxROS::sub_ss1_cb, this);
  sub2 = node_.subscribe(arr_in_topic2_, 1, &Vl53l5cxROS::sub_ss2_cb, this);
  sub3 = node_.subscribe(arr_in_topic3_, 1, &Vl53l5cxROS::sub_ss3_cb, this);
  sub4 = node_.subscribe(arr_in_topic4_, 1, &Vl53l5cxROS::sub_ss4_cb, this);
  // publishers
  laser_cloud_pub1_ = node_.advertise<sensor_msgs::PointCloud2>(pcl_out_topic1_, 1);
  laser_cloud_pub2_ = node_.advertise<sensor_msgs::PointCloud2>(pcl_out_topic2_, 1);
  laser_cloud_pub3_ = node_.advertise<sensor_msgs::PointCloud2>(pcl_out_topic3_, 1);
  laser_cloud_pub4_ = node_.advertise<sensor_msgs::PointCloud2>(pcl_out_topic4_, 1);

  // Calculate sin, cos table
  Vl53l5cxROS::ComputeSinCosTables();
}

/**
 * @brief Destroy the object
 *
 */
Vl53l5cxROS::~Vl53l5cxROS()
{
}

// Subcriber
void Vl53l5cxROS::sub_ss1_cb(const vl53l5cx::Vl53l5cxRanges::ConstPtr &msg)
{
  // ROS_INFO("myCallback activated: received value ");
  frame_id_ = "base_laser_1";
  ConvertDist2XYZCoords8x8(msg, &point_data);
  Vl53l5cxROS::publishMsg(laser_cloud_pub1_);
}

void Vl53l5cxROS::sub_ss2_cb(const vl53l5cx::Vl53l5cxRanges::ConstPtr &msg)
{
  // ROS_INFO("myCallback activated: received value ");
  frame_id_ = "base_laser_2";
  ConvertDist2XYZCoords8x8(msg, &point_data);
  Vl53l5cxROS::publishMsg(laser_cloud_pub2_);
}

void Vl53l5cxROS::sub_ss3_cb(const vl53l5cx::Vl53l5cxRanges::ConstPtr &msg)
{
  // ROS_INFO("myCallback activated: received value ");
  frame_id_ = "base_laser_3";
  ConvertDist2XYZCoords8x8(msg, &point_data);
  Vl53l5cxROS::publishMsg(laser_cloud_pub3_);
}

void Vl53l5cxROS::sub_ss4_cb(const vl53l5cx::Vl53l5cxRanges::ConstPtr &msg)
{
  // ROS_INFO("myCallback activated: received value ");
  frame_id_ = "base_laser_4";
  ConvertDist2XYZCoords8x8(msg, &point_data);
  Vl53l5cxROS::publishMsg(laser_cloud_pub4_);
}

// Convert range data to point clound msg
bool Vl53l5cxROS::MsgDataTest()
{
  // Compute point from distance data
  uint16_t arr_temp[64] = {1691, 1630, 1647, 1667, 1693, 1646, 1671, 1753, 666, 1618, 1613, 1599, 1619, 1629, 1694, 1776, 658, 1599, 1630, 1637, 1612, 1646, 1654, 1649, 1579, 1565, 1589, 1609, 1618, 1632, 1628, 1631, 1555, 1567, 1580, 1578, 1615, 1580, 1632, 1645, 1531, 1543, 1582, 1562, 1572, 1595, 1617, 1626, 1529, 1528, 1533, 1549, 1568, 1571, 1582, 1581, 1508, 1517, 29, 60, 1554, 1558, 1545, 1608};
  vl53l5cx::Vl53l5cxRanges::Ptr msg;
  for (int i = 0; i < 64; i++)
  {
    msg->range[i] = static_cast<float>(arr_temp[i]);
  }

  ConvertDist2XYZCoords8x8(boost::static_pointer_cast<const vl53l5cx::Vl53l5cxRanges>(msg), &point_data);
  publishMsg(laser_cloud_pub1_);
  std::cout << "Publish message test" << std::endl;
  return true;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

uint8_t Vl53l5cxROS::ComputeSinCosTables()
{
  // This function will save the math processing time of the code.  If the user wishes to not
  // perform this function, these tables can be generated and saved as a constant.
  uint8_t ZoneNum;
  for (ZoneNum = 0; ZoneNum < 64; ZoneNum++)
  {
    SinOfPitch[ZoneNum] = sin((VL53L5_Zone_Pitch8x8[ZoneNum]) * M_PI / 180);
    CosOfPitch[ZoneNum] = cos((VL53L5_Zone_Pitch8x8[ZoneNum]) * M_PI / 180);
    SinOfYaw[ZoneNum] = sin(VL53L5_Zone_Yaw8x8[ZoneNum] * M_PI / 180);
    CosOfYaw[ZoneNum] = cos(VL53L5_Zone_Yaw8x8[ZoneNum] * M_PI / 180);
  }

  return 0;
}

uint8_t Vl53l5cxROS::ConvertDist2XYZCoords8x8(const vl53l5cx::Vl53l5cxRanges::ConstPtr ResultsData, XYZ_ZoneCoordinates_t *XYZ_ZoneCoordinates)
{
  uint8_t ZoneNum;
  float Hyp;
  int row, index;
  for (ZoneNum = 0; ZoneNum < 64; ZoneNum++)
  {
    row = ZoneNum / 8;
    index = (row * 8 + 7) - (ZoneNum % 8);
    if (ResultsData->range[index] > 0)
    {
      Hyp = ResultsData->range[index] / SinOfPitch[ZoneNum];
      XYZ_ZoneCoordinates->Xpos[ZoneNum] = CosOfYaw[ZoneNum] * CosOfPitch[ZoneNum] * Hyp;
      XYZ_ZoneCoordinates->Ypos[ZoneNum] = SinOfYaw[ZoneNum] * CosOfPitch[ZoneNum] * Hyp;
      XYZ_ZoneCoordinates->Zpos[ZoneNum] = ResultsData->range[index];
    }
    else
    {
      XYZ_ZoneCoordinates->Xpos[ZoneNum] = 0;
      XYZ_ZoneCoordinates->Ypos[ZoneNum] = 0;
      XYZ_ZoneCoordinates->Zpos[ZoneNum] = 0;
    }
  }
  return 0;
}

void Vl53l5cxROS::publishMsg(ros::Publisher &laser_cloud_pub_)
{
  // Tạo message pointcloud

  // Tạo điểm cloud từ mảng dữ liệu
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < 64; i++)
  {
    pcl::PointXYZ point;
    point.x = point_data.Xpos[i] / 1000; // use met unit
    point.y = point_data.Ypos[i] / 1000;
    point.z = point_data.Zpos[i] / 1000;
    cloud->push_back(point);
  }

  // Tạo message pointcloud
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.frame_id = frame_id_;
  msg.header.stamp = ros::Time::now();
  laser_cloud_pub_.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Vl53l5cxROS");
  Vl53l5cxROS node;

  ROS_INFO("Init VL53L5CX ROS node.");

  ros::spin();
  // ros::Rate rate(10);
  // while (ros::ok())
  // {
  //   ros::spinOnce();
  //   node.MsgDataTest();
  //   rate.sleep();
  // }
  return 0;
}
