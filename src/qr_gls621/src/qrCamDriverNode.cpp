#include "agv_msgs/DataMatrixStamped.h"
#include "qrCamDriver.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "qrCamDriverNode");
  ros::NodeHandle nh;
  ros::Publisher data_pub_;

  std::string ip_address;
  int port;
  int get_io_rate;

  // Load parameters
  nh.param<std::string>("/qr_gls621/ip_address", ip_address, "192.168.0.150");
  nh.param<int>("/qr_gls621/port", port, 2111);
  nh.param<int>("/qr_gls621/get_io_rate", get_io_rate, 60); // Default 60Hz

  // ROS Publisher
  data_pub_ = nh.advertise<agv_msgs::DataMatrixStamped>("/data_gls621", 1);

  // Driver instance
  qrCamDriver camera_driver(ip_address, port);
  ros::Rate loop_rate(get_io_rate);

  ROS_INFO("Starting QR_CAM_DRIVER_NODE at %s:%d", ip_address.c_str(), port);
  if (!camera_driver.connect()) {
    ROS_ERROR("Không thể kết nối tới camera QR. Thoát.");
    return -1;
  }

  int last_label_x = -1;
  int last_label_y = -1;

  while (ros::ok()) {
    // Gửi trigger mỗi vòng
    camera_driver.sendTrigger();

    // Đọc dữ liệu
    // ROS_WARN("call read Data");
    std::string cam_data = camera_driver.readData();
    if (cam_data.empty()) {
      loop_rate.sleep();
      continue;
    }

    agv_msgs::DataMatrixStamped cam_msg;
    // ROS_WARN("call processData");
    try {
      cam_msg = camera_driver.processData(cam_data);
    } catch (const std::exception &e) {
      ROS_WARN("Không phân tích được QR: %s", e.what());
      loop_rate.sleep();
      continue;
    }

    // Chỉ log nếu nhãn QR mới
    if (cam_msg.lable.x != last_label_x || cam_msg.lable.y != last_label_y) {
      camera_driver.newQr = true;
      last_label_x = cam_msg.lable.x;
      last_label_y = cam_msg.lable.y;

      ROS_INFO("RAW_DATA: %s", cam_data.c_str());
      std::cout << "cam_data: [" << cam_data.size() << "] " << cam_data
                << std::endl;
    } else {
      camera_driver.newQr = false;
    }

    // Xuất dữ liệu
    data_pub_.publish(cam_msg);
    loop_rate.sleep();
  }

  return 0;
}
