#include "agv_msgs/DataMatrixStamped.h"
#include "qrIrayble.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "qrIraybleNode");
  ros::NodeHandle nh("~");  // Private node handle for parameters
  ros::Publisher data_pub_;

  // Parameters
  std::string ip_address;
  int port;
  int get_io_rate;
  std::string comm_mode;
  std::string result_format;
  std::string transfer_style;
  int site_id;
  bool enable_light_control;
  double light_control_interval;
  double angle_cam_qr;
  // Load parameters with defaults from the manual
  nh.param<std::string>("ip_address", ip_address, "192.168.0.52");
  nh.param<int>("port", port, 2111);  // Default TCP port for AGV readers
  nh.param<int>("get_io_rate", get_io_rate, 100);  // 100 FPS as per manual
  nh.param<std::string>("comm_mode", comm_mode, "TCP");
  nh.param<std::string>("result_format", result_format, "Common");  // "Common" or "P+F"
  nh.param<std::string>("transfer_style", transfer_style, "Upload");  // "Upload" or "Query"
  nh.param<int>("site_id", site_id, 0);  // For multi-device RS485
  nh.param<bool>("enable_light_control", enable_light_control, false);
  nh.param<double>("light_control_interval", light_control_interval, 5.0);  // seconds
  nh.param<double>("angle_cam_qr", angle_cam_qr, 1.57);  // seconds

  // Validate parameters
  if (comm_mode != "TCP" && comm_mode != "RS485") {
    ROS_ERROR("Invalid comm_mode: %s. Must be 'TCP' or 'RS485'", comm_mode.c_str());
    return -1;
  }

  if (result_format != "Common" && result_format != "P+F") {
    ROS_ERROR("Invalid result_format: %s. Must be 'Common' or 'P+F'", result_format.c_str());
    return -1;
  }

  if (transfer_style != "Upload" && transfer_style != "Query") {
    ROS_ERROR("Invalid transfer_style: %s. Must be 'Upload' or 'Query'", transfer_style.c_str());
    return -1;
  }

  // ROS Publisher
  data_pub_ = nh.advertise<agv_msgs::DataMatrixStamped>("/data_gls621", 1);

  // Driver instance
  AgvCodeReader reader(ip_address, port, comm_mode, result_format, transfer_style, site_id);
  ros::Rate loop_rate(get_io_rate);

  ROS_INFO("Starting AGV Code Reader Node");
  ROS_INFO("  IP: %s:%d", ip_address.c_str(), port);
  ROS_INFO("  Communication: %s", comm_mode.c_str());
  ROS_INFO("  Format: %s", result_format.c_str());
  ROS_INFO("  Style: %s", transfer_style.c_str());
  ROS_INFO("  Site ID: %d", site_id);
  ROS_INFO("  Rate: %d Hz", get_io_rate);

  if (!reader.connect()) {
    ROS_ERROR("Failed to connect to AGV Code Reader. Exiting.");
    return -1;
  }

  // Initialize light if using query mode with P+F format
  if (result_format == "P+F" && transfer_style == "Query" && enable_light_control) {
    reader.sendLightControl(true);  // Turn on light initially
  }

  uint64_t last_tag_num = 0;
  ros::Time last_light_toggle = ros::Time::now();
  bool light_state = true;

  while (ros::ok()) {
    ROS_INFO("Try to read");
    try {
      agv_msgs::DataMatrixStamped msg;
      bool data_received = false;

      if (result_format == "Common") {
        if (transfer_style == "Query") {
          reader.sendQuery();
        }

        auto frames = reader.readCommonData();
        for (const auto& frame : frames) {
          if (!frame.empty()) {
            agv_msgs::DataMatrixStamped msg_frame = reader.processCommonData(frame,angle_cam_qr);
            data_received = true;

            ROS_WARN("Raw frame: %s", frame.c_str());

            msg = msg_frame;
          }
        }

      } else if (result_format == "P+F") {
        if (transfer_style == "Query") {
          reader.sendQuery();
        }

        std::vector<uint8_t> data = reader.readBinaryData();
        if (!data.empty()) {
          msg = reader.processPepperFuchsData(data);
          data_received = true;

          std::stringstream ss;
          for (size_t i = 0; i < data.size(); ++i) {
            ss << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<int>(data[i]) << " ";
          }
          ROS_WARN("Raw binary data: %s", ss.str().c_str());
        }
      }


      if (data_received) {
        // Check if this is a new tag
        uint64_t current_tag = static_cast<uint64_t>(msg.lable.x) * 10000 + msg.lable.y;

        if (current_tag != last_tag_num) {
          reader.newQr = true;
          last_tag_num = current_tag;

          ROS_INFO("New tag detected: %lu at position (%.3f, %.3f) with angle %.2f°",
                   (unsigned long)current_tag, (double)msg.possition.x, (double)msg.possition.y,
                   (double)msg.possition.angle * 180.0 / M_PI);
        } else {
          reader.newQr = false;
        }

        // Publish the message
        data_pub_.publish(msg);
      }else{
        ROS_INFO("Receive no data");
      }

      // Handle periodic light control for query mode
      if (enable_light_control && result_format == "P+F" && transfer_style == "Query") {
        ros::Time now = ros::Time::now();
        if ((now - last_light_toggle).toSec() >= light_control_interval) {
          light_state = !light_state;
          reader.sendLightControl(light_state);
          last_light_toggle = now;
          ROS_WARN("Toggled light: %s", light_state ? "ON" : "OFF");
        }
      }

    } catch (const std::exception& e) {
      ROS_WARN("Error processing AGV code reader data: %s", e.what());
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  // Clean shutdown
  if (enable_light_control && result_format == "P+F" && transfer_style == "Query") {
    reader.sendLightControl(false);  // Turn off light
  }

  reader.disconnect();
  ROS_INFO("AGV Code Reader Node shutting down");

  return 0;
}