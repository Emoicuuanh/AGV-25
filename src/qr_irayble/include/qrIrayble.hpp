#ifndef QRIRAYBLE_HPP
#define QRIRAYBLE_HPP

#include "agv_msgs/DataMatrixStamped.h"
#include <boost/asio.hpp>
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <cmath>

class AgvCodeReader {
private:
  std::string server_ip_;
  int port_;
  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::socket socket_;
  
  // Communication parameters
  std::string comm_mode_;  // "TCP" or "RS485"
  std::string result_format_; // "Common" or "P+F" (Pepperl+Fuchs)
  std::string transfer_style_; // "Query" or "Upload"
  int site_id_;  // For multi-device RS485 communication
  
  // Internal buffer for TCP communication
  std::vector<uint8_t> read_buffer_;
  
  // Helper functions
  double normalizeAngle(double angle_degrees);
  bool parseCommonFormat(const std::string& data, agv_msgs::DataMatrixStamped& result);
  bool parsePepperFuchsFormat(const std::vector<uint8_t>& data, agv_msgs::DataMatrixStamped& result);
  std::vector<uint8_t> createQueryCommand(int address = 0);
  std::vector<uint8_t> createLightCommand(int address, bool light_on);
  uint8_t calculateParity(uint8_t byte);
  
public:
  bool newQr = false;
  
  // Constructor
  AgvCodeReader(const std::string &server_ip, int port, 
                const std::string& comm_mode = "TCP",
                const std::string& result_format = "Common",
                const std::string& transfer_style = "Upload",
                int site_id = 0);
  
  // Connection management
  bool connect();
  void disconnect();
  bool isConnected();
  
  // Communication functions
  bool sendQuery();
  bool sendLightControl(bool light_on);
  std::vector<std::string> readCommonData();
  std::vector<uint8_t> readBinaryData();
  
  // Data processing
  agv_msgs::DataMatrixStamped processCommonData(const std::string& input,double angle_cam);
  agv_msgs::DataMatrixStamped processPepperFuchsData(const std::vector<uint8_t>& input);
  
  // Configuration
  void setCommMode(const std::string& mode) { comm_mode_ = mode; }
  void setResultFormat(const std::string& format) { result_format_ = format; }
  void setTransferStyle(const std::string& style) { transfer_style_ = style; }
  void setSiteId(int id) { site_id_ = id; }
  
  // Status
  std::string getCommMode() const { return comm_mode_; }
  std::string getResultFormat() const { return result_format_; }
  std::string getTransferStyle() const { return transfer_style_; }
  int getSiteId() const { return site_id_; }
};

#endif