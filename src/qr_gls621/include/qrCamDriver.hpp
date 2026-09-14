#ifndef QRCAMDRIVER_HPP
#define QRCAMDRIVER_HPP

#include "agv_msgs/DataMatrixStamped.h"
#include <boost/asio.hpp>
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <regex>


class qrCamDriver {
private:
  std::string server_ip_;
  int port_;
  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::socket socket_;
  std::vector<std::string> stringData_;
  // void readData();
  double normalizeAngle(double angle);

public:
  bool newQr = false;
  void sendTrigger();
  qrCamDriver(const std::string &server_ip, int port)
      : server_ip_(server_ip), port_(port), socket_(io_service_){};
  bool connect();
  std::string readData();
  agv_msgs::DataMatrixStamped processData(const std::string);
};
#endif // QRCAMDRIVER_HPP