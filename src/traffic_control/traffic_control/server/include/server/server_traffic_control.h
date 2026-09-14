#ifndef SERVER_TRAFFIC_CONTROL_H
#define SERVER_TRAFFIC_CONTROL_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include "server/httplib.h"
#include <jsoncpp/json/json.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_stamped_msgs/StringStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tuw_multi_robot_msgs/GetRobotStatus.h>
#include <tuw_multi_robot_msgs/RobotGoals.h>
#include <tuw_multi_robot_msgs/RobotInfo.h>
#include <tuw_multi_robot_msgs/RobotInfoArray.h>
#include <tuw_multi_robot_msgs/Route.h>
#include <tuw_multi_robot_msgs/RouteArray.h>
#include <tuw_multi_robot_msgs/RouterStatus.h>

class Server {
public:
  Server();
  void startServer();
  void joinServerThread();

private:
  void handleReceiveGoal(const httplib::Request &req, httplib::Response &res);
  void handleReceiveRobotInfo(const httplib::Request &req,
                              httplib::Response &res);
  void handleReceiveGetLastPath(const httplib::Request &req,
                                httplib::Response &res);
  void plannerStatusCallback(
      const tuw_multi_robot_msgs::RouterStatus::ConstPtr &msg);
  void routeCallback(const tuw_multi_robot_msgs::RouteArray::ConstPtr &msg);
  geometry_msgs::Quaternion yawToQuaternion(double yaw_radian);
  Json::Value
  convertRouteMsgToJson(const tuw_multi_robot_msgs::Route &route_msg);
  Json::Value getRobotRouteJson(const std::string &robot_name_req);
  bool callGetRobotStatusService(const std::string &robot_name, bool &is_pause,
                                 std::string &detail_status_pause);
  void moduleStatusCallback(const ros::TimerEvent &);
  ros::ServiceClient client_;

  ros::Publisher goal_pub_;
  ros::Publisher robot_info_pub_;
  ros::Publisher robot_pose_pub_;
  ros::Publisher module_status_pub_;

  ros::Subscriber subRoute_;
  ros::Subscriber subplannerStatus_;
  std::vector<tuw_multi_robot_msgs::RobotInfo> robot_info_msgs;
  std::vector<tuw_multi_robot_msgs::RouterStatus> planner_status_msg;
  std::map<std::string, uint32_t> goals_id;
  std::map<std::string, bool> request_get_last_path;
  tuw_multi_robot_msgs::RouteArray route_array_msg;
  std::map<std::string, tuw_multi_robot_msgs::Route> previous_routes;
  std::thread server_thread_;
  ros::Timer module_status_timer_;

  httplib::Server svr_;
  std::string server_ip;
  int server_port;
};

#endif // SERVER_TRAFFIC_CONTROL_H
