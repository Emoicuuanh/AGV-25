#ifndef CLIENT_AGV_H
#define CLIENT_AGV_H

#include "httplib.h"
#include <geometry_msgs/PoseStamped.h>
#include <jsoncpp/json/json.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <std_stamped_msgs/StringStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tuw_multi_robot_msgs/RobotInfo.h>
#include <tuw_multi_robot_msgs/RobotInfoArray.h>
#include <tuw_multi_robot_msgs/Route.h>
#include <tuw_multi_robot_msgs/RouterStatus.h>
#include <tuw_multi_robot_msgs/requestGoal.h>

class Client {
public:
  Client(ros::NodeHandle &n, const std::string &server_address_);
  bool handle_service(tuw_multi_robot_msgs::requestGoal::Request &req,
                      tuw_multi_robot_msgs::requestGoal::Response &res);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void robotPoseCallback(const geometry_msgs::Pose::ConstPtr &msg);
  void
  robotStatusCallback(const std_stamped_msgs::StringStamped::ConstPtr &msg);
  void
  missionStatusCallback(const std_stamped_msgs::StringStamped::ConstPtr &msg);
  void missionFromServerCallback(
      const std_stamped_msgs::StringStamped::ConstPtr &msg);

  void sendRobotInfo();
  bool sendGoal(double x, double y, double quat_x = 0.0, double quat_y = 0.0,
                double quat_z = 0.0, double quat_w = 1.0);
  bool getLastPath();
  void getState();
  bool getTargetGoal(double &x, double &y);
  void ManagerSendGoal();
  void processJsonAndPublish(const Json::Value &jsonResponse);
  void
  movingControlCallback(const std_stamped_msgs::StringStamped::ConstPtr &msg);
  void
  directionMoveCallback(const std_stamped_msgs::StringStamped::ConstPtr &msg);

  void
  HubActionStatusCallback(const std_stamped_msgs::StringStamped::ConstPtr &msg);
  void ElevatorActionStatusCallback(
      const std_stamped_msgs::StringStamped::ConstPtr &msg);
  void MatehanActionStatusCallback(
      const std_stamped_msgs::StringStamped::ConstPtr &msg);
  void currentMapCallback(const std_stamped_msgs::StringStamped::ConstPtr &msg);
  void currentPathCallback(const nav_msgs::Path::ConstPtr &msg);
  void updateAutoMode(const std::string &mode_, Json::Value &root);
  void pauseRobot();
  void resumeRunning();
  int findClosestSegment(const Json::Value &waypoints, double position_x_,
                         double position_y_);
  double distanceToSegment(const std::pair<double, double> &p1,
                           const std::pair<double, double> &p2,
                           const std::pair<double, double> &point);
  double perpendicularDistance(const std::pair<double, double> &p1,
                               const std::pair<double, double> &p2,
                               const std::pair<double, double> &pt);
  double fullSegmentDistance(const std::pair<double, double> &p1,
                             const std::pair<double, double> &p2,
                             const std::pair<double, double> &pt);
  bool isBehind(const std::pair<double, double> &p1,
                const std::pair<double, double> &p2,
                const std::pair<double, double> &pt);
  std::pair<double, double> unitVector(const std::pair<double, double> &from,
                                       const std::pair<double, double> &to);
  std::pair<double, double> movePoint(const std::pair<double, double> &origin,
                                      double distance,
                                      const std::pair<double, double> &dir);
  void buildTrimmedWaypoints(const Json::Value &waypoints,
                             Json::Value &trimmed_waypoints);
  double calculateAngleBetweenSegments(const Json::Value &wp0,
                                       const Json::Value &wp1,
                                       const Json::Value &wp2);
  double calculateAngle(const std::pair<double, double> &p0,
                        const std::pair<double, double> &p1,
                        double orientation_);
  double distance(const std::pair<double, double> &p1,
                  const std::pair<double, double> &p2);
  void perpendicularProjectionInfo(const std::pair<double, double> &p1,
                                   const std::pair<double, double> &p2,
                                   const std::pair<double, double> &pt,
                                   double &dist_p1_to_proj,
                                   double &dist_proj_to_p2);
  bool isSpecialReturnChargeMission();
  void updatePauseStatus();
  void configureClient(std::shared_ptr<httplib::Client> &client);
  geometry_msgs::PoseStamped createPoseStamped(double x, double y, double qx,
                                               double qy, double qz, double qw);

private:
  // Các biến thành viên
  ros::NodeHandle nh;
  ros::NodeHandle n_param_;

  ros::Subscriber odom_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber mission_from_server_sub_;
  ros::Subscriber robot_status_sub_;
  ros::Subscriber moving_control_sub_;
  ros::Subscriber mission_status_sub_;
  ros::Subscriber direction_move_sub_;

  ros::Subscriber hub_action_status_sub_;
  ros::Subscriber matehan_action_status_sub_;
  ros::Subscriber elevator_action_status_sub_;
  ros::Subscriber current_map_sub_;
  ros::Subscriber current_path_sub_;

  ros::Publisher pause_status_pub_;
  ros::Publisher route_pub_;
  ros::Publisher status_planner_pub_;
  ros::Publisher pub_mission_run_pause_;
  ros::Publisher goal_pub_;
  ros::Publisher current_goal_id_pub_;

  std::shared_ptr<httplib::Client> cli_send_goal_;
  std::shared_ptr<httplib::Client> cli_get_last_path_;
  std::shared_ptr<httplib::Client> cli_update_robot_info_;
  std::string server_ip;
  int server_port;
  std::string robot_name_;
  std::vector<double> robot_radius_;

  // Các biến trạng thái của robot
  std::mutex data_mutex_;
  std::mutex status_mutex_;
  double last_goal_x, last_goal_y;
  double position_x_, position_y_, orientation_;
  std::array<double, 2> velocity_ = {
      0.0, 0.0}; // Velocity for linear and angular velocities
  std::string mission_manager_status_, current_action_type_, mission_group_,
      current_mission_name_, mode_, status_, detail_;
  int current_action_num_, total_action_, index_action_found_;
  Json::Value current_mission_data_;
  std::string _state, _pre_state;
  bool send_goal_begin;
  bool init_pose_success;
  uint32_t goal_id_ = 0;
  bool is_send_special_goal_ = false;
  bool is_robot_in_hub_begin_ = false;
  bool is_send_get_last_path_ = false;
  std::string direction_move_, state_move_;
  std::string moving_control_status_;
  bool is_request_trigger_resend_goal = false;
  std::string hub_action_state_go_out_, matehan_action_state_go_out_,
      elevator_action_state_go_out_;
  std::vector<std::string> listMission = {"PUSH",          "PULL",
                                          "TRANSFER",      "RETURN_CHARGER",
                                          "PULL_ELEVATOR", "PUSH_ELEVATOR"};

  ros::Time current_time_update_status;
  ros::Time last_time_update_status;
  ros::Time mode_change_time;
  bool last_mode_is_auto = false;
  uint32_t last_goal_id_send = 0;
  bool root_auto_mode =
      false; // trạng thái hiện tại đã set trong root["auto_mode"]
  bool trigger_resend_goal = false;
  bool receive_goal = false;
  bool need_to_wait_receive_path = false;
  bool receive_new_mission_data = false;
  bool pause_by_wait_path = false;
  bool path_is_valid = false;
  std::mutex cli_mutex_send_goal_;
  std::mutex cli_mutex_update_robot_info_;
  std::mutex cli_mutex_get_last_path_;
  std::string server_address;
  std::pair<double, double> last_goal_in_path = std::make_pair(0.0, 0.0);
  std::pair<double, double> first_goal_in_path = std::make_pair(0.0, 0.0);
  ros::Time last_successful_connection_time_;
};

#endif // CLIENT_AGV_H