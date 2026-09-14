#ifndef MODULE_MANAGER_H
#define MODULE_MANAGER_H

#include <jsoncpp/json/json.h>
#include <map>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_stamped_msgs/EmptyStamped.h>
#include <std_stamped_msgs/StringStamped.h>
#include <string>
#include <thread>
#include <vector>
#include <yaml-cpp/yaml.h>
namespace module_manager
{
class ModuleStatus
{
public:
  static const int WAITING = 0;
  static const int RUNNING = 1;
  static const int PAUSED = 2;
  static const int ERROR = 3;
  static int getStatusFromString(const std::string& status_string)
  {
    static const std::map<std::string, int> status_map = {
        {"WAITING", 0}, {"RUNNING", 1}, {"PAUSED", 2}, {"ERROR", 3}};

    auto it = status_map.find(status_string);
    if (it != status_map.end())
    {
      return it->second;
    }
    else
    {
      // Handle unknown status
      return -1;  // Or whatever default value you prefer
    }
  }
  static std::string toString(int status)
  {
    switch (status)
    {
    case WAITING:
      return "WAITING";
    case RUNNING:
      return "RUNNING";
    case PAUSED:
      return "PAUSED";
    case ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
    }
  }
};

class ModuleServer
{
public:
  ModuleServer(ros::NodeHandle nh, const std::string& name);
  ~ModuleServer();

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void requestStartMissionCallback(
      const std_stamped_msgs::StringStamped::ConstPtr& msg);
  void
  runPauseRequestCallback(const std_stamped_msgs::StringStamped::ConstPtr& msg);
  void resetErrorCallback(const std_stamped_msgs::EmptyStamped::ConstPtr& msg);

  void resetFlags();
  void sendFeedback(const ros::Publisher& action, const std::string& msg);
  void startROS();
  std::string name_;
  std::string error_code;
  bool reset_action_req;
  bool reset_error_request;
  bool resume_req;
  bool pause_req;
  bool resume_req_by_server;
  bool pause_req_by_server;
  bool action_running;
  std::string module_state;
  int module_status;

  double vel_x;
  ros::Publisher module_status_pub;

private:
  ros::NodeHandle nh_;
  ros::Subscriber run_pause_req_sub;
  ros::Subscriber reset_error_sub;
  ros::Subscriber request_start_mission_sub;
  ros::Subscriber odom_sub;
};

class ModuleClient
{
public:
  ModuleClient(ros::NodeHandle nh, const std::string& name,
               const YAML::Node& param_dict);
  ~ModuleClient();
  void setupSubscriber();
  void
  moduleStatusCallback(const std_stamped_msgs::StringStamped::ConstPtr& msg);
  void loop();
  void processStatusMessage(const std::string& data);
  std::string name_;
  YAML::Node param_dict_;
  std::string display_name;
  bool parse_json;
  bool handle_when_sim;
  double timeout;
  bool pause_if_error;
  std::vector<std::map<std::string, std::map<std::string, std::string>>>
      error_list;
  std::vector<std::map<std::string, std::map<std::string, std::string>>>
      state_list;
  std::string check_script;

  double last_module_status;
  bool module_alive;
  std::map<std::string, std::string> module_status_dict;
  std::string special_matching_error;
  std::string special_matching_state;
  std::string special_error_led;
  std::string special_state_led;
  std::string special_state_sound;
  std::string special_error_sound;
  std::string error_code;

private:
  std_stamped_msgs::StringStamped::ConstPtr module_status_msg;

  ros::NodeHandle nh_;
  ros::Subscriber module_status_sub;

  std::thread loop_thread;
};
};  // namespace module_manager
#endif