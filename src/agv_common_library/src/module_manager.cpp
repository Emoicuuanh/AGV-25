#include <agv_common_library/module_manager.h>

namespace module_manager
{
ModuleServer::ModuleServer(ros::NodeHandle nh, const std::string& name)
    : nh_(nh), name_(name), module_status(ModuleStatus::WAITING)
{
  module_status_pub = nh_.advertise<std_stamped_msgs::StringStamped>(
      name_ + "/module_status", 5);

  run_pause_req_sub =
      nh_.subscribe(name_ + "/run_pause_req", 10,
                    &ModuleServer::runPauseRequestCallback, this);
  reset_error_sub = nh_.subscribe(name_ + "/reset_error", 10,
                                  &ModuleServer::resetErrorCallback, this);
  request_start_mission_sub =
      nh_.subscribe("/request_start_mission", 10,
                    &ModuleServer::requestStartMissionCallback, this);
  odom_sub = nh_.subscribe("/odom", 10, &ModuleServer::odomCallback, this);

  error_code = "";
  reset_action_req = false;
  reset_error_request = false;
  resume_req = false;
  pause_req = false;
  resume_req_by_server = false;
  pause_req_by_server = false;
  action_running = false;
  module_state = "";
}

ModuleServer::~ModuleServer()
{
  // Destructor implementation, if needed
}

void ModuleServer::startROS()
{
    ros::spinOnce();
}

void ModuleServer::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  vel_x = msg->twist.twist.linear.x;
}

void ModuleServer::requestStartMissionCallback(
    const std_stamped_msgs::StringStamped::ConstPtr& msg)
{
  if (msg->data == "STOP")
  {
    reset_action_req = true;
    ROS_INFO("STOP MISSION");
  }
}

void ModuleServer::runPauseRequestCallback(
    const std_stamped_msgs::StringStamped::ConstPtr& msg)
{
  if (msg->data == "RUN")
  {
    ROS_INFO("Resume request");
    resume_req = true;
    pause_req = false;
    resume_req_by_server = true;
    pause_req_by_server = false;
  }
  else if (msg->data == "PAUSE")
  {
    ROS_INFO("Pause request");
    resume_req = false;
    pause_req = true;
  }
  else if (msg->data == "PAUSE_BY_SERVER")
  {
    ROS_INFO("Pause request");
    resume_req = false;
    pause_req = true;
    resume_req_by_server = false;
    pause_req_by_server = true;
  }
}

void ModuleServer::resetErrorCallback(
    const std_stamped_msgs::EmptyStamped::ConstPtr& msg)
{
  reset_error_request = true;
}

void ModuleServer::resetFlags()
{
  pause_req = false;
  resume_req = false;
  resume_req_by_server = false;
  pause_req_by_server = false;
  reset_action_req = false;
  reset_error_request = false;
  error_code = "";
}

void ModuleServer::sendFeedback(const ros::Publisher& action,
                                const std::string& msg)
{
  std_stamped_msgs::StringStamped feedback_msg;
  feedback_msg.data = msg;
  action.publish(feedback_msg);
}

ModuleClient::ModuleClient(ros::NodeHandle nh, const std::string& name,
                           const YAML::Node& param_dict)
    : nh_(nh), name_(name), param_dict_(param_dict), parse_json(true),
      handle_when_sim(true), timeout(2.0), pause_if_error(true),
      module_alive(false), check_script("")
{
  ROS_INFO("Init ModuleClient for: %s", name_.c_str());

  if (param_dict.IsMap())
  {
    ROS_INFO("Param_dict is Map");
    if (param_dict["display_name"])
    {
      display_name = param_dict["display_name"].as<std::string>();
    }
    if (param_dict["parse_json"])
    {
      parse_json = param_dict["parse_json"].as<bool>();
    }
    if (param_dict["handle_when_sim"])
    {
      handle_when_sim = param_dict["handle_when_sim"].as<bool>();
    }
    if (param_dict["timeout"])
    {
      timeout = param_dict["timeout"].as<double>();
    }
    if (param_dict["pause_if_error"])
    {
      pause_if_error = param_dict["pause_if_error"].as<bool>();
    }
    if (param_dict["check_script"])
    {
      check_script = param_dict["check_script"].as<std::string>();
    }

    if (param_dict["error_list"])
    {
      for (const auto& error_node : param_dict["error_list"])
      {
        for (const auto& it : error_node)
        {
          std::string error_type = it.first.as<std::string>();
          std::map<std::string, std::string> details_map;
          for (const auto& detail : it.second)
          {
            details_map[detail.first.as<std::string>()] =
                detail.second.as<std::string>();
          }
          error_list.push_back({{error_type, details_map}});
        }
      }
    }

    if (param_dict["state_list"])
    {
      for (const auto& error_node : param_dict["state_list"])
      {
        for (const auto& it : error_node)
        {
          std::string error_type = it.first.as<std::string>();
          std::map<std::string, std::string> details_map;
          for (const auto& detail : it.second)
          {
            details_map[detail.first.as<std::string>()] =
                detail.second.as<std::string>();
          }
          state_list.push_back({{error_type, details_map}});
        }
      }
    }
  }
  else
  {
    ROS_INFO("Param_dict not is Map");
  }
  last_module_status = ros::Time::now().toSec();
  module_alive = false;

  setupSubscriber();

  loop_thread = std::thread(&ModuleClient::loop, this);
}

ModuleClient::~ModuleClient()
{
  if (loop_thread.joinable())
  {
    loop_thread.join();
  }
}

void ModuleClient::setupSubscriber()
{
  std::string status_topic = (param_dict_["status_topic"])
                                 ? param_dict_["status_topic"].as<std::string>()
                                 : (name_ + "/module_status");
  module_status_sub = nh_.subscribe(status_topic, 10,
                                    &ModuleClient::moduleStatusCallback, this);
  ROS_INFO("Init Subcriber topic for: %s", name_.c_str());
}

void ModuleClient::moduleStatusCallback(
    const std_stamped_msgs::StringStamped::ConstPtr& msg)
{
  module_status_msg = msg;
  last_module_status = ros::Time::now().toSec();
  if (!parse_json)
  {
    return;
  }
  special_matching_error.clear();
  special_matching_state.clear();
  special_error_led.clear();
  special_state_led.clear();
  special_state_sound.clear();
  special_error_sound.clear();

  try
  {
    processStatusMessage(msg->data);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("module_status_cb: %s", e.what());
  }
}

void ModuleClient::processStatusMessage(const std::string& data)
{
  Json::Reader reader;
  Json::Value root;
  if (!reader.parse(data, root))
  {
    ROS_ERROR("Failed to parse JSON");
    return;
  }
  module_status_dict.clear();
  for (Json::Value::const_iterator it = root.begin(); it != root.end(); ++it)
  {
    module_status_dict[it.key().asString()] = it->asString();
  }

  if (module_status_dict.count("error_code"))
  {
    error_code = module_status_dict["error_code"];
  }

  // Handle error list
  if (!error_code.empty())
  {
    for (const auto& error : error_list)
    {
      if (error.count(error_code))
      {
        special_matching_error = error.at(error_code).at("detail_status");
        if (error.at(error_code).count("led_status"))
        {
          special_error_led = error.at(error_code).at("led_status");
        }
        if (error.at(error_code).count("sound_status"))
        {
          special_error_sound = error.at(error_code).at("sound_status");
        }
        return;
      }
    }
  }

  // Handle state list
  if (module_status_dict.count("state"))
  {
    std::string state = module_status_dict["state"];
    for (const auto& state_item : state_list)
    {
      if (state_item.count(state))
      {
        special_matching_state = state_item.at(state).at("detail_status");
        if (state_item.at(state).count("led_status"))
        {
          special_state_led = state_item.at(state).at("led_status");
        }
        if (state_item.at(state).count("sound_status"))
        {
          special_state_sound = state_item.at(state).at("sound_status");
        }
        return;
      }
    }
  }
}

void ModuleClient::loop()
{
  ros::Rate rate(2);
  double vel_x = 0;
  double start_time_check_disconnect = ros::Time::now().toSec();

  while (ros::ok())
  {
    if (param_dict_["status_topic"] &&
        param_dict_["status_topic"].as<std::string>() == "/followline_sensor")
    {
      if (vel_x == 0)
      {
        start_time_check_disconnect = ros::Time::now().toSec();
        last_module_status = ros::Time::now().toSec();
      }
      if (ros::Time::now().toSec() - start_time_check_disconnect <= 0.5)
      {
        last_module_status = ros::Time::now().toSec();
      }
    }
    if (check_script.empty())
    {
      module_alive = (ros::Time::now().toSec() - last_module_status <= timeout);
    }
    else
    {
      // Implement custom check script logic here
    }
    rate.sleep();
  }
}

}  // namespace module_manager