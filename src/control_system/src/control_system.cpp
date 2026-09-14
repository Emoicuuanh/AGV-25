#include <control_system.h>
using namespace module_manager;
namespace control_system
{

ControlSystem::ControlSystem(ros::NodeHandle& nh,
                             const std::map<std::string, std::string>& options)
    : nh_(nh)
{
  initVariable(options);
  if (!loadConfig())
  {
    return;
  }

  // Set up ROS shutdown callback
  ros::NodeHandle private_nh("~");
  shutdown_sub_ =
      private_nh.subscribe("shutdown", 1, &ControlSystem::shutdown, this);

  // Publishers
  pub_init_server_ = nh_.advertise<std_msgs::Empty>("/init_server", 1);
  pub_battery_ = nh_.advertise<std_msgs::Int8>("/battery", 10);
  pub_robot_mode_ = nh_.advertise<std_msgs::String>("/robot_mode", 10);
  pub_current_map_ = nh_.advertise<std_msgs::String>("/current_map", 10);
  pub_robot_status_ = nh_.advertise<std_msgs::String>("/robot_status", 10);
  pub_mission_control_ =
      nh_.advertise<std_msgs::String>("/mission_control", 10);
  pub_mission_run_pause_ =
      nh_.advertise<std_msgs::String>("/mission_manager/run_pause_req", 10);
  pub_mapping_req_ = nh_.advertise<std_msgs::String>("/mapping_request", 5);
  pub_map_load_req_ =
      nh_.advertise<std_msgs::String>("/map_load_request", 5, true);
  pub_map_save_req_ = nh_.advertise<std_msgs::String>("/map_save_request", 5);
  pub_temp_map_save_req_ =
      nh_.advertise<std_msgs::String>("/temp_savemap_request", 5);
  pub_led_status_ = nh_.advertise<std_msgs::String>("/led_status", 5, true);
  pub_trigger_ = nh_.advertise<std_msgs::String>("/trigger_complex", 5);
  pub_mission_reset_error_ =
      nh_.advertise<std_msgs::Empty>("/mission_manager/reset_error", 5);
  pub_stop_moving_ = nh_.advertise<std_msgs::Empty>("/stop_moving", 5);
  pub_request_start_mission_ =
      nh_.advertise<std_msgs::String>("/request_start_mission", 5);
  pub_mission_fr_server_ =
      nh_.advertise<std_msgs::String>("/mission_from_server", 5);
  pub_mission_fr_server_latch_ =
      nh_.advertise<std_msgs::String>("/mission_from_server_latch", 5, true);
  pub_request_sound_ = nh_.advertise<std_msgs::String>("/request_sound", 5);

  for (auto& i : module_config)
  {
    ModuleClient* client_module = nullptr;
    client_module = new ModuleClient(i.begin()->first, i.begin()->second);
    if (i.begin()->first == "mission_manager")
    {
      this->mission_manager_module = client_module;
    }
    else if (i.begin()->first == "slam_manager")
    {
      this->slam_manager_module = client_module;
    }

    this->module_list.push_back(client_module);
  }

  // Subscribers
  nh_.subscribe("/init_server", 1, &ControlSystem::initServerCB, this);
  nh_.subscribe("/mission_manager/module_status", 10,
                &ControlSystem::missionStatusCB, this);
  nh_.subscribe("/current_goal_monving_control", 10,
                &ControlSystem::currentMovingControlGoalCB, this);
  nh_.subscribe("/error_control_motor_right", 10,
                &ControlSystem::rightVelErrorCB, this);
  nh_.subscribe("/error_control_motor_left", 10, &ControlSystem::leftVelErrorCB,
                this);
  nh_.subscribe("/request_mode", 10, &ControlSystem::requestModeCB, this);
  nh_.subscribe("/request_savemap", 10, &ControlSystem::requestSavemapCB, this);
  nh_.subscribe("/request_temp_savemap", 10,
                &ControlSystem::requestTempSavemapCB, this);
  nh_.subscribe("/request_change_map", 10, &ControlSystem::requestChangeMapCB,
                this);
  nh_.subscribe("/request_reload_map", 10, &ControlSystem::requestReloadMapCB,
                this);
  nh_.subscribe("/request_run_stop", 10, &ControlSystem::requestRunStopCB,
                this);
  nh_.subscribe("/request_start_mission", 10,
                &ControlSystem::requestStartMissionCB, this);
  nh_.subscribe("/standard_io", 10, &ControlSystem::standardIoCB, this);
  nh_.subscribe("/arduino_error", 10, &ControlSystem::arduinoErrorCB, this);
  nh_.subscribe("/reset_error", 10, &ControlSystem::resetErrorCB, this);
  nh_.subscribe("/fake_battery", 10, &ControlSystem::fakeBatteryCB, this);
  nh_.subscribe("/arduino_driver/float_param/battery_percent", 10,
                &ControlSystem::realBatteryCB, this);
  nh_.subscribe("/arduino_driver/float_param/battery_ampe", 10,
                &ControlSystem::batteryAmpeCB, this);
  nh_.subscribe("/arduino_driver/float_param/battery_voltage", 10,
                &ControlSystem::batteryVoltageCB, this);
  nh_.subscribe("/trigger_mission_from_server", 10,
                &ControlSystem::triggerMissionFromServerCB, this);
  nh_.subscribe("/initialpose", 10, &ControlSystem::initialposeCB, this);
  nh_.subscribe("/safety_status", 10, &ControlSystem::safetyStatusCB, this);
  nh_.subscribe("/moving_control/module_status", 10,
                &ControlSystem::movingControlModuleStatusCB, this);
  nh_.subscribe("/odom", 10, &ControlSystem::odomCB, this);
  nh_.subscribe("/log_distance_move", 10, &ControlSystem::distanceMoveRecordCB,
                this);
  nh_.subscribe("/log_lift", 10, &ControlSystem::liftRecordCB, this);
  nh_.subscribe("/request_working_stt", 10, &ControlSystem::workingSttCB, this);

  // Start server connection thread
  server_thread_ = std::thread(&ControlSystem::connectServer, this);
  server_thread_.detach();

  // Main loop
  loop();
}

void ControlSystem::initVariable(
    const std::map<std::string, std::string>& kwargs)
{
  // Simulation
  simulation = kwargs.at("simulation") == "true";
  simulationStr = kwargs.at("simulation");

  // TF
  tf::TransformListener tf_listener;
  std::string odom_frame = "odom";
  std::string robot_frame = "base_footprint";
  std::string map_frame = "map";
  // Assuming the type of current_pose is similar to Pose in geometry_msgs
  geometry_msgs::Pose current_pose;
  // Initialize TF Listener and frames

  // Config
  std::string mapRequesting = "";
  std::stringcurrentMapFile = kwargs.at("current_map_file");
  std::string robotConfigFile = kwargs.at("robot_config_file");
  std::string serverConfigFile = kwargs.at("robot_define");
  CurrentMap current_map_cfg;
  current_map_cfg.mapFile = "mkac";
  current_map_cfg.mapDir = "/tmp/ros/maps/";

  // State
  SlamState slamState = SlamState::NONE;
  ModuleStatus missionStatus = ModuleStatus::WAITING;
  std::string runningMissionId = "";
  std::string lastDoneMissionId = "";
  LedStatus led_status = LedStatus::STARTING;
  MainState mainState = MainState::INIT;
  MainState prevState = MainState::NONE;
  RobotMode robotMode = RobotMode::AUTO;
  RobotMode robotModeReq = RobotMode::NONE;
  RobotStatus robotStatus = RobotStatus::NONE;
  std::string detailStatus = "ALL_OK";
  std::string pauseDetailStatus = "";
  int sound_status = -1;  // Assuming -1 represents None
  std::map<std::string, bool> std_io_status = {{"emg_button", false}};
  std::map<std::string, std::string> status_dict = {{"status", "NONE"},
                                                    {"detail", "NONE"}};
  std::vector<std::string> module_list;
  std::string error_code = "";
  std::string special_matching_state = "";
  StringStamped arduino_error_msg;
  int arduino_error_cnt = 0;
  int load_map_cnt = 0;
  int mapping_cnt = 0;

  // Flag
  bool save_map_request = false;
  bool reset_error_request = false;
  bool mission_from_server_req = false;
  bool init_pose_received = false;
  int battery_percent = 90;
  int battery_ampe = 0;
  int battery_voltage = 0;
  double last_battery_fake = ros::Time::now().toSec();
  double last_std_io_msg = ros::Time::now().toSec();
  double last_odom_msg = ros::Time::now().toSec();
  double time_start = ros::Time::now().toSec();

  // Button
  const int emgButton = SENSOR_DEACTIVATE;
  const int start1Button = SENSOR_DEACTIVATE;
  const int start2Button = SENSOR_DEACTIVATE;
  const int stop1Button = SENSOR_DEACTIVATE;
  const int stop2Button = SENSOR_DEACTIVATE;
  const int autoManualSw = SENSOR_DEACTIVATE;
  const int motorEnableSw = SENSOR_DEACTIVATE;
  bool updateStatusButton = true;

  // Safety
  double lastSafetyTime = ros::Time::now().toSec();
  bool isSafety = false;

  // Ros params
  bool poseInitiated;
  ros::param::get("/pose_initiated", poseInitiated);

  std::map<std::string, std::string> movingControlStatusDict;
  movingControlStatusDict["status"] = "NONE";

  int bumper = SENSOR_DEACTIVATE;

  // Motor
  bool controlMotorLeftError = false;
  bool controlMotorRightError = false;
  bool motorLeftOverAmpe = false;
  bool motorRightOverAmpe = false;

  // Pose
  tf::TransformListener tf_listener;
  std::string followline_goal = "";
  std::string moving_control_goal = "";
  bool pause_by_cross_function = false;
  std::string current_action_type = "";

  // Database
  std::string db_address =
      "/mongodb_address";  // Modify according to your ROS parameter server setup
  ros::param::get(db_address, db_address);
  ROS_DEBUG_STREAM("mongodb_address: " << db_address);
  // mongodb db(db_address);

  // Server
  std::string agvDbObjectId = "";
  std::string working_status = "PRODUCTION";
  bool reset_get_mission = false;
  std::string map_server_name = "";
  int odom_moved_record = 0;
  int lift_record = 0;
  std::string detail_status_update_to_server = "";
  std::string robot_status_update_to_server = "";
  std::string last_detail_status_update_to_server = "";
  std::string last_robot_status_update_to_server = "";
  std::vector<std::string> list_action_move = {
      "move", "hub", "matehand", "docking_charger", "un_docking", "elevator"};
}

///*
//######## ##     ## ##    ##  ######  ######## ####  #######  ##    ##
//##       ##     ## ###   ## ##    ##    ##     ##  ##     ## ###   ##
//##       ##     ## ####  ## ##          ##     ##  ##     ## ####  ##
//######   ##     ## ## ## ## ##          ##     ##  ##     ## ## ## ##
//##       ##     ## ##  #### ##          ##     ##  ##     ## ##  ####
//##       ##     ## ##   ### ##    ##    ##     ##  ##     ## ##   ###
//##        #######  ##    ##  ######     ##    ####  #######  ##    ##
//*/

void ControlSystem::initServer()
{
  if (!serverConfig)
  {
    ROS_INFO_THROTTLE(30, "Server was not configured!");
    return;
  }

  apiUrl = serverConfig->serverAddress;
  header = {{"X-Parse-Application-Id", "APPLICATION_ID"},
            {"X-Parse-Master-Key", "YOUR_MASTER_KEY"},
            {"Content-Type", "application/json"}};

  // Get AGV object ID in Database
  std::string queryParam =
      "where=" + json.dumps({"name" : serverConfig->agvName});
  ROS_DEBUG("AGV name for system: %s", serverConfig->agvName.c_str());
  try
  {
    std::string agvDbObj =
        requests.get(apiUrl + "classes/agv", queryParam, header);
    ROS_DEBUG("Request server result:\n%s", agvDbObj.c_str());
    auto jsonResult = json.loads(agvDbObj);
    agvDbObjectId = jsonResult["results"][0]["objectId"];
    ROS_DEBUG("AGV obj db id: %s", agvDbObjectId.c_str());
  }
  catch (const std::exception& e)
  {
    agvDbObjectId = "";
    ROS_ERROR_STREAM(e.what());
  }
}

void ControlSystem::connectServer()
{
  double last_update_pose = ros::Time::now().toSec();
  double last_send_traffic_control = ros::Time::now().toSec();
  double begin_request_mission = ros::Time::now().toSec();
  std::string mission_queue_id = "";
  std::string pre_mission_queue_id = "";
  bool allow_update_mission = false;

  while (ros::ok())
  {
    double begin_time = ros::Time::now().toSec();

    // Fetch and update mission
    if (robot_status == "WAITING" && robot_mode == "AUTO")
    {
      if (mission_queue_id != pre_mission_queue_id)
      {
        pre_mission_queue_id = mission_queue_id;
        allow_update_mission = true;
      }
      if (ros::Time::now().toSec() - begin_request_mission > 2.0)
      {
        if (!agv_db_object_id.empty())
        {
          if (reset_get_mission)
          {
            reset_get_mission = false;
            allow_update_mission = false;
          }
          if (last_done_mission_id == mission_queue_id &&
              !mission_queue_id.empty() && allow_update_mission)
          {
            ROS_WARN("Mission done id: %s", last_done_mission_id.c_str());
            Json::Value data;
            data["function"] = "UPDATE_MISSION";
            data["missionId"] = last_done_mission_id;
            data["progress"] = "done";
            std::string response =
                postRequest(api_url + "functions/agvapi", data);
            Json::Reader reader;
            Json::Value result;
            reader.parse(response, result);
            if (result["result"] == "OK")
            {
              allow_update_mission = false;
              mission_queue_id.clear();
              pre_mission_queue_id.clear();
              ROS_WARN("Update mission done succeed!");
            }
            else
            {
              ROS_ERROR("UPDATE MISSION FAIL --> CANNOT GET NEW MISSION");
            }
          }
          if (!allow_update_mission)
          {
            try
            {
              Json::Value data;
              data["agv"] = server_config["agv_name"];
              std::string response =
                  postRequest(api_url + "functions/get_mission", data);
              Json::Reader reader;
              Json::Value result;
              reader.parse(response, result);
              std::string mission_json = Json::StyledWriter().write(result);
              ROS_INFO("Mission from server:\n%s", mission_json.c_str());

              if (!result["result"].isNull() &&
                  result["result"]["actions"].size() > 0)
              {
                for (auto& action : result["result"]["actions"])
                {
                  if (!action["waypoints"].isNull())
                  {
                    map_server_name = action["waypoints"][0]["map"].asString();
                    break;
                  }
                }
                pub_mission_fr_server.publish(std_msgs::String{mission_json});
                pub_mission_fr_server_latch.publish(
                    std_msgs::String{mission_json});
                mission_queue_id =
                    result["result"]["mission_queue_id"].asString();
                nh.setParam("/current_mission_queue_id", mission_queue_id);
                ROS_WARN("Received new mission_queue_id : %s",
                         mission_queue_id.c_str());
              }
            }
            catch (const std::exception& e)
            {
              ROS_ERROR("Exception: %s", e.what());
              agv_db_object_id.clear();
            }
          }
        }
        else if (!server_config.empty())
        {
          std_msgs::Empty msg;
          pub_init_server.publish(msg);
        }
        begin_request_mission = ros::Time::now().toSec();
      }
    }

    // Update position to server
    if (!agv_db_object_id.empty())
    {
      double now = ros::Time::now().toSec();
      bool update_when_change_state = false;
      if (detail_status_update_to_server !=
              last_detail_status_update_to_server ||
          robot_status_update_to_server != last_robot_status_update_to_server)
      {
        update_when_change_state = true;
      }
      if (now - last_update_pose >= 2 || update_when_change_state)
      {
        last_update_pose = now;
        if (current_pose != nullptr)
        {
          Json::Value data;
          data["function"] = "UPDATE_AGV";
          data["agv"] = server_config["agv_name"];
          data["status"]["status"] = robot_status_update_to_server;
          data["status"]["detail"] = detail_status_update_to_server;
          data["position"]["x"] = current_pose->position.x;
          data["position"]["y"] = current_pose->position.y;
          data["position"]["z"] = 0.0;
          data["battery"]["capacity"] = battery_percent;
          data["battery"]["voltage"] = round(battery_voltage * 100) / 100.0;
          data["battery"]["current"] = round(battery_ampe * 100) / 100.0;
          data["params"]["odo"] = odom_moved_record;
          data["params"]["liftup"] = lift_record;
          data["params"]["working_status"] = working_status;
          data["version"] = 3;

          std::string response =
              postRequest(api_url + "functions/agvapi", data);
          Json::Reader reader;
          Json::Value result;
          reader.parse(response, result);
          ROS_WARN("detail_status_update_to_server: %s",
                   detail_status_update_to_server.c_str());
          ROS_WARN("robot_status_update_to_server: %s",
                   robot_status_update_to_server.c_str());
          last_detail_status_update_to_server = detail_status_update_to_server;
          last_robot_status_update_to_server = robot_status_update_to_server;
        }
      }
    }

    // Traffic control
    if (!server_config.empty() &&
        ((robot_status == "RUNNING" && robot_mode == "AUTO") ||
         (robot_status == "PAUSED" && robot_mode == "AUTO")))
    {
      double time_send_traffic_control = ros::Time::now().toSec();
      if (time_send_traffic_control - last_send_traffic_control >= 0.3)
      {
        if (isNearIntersectArea(2.5))
        {
          if (current_pose != nullptr)
          {
            Json::Value data;
            data["function"] = "TRAFFIC_CONTROL";
            data["agv"] = server_config["agv_name"];
            data["position"]["x"] = current_pose->position.x;
            data["position"]["y"] = current_pose->position.y;
            data["position"]["z"] = 0.0;

            std::string response =
                postRequest(api_url + "functions/agvapi", data);
            Json::Reader reader;
            Json::Value result;
            reader.parse(response, result);
            if (result["result"] == "STOP")
            {
              if (robot_status == "RUNNING")
              {
                ROS_WARN("Stop by traffic control");
                pauseRobotByServer();
                pause_detail_status = "Pause by traffic control";
              }
            }
            else
            {
              if (robot_status == "PAUSED" && pause_by_cross_function)
              {
                ROS_WARN("Resume by traffic control");
                resumeRunningByServer();
              }
            }
            last_send_traffic_control = time_send_traffic_control;
          }
        }
      }
    }

    if (ros::Time::now().toSec() - begin_time > 0.5)
    {
      ROS_ERROR("Connect server consume time: %f",
                ros::Time::now().toSec() - begin_time);
    }
  }
}

void ControlSystem::shutdown(const std_msgs::Empty::ConstPtr& msg)
{
  // Handle ROS shutdown
  ROS_INFO("Shutting down...");
}

bool ControlSystem::isNearIntersectArea(double distCheck = 2)
{
  try
  {
    if (currentActionType == "move")
    {
      if (!movingControlGoal.empty())
      {
        // Calculate distance between current pose and moving control goal
        double distance = distanceTwoPose(currentPose, movingControlGoal);
        if (distance < distCheck)
        {
          return true;
        }
        return false;
      }
    }
    else if (currentActionType == "hub" || currentActionType == "matehand")
    {
      if (!followlineGoal.empty())
      {
        // Calculate distance between current pose and follow line goal
        double distance = distanceTwoPose(currentPose, followlineGoal);
        if (distance < distCheck)
        {
          return true;
        }
        else
        {
          return false;
        }
      }
    }
    return false;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Error checking near intersection: " << e.what());
    return false;
  }
}

void ControlSystem::dumpConfig()
{
  // TODO: Only dump when load map successful
  std::ofstream file(currentMapFile);
  file << YAML::Dump(currentMapCfg);
}

bool ControlSystem::loadConfig()
{
  try
  {
    // Current map
    ROS_INFO_STREAM("Check current_map_file: " << currentMapFile);
    if (std::filesystem::exists(currentMapFile))
    {
      YAML::Node configNode = YAML::LoadFile(currentMapFile);
      if (configNode.IsNull())
      {
        ROS_WARN("Current map file empty");
        dumpConfig();
      }
      else
      {
        CurrentMap configDict = configNode.as<CurrentMap>();
        configDict.map_dir =
            currentMapCfg.map_dir;  // Overwrite map_dir for different user name
        currentMapCfg = configDict;
        dumpConfig();
        ROS_INFO_STREAM("Current map file:\n" << YAML::Dump(currentMapCfg));
      }
    }
    else
    {
      ROS_WARN("Check current_map_file not exist. Write default value");
      dumpConfig();
    }

    // Robot config
    if (std::filesystem::exists(robotConfigFile))
    {
      YAML::Node configNode = YAML::LoadFile(robotConfigFile);
      if (configNode.IsNull())
      {
        ROS_ERROR("Robot config file empty");
      }
      else
      {
        robotConfig = configNode.as<RobotConfig>();
        batteryConfig = robotConfig["battery"];
        moduleConfig = robotConfig["module_list"];
        optionConfig = robotConfig["option"];
        soundConfig = robotConfig["sound"];

        if (robotConfig["motor"])
        {
          motorConfig = robotConfig["motor"];
          // Subscribe to motor topics
          ampeMotorLeftSub =
              nh.subscribe("/ampe_left", 10, &Robot::ampeMotorLeftCb, this);
          ampeMotorRightSub =
              nh.subscribe("/ampe_right", 10, &Robot::ampeMotorRightCb, this);
        }

        // Handle missing options
        if (optionConfig["use_followline"].IsNull())
        {
          optionConfig["use_followline"] = true;
          // TODO: Import follow_line.msg
        }
        if (optionConfig["check_init_pose"].IsNull())
        {
          optionConfig["check_init_pose"] = true;
        }
        if (optionConfig["use_slam"].IsNull())
        {
          optionConfig["use_slam"] = true;
        }
        if (optionConfig["use_bumper"].IsNull())
        {
          optionConfig["use_bumper"] = false;
        }

        ROS_INFO_STREAM("Robot config file:\n" << YAML::Dump(robotConfig));
      }
    }
    // TODO: Handle server config file
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Load_config: " << e.what());
    return false;
  }
  return true;
}

void ControlSystem::setPoseInit(bool value)
{
  poseInitiated = value;
  ros::param::set("/pose_initiated", value);
}

void ControlSystem::pauseRobot()
{
  pauseByCrossFunction = false;
  std_msgs::String msg;
  msg.data = "PAUSE";
  pubMissionRunPause.publish(msg);
}

void ControlSystem::pauseRobotByServer()
{
  ROS_WARN("Pause by cross function");
  pauseByCrossFunction = true;
  std_msgs::String msg;
  msg.data = "PAUSE_BY_SERVER";
  pubMissionRunPause.publish(msg);
}

void ControlSystem::stopMoving()
{
  std_msgs::Empty msg;
  pubStopMoving.publish(msg);
}

void ControlSystem::resumeRunning()
{
  std_msgs::String msg;
  msg.data = "RUN";
  pubMissionRunPause.publish(msg);
  pauseDetailStatus = "";
}

void ControlSystem::resumeRunningByServer()
{
  ROS_WARN("Resume running by cross function");
  std_msgs::String msg;
  msg.data = "RUN";
  pubMissionRunPause.publish(msg);
  pauseDetailStatus = "";
}

void ControlSystem::loadMap(std::string mapFile, bool absPath = true,
                            bool reloadMap = false)
{
  std_msgs::String msg;
  msg.stamp = ros::Time::now();
  if (absPath)
  {
    msg.data = currentMapCfg.map_dir + mapFile;
  }
  else
  {
    msg.data = mapFile;
  }
  ROS_WARN(msg.data);
  if (!reloadMap)
  {
    pubMapLoadReq.publish(msg);
    ROS_INFO("load_map: %s", msg.data.c_str());
  }
  else
  {
    pubMapLoadReq.publish(msg);
    ROS_INFO("reload_map: %s", msg.data.c_str());
  }
}

bool ControlSystem::checkMap(std::string mapFile)
{
  ROS_INFO("Check map file: %s", mapFile.c_str());
  ros::ServiceClient client =
      nh.serviceClient<your_package::StringService>("check_map");
  your_package::StringService srv;
  srv.request.input = mapFile;
  if (client.call(srv))
  {
    if (srv.response.output == "OK")
    {
      return true;
    }
  }
  else
  {
    ROS_ERROR("Failed to call service check_map");
  }
  return false;
}

// /*
//  ######     ###    ##       ##       ########     ###     ######  ##    ##
// ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
// ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
// ##       ##     ## ##       ##       ########  ##     ## ##       #####
// ##       ######### ##       ##       ##     ## ######### ##       ##  ##
// ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
//  ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
// */

void ControlSystem::initServerCB(
    const std_stamped_msgs::EmptyStamped::ConstPtr& msg)
{
  if (!server_config.empty())
  {  // Retry init server
    initServer();
    if (!agv_db_object_id.empty())
    {
      ROS_WARN("Re-init server");
    }
  }
}

void ControlSystem::workingSttCb(const std_msgs::String::ConstPtr& msg)
{
  working_status = msg->data;
}

void ControlSystem::distanceMoveRecordCb(const std_msgs::UInt32::ConstPtr& msg)
{
  odom_moved_record = msg->data;
}

void ControlSystem::liftRecordCb(const std_msgs::UInt32::ConstPtr& msg)
{
  lift_record = msg->data;
}

void ControlSystem::missionStatusCb(const std_msgs::String::ConstPtr& msg)
{
  Json::Value data_dict;
  Json::Reader reader;
  reader.parse(msg->data, data_dict);
  current_action_type = data_dict["current_action_type"].asString();
}

void ControlSystem::currentMovingControlGoalCb(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  moving_control_goal = msg->pose;
}

void ControlSystem::followlineActionCB(
    const follow_line::FollowLineActionGoal::ConstPtr& msg)
{
  followline_goal = msg->goal.target_pose.pose;
}

void ControlSystem::ampeMotorLeftCB(const std_msgs::Int16::ConstPtr& msg)
{
  if (msg->data > motor_config["max_ampe"])
  {
    motor_left_over_ampe = true;
  }
  else
  {
    motor_left_over_ampe = false;
  }
}

void ControlSystem::ampeMotorRightCB(const std_msgs::Int16::ConstPtr& msg)
{
  if (msg->data > motor_config["max_ampe"])
  {
    motor_right_over_ampe = true;
  }
  else
  {
    motor_right_over_ampe = false;
  }
}

void ControlSystem::leftVelErrorCB(const std_msgs::Int16::ConstPtr& msg)
{
  if (msg->data == 1)
  {
    control_motor_left_error = true;
  }
  else
  {
    control_motor_left_error = false;
  }
}

void ControlSystem::rightVelErrorCB(const std_msgs::Int16::ConstPtr& msg)
{
  if (msg->data == 1)
  {
    control_motor_right_error = true;
  }
  else
  {
    control_motor_right_error = false;
  }
}

void ControlSystem::requestModeCb(const std_msgs::String::ConstPtr& msg)
{
  robot_mode_req = msg->data;
  ROS_INFO("Request mode: %s", robot_mode_req.c_str());
  if (robot_mode_req == "MAPPING")
  {
    mapping_cnt++;
    ROS_INFO("Mapping request time: %d", mapping_cnt);
  }
}

void ControlSystem::requestSavemapCb(const std_msgs::String::ConstPtr& msg)
{
  if (!option_config["use_slam"])
    return;
  ROS_INFO("Request save map: %s", msg->data.c_str());
  try
  {
    Json::Value data_dict;
    Json::Reader reader;
    reader.parse(msg->data, data_dict);
    map_requesting = data_dict["name"].asString();
    std::string map_name =
        current_map_cfg["map_dir"].asString() + map_requesting;
    std_msgs::String map_msg;
    map_msg.data = map_name;
    pub_map_save_req.publish(map_msg);
    save_map_request = true;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("request_savemap_cb: %s", e.what());
  }
}

void ControlSystem::requestTempSavemapCb(const std_msgs::String::ConstPtr& msg)
{
  if (!option_config["use_slam"])
    return;
  ROS_INFO("Temp save map request: %s", msg->data.c_str());
  try
  {
    Json::Value data_dict;
    Json::Reader reader;
    reader.parse(msg->data, data_dict);
    map_requesting = data_dict["name"].asString();
    std::string map_name =
        current_map_cfg["map_dir"].asString() + map_requesting;
    std_msgs::String map_msg;
    map_msg.data = map_name;
    pub_temp_map_save_req.publish(map_msg);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("request_temp_savemap_cb: %s", e.what());
  }
}

void ControlSystem::requestChangeMapCb(const std_msgs::String::ConstPtr& msg)
{
  if (!option_config["use_slam"])
    return;
  ROS_INFO("Request change map: %s", msg->data.c_str());
  std::string file_check = current_map_cfg["map_dir"].asString() + msg->data;
  if (check_map(file_check))
  {
    main_state = "INIT_LOAD_MAP";
    current_map_cfg["map_file"] = msg->data;
    dump_config();
  }
  else
  {
    ROS_ERROR("The requesting map does not exist: %s", file_check.c_str());
  }
}

void ControlSystem::requestReloadMapCb(const std_msgs::String::ConstPtr& msg)
{
  if (!option_config["use_slam"])
    return;
  ROS_INFO("Request reload map: %s", msg->data.c_str());
  std::string file_check = current_map_cfg["map_dir"].asString() + msg->data;
  if (check_map(file_check))
  {
    main_state = "INIT_LOAD_MAP";
  }
  else
  {
    ROS_ERROR("The requesting map does not exist: %s", file_check.c_str());
  }
}

void ControlSystem::requestRunStopCb(const std_msgs::String::ConstPtr& msg)
{
  std_msgs::String mod_msg = *msg;
  if (msg->data == "STOP")
  {
    mod_msg.data = "PAUSE";
  }
  pub_mission_run_pause.publish(mod_msg);
  if (msg->data == "PAUSE")
  {
    pause_detail_status = "Pause by request pause";
  }
}

void ControlSystem::triggerMissionFromServerCb(
    const std_msgs::String::ConstPtr& msg)
{
  mission_from_server_req = true;
}

void ControlSystem::initialposeCb(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  init_pose_received = true;
}

void ControlSystem::requestStartMissionCb(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "START")
  {
    if (robot_mode == "AUTO")
    {
      pub_mission_control.publish(msg);
    }
  }
  else
  {
    reset_get_mission = true;
    pub_mission_control.publish(msg);
  }
}

void ControlSystem::standardIOCB(const std_msgs::String::ConstPtr& msg)
{
  try
  {
    std_io_status = json::parse(msg->data);
    if (std_io_status.contains("bumper"))
    {
      bumper = std_io_status["bumper"];
    }
    if (std_io_status.contains("emg_button"))
    {
      emg_button = std_io_status["emg_button"];
    }
    if (update_status_button)
    {
      update_status_button = false;
      if (std_io_status.contains("start_1_button"))
      {
        start_1_button = std_io_status["start_1_button"];
      }
      if (std_io_status.contains("start_2_button"))
      {
        start_2_button = std_io_status["start_2_button"];
      }
      if (std_io_status.contains("stop_1_button"))
      {
        stop_1_button = std_io_status["stop_1_button"];
      }
      if (std_io_status.contains("stop_2_button"))
      {
        stop_2_button = std_io_status["stop_2_button"];
      }
      if (std_io_status.contains("auto_manual_sw"))
      {
        auto_manual_sw = std_io_status["auto_manual_sw"];
      }
      if (std_io_status.contains("motor_enable_sw"))
      {
        motor_enable_sw = std_io_status["motor_enable_sw"];
      }
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("standard_io error: %s", e.what());
  }
  last_std_io_msg = ros::Time::now().toSec();
}

void ControlSystem::odomCb(const nav_msgs::Odometry::ConstPtr& msg)
{
  last_odom_msg = ros::Time::now().toSec();
}

void ControlSystem::arduinoErrorCb(const std_msgs::String::ConstPtr& msg)
{
  arduino_error_msg = msg->data;
}

void ControlSystem::resetErrorCb(const std_msgs::String::ConstPtr& msg)
{
  reset_error_request = true;
  std_msgs::EmptyStamped empty_msg;
  empty_msg.header.stamp = ros::Time::now();
  pub_mission_reset_error.publish(empty_msg);
}

void ControlSystem::fakeBatteryCb(const std_msgs::Float32::ConstPtr& msg)
{
  battery_percent = msg->data;
  last_battery_fake = ros::Time::now().toSec();
}

void ControlSystem::realBatteryCb(const std_msgs::Float32::ConstPtr& msg)
{
  if (ros::Time::now().toSec() - last_battery_fake > 1.0)
  {
    try
    {
      battery_percent = static_cast<int>(msg->data);
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("real_battery_cb: %s", e.what());
    }
  }
}

void ControlSystem::batteryAmpeCb(const std_msgs::Float32::ConstPtr& msg)
{
  battery_ampe = msg->data;
}

void ControlSystem::batteryVoltageCb(const std_msgs::Float32::ConstPtr& msg)
{
  battery_voltage = msg->data;
}

void ControlSystem::ledControlAliveCb(const std_msgs::Empty::ConstPtr& msg)
{
  last_led_alive = ros::Time::now().toSec();
}

void ControlSystem::safetyStatusCb(
    const std_msgs::UInt32MultiArray::ConstPtr& msg)
{
  last_safety_time = ros::Time::now().toSec();
  if (!msg->data.empty())
  {
    is_safety = msg->data[0] == 1;
  }
  else
  {
    is_safety = false;
  }
}

void ControlSystem::updateFeedback()
{
  StringStamped msg;
  msg.stamp = ros::Time::now();

  // Publish robot_mode
  msg.data = robotMode.toString();
  pubRobotMode.publish(msg);

  // Publish robot_status
  statusDict["working_status"] = workingStatus;
  statusDict["status"] = robotStatus.toString();
  statusDict["state"] = mainState.toString();
  statusDict["mode"] = robotMode.toString();
  statusDict["sound"] = (soundStatus == "") ? "OFF" : soundStatus;
  statusDict["detail"] = detailStatus;
  statusDict["error_code"] = errorCode;  // TODO: Display in app
  statusDict["special_matching_state"] = specialMatchingState;
  msg.data = json.dumps(statusDict, 2);
  pubRobotStatus.publish(msg);

  // Current map
  msg.data = currentMapCfg.mapFile;
  if (optionConfig["use_slam"])
  {
    pubCurrentMap.publish(msg);
  }

  Int8Stamped msgBattery;
  msgBattery.data = batteryPercent;
  if (msgBattery.data > 100)
  {
    msgBattery.data = 100;
  }
  if (msgBattery.data < 0)
  {
    msgBattery.data = 0;
  }
  msgBattery.stamp = ros::Time::now();
  pubBattery.publish(msgBattery);
}

void ControlSystem::updateErrorCode(const std::string& moduleDisconnected,
                                    const std::string& msg)
{
  if (!moduleDisconnected.empty())
  {
    errorCode = '"' + moduleDisconnected + '" module disconnected';
  }
  else if (!msg.empty())
  {
    errorCode = msg;
  }
}

void ControlSystem::clearErrorCode() { errorCode = ""; }

void ControlSystem::movingControlModuleStatusCB(
    const std_msgs::String::ConstPtr& msg)
{
  try
  {
    movingControlStatusDict = json::parse(msg->data);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("moving_control_module_status_cb: %s", e.what());
  }
}

/*
##        #######   #######  ########
##       ##     ## ##     ## ##     ##
##       ##     ## ##     ## ##     ##
##       ##     ## ##     ## ########
##       ##     ## ##     ## ##
##       ##     ## ##     ## ##
########  #######   #######  ##
*/

void ControlSystem::checkConditionResetErr()
{
  if (robotModeReq == RobotMode::MAPPING)
  {
    robotModeReq = RobotMode::NONE;
    mainState = MainState::INIT_MAPPING;
  }
  if (slamState == SlamState::LOCALIZING)
  {
    mainState = MainState::MANUAL;
    robotModeReq = RobotMode::NONE;
  }
  if (slamState == SlamState::SWITCH_LOCALIZING)
  {
    mainState = MainState::LOADING_MAP;
  }
  if (slamState == SlamState::SWITCH_MAPPING)
  {
    mainState = MainState::WAIT_MAPPING;
  }
  if (slamState == SlamState::MAPPING)
  {
    mainState = MainState::MAPPING;
    robotModeReq = RobotMode::NONE;
  }
  if (robotModeReq == RobotMode::MANUAL)
  {
    robotModeReq = RobotMode::NONE;
    mainState = MainState::INIT_LOAD_MAP;
  }
}

void ControlSystem::loop()
{
  ros::Duration(2.0).sleep();
  ros::Time begin_load_map = ros::Time::now();
  ros::Time last_pub_time = ros::Time::now();
  ros::Time begin_hold_stop_button = ros::Time::now();
  ros::Time begin_load_map_error = ros::Time::now();
  ros::Time begin_check_mode_change = ros::Time::now();
  ros::Time begin_check_status_change = ros::Time::now();
  ros::Time last_sound_time = ros::Time::now();
  ros::Time begin_loop = ros::Time::now();
  ros::Time trigger_button_time = ros::Time::now();
  std::string trigger_button = "";
  bool first_check_mode = true;
  bool pushed_stop_mission = false;
  bool odom_ready = true;
  ModuleStatus prev_led_stt = ModuleStatus::NONE;
  ModuleStatus prev_sound_stt = ModuleStatus::NONE;
  bool prev_auto_man_switch = SWITCH_MANUAL_MODE;
  bool last_auto_man_switch = SWITCH_MANUAL_MODE;
  std::string current_safety_job = "";
  bool pause_by_error = false;
  RobotStatus last_robot_status = RobotStatus::NONE;
  RobotMode last_robot_mode = RobotMode::NONE;
  bool last_start_1_button = SENSOR_DEACTIVATE;
  bool last_start_2_button = SENSOR_DEACTIVATE;
  bool last_stop_1_button = SENSOR_DEACTIVATE;
  bool last_stop_2_button = SENSOR_DEACTIVATE;
  bool pre_start_1_button = SENSOR_DEACTIVATE;
  bool pre_start_2_button = SENSOR_DEACTIVATE;
  bool pre_stop_1_button = SENSOR_DEACTIVATE;
  bool pre_stop_2_button = SENSOR_DEACTIVATE;
  ros::Time time_update_button = ros::Time::now();
  std::string last_module_disconnected = "";
  ros::Rate r(10.0);
  while (ros::ok())
  {
    update_status_button = true;
    if (main_state != prev_state)
    {
      ROS_INFO("Main state: %s -> %s", prev_state.toString().c_str(),
               main_state.toString().c_str());
      prev_state = main_state;
    }
    detail_status = DetailStatus::ALL_OK.toString();
    // State
    if (slam_manager_module.module_status_dict.find("state") !=
        slam_manager_module.module_status_dict.end())
    {
      slam_state = SlamState[slam_manager_module.module_status_dict["state"]];
    }
    // Status
    if (mission_manager_module.module_status_dict.find("status") !=
        mission_manager_module.module_status_dict.end())
    {
      mission_status =
          ModuleStatus[mission_manager_module.module_status_dict["status"]];
    }
    // Running mission id
    if (mission_manager_module.module_status_dict.find("running_mission_id") !=
        mission_manager_module.module_status_dict.end())
    {
      running_mission_id =
          mission_manager_module.module_status_dict["running_mission_id"];
    }
    if (mission_manager_module.module_status_dict.find(
            "last_done_mission_id") !=
        mission_manager_module.module_status_dict.end())
    {
      last_done_mission_id =
          mission_manager_module.module_status_dict["last_done_mission_id"];
    }
    // Safety job
    try
    {
      bool use_safety = false;
      if (option_config.find("use_safety") != option_config.end())
      {
        use_safety = option_config["use_safety"];
      }
      if (use_safety)
      {
        current_safety_job =
            json.loads(rospy.get_param("/scan_safety_node_ng/job"));
        if (current_safety_job["safety"].size() == 0)
        {
          safety_disabled = true;
        }
      }
    }
    catch (...)
    {
    }
    // Update current position
    geometry_msgs::PoseStamped temp_pose =
        lockup_pose(tf_listener, map_frame, robot_frame);
    if (temp_pose != nullptr)
    {
      current_pose = temp_pose;
    }
    // State: INIT
    if (main_state == MainState::INIT)
    {
      // slam_manager is ready
      if (slam_state == SlamState::MAPPING)
      {
        main_state = MainState::MAPPING;
        robot_mode_req = RobotMode::NONE;
        continue;
      }
      else if (slam_state == SlamState::LOCALIZING)
      {
        if (auto_manual_sw == SENSOR_DEACTIVATE &&
            option_config.find("use_auto_man_sw") != option_config.end() &&
            option_config["use_auto_man_sw"])
        {
          main_state = MainState::MANUAL;
        }
        else
        {
          main_state = MainState::AUTO;
        }
        continue;
      }
      else if (slam_state == SlamState::SWITCH_LOCALIZING)
      {
        if (auto_manual_sw == SENSOR_DEACTIVATE &&
            option_config.find("use_auto_man_sw") != option_config.end() &&
            option_config["use_auto_man_sw"])
        {
          main_state = MainState::WAIT_MANUAL;
        }
        else
        {
          main_state = MainState::WAIT_AUTO;
        }
        continue;
      }
      else if (slam_state == SlamState::SWITCH_MAPPING)
      {
        main_state = MainState::WAIT_MAPPING;
        continue;
      }
      // slam_manager is not ready
      else if (robot_mode_req == RobotMode::MANUAL)
      {
        robot_mode_req = RobotMode::NONE;
        pause_robot();
        main_state = MainState::WAIT_MANUAL;
        if (slam_state != SlamState::LOCALIZING)
        {
          main_state = MainState::INIT_LOAD_MAP;
        }
      }
      else if (ros::Time::now() - begin_loop > ros::Duration(2.0))
      {
        if (slam_state != SlamState::LOCALIZING)
        {
          main_state = MainState::INIT_LOAD_MAP;
        }
      }
      // State: WAIT_MAPPING
      else if (main_state == MainState::WAIT_MAPPING)
      {
        robot_mode = RobotMode::WAIT_RESPOND;
        if (slam_state == SlamState::MAPPING)
        {
          main_state = MainState::MAPPING;
        }
        else if (slam_state == SlamState::ERROR_MAPPING)
        {
          main_state = MainState::MAPPING_ERROR;
        }
        if (ros::Time::now() - slam_manager_module.last_module_status >
            ros::Duration(2.0))
        {
          main_state = MainState::LOAD_MAP_ERROR;
        }
      }
      // State: WAIT_MANUAL
      else if (main_state == MainState::WAIT_MANUAL)
      {
        robot_mode = RobotMode::WAIT_RESPOND;
        main_state = MainState::MANUAL;
      }
      // State: WAIT_AUTO
      else if (main_state == MainState::WAIT_AUTO)
      {
        robot_mode = RobotMode::WAIT_RESPOND;
        if (slam_state == SlamState::LOCALIZING)
        {
          main_state = MainState::AUTO;
        }
        else
        {
          ROS_ERROR("Not yet switch to AUTO");
        }
      }
      // State: MANUAL
      else if (main_state == MainState::MANUAL)
      {
        robot_mode = RobotMode::MANUAL;
        if (init_pose_received)
        {
          init_pose_received = false;
          set_pose_init(true);
        }
        if (robot_mode_req == RobotMode::AUTO ||
            (auto_manual_sw == SWITCH_AUTO_MODE && !simulation))
        {
          robot_mode_req = RobotMode::NONE;
          main_state = MainState::WAIT_AUTO;
          continue;
        }
        else if (robot_mode_req == RobotMode::MAPPING)
        {
          robot_mode_req = RobotMode::NONE;
          main_state = MainState::INIT_MAPPING;
          continue;
        }
        if (slam_state == SlamState::ERROR_LOCALIZING && odom_ready)
        {
          main_state = MainState::LOAD_MAP_ERROR;
          continue;
        }
        check_condition_reset_err();
      }
      // State: AUTO
      else if (main_state == MainState::AUTO)
      {
        robot_mode = RobotMode::AUTO;
        if (init_pose_received)
        {
          init_pose_received = false;
          set_pose_init(true);
        }
        if (robot_mode_req == RobotMode::MANUAL ||
            (auto_manual_sw == SENSOR_DEACTIVATE && !simulation))
        {
          robot_mode_req = RobotMode::NONE;
          pause_robot();
          pause_detail_status = "Pause by switch mode to MANUAL";
          main_state = MainState::WAIT_MANUAL;
        }
        else if (robot_mode_req == RobotMode::MAPPING)
        {
          robot_mode_req = RobotMode::NONE;
          main_state = MainState::INIT_MAPPING;
        }
        if (slam_state == SlamState::ERROR_LOCALIZING && odom_ready)
        {
          main_state = MainState::LOAD_MAP_ERROR;
        }
      }
      // State: MAPPING
      else if (main_state == MainState::MAPPING)
      {
        robot_mode = RobotMode::MAPPING;
        if (robot_mode_req == RobotMode::MANUAL)
        {
          robot_mode_req = RobotMode::NONE;
          main_state = MainState::INIT_LOAD_MAP;
          continue;
        }
        if (save_map_request)
        {
          if (current_map_cfg.map_dir + map_requesting ==
              slam_manager_module.module_status_dict["current_map"])
          {
            ROS_INFO("MAPPING stop -> load new map");
            save_map_request = false;
            current_map_cfg.map_file = map_requesting;
            dump_config();
            main_state = MainState::INIT_LOAD_MAP;
            continue;
          }
        }
        check_condition_reset_err();
      }
      // State: INIT_LOAD_MAP
      else if (main_state == MainState::INIT_LOAD_MAP)
      {
        std::string map_file_check =
            current_map_cfg.map_dir + current_map_cfg.map_file;
        if (check_map(map_file_check))
        {
          load_map_cnt++;
          ROS_INFO("Load map request time: %d", load_map_cnt);
          load_map(current_map_cfg.map_file);
          begin_load_map = ros::Time::now();
          main_state = MainState::LOADING_MAP;
        }
        else
        {
          main_state = MainState::LOAD_MAP_ERROR;
        }
      }
      // State: INIT_MAPPING
      else if (main_state == MainState::INIT_MAPPING)
      {
        pub_mapping_req.publish(StringStamped());
        main_state = MainState::WAIT_MAPPING;
      }
      // State: LOADING_MAP
      else if (main_state == MainState::LOADING_MAP)
      {
        robot_mode = RobotMode::WAIT_RESPOND;
        if (slam_state == SlamState::LOCALIZING)
        {
          if (ros::Time::now() - begin_load_map > ros::Duration(2.0))
          {
            if (first_check_mode && auto_manual_sw == SWITCH_AUTO_MODE)
            {
              main_state = MainState::AUTO;
            }
            else
            {
              main_state = MainState::MANUAL;
            }
            robot_mode_req = RobotMode::NONE;
            first_check_mode = false;
          }
        }
        if (slam_state == SlamState::ERROR_LOCALIZING)
        {
          main_state = MainState::LOAD_MAP_ERROR;
        }
        if (ros::Time::now() - slam_manager_module.last_module_status >
            ros::Duration(2.0))
        {
          main_state = MainState::LOAD_MAP_ERROR;
        }
      }
      // State: LOAD_MAP_ERROR
      else if (main_state == MainState::LOAD_MAP_ERROR)
      {
        detail_status = DetailStatus::LOAD_MAP_ERROR.toString();
        check_condition_reset_err();
        bool auto_reset_load_map = false;
        if (ros::Time::now() - begin_load_map_error > ros::Duration(30.0))
        {
          auto_reset_load_map = true;
          ROS_WARN("Auto reset LOAD_MAP_ERROR");
        }
        if (reset_error_request || auto_reset_load_map)
        {
          reset_error_request = false;
          main_state = MainState::INIT_LOAD_MAP;
        }
      }
      // State: MAPPING_ERROR
      else if (main_state == MainState::MAPPING_ERROR)
      {
        robot_mode = RobotMode::MAPPING;
        detail_status = DetailStatus::MAPPING_ERROR.toString();
        check_condition_reset_err();
        if (reset_error_request)
        {
          reset_error_request = false;
          main_state = MainState::INIT_MAPPING;
        }
      }

      // Set timeout
      if (main_state == MainState::LOAD_MAP_ERROR &&
          prev_state != MainState::LOAD_MAP_ERROR)
      {
        begin_load_map_error = ros::Time::now();
      }
    }
    std::string module_disconnected = "";
    std::string special_matching_error = "";
    std::string special_matching_state = "";
    std::string special_error_led = "";
    std::string special_error_sound = "";
    std::string special_state_led = "";
    std::string special_state_sound = "";
    std::string module_error_code = "";
    bool pause_if_error = true;

    for (auto& m : module_list)
    {
      if (!m.module_alive && (m.handle_when_sim || !simulation))
      {
        updateErrorCode("module_disconnected", m.display_name);
        module_disconnected = m.display_name;
        break;
      }
    }

    for (auto& m : module_list)
    {
      if (!m.special_matching_error.empty())
      {
        special_matching_error = m.special_matching_error;
        special_error_led = m.special_error_led;
        special_error_sound = m.special_error_sound;
        updateErrorCode("special_matching_error", special_matching_error);
        break;
      }
    }

    for (auto& m : module_list)
    {
      if (!m.error_code.empty())
      {
        module_error_code = m.error_code;
        pause_if_error = m.pause_if_error;
        break;
      }
    }

    for (auto& m : module_list)
    {
      if (!m.special_matching_state.empty())
      {
        special_matching_state = m.special_matching_state;
        special_state_led = m.special_state_led;
        special_state_sound = m.special_state_sound;
        break;
      }
    }

    special_matching_state = special_matching_state;
    if (module_disconnected.empty() && special_matching_error.empty())
    {
      clearErrorCode();
    }

    robot_status = RobotStatus::NONE;

    if (ros::Time::now().toSec() - last_std_io_msg > 1.0 && !simulation)
    {
      robot_status = RobotStatus::ERROR;
      detail_status = DetailStatus::FATAL_ERROR.toString();
      updateErrorCode("IO_BOARD_DISCONNECT");
      odom_ready = false;
    }
    else if (ros::Time::now().toSec() - arduino_error_msg.stamp.toSec() < 0.5 &&
             !simulation)
    {
      arduino_error_cnt++;
      if (arduino_error_cnt > 30)
      {
        robot_status = RobotStatus::ERROR;
        detail_status = DetailStatus::FATAL_ERROR.toString();
        updateErrorCode("IO_BOARD_ERROR");
        odom_ready = false;
      }
    }
    else if ((emg_button == SENSOR_DEACTIVATE ||
              (bumper == SENSOR_DEACTIVATE && option_config["use_bumper"])) &&
             !simulation)
    {
      robot_status = RobotStatus::EMG;
      if (last_robot_status != robot_status)
      {
        pauseRobot();
        pause_detail_status = emg_button == SENSOR_DEACTIVATE
                                  ? "Pause by push EMG button"
                                  : "Pause by detect bumper collision";
      }
    }
    else if (motor_enable_sw == SENSOR_DEACTIVATE && !simulation &&
             option_config["use_motor_release"])
    {
      robot_status = RobotStatus::ERROR;
      detail_status = DetailStatus::MOTOR_OFF.toString();
      updateErrorCode(detail_status);
    }
    else if (!module_disconnected.empty())
    {
      robot_status = RobotStatus::ERROR;
      detail_status = DetailStatus::FATAL_ERROR.toString();
    }
    else if (main_state == MainState::LOAD_MAP_ERROR ||
             main_state == MainState::MAPPING_ERROR)
    {
      robot_status = RobotStatus::ERROR;
      updateErrorCode(detail_status);
    }
    else if (!special_matching_error.empty())
    {
      robot_status = RobotStatus::ERROR;
      detail_status = special_matching_error;
    }
    else if (!module_error_code.empty())
    {
      robot_status = RobotStatus::ERROR;
      detail_status = DetailStatus::GENERAL_ERROR.toString();
      updateErrorCode(module_error_code);
    }
    else if (battery_percent < battery_config["empty_threshold"] &&
             special_matching_state != DetailStatus::AUTO_CHARGING.toString() &&
             special_matching_state != DetailStatus::MANUAL_CHARGING.toString())
    {
      robot_status = RobotStatus::ERROR;
      detail_status = DetailStatus::BATTERY_EMPTY.toString();
      updateErrorCode(detail_status);
    }
    else if (motor_left_over_ampe || motor_right_over_ampe)
    {
      robot_status = RobotStatus::ERROR;
      detail_status = DetailStatus::MOTOR_OVER_CURRENT.toString();
      updateErrorCode(detail_status);
    }
    else if (!pose_initiated && option_config["check_init_pose"])
    {
      robot_status = RobotStatus::WAITING_INIT_POSE;
    }
    else if (mission_status == ModuleStatus::WAITING)
    {
      robot_status = RobotStatus::WAITING;
    }
    else if (mission_status == ModuleStatus::PAUSED)
    {
      robot_status = RobotStatus::PAUSED;
    }
    else if (mission_status == ModuleStatus::RUNNING)
    {
      robot_status = RobotStatus::RUNNING;
    }

    // Check for FATAL_ERROR with higher priority than ODOMETRY_ERROR
    if (ros::Time::now().toSec() - last_odom_msg > 1.0)
    {
      setPoseInit(false);
    }

    if (!module_disconnected.empty() && last_module_disconnected.empty())
    {
      ROS_ERROR("Module disconnected: %s", module_disconnected.c_str());
    }

    if (detail_status != DetailStatus::IO_BOARD_DISCONNECT.toString() &&
        detail_status != DetailStatus::IO_BOARD_ERROR.toString())
    {
      odom_ready = true;
    }

    // Normal status, no need to update robot_status

    if (robot_status != RobotStatus::ERROR)
    {
      if (battery_percent < battery_config["low_threshold"] &&
          special_matching_state != DetailStatus::AUTO_CHARGING.toString() &&
          special_matching_state != DetailStatus::MANUAL_CHARGING.toString())
      {
        detail_status = DetailStatus::BATTERY_LOW.toString();
      }
      else if (safety_disabled && robot_status == RobotStatus::RUNNING &&
               special_matching_state !=
                   DetailStatus::AUTO_CHARGING.toString() &&
               special_matching_state !=
                   DetailStatus::MANUAL_CHARGING.toString() &&
               special_matching_state == "GOING_TO_POS")
      {
        detail_status = DetailStatus::SAFETY_DISABLED.toString();
      }
      else if ((is_safety ||
                ros::Time::now().toSec() - last_safety_time > 0.5) &&
               special_matching_state == "GOING_TO_POS")
      {
        detail_status = DetailStatus::SAFETY_STOP.toString();
      }
      else if (!special_matching_state.empty())
      {
        detail_status = special_matching_state;
      }
      else if (std::find(list_action_move.begin(), list_action_move.end(),
                         current_action_type) != list_action_move.end())
      {
        detail_status = "GOING_TO_POS";
      }
    }
    if (working_status == "PRODUCTION")
    {
      if (detail_status == DetailStatus::FATAL_ERROR.toString())
      {
        led_status = LedStatus::FATAL_ERROR.toString();
      }
      else if (robot_status == RobotStatus::EMG)
      {
        led_status = LedStatus::EMG.toString();
      }
      else if (special_error_led != "")
      {
        led_status = special_error_led;
      }
      else if (detail_status == DetailStatus::MOTOR_OFF.toString())
      {
        led_status = LedStatus::MOTOR_OFF.toString();
      }
      else if (robot_status == RobotStatus::ERROR)
      {
        led_status = LedStatus::GENERAL_ERROR.toString();
      }
      else if (main_state == MainState::WAIT_MAPPING ||
               main_state == MainState::WAIT_MANUAL ||
               main_state == MainState::WAIT_AUTO ||
               main_state == MainState::LOADING_MAP)
      {
        led_status = LedStatus::WAIT_RESPOND.toString();
      }
      else if (detail_status == DetailStatus::SAFETY_DISABLED.toString())
      {
        led_status = LedStatus::SAFETY_DISABLED.toString();
      }
      else if (detail_status == DetailStatus::AUTO_CHARGING.toString() &&
               mission_status == ModuleStatus::WAITING &&
               robot_mode == RobotMode::AUTO)
      {
        led_status = LedStatus::CHARGING_READY.toString();
      }
      else if (detail_status == DetailStatus::SAFETY_STOP.toString())
      {
        led_status = LedStatus::SAFETY_STOP.toString();
      }
      else if (special_state_led != "")
      {
        led_status = special_state_led;
      }
      else if (detail_status == DetailStatus::BATTERY_EMPTY.toString())
      {
        led_status = LedStatus::BATTERY_EMPTY.toString();
      }
      else if (robot_status == RobotStatus::WAITING_INIT_POSE)
      {
        led_status = LedStatus::WAITING_INIT_POSE.toString();
      }
      else if (detail_status == DetailStatus::BATTERY_LOW.toString())
      {
        led_status = LedStatus::BATTERY_LOW.toString();
      }
      else if (main_state == MainState::MANUAL)
      {
        led_status = LedStatus::MANUAL.toString();
      }
      else if (main_state == MainState::MAPPING)
      {
        led_status = LedStatus::MAPPING.toString();
      }
      else if (robot_status == RobotStatus::PAUSED)
      {
        led_status = LedStatus::PAUSED.toString();
      }
      else if (robot_status == RobotStatus::WAITING)
      {
        led_status = LedStatus::WAITING.toString();
      }
      else if (mission_status == ModuleStatus::RUNNING)
      {
        led_status = LedStatus::RUNNING_NORMAL.toString();
      }
      else
      {
        ROS_WARN("Led status was not set");
      }
    }
    else
    {
      led_status = LedStatus::MAINTENANCE.toString();
    }

    // Clear Arduino Error
    if (ros::Time::now().toSec() - arduino_error_msg.stamp.toSec() > 5.0)
    {
      arduino_error_cnt = 0;
    }

    // Record log
    if (robot_status == RobotStatus::ERROR &&
        last_robot_status != RobotStatus::ERROR)
    {
      recordLog("ERROR: " + detail_status, ros::this_node::getName(),
                LogLevel::ERROR.toString());
      ROS_ERROR("ERROR: %s", detail_status.c_str());
    }

    // Pause if error
    if (robot_status == RobotStatus::ERROR &&
        last_robot_status != RobotStatus::ERROR &&
        last_robot_status != RobotStatus::PAUSED && pause_if_error &&
        robot_mode == RobotMode::AUTO)
    {
      ROS_WARN("Pause by error");
      pauseRobot();
    }

    // Check clear pause_by_error
    if (robot_status == RobotStatus::WAITING && pause_by_error &&
        robot_mode == RobotMode::AUTO)
    {
      pause_by_error = false;
      ROS_WARN("Clear pause by error");
    }

    // FIXME: Sometime MANUAL or ERROR but Robot still run
    if (robot_mode != RobotMode::AUTO && last_robot_mode == RobotMode::AUTO)
    {
      begin_check_mode_change = ros::Time::now().toSec();
    }
    if (robot_status != RobotStatus::RUNNING &&
        robot_status == RobotStatus::RUNNING)
    {
      begin_check_status_change = ros::Time::now().toSec();
    }
    if (robot_mode != RobotMode::AUTO &&
        ros::Time::now().toSec() - begin_check_mode_change > 1.0 &&
        ros::Time::now().toSec() - begin_check_mode_change < 3.0)
    {
      if (moving_control_status_dict["status"] ==
          ModuleStatus::RUNNING.toString())
      {
        pauseRobot();
        stopMoving();
        ROS_ERROR("Not yet PAUSED after switch mode to MANUAL");
      }
    }
    if (robot_status != RobotStatus::RUNNING &&
        ros::Time::now().toSec() - begin_check_status_change > 1.0 &&
        ros::Time::now().toSec() - begin_check_status_change < 3.0)
    {
      if (moving_control_status_dict["status"] ==
          ModuleStatus::RUNNING.toString())
      {
        pauseRobot();
        stopMoving();
        ROS_ERROR("Not yet PAUSED after status != RUNNING");
      }
    }
    // Button handle
    // Hold PAUSE button to stop mission
    // Không check Mode khi gửi stop mission vì không chắc chắn lúc đó
    // mission_manager có đang hoạt động không
    if (robot_mode == RobotMode::AUTO)
    {
      if ((stop_1_button == SENSOR_ACTIVATE &&
           last_stop_1_button == SENSOR_DEACTIVATE) ||
          (stop_2_button == SENSOR_ACTIVATE &&
           last_stop_2_button == SENSOR_DEACTIVATE))
      {
        pushed_stop_mission = false;
      }
      if ((stop_1_button == SENSOR_ACTIVATE &&
           last_stop_1_button == SENSOR_ACTIVATE) ||
          (stop_2_button == SENSOR_ACTIVATE &&
           last_stop_2_button == SENSOR_ACTIVATE))
      {
        if (ros::Time::now().toSec() - begin_hold_stop_button >= 3.0)
        {
          if (!pushed_stop_mission)
          {
            reset_get_mission = true;
            pushed_stop_mission = true;
            ROS_INFO("Stop all mission when hold PAUSE button");
            pub_request_start_mission.publish("STOP");
          }
        }
      }
      else
      {
        begin_hold_stop_button = ros::Time::now().toSec();
      }
    }

    // Reset error
    if (robot_status == RobotStatus::ERROR && robot_mode == RobotMode::AUTO)
    {
      if ((start_1_button == SENSOR_ACTIVATE &&
           last_start_1_button == SENSOR_DEACTIVATE) ||
          (start_2_button == SENSOR_ACTIVATE &&
           last_start_2_button == SENSOR_DEACTIVATE))
      {
        pub_mission_reset_error.publish(std_msgs::Empty());
        ROS_INFO("Reset error when push RUN button");
      }
    }

    // Trigger
    else if (robot_status == RobotStatus::WAITING &&
             robot_mode == RobotMode::AUTO)
    {
      Json::Value trigger_dict;
      trigger_dict["topic"] = "";
      trigger_dict["run_immediately"] = true;

      if (current_pose != nullptr)
      {
        // Only press
        if (start_1_button == SENSOR_ACTIVATE &&
            last_start_1_button == SENSOR_DEACTIVATE)
        {
          trigger_dict["topic"] = "FRONT_RUN_BUTTON";
        }
        else if (start_2_button == SENSOR_ACTIVATE &&
                 last_start_2_button == SENSOR_DEACTIVATE)
        {
          trigger_dict["topic"] = "REAR_RUN_BUTTON";
        }
        else if (stop_1_button == SENSOR_ACTIVATE &&
                 last_stop_1_button == SENSOR_DEACTIVATE)
        {
          trigger_dict["topic"] = "FRONT_PAUSE_BUTTON";
        }
        else if (stop_2_button == SENSOR_ACTIVATE &&
                 last_stop_2_button == SENSOR_DEACTIVATE)
        {
          trigger_dict["topic"] = "REAR_PAUSE_BUTTON";
        }

        std_msgs::String trigger_msg;
        trigger_msg.data = Json::FastWriter().write(trigger_dict);
        pub_trigger.publish(trigger_msg);
        ROS_INFO_STREAM("Send trigger when push button: "
                        << trigger_dict["topic"].asString());
      }
    }

    // Resume
    else if (robot_status == RobotStatus::PAUSED &&
             robot_mode == RobotMode::AUTO)
    {
      if ((start_1_button == SENSOR_ACTIVATE &&
           last_start_1_button == SENSOR_DEACTIVATE) ||
          (start_2_button == SENSOR_ACTIVATE &&
           last_start_2_button == SENSOR_DEACTIVATE))
      {
        ROS_INFO("Resume robot when push RUN button");
        resume_running();
      }
    }

    // Pause
    else if (robot_status == RobotStatus::RUNNING)
    {
      if ((stop_1_button == SENSOR_ACTIVATE &&
           last_stop_1_button == SENSOR_DEACTIVATE) ||
          (stop_2_button == SENSOR_ACTIVATE &&
           last_stop_2_button == SENSOR_DEACTIVATE))
      {
        ROS_INFO("Pause robot when push PAUSE button");
        pause_robot();
        pause_detail_status = "Pause by push PAUSE button";
      }
    }

    // Publish status
    double now = ros::Time::now().toSec();
    if (now - last_pub_time >= 0.2)
    {
      last_pub_time = now;
      update_feedback();
    }

    // Publish led status
    if (led_status != prev_led_stt)
    {
      std_msgs::String msg;
      msg.data = led_status;
      pub_led_status.publish(msg);
      prev_led_stt = led_status;
    }
    // Special sound
    if (ros::Time::now().toSec() - time_start > 60)
    {
      if (sound_config["play_sound_when_error"])
      {
        if (detail_status == DetailStatus::FATAL_ERROR.toString())
        {
          sound_status = LedStatus::FATAL_ERROR.toString();
        }
        else if (robot_status == RobotStatus::EMG)
        {
          sound_status = LedStatus::EMG.toString();
        }
        else if (special_error_sound != "")
        {
          sound_status = special_error_sound;
        }
        else if (detail_status == DetailStatus::BATTERY_EMPTY.toString())
        {
          sound_status = LedStatus::BATTERY_EMPTY.toString();
        }
        else if (robot_status == RobotStatus::ERROR)
        {
          sound_status = LedStatus::GENERAL_ERROR.toString();
        }
        else if (detail_status == DetailStatus::SAFETY_DISABLED.toString())
        {
          sound_status = LedStatus::SAFETY_DISABLED.toString();
        }
        else if (detail_status == DetailStatus::SAFETY_STOP.toString())
        {
          sound_status = LedStatus::SAFETY_STOP.toString();
        }
        else if (special_state_sound != "")
        {
          sound_status = special_state_sound;
        }
        else if (robot_status == RobotStatus::PAUSED)
        {
          sound_status = LedStatus::PAUSED.toString();
        }
        else
        {
          sound_status = "";
        }
      }
      else
      {
        if (detail_status == DetailStatus::SAFETY_DISABLED.toString())
        {
          sound_status = detail_status;
        }
        else
        {
          sound_status = special_state_sound;
        }
      }

      if (sound_status != prev_sound_stt)
      {
        last_sound_time = ros::Time::now().toSec();
        pub_request_sound.publish(
            StringStamped(ros::Time::now(), sound_status));
      }
      prev_sound_stt = sound_status;
    }

    // Test server by topic
    if (mission_from_server_req && robot_mode == RobotMode::AUTO &&
        robot_status == RobotStatus::WAITING)
    {
      try
      {
        std::string mission_fr_server_file =
            ros::package::getPath("control_system") + "/mission/mission.json";
        std::ifstream j(mission_fr_server_file);
        if (j.is_open())
        {
          ROS_WARN_STREAM(mission_fr_server_file);
          Json::Value mission_dict;
          j >> mission_dict;
          std::string mission_json = Json::FastWriter().write(mission_dict);
          pub_mission_fr_server.publish(mission_json);
          pub_mission_fr_server_latch.publish(mission_json);
        }
      }
      catch (const std::exception& e)
      {
        ROS_ERROR_STREAM("Test mission from server: " << e.what());
      }
    }

    // Update last button states
    last_start_1_button = start_1_button;
    last_start_2_button = start_2_button;
    last_stop_1_button = stop_1_button;
    last_stop_2_button = stop_2_button;
    last_auto_man_switch = auto_manual_sw;
    last_robot_status = robot_status;
    last_robot_mode = robot_mode;
    last_module_disconnected = module_disconnected;

    // Sleep to control loop rate
    ros::Duration(0.1).sleep();
  }
}

};  // namespace control_system

namespace po = boost::program_options;

struct Options
{
  bool simulation;
  bool log_debug;
  std::string current_map_file;
  std::string robot_config_file;
  std::string robot_define;
};

Options parse_opts(int argc, char* argv[])
{
  Options options;

  po::options_description desc("Options");
  desc.add_options()("simulation,s", "type '-s' if simulation")(
      "ros_debug,d", "log_level=rospy.DEBUG")(
      "current_map_file,c", po::value<std::string>()->default_value(
                                "/tmp/ros/maps/current_map.yaml"))(
      "robot_config_file,r",
      po::value<std::string>()->default_value("/path/to/robot_config.yaml"))(
      "robot_define",
      po::value<std::string>()->default_value("/path/to/robot_define.yaml"));

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  options.simulation = vm.count("simulation");
  options.log_debug = vm.count("ros_debug");
  options.current_map_file = vm["current_map_file"].as<std::string>();
  options.robot_config_file = vm["robot_config_file"].as<std::string>();
  options.robot_define = vm["robot_define"].as<std::string>();

  return options;
}

int main(int argc, char* argv[])
{
  Options options = parse_opts(argc, argv);

  // Create directories if not exists
  std::system("mkdir -p /tmp/ros/maps/");

  // Initialize node
  ros::init(argc, argv, "control_system");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Set log level
  if (options.log_debug)
  {
    ROS_DEBUG("Log level set to DEBUG");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Debug);
    ros::console::notifyLoggerLevelsChanged();
  }

  // Log initialization
  ROS_INFO_STREAM("Init node " << ros::this_node::getName());

  // Create ControlSystem object
  ControlSystem control_system(options);

  ros::spin();

  return 0;
}
