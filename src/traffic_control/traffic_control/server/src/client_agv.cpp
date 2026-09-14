#include "server/client_agv.h"

void sigintHandler(int sig) {
  ROS_INFO("Shutting down...");

  // Perform any necessary shutdown procedures here
  // For example, stop any ongoing actions or cleanup resources

  // Shutdown ROS
  ros::shutdown();
}

Client::Client(ros::NodeHandle &n, const std::string &server_address_)
    : nh(n), n_param_("~") {

  cli_send_goal_ = std::make_shared<httplib::Client>(server_address_.c_str());
  cli_get_last_path_ =
      std::make_shared<httplib::Client>(server_address_.c_str());
  cli_update_robot_info_ =
      std::make_shared<httplib::Client>(server_address_.c_str());

  // Đặt timeout & cấu hình
  configureClient(cli_send_goal_);
  configureClient(cli_get_last_path_);
  configureClient(cli_update_robot_info_);

  // Ros
  nh.param<std::string>("agv_name", robot_name_, "r0");
  // Initialize robot_radius_ as a vector
  std::string robot_radius_str;
  nh.param<std::string>("robot_shape", robot_radius_str, "[0.3]");
  nh.param<std::string>("server_traffic_control_ip", server_ip,
                        "192.168.226.220");
  nh.param<int>("server_traffic_control_port", server_port, 8080);
  server_address = server_address_;
  std::stringstream ss(robot_radius_str.substr(
      1, robot_radius_str.size() - 2)); // remove the brackets

  double radius;
  while (ss >> radius) {
    robot_radius_.push_back(radius);
    if (ss.peek() == ',')
      ss.ignore();
  }
  // Now robot_radius_ should contain the values from the string
  ROS_INFO("robot_radius_: [%f, %f]", robot_radius_[0], robot_radius_[1]);

  // /*
  //   #####  #     # ######   #####  ######  ### ######  ####### ######
  //  #     # #     # #     # #     # #     #  #  #     # #       #     #
  //  #       #     # #     # #       #     #  #  #     # #       #     #
  //   #####  #     # ######  #       ######   #  ######  #####   ######
  //        # #     # #     # #       #   #    #  #     # #       #   #
  //  #     # #     # #     # #     # #    #   #  #     # #       #    #
  //   #####   #####  ######   #####  #     # ### ######  ####### #     #

  // */
  odom_sub_ = nh.subscribe("/odom", 10, &Client::odometryCallback, this);
  pose_sub_ = nh.subscribe("/robot_pose", 10, &Client::robotPoseCallback, this);
  moving_control_sub_ = nh.subscribe("/moving_control/module_status", 10,
                                     &Client::movingControlCallback, this);
  mission_status_sub_ = nh.subscribe("/mission_manager/module_status", 10,
                                     &Client::missionStatusCallback, this);
  mission_from_server_sub_ =
      nh.subscribe("/mission_from_server_latch", 10,
                   &Client::missionFromServerCallback, this);
  robot_status_sub_ =
      nh.subscribe("/robot_status", 10, &Client::robotStatusCallback, this);
  direction_move_sub_ = nh.subscribe("/robot_state_direction", 10,
                                     &Client::directionMoveCallback, this);

  hub_action_status_sub_ = nh.subscribe("/hub_server/module_status", 10,
                                        &Client::HubActionStatusCallback, this);
  matehan_action_status_sub_ =
      nh.subscribe("/matehan_server/module_status", 10,
                   &Client::MatehanActionStatusCallback, this);
  elevator_action_status_sub_ =
      nh.subscribe("/elevator_server/module_status", 10,
                   &Client::ElevatorActionStatusCallback, this);

  current_map_sub_ = nh.subscribe("/current_map_server", 10,
                                  &Client::currentMapCallback, this);
  current_path_sub_ =
      nh.subscribe("/current_path", 10, &Client::currentPathCallback, this);

  // /*
  //  ######  #     # ######  #       ###  #####  #     # ####### ######
  //  #     # #     # #     # #        #  #     # #     # #       #     #
  //  #     # #     # #     # #        #  #       #     # #       #     #
  //  ######  #     # ######  #        #   #####  ####### #####   ######
  //  #       #     # #     # #        #        # #     # #       #   #
  //  #       #     # #     # #        #  #     # #     # #       #    #
  //  #        #####  ######  ####### ###  #####  #     # ####### #     #

  // */
  current_goal_id_pub_ = nh.advertise<std_msgs::UInt32>("/goal_id", 10, true);
  status_planner_pub_ = nh.advertise<tuw_multi_robot_msgs::RouterStatus>(
      "/planner_status_receive", 10);
  route_pub_ =
      nh.advertise<std_stamped_msgs::StringStamped>("/route_receive", 10, true);
  pause_status_pub_ = nh.advertise<std_stamped_msgs::StringStamped>(
      "/pause_by_traffic_status", 10);

  pub_mission_run_pause_ = nh.advertise<std_stamped_msgs::StringStamped>(
      "/mission_manager/run_pause_req", 10);

  goal_pub_ =
      nh.advertise<geometry_msgs::PoseStamped>("goal_send_to_server", 10, true);

  // Start sendRobotInfo in a separate thread
  std::thread(&Client::sendRobotInfo, this).detach();
  std::thread(&Client::updatePauseStatus, this).detach();

  ROS_INFO("Client connecting to %s", server_address.c_str());
}

///*
//  #####     #    #       #       ######     #     #####  #    #
// #     #   # #   #       #       #     #   # #   #     # #   #
// #        #   #  #       #       #     #  #   #  #       #  #
// #       #     # #       #       ######  #     # #       ###
// #       ####### #       #       #     # ####### #       #  #
// #     # #     # #       #       #     # #     # #     # #   #
//  #####  #     # ####### ####### ######  #     #  #####  #    #

//*/

void Client::currentPathCallback(const nav_msgs::Path::ConstPtr &msg) {
  if (!msg->poses.empty()) {
    // Lấy điểm cuối
    const geometry_msgs::PoseStamped &last_pose = msg->poses.back();
    double x_last = last_pose.pose.position.x;
    double y_last = last_pose.pose.position.y;
    last_goal_in_path = std::make_pair(x_last, y_last);

    // Lấy điểm đầu
    const geometry_msgs::PoseStamped &first_pose = msg->poses.front();
    double x_first = first_pose.pose.position.x;
    double y_first = first_pose.pose.position.y;
    first_goal_in_path = std::make_pair(x_first, y_first);

    ROS_INFO("First waypoint: x = %f, y = %f", x_first, y_first);
    ROS_INFO("Last waypoint: x = %f, y = %f", x_last, y_last);
  } else {
    last_goal_in_path = std::make_pair(0.0, 0.0);
    first_goal_in_path = std::make_pair(0.0, 0.0);
    ROS_INFO_STREAM("Received path with no poses.");
  }
}

void Client::missionFromServerCallback(
    const std_stamped_msgs::StringStamped::ConstPtr &msg) {
  if (msg->data.empty()) {
    ROS_INFO_STREAM("Received empty mission data");
    return;
  }

  Json::Value root;
  Json::CharReaderBuilder reader;
  std::string errs;
  std::istringstream ss(msg->data);

  if (Json::parseFromStream(reader, ss, &root, &errs)) {
    if (!root.isMember("result")) {
      ROS_INFO_STREAM("Missing 'result' field in JSON");
      current_mission_data_.clear();
      return;
    }

    current_mission_data_ = root;
    receive_new_mission_data = true;
  } else {
    ROS_INFO_STREAM("Failed to parse mission from server JSON: " << errs);
    current_mission_data_.clear();
  }
}

void Client::movingControlCallback(
    const std_stamped_msgs::StringStamped::ConstPtr &msg) {
  Json::Value root;
  Json::CharReaderBuilder reader;
  std::string errs;
  std::istringstream ss(msg->data);

  if (Json::parseFromStream(reader, ss, &root, &errs)) {
    // std::lock_guard<std::mutex> lock(status_mutex_);
    moving_control_status_ = root["status"].asString();
    if (root.isMember("trigger_resend_goal") &&
        root["trigger_resend_goal"].isBool()) {
      is_request_trigger_resend_goal = root["trigger_resend_goal"].asBool();
      if (is_request_trigger_resend_goal) {
        trigger_resend_goal = true;
        need_to_wait_receive_path = true;
        ROS_INFO_STREAM("Trigger resend goal: true");
      } else {
        ROS_INFO_STREAM_THROTTLE(
            20.0, "Log throttle 20 giây, Trigger resend goal: false");
      }
    }

  } else {
    ROS_INFO_STREAM("Failed to parse moving control JSON");
  }
}

void Client::missionStatusCallback(
    const std_stamped_msgs::StringStamped::ConstPtr &msg) {
  Json::Value root;
  Json::CharReaderBuilder reader;
  std::string errs;
  std::istringstream ss(msg->data);

  if (Json::parseFromStream(reader, ss, &root, &errs)) {
    // std::lock_guard<std::mutex> lock(status_mutex_);
    mission_manager_status_ = root["status"].asString();
    mission_group_ = root["mission_group"].asString();
    current_action_type_ = root["current_action_type"].asString();
    current_mission_name_ = root["current_mission_name"].asString();
    current_action_num_ = root["current_action_num"].asInt();
    total_action_ = root["total_action"].asInt();
    if (mission_manager_status_ == "WAITING") {
      _state = "WAITING";
      send_goal_begin = true;
      receive_goal = false;
    } else {
      // _state = "WAITING"; xu ly khi khong phai waiitng thi lam gi tiep theo
      if (send_goal_begin && receive_new_mission_data) {
        receive_new_mission_data = false;
        _state = "SEND_GOAL";
        send_goal_begin = false;
      }
    }
  } else {
    ROS_INFO_STREAM("Failed to parse module_status JSON");
  }
}

void Client::HubActionStatusCallback(
    const std_stamped_msgs::StringStamped::ConstPtr &msg) {
  Json::Value root;
  Json::CharReaderBuilder reader;
  std::string errs;
  std::istringstream ss(msg->data);

  if (Json::parseFromStream(reader, ss, &root, &errs)) {
    // std::lock_guard<std::mutex> lock(status_mutex_);
    hub_action_state_go_out_ = root["state"].asString();
  } else {
    ROS_INFO_STREAM("Failed to parse hub action status JSON");
  }
}

void Client::MatehanActionStatusCallback(
    const std_stamped_msgs::StringStamped::ConstPtr &msg) {
  Json::Value root;
  Json::CharReaderBuilder reader;
  std::string errs;
  std::istringstream ss(msg->data);

  if (Json::parseFromStream(reader, ss, &root, &errs)) {
    // std::lock_guard<std::mutex> lock(status_mutex_);
    matehan_action_state_go_out_ = root["state"].asString();
  } else {
    ROS_INFO_STREAM("Failed to parse hub action status JSON");
  }
}

void Client::ElevatorActionStatusCallback(
    const std_stamped_msgs::StringStamped::ConstPtr &msg) {
  Json::Value root;
  Json::CharReaderBuilder reader;
  std::string errs;
  std::istringstream ss(msg->data);

  if (Json::parseFromStream(reader, ss, &root, &errs)) {
    // std::lock_guard<std::mutex> lock(status_mutex_);
    elevator_action_state_go_out_ = root["state"].asString();
  } else {
    ROS_INFO_STREAM("Failed to parse hub action status JSON");
  }
}

void Client::currentMapCallback(
    const std_stamped_msgs::StringStamped::ConstPtr &msg) {
  std::string data = msg->data;
  int new_server_port = 0;

  if (data == "Tendo_1F") {
    new_server_port = 2201;
  } else if (data == "Tendo_2F") {
    new_server_port = 2202;
  } else if (data == "Tendo_3F") {
    new_server_port = 2203;
  } else {
    new_server_port = 2204; // not use
    ROS_INFO_STREAM("Received unknown map data: " << data);
    return; // không thay đổi nếu không nhận diện được map
  }

  if (server_port != new_server_port) {
    std::lock_guard<std::mutex> lock_send(cli_mutex_send_goal_);
    std::lock_guard<std::mutex> lock_update(cli_mutex_update_robot_info_);
    std::lock_guard<std::mutex> lock_last(cli_mutex_get_last_path_);

    server_port = new_server_port;
    server_address = "http://" + server_ip + ":" + std::to_string(server_port);

    ROS_INFO("Server port updated to: %d based on map: %s", server_port,
             data.c_str());
    ROS_INFO("Updated server address: %s", server_address.c_str());

    // Reset old clients (giải phóng kết nối cũ)
    cli_send_goal_.reset();
    cli_get_last_path_.reset();
    cli_update_robot_info_.reset();

    // Tạo client mới
    cli_send_goal_ = std::make_shared<httplib::Client>(server_address.c_str());
    cli_get_last_path_ =
        std::make_shared<httplib::Client>(server_address.c_str());
    cli_update_robot_info_ =
        std::make_shared<httplib::Client>(server_address.c_str());

    configureClient(cli_send_goal_);
    configureClient(cli_get_last_path_);
    configureClient(cli_update_robot_info_);
  }
}

void Client::directionMoveCallback(
    const std_stamped_msgs::StringStamped::ConstPtr &msg) {
  Json::Value root;
  Json::CharReaderBuilder reader;
  std::string errs;
  std::istringstream ss(msg->data);

  if (Json::parseFromStream(reader, ss, &root, &errs)) {
    // std::lock_guard<std::mutex> lock(status_mutex_);
    direction_move_ = root["direction"].asString();
    state_move_ = root["state"].asString();
  } else {
    ROS_INFO_STREAM("Failed to parse direction move JSON");
  }
}

void Client::robotStatusCallback(
    const std_stamped_msgs::StringStamped::ConstPtr &msg) {
  Json::Value root;
  Json::CharReaderBuilder reader;
  std::string errs;
  std::istringstream ss(msg->data);

  if (Json::parseFromStream(reader, ss, &root, &errs)) {
    // std::lock_guard<std::mutex> lock(status_mutex_);
    mode_ = root["mode"].asString();
    status_ = root["status"].asString();
    detail_ = root["detail"].asString();
    if (status_ == "WAITING" || !root_auto_mode) {
      last_goal_in_path = std::make_pair(0.0, 0.0); // Clear to default
      first_goal_in_path = std::make_pair(0.0, 0.0);
      // ROS_INFO("last_goal_in_path cleared due to WAITING or MANUAL mode");
    }
  } else {
    ROS_INFO_STREAM("Failed to parse robot_status JSON");
  }
}

void Client::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  // std::lock_guard<std::mutex> lock(data_mutex_);  // Tránh điều kiện race
  velocity_[0] = msg->twist.twist.linear.x;  // Vận tốc thẳng
  velocity_[1] = msg->twist.twist.angular.z; // Vận tốc góc
}

void Client::robotPoseCallback(const geometry_msgs::Pose::ConstPtr &msg) {
  // std::lock_guard<std::mutex> lock(data_mutex_);
  position_x_ = msg->position.x;
  position_y_ = msg->position.y;

  // Convert quaternion to yaw (orientation angle)
  tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z,
                    msg->orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  orientation_ = yaw; // Store orientation in radians
}

// /*
//  ####### #     # #     #  #####  ####### ### ####### #     #
//  #       #     # ##    # #     #    #     #  #     # ##    #
//  #       #     # # #   # #          #     #  #     # # #   #
//  #####   #     # #  #  # #          #     #  #     # #  #  #
//  #       #     # #   # # #          #     #  #     # #   # #
//  #       #     # #    ## #     #    #     #  #     # #    ##
//  #        #####  #     #  #####     #    ### ####### #     #

// */

void Client::pauseRobot() {
  pause_by_wait_path = true;
  std_stamped_msgs::StringStamped msg;
  msg.stamp = ros::Time::now();
  msg.data = "PAUSE";
  pub_mission_run_pause_.publish(msg);
}

void Client::resumeRunning() {
  pause_by_wait_path = false;
  std_stamped_msgs::StringStamped msg;
  msg.data = "RUN";
  msg.stamp = ros::Time::now();
  pub_mission_run_pause_.publish(msg);
}

bool Client::sendGoal(double x, double y, double quat_x, double quat_y,
                      double quat_z, double quat_w) {
  // Kiểm tra client hợp lệ
  if (!cli_send_goal_) {
    ROS_INFO_THROTTLE(10, "HTTP cli_send_goal_ is not initialized!");

    return false;
  }

  std_msgs::UInt32 msg;
  msg.data = goal_id_;
  current_goal_id_pub_.publish(msg);
  ROS_INFO_STREAM("Published goal_id: " << goal_id_);

  Json::Value root;
  root["id"] = goal_id_;
  root["header"]["seq"] = 0;
  root["header"]["stamp"] = ros::Time::now().toSec();
  root["header"]["frame_id"] = "base_link";

  root["pose"]["position"]["x"] = x;
  root["pose"]["position"]["y"] = y;
  root["pose"]["position"]["z"] = 0.0;

  root["pose"]["orientation"]["x"] = quat_x;
  root["pose"]["orientation"]["y"] = quat_y;
  root["pose"]["orientation"]["z"] = quat_z;
  root["pose"]["orientation"]["w"] = quat_w;

  root["robot_name"] = robot_name_;

  Json::StreamWriterBuilder writer;
  std::string data = Json::writeString(writer, root);

  std::lock_guard<std::mutex> lock(cli_mutex_send_goal_);

  // Gửi POST request
  auto response = cli_send_goal_->Post(
      "/traffic_control/update_target_position", data, "application/json");

  if (response && response->status == 200) {
    last_goal_id_send = goal_id_;
    goal_id_++;
    ROS_INFO("Goal sent successfully");

    goal_pub_.publish(createPoseStamped(x, y, quat_x, quat_y, quat_z, quat_w));
    need_to_wait_receive_path = true;
    return true;
  } else {
    if (response) {
      ROS_INFO_STREAM("Failed to send goal, status: "
                      << response->status << ", body: " << response->body);
    } else {
      ROS_INFO(
          "No response from server (possibly timeout or connection failure)");
    }
    return false;
  }
}

bool Client::getLastPath() {
  // Kiểm tra client có được khởi tạo không
  if (!cli_get_last_path_) {
    ROS_INFO_THROTTLE(10, "HTTP cli_get_last_path_ is not initialized!");
    return false;
  }

  // Tạo JSON request
  Json::Value root;
  root["robot_name"] = robot_name_;

  Json::StreamWriterBuilder writer;
  std::string data = Json::writeString(writer, root);

  // Gửi yêu cầu POST
  std::lock_guard<std::mutex> lock(cli_mutex_get_last_path_);
  auto response = cli_get_last_path_->Post("/traffic_control/get_last_path",
                                           data, "application/json");

  // Kiểm tra phản hồi
  if (response && response->status == 200) {
    ROS_INFO("Get last path successfully");
    return true;
  } else {
    if (response) {
      ROS_INFO_STREAM("Failed to get last path. Status code: "
                      << response->status << ", body: " << response->body);
    } else {
      ROS_INFO("No response from server when calling getLastPath (timeout or "
               "connection issue)");
    }
    return false;
  }
}

bool Client::isSpecialReturnChargeMission() {
  if (!current_mission_data_.isMember("result") ||
      !current_mission_data_["result"].isMember("actions")) {
    return false;
  }

  const Json::Value &actions = current_mission_data_["result"]["actions"];
  if (!actions.isArray() || actions.size() != 2) {
    return false;
  }

  const Json::Value &action0 = actions[0];
  const Json::Value &action1 = actions[1];

  if (action0["type"].asString() != "move" ||
      action1["type"].asString() != "docking_charger") {
    return false;
  }

  // Check if action0 has at least one waypoint with valid position
  if (!action0.isMember("waypoints") || !action0["waypoints"].isArray() ||
      action0["waypoints"].empty()) {
    return false;
  }

  const Json::Value &waypoint = action0["waypoints"][0];
  if (!waypoint.isMember("position") ||
      !waypoint["position"].isMember("position")) {
    return false;
  }

  const Json::Value &pos0 = waypoint["position"]["position"];

  // Check if action1 has a valid target position
  if (!action1.isMember("params") || !action1["params"].isMember("position") ||
      !action1["params"]["position"].isMember("position")) {
    return false;
  }

  const Json::Value &pos1 = action1["params"]["position"]["position"];

  double x0 = pos0["x"].asDouble();
  double y0 = pos0["y"].asDouble();
  double x1 = pos1["x"].asDouble();
  double y1 = pos1["y"].asDouble();

  // Compare positions with a small tolerance
  const double EPSILON = 0.01;
  return (std::abs(x0 - x1) < EPSILON && std::abs(y0 - y1) < EPSILON);
}

bool Client::getTargetGoal(double &x, double &y) {
  if (!current_mission_data_.isNull() &&
      current_mission_data_.isMember("result") &&
      current_mission_data_["result"].isMember("actions") &&
      current_mission_data_["result"]["actions"].isArray()) {
    int length = current_mission_data_["result"]["actions"].size();
    ROS_INFO("actions array exists with %d elements", length);
    std::string current_mission_name =
        current_mission_data_["result"]["name"].asString();
    ROS_INFO("current_mission_name: %s", current_mission_name.c_str());
    if (current_mission_name != current_mission_name_) {
      return false;
    }
    // cacs mission special
    static const std::set<std::string> special_names = {
        "hub", "matehand", "docking_charger", "elevator"};
    int last_index = -1;
    int selected_index = -1;
    std::string action_name;
    bool is_special_mission_return_charge = isSpecialReturnChargeMission();

    // Truong hop AGV nhan mission ve sac khi dang dung o sac
    if (is_special_mission_return_charge) {
      for (int i = 0; i < current_mission_data_["result"]["actions"].size();
           ++i) {
        action_name =
            current_mission_data_["result"]["actions"][i]["type"].asString();
        // tim action specail tiep theo
        if (i >= current_action_num_ - 1) {
          index_action_found_ = i + 1;
          if (action_name == "move" &&
              current_mission_data_["result"]["actions"][i].isMember(
                  "waypoints") &&
              current_mission_data_["result"]["actions"][i]["waypoints"]
                  .isArray() &&
              !current_mission_data_["result"]["actions"][i]["waypoints"]
                   .empty()) {
            // Lấy phần tử cuối cùng trong danh sách waypoints
            auto &last_waypoint =
                current_mission_data_["result"]["actions"][i]["waypoints"];
            if (!last_waypoint.isArray() || last_waypoint.empty()) {
              // Xử lý trường hợp mảng rỗng hoặc không hợp lệ
              return false;
            }
            Json::ArrayIndex last_wp_index =
                static_cast<Json::ArrayIndex>(last_waypoint.size() - 1);

            x = last_waypoint[last_wp_index]["position"]["position"]["x"]
                    .asDouble();
            y = last_waypoint[last_wp_index]["position"]["position"]["y"]
                    .asDouble();
            return true;
          } else if (action_name == "docking_charger") {
            if (current_mission_data_["result"]["actions"][i]["params"]
                                     ["position"]
                                         .isMember("position")) {
              x = current_mission_data_["result"]["actions"][i]["params"]
                                       ["position"]["position"]["x"]
                                           .asDouble();
              y = current_mission_data_["result"]["actions"][i]["params"]
                                       ["position"]["position"]["y"]
                                           .asDouble();
              return true;
            } else {
              x = current_mission_data_["result"]["actions"][i]["params"]
                                       ["position"]["x"]
                                           .asDouble();
              y = current_mission_data_["result"]["actions"][i]["params"]
                                       ["position"]["y"]
                                           .asDouble();
              return true;
            }
          }
        }
      }

    }
    // truong hop thong thuong khac
    else {
      for (int i = 0; i < current_mission_data_["result"]["actions"].size();
           ++i) {
        action_name =
            current_mission_data_["result"]["actions"][i]["type"].asString();
        // tim action specail tiep theo
        if (current_action_num_ == 1 && i == 0 &&
            current_mission_data_["result"]["actions"].size() > 1) {
          const Json::Value &action0 =
              current_mission_data_["result"]["actions"][0];
          const Json::Value &action1 =
              current_mission_data_["result"]["actions"][1];
          if (action0["type"].asString() == "move" &&
              action1["type"].asString() == "hub") {

            Json::Value pos0; // Khai báo biến ngoài if
            Json::Value pos1;
            bool has_pos0 = false;
            bool has_pos1 = false;

            if (action0.isMember("waypoints") &&
                action0["waypoints"].isArray() &&
                !action0["waypoints"].empty()) {
              // Lấy phần tử đầu tiên trong danh sách waypoints (theo code gốc)
              pos0 = action0["waypoints"][0]["position"]["position"];
              has_pos0 = true;
            }

            if (action1.isMember("params") &&
                action1["params"].isMember("position")) {
              pos1 = action1["params"]["position"];
              has_pos1 = true;
            }
            if (has_pos0 && has_pos1) {

              double x0 = pos0["x"].asDouble();
              double y0 = pos0["y"].asDouble();
              double x1 = pos1["x"].asDouble();
              double y1 = pos1["y"].asDouble();

              // So sánh vị trí với EPSILON ...
              const double EPSILON = 0.01;
              if (std::abs(x0 - x1) < EPSILON && std::abs(y0 - y1) < EPSILON) {
                index_action_found_ = i + 1;
                auto &last_waypoint = action0["waypoints"];
                Json::ArrayIndex last_wp_index =
                    static_cast<Json::ArrayIndex>(last_waypoint.size() - 1);

                x = last_waypoint[last_wp_index]["position"]["position"]["x"]
                        .asDouble();
                y = last_waypoint[last_wp_index]["position"]["position"]["y"]
                        .asDouble();
                is_robot_in_hub_begin_ = true;
                return true;
                // TODO: xử lý cái index_action_found_ xung đột vơi dk bên dưới,
                // vì nếu vậy thì bên dưới sẽ check i >= 2 (action đã chueyenr
                // sang 2)
              }
            }
          }
        }
        int index_offset = 0;
        if (is_robot_in_hub_begin_) {
          index_offset = 1;
        }
        if (i >= current_action_num_ - index_offset &&
            special_names.count(action_name)) {
          is_robot_in_hub_begin_ = false;
          index_action_found_ = i + 1;
          if (action_name != "docking_charger") {
            x = current_mission_data_["result"]["actions"][i]["params"]
                                     ["position"]["x"]
                                         .asDouble();
            y = current_mission_data_["result"]["actions"][i]["params"]
                                     ["position"]["y"]
                                         .asDouble();
            return true;
          } else {
            if (current_mission_data_["result"]["actions"][i]["params"]
                                     ["position"]
                                         .isMember("position")) {
              x = current_mission_data_["result"]["actions"][i]["params"]
                                       ["position"]["position"]["x"]
                                           .asDouble();
              y = current_mission_data_["result"]["actions"][i]["params"]
                                       ["position"]["position"]["y"]
                                           .asDouble();
              return true;
            } else {
              x = current_mission_data_["result"]["actions"][i]["params"]
                                       ["position"]["x"]
                                           .asDouble();
              y = current_mission_data_["result"]["actions"][i]["params"]
                                       ["position"]["y"]
                                           .asDouble();
              return true;
            }
          }
        }
        last_index = i;
      }

      // neu khong tim duoc mission special thi lay thong tin action cuoi cung
      if (last_index >= current_action_num_ - 1) {
        is_robot_in_hub_begin_ = false;
        index_action_found_ = last_index + 1;
        // neu  action cuoi cung la action move
        if (action_name == "move" &&
            current_mission_data_["result"]["actions"][last_index].isMember(
                "waypoints") &&
            current_mission_data_["result"]["actions"][last_index]["waypoints"]
                .isArray() &&
            !current_mission_data_["result"]["actions"][last_index]["waypoints"]
                 .empty()) {
          // Lấy phần tử cuối cùng trong danh sách waypoints
          auto &last_waypoint = current_mission_data_["result"]["actions"]
                                                     [last_index]["waypoints"];
          if (!last_waypoint.isArray() || last_waypoint.empty()) {
            // Xử lý trường hợp mảng rỗng hoặc không hợp lệ
            return false;
          }
          Json::ArrayIndex last_wp_index =
              static_cast<Json::ArrayIndex>(last_waypoint.size() - 1);

          x = last_waypoint[last_wp_index]["position"]["position"]["x"]
                  .asDouble();
          y = last_waypoint[last_wp_index]["position"]["position"]["y"]
                  .asDouble();
          return true;
        } else {
          // kiem tra action cuoi cung co phai la action dac biet khong, neu co
          // thi goal la waiting pose
          if (special_names.count(action_name)) {
            x = current_mission_data_["result"]["actions"][last_index]["params"]
                                     ["waiting_position"]["x"]
                                         .asDouble();
            y = current_mission_data_["result"]["actions"][last_index]["params"]
                                     ["waiting_position"]["y"]
                                         .asDouble();
            ROS_INFO_STREAM(
                "found special goal x,y in action last name: " << action_name);
            return true;
          }
          ROS_INFO_STREAM("not found special goal x,y in action last name: "
                          << action_name);
        }
      }
    }
  }
  return false;
}

// /*
//  ####### #     # #     #  #####  ####### ### ####### #     #
//  #       #     # ##    # #     #    #     #  #     # ##    #
//  #       #     # # #   # #          #     #  #     # # #   #
//  #####   #     # #  #  # #          #     #  #     # #  #  #
//  #       #     # #   # # #          #     #  #     # #   # #
//  #       #     # #    ## #     #    #     #  #     # #    ##
//  #        #####  #     #  #####     #    ### ####### #     #

// */
// Tính góc giữa vector p0→p1 và hướng robot (orientation_)
double Client::calculateAngle(const std::pair<double, double> &p0,
                              const std::pair<double, double> &p1,
                              double orientation_) {
  // Vector từ p0 tới p1
  double dx = p1.first - p0.first;
  double dy = p1.second - p0.second;

  // Tính hướng của vector (theo radian)
  double segment_angle = std::atan2(dy, dx);

  // Tính hiệu giữa 2 góc
  double angle_diff = segment_angle - orientation_;

  // Đưa về khoảng [-π, π]
  while (angle_diff > M_PI)
    angle_diff -= 2 * M_PI;
  while (angle_diff < -M_PI)
    angle_diff += 2 * M_PI;

  // Đổi sang độ và lấy trị tuyệt đối (vì không quan tâm trái/phải)
  double angle_deg = std::abs(angle_diff * 180.0 / M_PI);
  return angle_deg;
}

/**
 * @brief Tính toán thông tin liên quan đến hình chiếu vuông góc của một điểm
 * lên một đoạn thẳng.
 *
 * Hàm này thực hiện:
 *  - Tìm tọa độ điểm hình chiếu vuông góc từ điểm pt xuống đoạn thẳng p1-p2.
 *  - Tính khoảng cách từ p1 đến điểm chiếu và từ điểm chiếu đến p2.
 *  - Tính khoảng cách vuông góc từ pt đến đoạn thẳng (khoảng cách ngắn nhất).
 *
 * @param p1     Điểm đầu đoạn thẳng (x1, y1)
 * @param p2     Điểm cuối đoạn thẳng (x2, y2)
 * @param pt     Điểm cần chiếu xuống đoạn thẳng (x, y)
 */
void Client::perpendicularProjectionInfo(const std::pair<double, double> &p1,
                                         const std::pair<double, double> &p2,
                                         const std::pair<double, double> &pt,
                                         double &dist_p1_to_proj,
                                         double &dist_proj_to_p2) {
  double dx = p2.first - p1.first;
  double dy = p2.second - p1.second;
  double vx = dx;
  double vy = dy;
  double wx = pt.first - p1.first;
  double wy = pt.second - p1.second;

  double dot = vx * wx + vy * wy;
  double len_sq = vx * vx + vy * vy;
  double t = dot / len_sq;

  // Tọa độ điểm vuông góc
  double proj_x = p1.first + t * vx;
  double proj_y = p1.second + t * vy;

  // Truyền kết quả ra ngoài thông qua tham chiếu
  dist_p1_to_proj = std::hypot(proj_x - p1.first, proj_y - p1.second);
  dist_proj_to_p2 = std::hypot(proj_x - p2.first, proj_y - p2.second);

  // std::cout << "Chiều dọc từ pt đến đoạn: "
  //           << std::abs(vx * wy - vy * wx) / std::sqrt(len_sq) << "\n";
  // std::cout << "T = " << t << "\n";
  // std::cout << "pt_vuong_goc: (" << proj_x << ", " << proj_y << ")\n";
  // std::cout << "Distance from p1 to projection: " << dist_p1_to_proj << "\n";
  // std::cout << "Distance from projection to p2: " << dist_proj_to_p2 << "\n";
}

// Hàm tính khoảng cách vuông góc từ một điểm đến một đoạn thẳng
double Client::perpendicularDistance(const std::pair<double, double> &p1,
                                     const std::pair<double, double> &p2,
                                     const std::pair<double, double> &pt) {
  double dx = p2.first - p1.first;
  double dy = p2.second - p1.second;
  double numerator = std::abs(dy * pt.first - dx * pt.second +
                              p2.first * p1.second - p2.second * p1.first);
  double denominator = std::hypot(dx, dy);
  return numerator / denominator;
}

// Hàm tính khoảng cách từ điểm đến đoạn theo đúng 3 trường hợp
double Client::fullSegmentDistance(const std::pair<double, double> &p1,
                                   const std::pair<double, double> &p2,
                                   const std::pair<double, double> &pt) {
  double dx = p2.first - p1.first;
  double dy = p2.second - p1.second;
  double length_sq = dx * dx + dy * dy;
  if (length_sq == 0.0)
    return std::hypot(pt.first - p1.first, pt.second - p1.second);

  double t =
      ((pt.first - p1.first) * dx + (pt.second - p1.second) * dy) / length_sq;
  t = std::max(0.0, std::min(1.0, t));
  double proj_x = p1.first + t * dx;
  double proj_y = p1.second + t * dy;
  return std::hypot(pt.first - proj_x, pt.second - proj_y);
}

// Kiểm tra xem điểm có nằm sau p2 trên đoạn p1->p2 không
bool Client::isBehind(const std::pair<double, double> &p1,
                      const std::pair<double, double> &p2,
                      const std::pair<double, double> &pt) {
  double dx = p2.first - p1.first;
  double dy = p2.second - p1.second;
  double px = pt.first - p2.first;
  double py = pt.second - p2.second;
  return (dx * px + dy * py) > 0;
}

// Hàm tính vector đơn vị
std::pair<double, double>
Client::unitVector(const std::pair<double, double> &from,
                   const std::pair<double, double> &to) {
  double dx = to.first - from.first;
  double dy = to.second - from.second;
  double len = std::hypot(dx, dy);
  return {dx / len, dy / len};
}

// Hàm tạo điểm cách điểm gốc một khoảng theo hướng vector
std::pair<double, double>
Client::movePoint(const std::pair<double, double> &origin, double distance,
                  const std::pair<double, double> &dir) {
  return {origin.first + dir.first * distance,
          origin.second + dir.second * distance};
}

// Hàm tính khoảng cách Euclidean giữa hai điểm
double Client::distance(const std::pair<double, double> &p1,
                        const std::pair<double, double> &p2) {
  double dx = p2.first - p1.first;
  double dy = p2.second - p1.second;
  return std::hypot(dx, dy);
}

// Hàm xử lý trimmed_waypoints, sử dụng để create exact pose start when receive
// change path use for AGV300
void Client::buildTrimmedWaypoints(const Json::Value &waypoints,
                                   Json::Value &trimmed_waypoints) {
  std::pair<double, double> current_pos = {position_x_, position_y_};

  if (waypoints.size() < 2) {
    // Không đủ waypoint để xử lý
    // Path not valid
    ROS_INFO_STREAM("Robot out of path TH1");
    trimmed_waypoints.clear();
    return;
  }
  if (waypoints.size() < 3) {
    // Không đủ waypoint để xử lý
    std::pair<double, double> wp0 = {
        waypoints[0]["position"]["position"]["x"].asDouble(),
        waypoints[0]["position"]["position"]["y"].asDouble()};
    std::pair<double, double> wp1 = {
        waypoints[1]["position"]["position"]["x"].asDouble(),
        waypoints[1]["position"]["position"]["y"].asDouble()};
    // TODO: fix bug return Nan
    double dist_perpendicular_robot_to_wp01 =
        perpendicularDistance(wp0, wp1, current_pos);
    ROS_INFO_STREAM("wp0: (" << wp0.first << ", " << wp0.second << ")");
    ROS_INFO_STREAM("wp1: (" << wp1.first << ", " << wp1.second << ")");
    ROS_INFO_STREAM("current_pos: (" << current_pos.first << ", "
                                     << current_pos.second << ")");

    ROS_INFO_STREAM("dist_perpendicular_robot_to_wp01: "
                    << dist_perpendicular_robot_to_wp01);
    if (dist_perpendicular_robot_to_wp01 > 0.05) {
      // Path not valid, robot out of path > 5 cm
      ROS_INFO_STREAM("Robot out of path TH2");
      trimmed_waypoints.clear();
      return;
    } else {
      trimmed_waypoints = waypoints;
      return;
    }
  }
  std::pair<double, double> wp0 = {
      waypoints[0]["position"]["position"]["x"].asDouble(),
      waypoints[0]["position"]["position"]["y"].asDouble()};
  std::pair<double, double> wp1 = {
      waypoints[1]["position"]["position"]["x"].asDouble(),
      waypoints[1]["position"]["position"]["y"].asDouble()};
  std::pair<double, double> wp2 = {
      waypoints[2]["position"]["position"]["x"].asDouble(),
      waypoints[2]["position"]["position"]["y"].asDouble()};
  double dist_robot_to_wp0 = distance(wp0, current_pos);
  double dist_robot_to_wp1 = distance(wp1, current_pos);
  double angle_robot_to_wp01 = calculateAngle(wp0, wp1, orientation_);
  double angle_robot_to_wp12 = calculateAngle(wp1, wp2, orientation_);
  double angle_wp01_to_wp12 =
      calculateAngleBetweenSegments(waypoints[0], waypoints[1], waypoints[2]);
  double dist_perpendicular_robot_to_wp12 =
      perpendicularDistance(wp1, wp2, current_pos);
  double dist_perpendicular_robot_to_wp01 =
      perpendicularDistance(wp0, wp1, current_pos);
  double dist_project_to_wp0_in_wp01, dist_project_to_wp1_in_wp01;
  perpendicularProjectionInfo(wp0, wp1, current_pos,
                              dist_project_to_wp0_in_wp01,
                              dist_project_to_wp1_in_wp01);
  double dist_project_to_wp1_in_wp12, dist_project_to_wp2_in_wp12;
  perpendicularProjectionInfo(wp1, wp2, current_pos,
                              dist_project_to_wp1_in_wp12,
                              dist_project_to_wp2_in_wp12);

  ROS_INFO_STREAM("wp0: (" << wp0.first << ", " << wp0.second << ")");
  ROS_INFO_STREAM("wp1: (" << wp1.first << ", " << wp1.second << ")");
  ROS_INFO_STREAM("wp2: (" << wp2.first << ", " << wp2.second << ")");
  ROS_INFO_STREAM("current_pos: (" << current_pos.first << ", "
                                   << current_pos.second << ")");

  ROS_INFO_STREAM("dist_robot_to_wp0: " << dist_robot_to_wp0);
  ROS_INFO_STREAM("dist_robot_to_wp1: " << dist_robot_to_wp1);
  ROS_INFO_STREAM("angle_robot_to_wp01: " << angle_robot_to_wp01);
  ROS_INFO_STREAM("angle_robot_to_wp12: " << angle_robot_to_wp12);
  ROS_INFO_STREAM("angle_wp01_to_wp12: " << angle_wp01_to_wp12);
  ROS_INFO_STREAM(
      "dist_perpendicular_robot_to_wp12: " << dist_perpendicular_robot_to_wp12);
  ROS_INFO_STREAM(
      "dist_perpendicular_robot_to_wp01: " << dist_perpendicular_robot_to_wp01);
  ROS_INFO_STREAM(
      "dist_project_to_wp0_in_wp01: " << dist_project_to_wp0_in_wp01);
  ROS_INFO_STREAM(
      "dist_project_to_wp1_in_wp01: " << dist_project_to_wp1_in_wp01);
  ROS_INFO_STREAM(
      "dist_project_to_wp1_in_wp12: " << dist_project_to_wp1_in_wp12);
  ROS_INFO_STREAM(
      "dist_project_to_wp2_in_wp12: " << dist_project_to_wp2_in_wp12);

  if (last_goal_in_path.first == 0.0 && last_goal_in_path.second == 0.0) {
    // AGV bat dau chay hoac o che do manual trước đó
    if (dist_perpendicular_robot_to_wp01 > 0.05) {
      // Path not valid, robot out of path > 5 cm
      ROS_INFO_STREAM("Robot out of path TH3");
      trimmed_waypoints.clear();
      return;
    } else {
      trimmed_waypoints = waypoints;
      return;
    }
  }

  // Trường hợp robot gần waypoint đầu tiên (wp0) hơn
  if (dist_robot_to_wp0 < dist_robot_to_wp1) {
    // Robot vẫn nằm trên đoạn wp0-wp1
    if (dist_perpendicular_robot_to_wp01 < 0.05) {
      // Không cần chỉnh sửa path, giữ nguyên
      trimmed_waypoints = waypoints;
      return;
    } else {
      // Nếu điểm chiếu vuông góc từ robot tới đoạn wp0-wp1 gần wp0
      if (dist_project_to_wp0_in_wp01 < 0.05) {
        // Chèn thêm vị trí hiện tại của robot thành một waypoint ảo vào đầu
        // path
        Json::Value virtual_json = waypoints[0];
        virtual_json["position"]["position"]["x"] = position_x_;
        virtual_json["position"]["position"]["y"] = position_y_;
        trimmed_waypoints.append(virtual_json);

        // Giữ nguyên các waypoint ban đầu
        for (int i = 0; i < waypoints.size(); ++i) {
          trimmed_waypoints.append(waypoints[i]);
        }
        return;
      } else {
        // Robot lệch khỏi path quá 5cm => path không hợp lệ
        ROS_INFO_STREAM("Robot out of path TH4");
        trimmed_waypoints.clear();
        return;
      }
    }
  }

  // Trường hợp robot gần waypoint thứ 2 (wp1) hơn
  else {
    // Nếu robot vẫn nằm gần đoạn wp0-wp1
    if (dist_perpendicular_robot_to_wp01 < 0.05) {
      // Kiểm tra xem wp1 có trùng với last_goal_in_path không
      if (last_goal_in_path.first != 0.0 || last_goal_in_path.second != 0.0) {
        bool isDifferent =
            std::abs(wp1.first - last_goal_in_path.first) > 0.01 ||
            std::abs(wp1.second - last_goal_in_path.second) > 0.01;
        if (!isDifferent) {
          // Nếu giống thì giữ nguyên path
          trimmed_waypoints = waypoints;
          return;
        }
      }

      // Kiểm tra xem wp1 có trùng với first_goal_in_path không
      if (first_goal_in_path.first != 0.0 || first_goal_in_path.second != 0.0) {
        bool isDifferent =
            std::abs(wp1.first - first_goal_in_path.first) > 0.01 ||
            std::abs(wp1.second - first_goal_in_path.second) > 0.01;
        if (!isDifferent) {
          // Nếu giống thì bỏ qua waypoint đầu tiên (wp0)
          for (int i = 1; i < waypoints.size(); ++i) {
            trimmed_waypoints.append(waypoints[i]);
          }
          return;
        }
      }

      // Nếu path rẽ nhiều (góc giữa wp0-wp1 và wp1-wp2 > 45° và < 135°)
      if (angle_wp01_to_wp12 > 45 && angle_wp01_to_wp12 < 135) {
        // Kiểm tra xem robot có gần đoạn wp1-wp2 không
        if (dist_perpendicular_robot_to_wp12 < 0.05) {
          // Nếu robot đang nhìn theo đúng hướng path
          if (angle_robot_to_wp01 > 5 && angle_robot_to_wp01 < 175) {
            // Cắt bỏ waypoint đầu tiên (wp0), giữ lại phần còn lại
            for (int i = 1; i < waypoints.size(); ++i) {
              trimmed_waypoints.append(waypoints[i]);
            }
            return;
          } else {
            // Robot chưa gần đoạn wp1-wp2, thêm một waypoint ảo là vị trí hiện
            // tại
            Json::Value virtual_json = waypoints[1];
            virtual_json["position"]["position"]["x"] = position_x_;
            virtual_json["position"]["position"]["y"] = position_y_;
            trimmed_waypoints.append(virtual_json);

            // Giữ lại toàn bộ path
            for (int i = 0; i < waypoints.size(); ++i) {
              trimmed_waypoints.append(waypoints[i]);
            }
            return;
          }
        } else {
          // Robot chưa gần đoạn wp1-wp2, thêm một waypoint ảo là vị trí hiện
          // tại
          Json::Value virtual_json = waypoints[1];
          virtual_json["position"]["position"]["x"] = position_x_;
          virtual_json["position"]["position"]["y"] = position_y_;
          trimmed_waypoints.append(virtual_json);

          // Giữ lại toàn bộ path
          for (int i = 0; i < waypoints.size(); ++i) {
            trimmed_waypoints.append(waypoints[i]);
          }
          return;
        }
      } else {
        // Góc rẽ không đáng kể, giữ nguyên path
        trimmed_waypoints = waypoints;
        return;
      }
    }

    // Nếu robot không còn nằm gần đoạn wp0-wp1, nhưng gần đoạn wp1-wp2
    else if (dist_perpendicular_robot_to_wp12 < 0.05) {
      // Bỏ qua waypoint đầu tiên, giữ lại phần từ wp1 trở đi
      for (int i = 1; i < waypoints.size(); ++i) {
        trimmed_waypoints.append(waypoints[i]);
      }
      return;
    }

    // Robot lệch khỏi path nhiều, kiểm tra hướng di chuyển
    else {
      // Nếu góc giữa wp0-wp1 và wp1-wp2 là đường thẳng (góc gần 0 hoặc 180)
      if (angle_wp01_to_wp12 < 5 || angle_wp01_to_wp12 > 175) {
        // Kiểm tra điểm chiếu vuông góc gần với wp1
        if (dist_project_to_wp1_in_wp01 < 0.05) {
          // Thêm waypoint ảo tại vị trí robot
          Json::Value virtual_json = waypoints[1];
          virtual_json["position"]["position"]["x"] = position_x_;
          virtual_json["position"]["position"]["y"] = position_y_;
          trimmed_waypoints.append(virtual_json);

          // Giữ lại phần từ wp1 trở đi
          for (int i = 1; i < waypoints.size(); ++i) {
            trimmed_waypoints.append(waypoints[i]);
          }
          return;
        } else {
          // Robot lệch quá xa khỏi path, bỏ path
          ROS_INFO_STREAM("Robot out of path TH5");
          trimmed_waypoints.clear();
          return;
        }
      } else {
        // Góc không đủ điều kiện, path không hợp lệ
        ROS_INFO_STREAM("Robot out of path TH6");
        trimmed_waypoints.clear();
        return;
      }
    }
  }
  trimmed_waypoints = waypoints;
  return;
}

// Tính góc giữa 2 đoạn thẳng wp0→wp1 và wp1→wp2
double Client::calculateAngleBetweenSegments(const Json::Value &wp0,
                                             const Json::Value &wp1,
                                             const Json::Value &wp2) {
  // Tạo vector từ wp0 → wp1
  double v1_x = wp1["position"]["position"]["x"].asDouble() -
                wp0["position"]["position"]["x"].asDouble();
  double v1_y = wp1["position"]["position"]["y"].asDouble() -
                wp0["position"]["position"]["y"].asDouble();

  // Tạo vector từ wp1 → wp2
  double v2_x = wp2["position"]["position"]["x"].asDouble() -
                wp1["position"]["position"]["x"].asDouble();
  double v2_y = wp2["position"]["position"]["y"].asDouble() -
                wp1["position"]["position"]["y"].asDouble();

  double dot = v1_x * v2_x + v1_y * v2_y;
  double len1 = std::sqrt(v1_x * v1_x + v1_y * v1_y);
  double len2 = std::sqrt(v2_x * v2_x + v2_y * v2_y);

  if (len1 < 1e-6 || len2 < 1e-6) {
    return 0.0; // Đoạn quá ngắn, không xác định góc
  }

  double cos_theta = dot / (len1 * len2);
  cos_theta = std::max(-1.0, std::min(1.0, cos_theta)); // clamp
  double angle_rad = std::acos(cos_theta);
  return angle_rad * 180.0 / M_PI; // Đổi sang độ
}

int Client::findClosestSegment(const Json::Value &waypoints, double position_x_,
                               double position_y_) {
  if (waypoints.size() < 3)
    return 0; // Cần ít nhất 3 waypoint để có 2 đoạn đầu

  double min_dist = std::numeric_limits<double>::max();
  int closest_segment = 0;

  std::pair<double, double> robot_pos(position_x_, position_y_);

  // Đoạn 0-1
  {
    const auto &wp0 = waypoints[0];
    const auto &wp1 = waypoints[1];
    std::pair<double, double> p0(wp0["position"]["position"]["x"].asDouble(),
                                 wp0["position"]["position"]["y"].asDouble());
    std::pair<double, double> p1(wp1["position"]["position"]["x"].asDouble(),
                                 wp1["position"]["position"]["y"].asDouble());
    double dist = distanceToSegment(p0, p1, robot_pos);
    if (dist < min_dist) {
      min_dist = dist;
      closest_segment = 0;
    }
  }

  // Đoạn 1-2
  {
    const auto &wp1 = waypoints[1];
    const auto &wp2 = waypoints[2];
    std::pair<double, double> p1(wp1["position"]["position"]["x"].asDouble(),
                                 wp1["position"]["position"]["y"].asDouble());
    std::pair<double, double> p2(wp2["position"]["position"]["x"].asDouble(),
                                 wp2["position"]["position"]["y"].asDouble());
    double dist = distanceToSegment(p1, p2, robot_pos);
    if (dist < min_dist) {
      min_dist = dist;
      closest_segment = 1;
    }
  }

  return closest_segment;
}

// Hàm tính khoảng cách từ điểm tới đoạn thẳng (không sử dụng Eigen)
double Client::distanceToSegment(const std::pair<double, double> &p1,
                                 const std::pair<double, double> &p2,
                                 const std::pair<double, double> &point) {
  double seg_dir_x = p2.first - p1.first;
  double seg_dir_y = p2.second - p1.second;
  double vec_to_point_x = point.first - p1.first;
  double vec_to_point_y = point.second - p1.second;

  double c = seg_dir_x * vec_to_point_x + seg_dir_y * vec_to_point_y;

  // Nằm phía trước p1
  if (c <= 0.0)
    return std::sqrt(vec_to_point_x * vec_to_point_x +
                     vec_to_point_y * vec_to_point_y);

  // Nằm phía sau p2
  double seg_len_sq = seg_dir_x * seg_dir_x + seg_dir_y * seg_dir_y;
  if (c >= seg_len_sq)
    return std::sqrt((point.first - p2.first) * (point.first - p2.first) +
                     (point.second - p2.second) * (point.second - p2.second));

  // Nằm giữa segment
  double cross_prod = seg_dir_x * vec_to_point_y - seg_dir_y * vec_to_point_x;
  return std::abs(cross_prod) /
         std::sqrt(seg_dir_x * seg_dir_x + seg_dir_y * seg_dir_y);
}

void Client::processJsonAndPublish(const Json::Value &jsonResponse) {
  // Kiểm tra điều kiện "result" == "OK"
  if (jsonResponse.isMember("result") &&
      jsonResponse["result"].asString() == "OK") {
    // Kiểm tra xem "status_get_path" có tồn tại và có "waypoints"
    // pub path
    if (jsonResponse.isMember("status_get_path") &&
        jsonResponse["status_get_path"]
            .isObject() && // Kiểm tra "status_get_path" có phải là object
        jsonResponse["status_get_path"].isMember("waypoints") &&
        jsonResponse["status_get_path"]["waypoints"].isArray() &&
        !jsonResponse["status_get_path"]["waypoints"].empty()) {
      // ROS_INFO_STREAM("[Client] Begin get waypoints ");
      // ros::Duration(5).sleep();
      const auto &waypoints = jsonResponse["status_get_path"]["waypoints"];

      // not accurracy
      // int closest_segment =
      //     findClosestSegment(waypoints, position_x_, position_y_);
      // Json::Value trimmed_waypoints(Json::arrayValue);
      // for (int i = closest_segment; i < waypoints.size(); ++i) {
      //   trimmed_waypoints.append(waypoints[i]);
      // }
      Json::Value trimmed_waypoints(Json::arrayValue);
      buildTrimmedWaypoints(waypoints, trimmed_waypoints);

      if (trimmed_waypoints.empty()) {
        ROS_INFO_STREAM("Robot is out of path receive");
        path_is_valid = false;
      } else {
        ROS_INFO_STREAM("Path is OK");
        path_is_valid = true;
      }
      // ROS_INFO_STREAM("Trimmed waypoints from index: "
      //                 << closest_segment
      //                 << ", total: " << trimmed_waypoints.size());
      if (path_is_valid) {
        // Gán lại vào json
        Json::Value modified_status = jsonResponse["status_get_path"];
        modified_status["waypoints"] = trimmed_waypoints;

        // Tạo và publish message
        std_stamped_msgs::StringStamped msg;
        msg.stamp = ros::Time::now();
        Json::StreamWriterBuilder writer;
        msg.data = Json::writeString(writer, modified_status);
        route_pub_.publish(msg);
        // TODO: add here
        // ROS_INFO("Published new path: %s", msg.data.c_str());
      } else {
        ROS_ERROR_THROTTLE(5, "Path not correct to current pose");
      }
    }
    // else
    // {
    //   ROS_INFO_STREAM("Invalid or missing 'status_get_path' in JSON");
    // }

    // pub status RUN or STOP
    if (jsonResponse.isMember("status_traffic_control")) {
      std::string trafficStatus =
          jsonResponse["status_traffic_control"].asString();

      // Tạo đối tượng StringStamped và gán giá trị
      std_stamped_msgs::StringStamped pause_msg;
      pause_msg.stamp = ros::Time::now(); // Thêm thời gian hiện tại vào header
      if (path_is_valid) {
        pause_msg.data =
            trafficStatus; // Gán giá trị từ 'status_traffic_control'
      } else {
        pause_msg.data = "STOP - Robot not in path when receive new path";
      }

      // Publish lên topic
      pause_status_pub_.publish(pause_msg);

      // ROS_INFO_THROTTLE(10, "Published status_traffic_control: %s",
      // trafficStatus.c_str());
    } else {
      ROS_INFO_STREAM(
          "Field 'status_traffic_control' not found in response JSON.");
    }

    if (jsonResponse.isMember("last_success_id") &&
        jsonResponse["last_success_id"].isUInt()) {
      uint32_t last_success_id = jsonResponse["last_success_id"].asUInt();
      // Log chi tiết các giá trị
      // ROS_INFO_STREAM("========= DEBUG COMPARISON =========");
      // ROS_INFO_STREAM("last_success_id (from JSON): %u", last_success_id);
      // ROS_INFO_STREAM("goal_id_ (current value):    %u", goal_id_);
      // ROS_INFO_STREAM("Values in hex:");
      // ROS_INFO_STREAM("last_success_id: 0x%08X", last_success_id);
      // ROS_INFO_STREAM("goal_id_:        0x%08X", goal_id_);
      // ROS_INFO_STREAM("Type sizes:");
      // ROS_INFO_STREAM("last_success_id size: %zu bytes",
      // sizeof(last_success_id)); ROS_INFO_STREAM("goal_id_ size:        %zu
      // bytes", sizeof(goal_id_));
      if (last_success_id == last_goal_id_send) {
        need_to_wait_receive_path = false;
      }
      // Use last_success_id ...
    }
  } else {
    ROS_INFO_STREAM("JSON 'result' is not 'OK'");
  }
}

void Client::updateAutoMode(const std::string &mode_, Json::Value &root) {
  bool current_mode_is_auto = (mode_ == "AUTO");
  ros::Time now = ros::Time::now();

  root["auto_mode"] = current_mode_is_auto;

  static bool first_call = true;

  if (first_call) {
    mode_change_time = now;
    last_mode_is_auto = current_mode_is_auto;
    root_auto_mode = current_mode_is_auto;
    root["auto_mode"] = current_mode_is_auto;
    first_call = false;
    return;
  }

  // Nếu trạng thái mode_ thay đổi so với lần trước, reset thời gian đếm
  if (current_mode_is_auto != last_mode_is_auto) {
    mode_change_time = now;
    last_mode_is_auto = current_mode_is_auto;
  }

  // Nếu là chế độ AUTO, cập nhật ngay lập tức
  if (current_mode_is_auto) {
    if (root_auto_mode != current_mode_is_auto) {
      root["auto_mode"] = current_mode_is_auto;
      trigger_resend_goal = true;
      root_auto_mode = current_mode_is_auto;
    }
  } else {
    // Kiểm tra xem đã giữ trạng thái mới đủ 2 giây chưa (chỉ cho chế độ không
    // phải AUTO)
    ros::Duration duration = now - mode_change_time;

    if (duration.toSec() >= 2.0) { // 2 giây
      if (root_auto_mode != current_mode_is_auto) {
        root["auto_mode"] = current_mode_is_auto;
        root_auto_mode = current_mode_is_auto;
      }
    } else {
      // Chưa đủ 2 giây, giữ nguyên trạng thái cũ
      root["auto_mode"] = root_auto_mode;
    }
  }
}

geometry_msgs::PoseStamped Client::createPoseStamped(double x, double y,
                                                     double qx, double qy,
                                                     double qz, double qw) {
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.seq = 0;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = "base_link";

  pose_msg.pose.position.x = x;
  pose_msg.pose.position.y = y;
  pose_msg.pose.position.z = 0.0;

  pose_msg.pose.orientation.x = qx;
  pose_msg.pose.orientation.y = qy;
  pose_msg.pose.orientation.z = qz;
  pose_msg.pose.orientation.w = qw;

  return pose_msg;
}

void Client::configureClient(std::shared_ptr<httplib::Client> &client) {
  client->set_read_timeout(5);
  client->set_write_timeout(5);
  client->set_connection_timeout(5);
  client->set_keep_alive(false); // Tránh giữ kết nối lâu
}

void Client::sendRobotInfo() {
  static const std::set<std::string> special_state_actions = {
      "GOING_TO_OUT_OF_HUB", "SEND_GOTO_OUT_OF_HUB"};
  static uint32_t current_seq = 0; // Biến đếm seq
  while (ros::ok()) {
    double frequency;
    {
      // std::lock_guard<std::mutex> lock(status_mutex_);
      frequency = (status_ == "RUNNING" || status_ == "PAUSED") ? 5 : 1;
    }
    ros::Rate rate(frequency);
    if (!cli_update_robot_info_) {
      ROS_INFO_THROTTLE(10, "HTTP cli_update_robot_info_ is not initialized!");
      continue;
    }

    int32_t good_id =
        (special_state_actions.count(hub_action_state_go_out_) ||
         special_state_actions.count(matehan_action_state_go_out_) ||
         special_state_actions.count(elevator_action_state_go_out_))
            ? 0
            : 1;

    Json::Value root;
    {
      root["header"]["seq"] = current_seq;
      // std::lock_guard<std::mutex> lock(data_mutex_);
      root["robot_name"] = robot_name_;
      root["position"]["x"] = position_x_;
      root["position"]["y"] = position_y_;
      root["position"]["orient"] = orientation_;

      root["vel"] = Json::arrayValue;
      root["vel"].append(velocity_[0]);
      root["vel"].append(velocity_[1]);

      root["shape"] = Json::arrayValue;
      root["shape"].append(robot_radius_[0]);
      root["shape"].append(robot_radius_[1]);

      root["mission_group"] = mission_group_;
      root["current_action_type"] = current_action_type_;
      root["mode"] = mode_;
      root["status"] = status_;
      root["detail_status"] = detail_;
    }

    root["current_id_goal"] = 1;

    if (moving_control_status_ != "WAITING") {
      root["direction_move"] = direction_move_;
      root["state_move"] = state_move_;
    } else {
      root["direction_move"] = "";
      root["state_move"] = "";
    }

    updateAutoMode(mode_, root);

    if (trigger_resend_goal) {
      need_to_wait_receive_path = true;
    }

    root["order_status"] = need_to_wait_receive_path ? 1 : 0;
    root["pause"] = (status_ == "PAUSED");

    if (status_ == "WAITING_INIT_POSE") {
      init_pose_success = false;
    } else if (status_ == "WAITING") {
      init_pose_success = true;
    }

    root["init_pose"] = init_pose_success;
    root["good_id"] = good_id;
    root["allow_estimate_pose"] =
        !(last_goal_in_path.first == 0.0 && last_goal_in_path.second == 0.0);

    Json::StreamWriterBuilder writer;
    std::string data = Json::writeString(writer, root);

    std::lock_guard<std::mutex> lock(cli_mutex_update_robot_info_);
    auto response = cli_update_robot_info_->Post(
        "/traffic_control/update_status", data, "application/json");
    ROS_INFO("Sent robot info with seq=%u", current_seq);
    if (response && response->status == 200) {
      last_successful_connection_time_ = ros::Time::now();
      ROS_INFO_THROTTLE(10, "Robot info sent successfully");

      Json::Value jsonResponse;
      Json::CharReaderBuilder reader;
      std::string errs;
      std::istringstream s(response->body);

      if (Json::parseFromStream(reader, s, &jsonResponse, &errs)) {
        std::string response_str = Json::writeString(writer, jsonResponse);
        ROS_INFO_THROTTLE(10, "Response: %s", response_str.c_str());
        if (jsonResponse.isObject()) {
          if (jsonResponse.isMember("seq")) {
            uint32_t seq_received = jsonResponse["seq"].asUInt();
            ROS_INFO("Received seq from response: %u", seq_received);
          } else {
            ROS_INFO("Response JSON does not contain 'seq' field");
          }
          processJsonAndPublish(jsonResponse);
        }
      } else {
        ROS_INFO_STREAM("Failed to parse JSON response: "
                        << errs << ", current_seq = " << current_seq);
      }
    } else {
      if (response) {
        ROS_INFO_STREAM("Failed to send robot info. Status: "
                        << response->status << ", Body: " << response->body
                        << ", current_seq = " << current_seq);
      } else {
        ROS_INFO("No response from server (robot info), current_seq = %u",
                 current_seq);
      }
    }
    current_seq++;

    rate.sleep();
  }
}

void Client::updatePauseStatus() {
  ros::Rate rate(2); // kiểm tra mỗi 100ms
  while (ros::ok()) {
    ros::Time now = ros::Time::now();
    ros::Duration duration = now - last_successful_connection_time_;

    if (duration.toSec() > 0.5) {
      if (moving_control_status_ != "WAITING") {
        // Tạo đối tượng StringStamped và gán giá trị
        std_stamped_msgs::StringStamped pause_msg;
        pause_msg.stamp =
            ros::Time::now(); // Thêm thời gian hiện tại vào header
        pause_msg.data = "STOP - Disconected to server";
        // Publish lên topic
        pause_status_pub_.publish(pause_msg);
      }
    }
    rate.sleep();
  }
}

void Client::ManagerSendGoal() {

  if (trigger_resend_goal) {
    ROS_INFO_STREAM("Resend goal when change mode to AUTO");
    if (receive_goal) {
      if (sendGoal(last_goal_x, last_goal_y)) {
        trigger_resend_goal = false;
      }
    } else {
      trigger_resend_goal = false;
    }
  }
  if (_state != _pre_state) {
    // Ghi log thông báo sự thay đổi trạng thái
    ROS_INFO_STREAM("State changed: " << _pre_state << " -> " << _state);

    // Cập nhật _pre_state bằng _state mới
    _pre_state = _state;
  }
  if (_state == "WAITING") {
    index_action_found_ = 0;
    is_send_special_goal_ = false;
    is_robot_in_hub_begin_ = false;
    is_send_get_last_path_ = false;
  } else if (_state == "SEND_GOAL") {
    if (current_action_type_ != "" && current_action_type_ != "un_docking" &&
        current_action_num_ > 0) {
      double x, y;
      if (getTargetGoal(x, y)) {
        last_goal_x = x;
        last_goal_y = y;
        receive_goal = true;
        ROS_INFO_STREAM("Target goal found at x: " << x << ", y: " << y);

        // Call sendGoal here
        if (sendGoal(x, y)) {
          trigger_resend_goal = false;
          _state = "WAITING_RESULT";
          ROS_INFO_STREAM(
              "Goal sent successfully. State changed to WAITING_RESULT");
        } else {
          ROS_INFO_STREAM("Failed to send goal");
        }
      } else {
        ROS_INFO_STREAM("Failed to get target goal");
      }
    }
  } else if (_state == "WAITING_RESULT") {
    // Placeholder for state transition logic to SEND_GOAL if necessary
    getState();
  } else if (_state == "SEND_GET_LAST_PATH") {
    if (getLastPath()) {
      _state = "WAITING_RESULT";
      ROS_INFO_STREAM(
          "Send get last path successfully. State changed to WAITING_RESULT");
    } else {
      ROS_INFO_STREAM("Failed to send get last path");
    }
  } else if (_state == "DONE") {
  }

  // Handle resend goal when AGV change state
}

void Client::getState() {
  if (_state != "WAITING") {
    static const std::set<std::string> special_state_actions = {
        "GOING_TO_OUT_OF_HUB"};
    bool _state_action_special_is_go_out;
    // get special goal
    if (special_state_actions.count(hub_action_state_go_out_) ||
        special_state_actions.count(matehan_action_state_go_out_) ||
        special_state_actions.count(elevator_action_state_go_out_)) {
      _state_action_special_is_go_out = true;
    } else {
      _state_action_special_is_go_out = false;
    }

    if (current_action_num_ > index_action_found_ &&
        index_action_found_ < total_action_) {
      is_send_special_goal_ = false;
      is_send_get_last_path_ = false;
      _state = "SEND_GOAL";
    } else if (is_send_get_last_path_ && current_action_type_ == "move") {
      is_send_get_last_path_ = false;
      is_send_special_goal_ = false;
      _state = "SEND_GET_LAST_PATH";
    } else if (current_action_num_ == index_action_found_ &&
               _state_action_special_is_go_out && !is_send_special_goal_) {
      is_send_special_goal_ = true;
      is_send_get_last_path_ = true;
      ROS_INFO_STREAM(
          "SEND SPECIAL GOAL BECAUSE CURRENT STATE OF ACTION IS GO OUT "
          "OF HUB");
      _state = "SEND_GOAL";
    } else if (index_action_found_ == total_action_ &&
               current_action_type_ == "" &&
               current_action_num_ == total_action_) {
      _state = "DONE";
    }
  }
}

// /*
//  ****     ****     **     ** ****     **
// /**/**   **/**    ****   /**/**/**   /**
// /**//** ** /**   **//**  /**/**//**  /**
// /** //***  /**  **  //** /**/** //** /**
// /**  //*   /** **********/**/**  //**/**
// /**   /    /**/**//////**/**/**   //****
// /**        /**/**     /**/**/**    //***
// //         // //      // // //      ///
// */

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_a_send_data");
  ros::NodeHandle n;
  std::string server_ip;
  int server_port;
  n.param<std::string>("server_traffic_control_ip", server_ip,
                       "192.168.226.220");
  n.param<int>("server_traffic_control_port", server_port, 8080);
  std::string server_address =
      "http://" + server_ip + ":" + std::to_string(server_port);
  Client client_agv(n, server_address);

  signal(SIGINT, sigintHandler);
  ROS_INFO("Robot A ready to send goal and robot info");
  ros::Rate r(10);
  while (ros::ok()) {
    r.sleep();
    ros::spinOnce();
    client_agv.ManagerSendGoal();
  }
  // ros::spin();

  return 0;
}