#include "server/server_traffic_control.h"

void sigintHandler(int sig) {
  ROS_INFO("Shutting down...");

  // Perform any necessary shutdown procedures here
  // For example, stop any ongoing actions or cleanup resources

  // Shutdown ROS
  ros::shutdown();
}

Server::Server() {
  // POST: /traffic_control/update_target_position
  svr_.Post("/traffic_control/update_target_position",
            [&](const httplib::Request &req, httplib::Response &res) {
              try {
                handleReceiveGoal(req, res);
              } catch (const std::exception &e) {
                ROS_INFO_STREAM(
                    "[update_target_position] Exception: " << e.what());
                ROS_INFO_STREAM(
                    "[update_target_position] Request body: " << req.body);
                res.status = 500;
                res.set_content("Internal server error", "text/plain");
              } catch (...) {
                ROS_INFO("[update_target_position] Unknown exception caught!");
                ROS_INFO_STREAM(
                    "[update_target_position] Request body: " << req.body);
                res.status = 500;
                res.set_content("Unknown internal error", "text/plain");
              }
            });

  // POST: /traffic_control/update_status
  svr_.Post("/traffic_control/update_status",
            [&](const httplib::Request &req, httplib::Response &res) {
              try {
                handleReceiveRobotInfo(req, res);
              } catch (const std::exception &e) {
                ROS_INFO_STREAM("[update_status] Exception: " << e.what());
                ROS_INFO_STREAM("[update_status] Request body: " << req.body);
                res.status = 500;
                res.set_content("Internal server error", "text/plain");
              } catch (...) {
                ROS_INFO("[update_status] Unknown exception caught!");
                ROS_INFO_STREAM("[update_status] Request body: " << req.body);
                res.status = 500;
                res.set_content("Unknown internal error", "text/plain");
              }
            });

  // POST: /traffic_control/get_last_path
  svr_.Post("/traffic_control/get_last_path",
            [&](const httplib::Request &req, httplib::Response &res) {
              try {
                handleReceiveGetLastPath(req, res);
              } catch (const std::exception &e) {
                ROS_INFO_STREAM("[get_last_path] Exception: " << e.what());
                ROS_INFO_STREAM("[get_last_path] Request body: " << req.body);
                res.status = 500;
                res.set_content("Internal server error", "text/plain");
              } catch (...) {
                ROS_INFO("[get_last_path] Unknown exception caught!");
                ROS_INFO_STREAM("[get_last_path] Request body: " << req.body);
                res.status = 500;
                res.set_content("Unknown internal error", "text/plain");
              }
            });
}
// Hàm chuyển từ góc orient (yaw) sang quaternion
geometry_msgs::Quaternion Server::yawToQuaternion(double yaw_radian) {
  tf2::Quaternion q_tf;
  q_tf.setRPY(0, 0, yaw_radian); // Roll = 0, Pitch = 0, Yaw = yaw_radian

  geometry_msgs::Quaternion q_msg;
  q_msg.x = q_tf.x();
  q_msg.y = q_tf.y();
  q_msg.z = q_tf.z();
  q_msg.w = q_tf.w();
  return q_msg;
}
void Server::startServer() {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param<std::string>("server_traffic_control_ip", server_ip,
                         "192.168.226.220");
  pnh.param<int>("server_traffic_control_port", server_port, 8080);
  goal_pub_ =
      nh.advertise<tuw_multi_robot_msgs::RobotGoals>("goal_from_robot", 100);
  // TODO: de ys topic nay neu thuc the thi de ten la robot_info
  robot_info_pub_ = nh.advertise<tuw_multi_robot_msgs::RobotInfo>(
      "robot_info_receive_from_robot", 100);
  robot_pose_pub_ = nh.advertise<nav_msgs::Odometry>("robot_pose_topic", 10);
  module_status_pub_ = nh.advertise<std_stamped_msgs::StringStamped>(
      "server_traffic/module_status", 5);
  subRoute_ = nh.subscribe("routes", 10, &Server::routeCallback, this);
  subplannerStatus_ =
      nh.subscribe("planner_status", 10, &Server::plannerStatusCallback, this);
  module_status_timer_ =
      nh.createTimer(ros::Duration(1.0), &Server::moduleStatusCallback, this);
  client_ = nh.serviceClient<tuw_multi_robot_msgs::GetRobotStatus>(
      "get_robot_status");
  // Start the server in a separate thread
  server_thread_ = std::thread([this]() {
    ROS_INFO("Starting server on %s:%d", server_ip.c_str(), server_port);
    bool result = svr_.listen(server_ip.c_str(), server_port);
    if (!result) {
      ROS_INFO("HTTP server stopped listening. Check if port is already used "
               "or another error occurred.");
    } else {
      ROS_INFO("HTTP server is listening successfully.");
    }
  });
  // server_thread_.detach();
}

void Server::joinServerThread() {
  if (server_thread_.joinable()) {
    server_thread_.join();
  }
}

/*
       #       #     #####     #    #       #       ######     #     #####  # #
      #       #     #     #   # #   #       #       #     #   # #   #     # # #
     #       #      #        #   #  #       #       #     #  #   #  #       #  #
    #       #       #       #     # #       #       ######  #     # #       ###
   #       #        #       ####### #       #       #     # ####### #       #  #
  #       #         #     # #     # #       #       #     # #     # #     # # #
 #       #           #####  #     # ####### ####### ######  #     #  #####  # #

*/

void Server::routeCallback(
    const tuw_multi_robot_msgs::RouteArray::ConstPtr &msg) {
  route_array_msg = *msg;
  // ROS_INFO("Received route message with seq: %u", msg->header.seq);
}

void Server::plannerStatusCallback(
    const tuw_multi_robot_msgs::RouterStatus::ConstPtr &msg) {
  bool found = false;

  // Loop through all elements in planner_status_msg
  for (auto &status : planner_status_msg) {
    // Check if the current_planning_robot matches
    if (status.current_planning_robot == msg->current_planning_robot) {
      // Replace the existing entry with the new message
      status = *msg;
      found = true;
      // break;
    } else {
      // Check if the current_planning_robot exists in msg->active_robots
      if (std::find(msg->active_robots.begin(), msg->active_robots.end(),
                    status.current_planning_robot) !=
          msg->active_robots.end()) {
        // Update the active_robots field of the existing status
        status.active_robots = msg->active_robots;
      }
    }
  }

  // If no match was found, add the new message to the vector
  if (!found) {
    planner_status_msg.push_back(*msg);
  }
}

void Server::handleReceiveGoal(const httplib::Request &req,
                               httplib::Response &res) {
  Json::CharReaderBuilder reader;
  Json::Value root;
  std::string errs;
  std::istringstream iss(req.body);
  if (!Json::parseFromStream(reader, iss, &root, &errs)) {
    res.status = 400;
    res.set_content("Invalid JSON data", "text/plain");
    return;
  }

  geometry_msgs::PoseStamped goal_msg;
  goal_msg.header.seq = root["header"]["seq"].asUInt();
  goal_msg.header.stamp = ros::Time(root["header"]["stamp"].asDouble());
  goal_msg.header.frame_id = root["header"]["frame_id"].asString();
  goal_msg.pose.position.x = root["pose"]["position"]["x"].asDouble();
  goal_msg.pose.position.y = root["pose"]["position"]["y"].asDouble();
  goal_msg.pose.position.z = root["pose"]["position"]["z"].asDouble();
  goal_msg.pose.orientation.x = root["pose"]["orientation"]["x"].asDouble();
  goal_msg.pose.orientation.y = root["pose"]["orientation"]["y"].asDouble();
  goal_msg.pose.orientation.z = root["pose"]["orientation"]["z"].asDouble();
  goal_msg.pose.orientation.w = root["pose"]["orientation"]["w"].asDouble();

  tuw_multi_robot_msgs::RobotGoals robot_goals;
  // Set the goal ID if available in the request
  if (root.isMember("id") && root["id"].isUInt()) {
    robot_goals.id = root["id"].asUInt();
  } else {
    robot_goals.id = 0; // Set a default ID if not provided
  }
  // TODO: fix robot name
  robot_goals.robot_name = root["robot_name"].asString();

  geometry_msgs::Pose pose;
  pose.position = goal_msg.pose.position;
  pose.orientation = goal_msg.pose.orientation;

  // Add the pose to the destinations array in robot_goals
  robot_goals.destinations.push_back(pose);
  goal_pub_.publish(robot_goals);

  // save goal to check history
  goals_id[robot_goals.robot_name] = robot_goals.id;

  res.set_content("Goal received and published successfully", "text/plain");
  res.status = 200;
}

void Server::handleReceiveGetLastPath(const httplib::Request &req,
                                      httplib::Response &res) {
  Json::CharReaderBuilder reader;
  Json::Value root;
  std::string errs;
  std::istringstream iss(req.body);
  if (!Json::parseFromStream(reader, iss, &root, &errs)) {
    res.status = 400;
    res.set_content("Invalid JSON data", "text/plain");
    return;
  }

  // TODO: fix robot name

  // save goal to check history
  std::string robot_name = root["robot_name"].asString();
  ROS_INFO("RECEIVE REQUEST GET LAST PATH FROM : %s", robot_name.c_str());

  // save robot name as requested
  request_get_last_path[robot_name] = true;

  res.set_content("Receive command get last path successfully", "text/plain");
  res.status = 200;
}

void Server::handleReceiveRobotInfo(const httplib::Request &req,
                                    httplib::Response &res) {

  // read data sent from robot
  Json::CharReaderBuilder reader;
  Json::Value root;
  std::string errs;
  std::istringstream iss(req.body);
  if (!Json::parseFromStream(reader, iss, &root, &errs)) {
    res.status = 400;
    res.set_content("Invalid JSON data", "text/plain");
    ROS_INFO_STREAM("Fail to [handleReceiveRobotInfo]");
    return;
  }
  // ROS_INFO("RECEIVE ROBOT INFO");
  tuw_multi_robot_msgs::RobotInfo robot_info_msg;
  robot_info_msg.header.seq = root["header"]["seq"].asUInt();
  robot_info_msg.header.stamp = ros::Time(root["header"]["stamp"].asDouble());
  robot_info_msg.header.frame_id = "map";
  robot_info_msg.robot_name = root["robot_name"].asString();
  ROS_INFO_STREAM("[handleReceiveRobotInfo] robot_name = "
                  << robot_info_msg.robot_name
                  << ", seq = " << robot_info_msg.header.seq);

  robot_info_msg.pose.pose.position.x = root["position"]["x"].asDouble();
  robot_info_msg.pose.pose.position.y = root["position"]["y"].asDouble();
  geometry_msgs::Quaternion q =
      yawToQuaternion(root["position"]["orient"].asDouble());
  robot_info_msg.pose.pose.orientation.x = q.x;
  robot_info_msg.pose.pose.orientation.y = q.y;
  robot_info_msg.pose.pose.orientation.z = q.z;
  robot_info_msg.pose.pose.orientation.w = q.w;
  for (int i = 0; i < 36; ++i) {
    robot_info_msg.pose.covariance[i] =
        root["pose"]["covariance"][i].asDouble();
  }
  robot_info_msg.layout_map = 0;
  robot_info_msg.mission_group = root["mission_group"].asString();
  robot_info_msg.shape = 1;
  for (const auto &var : root["shape"]) {
    robot_info_msg.shape_variables.push_back(var.asFloat());
  }
  robot_info_msg.current_action_type = root["current_action_type"].asString();
  robot_info_msg.direction_move = root["direction_move"].asString();
  robot_info_msg.state_move = root["state_move"].asString();
  robot_info_msg.mode = root["mode"].asString();
  robot_info_msg.status = root["status"].asString();
  robot_info_msg.detail_status = root["detail_status"].asString();
  robot_info_msg.auto_mode = root["auto_mode"].asBool();
  robot_info_msg.pause = root["pause"].asBool();
  robot_info_msg.init_pose = root["init_pose"].asBool();
  robot_info_msg.allow_estimate_pose = root["allow_estimate_pose"].asBool();
  robot_info_msg.good_id = root["good_id"].asInt();
  robot_info_msg.order_status = root["order_status"].asInt();

  // Xử lý velocity
  if (root.isMember("vel") && root["vel"].isArray() &&
      root["vel"].size() >= 2) {
    robot_info_msg.velocity.linear.x =
        root["vel"][0].asDouble(); // vận tốc thẳng
    robot_info_msg.velocity.linear.y = 0.0;
    robot_info_msg.velocity.linear.z = 0.0;
    robot_info_msg.velocity.angular.z =
        root["vel"][1].asDouble(); // vận tốc góc
    robot_info_msg.velocity.angular.x = 0.0;
    robot_info_msg.velocity.angular.y = 0.0;
  } else {
    // Nếu không có trường velocity thì gán 0
    robot_info_msg.velocity.linear.x = 0.0;
    robot_info_msg.velocity.linear.y = 0.0;
    robot_info_msg.velocity.linear.z = 0.0;
    robot_info_msg.velocity.angular.x = 0.0;
    robot_info_msg.velocity.angular.y = 0.0;
    robot_info_msg.velocity.angular.z = 0.0;
  }

  robot_info_pub_.publish(robot_info_msg);

  // create response data
  // Convert response array to JSON string and set it as response content
  Json::Value response_root;
  // ROS_INFO("Robot name: %s", robot_info_msg.robot_name.c_str());
  response_root["result"] = "OK";
  response_root["status_get_path"] = "";
  response_root["status_traffic_control"] = "";
  response_root["last_success_id"] = -1;
  if (robot_info_msg.status == "WAITING") {
    // If status wait -> set default value
    response_root["status_get_path"] = "Dont have new path";
    response_root["status_traffic_control"] = "STOP - Dont have new path";
  } else {
    // if RUN, PAUSE, ERROR
    auto goal_id = goals_id.find(robot_info_msg.robot_name);
    // found goal id with robot name
    if (goal_id != goals_id.end()) {
      uint32_t current_id_goal_request = goals_id[robot_info_msg.robot_name];
      // get last planner status with robot_name
      response_root["status_get_path"] = "Wait for create path";
      response_root["status_traffic_control"] = "STOP - Wait for create path";
      // ROS_INFO("4");
      for (const auto &status : planner_status_msg) {
        if (status.current_planning_robot == robot_info_msg.robot_name) {
          if (status.success) {
            response_root["last_success_id"] = status.id;
          }
        }
        // Check if the current_planning_robot matches the robot_name
        if (status.current_planning_robot == robot_info_msg.robot_name) {
          // Create a JSON response with the matched element
          if (status.id == current_id_goal_request) {
            if (status.success) {
              response_root["status_get_path"] =
                  getRobotRouteJson(robot_info_msg.robot_name); // set path here

              bool is_pause = false;
              std::string detail_status_pause = "";
              bool success = callGetRobotStatusService(
                  robot_info_msg.robot_name, is_pause, detail_status_pause);

              if (success) {
                if (is_pause) {
                  response_root["status_traffic_control"] =
                      "STOP" + std::string(" by ") + detail_status_pause;
                } else {
                  response_root["status_traffic_control"] = "RUN";
                }
              } else {
                ROS_INFO("Failed to retrieve robot status.");
              }
              // ROS_INFO("0");
            } else {
              response_root["status_get_path"] = status.error_code;
              response_root["status_traffic_control"] =
                  "STOP - " + status.error_code;
              // ROS_INFO("1");
            }
          } else {
            // ROS_INFO("3");
            response_root["status_get_path"] = "Wait for create path";
            response_root["status_traffic_control"] =
                "STOP- Wait for create path";
          }
          break;
        }
      }
    } else {
      // If not found goal with robot_name in history
      if (robot_info_msg.current_action_type != "un_docking") {
        response_root["status_get_path"] = "Dont have new path";
        response_root["status_traffic_control"] = "STOP - Dont have new path";
      } else {
        response_root["status_get_path"] = "Dont have new path";
        response_root["status_traffic_control"] = "RUN";
      }
    }
  }
  response_root["seq"] = robot_info_msg.header.seq;

  ROS_INFO_STREAM(" Response [handleReceiveRobotInfo] robot_name = "
                  << robot_info_msg.robot_name
                  << ", seq = " << robot_info_msg.header.seq);

  // Convert response JSON to string and set it as response content
  Json::StreamWriterBuilder writer;
  std::string response_str = Json::writeString(writer, response_root);
  res.set_content(response_str, "application/json");

  // Set response status
  res.status = 200;
}

bool Server::callGetRobotStatusService(const std::string &robot_name,
                                       bool &is_pause,
                                       std::string &detail_status_pause) {

  tuw_multi_robot_msgs::GetRobotStatus srv;
  srv.request.robot_name = robot_name;
  if (client_.call(srv)) {
    // ROS_INFO("Response received:");
    // ROS_INFO("Detail Pause by Route: %s",
    //          srv.response.detail_status_run_pause_by_route.c_str());
    // ROS_INFO("Detail Pause by Collision: %s",
    //          srv.response.detail_status_run_pause_by_detect_collision.c_str());
    // ROS_INFO("Is Pause by Route: %s",
    //          srv.response.is_pause_by_route ? "True" : "False");
    // ROS_INFO("Is Pause by Collision: %s",
    //          srv.response.is_pause_by_detect_collision ? "True" : "False");
    if (srv.response.is_pause_by_route) {
      is_pause = true;
      detail_status_pause = srv.response.detail_status_run_pause_by_route;
    } else if (srv.response.is_pause_by_detect_collision) {
      is_pause = true;
      detail_status_pause =
          srv.response.detail_status_run_pause_by_detect_collision;
    }
    return true; // Thành công
  } else {
    ROS_INFO("Failed to call service get_robot_status");
    return false; // Thất bại
  }
}

/*
       #       #    #######
      #       #     #       #    # #    #  ####  ##### #  ####  #    #
     #       #      #       #    # ##   # #    #   #   # #    # ##   #
    #       #       #####   #    # # #  # #        #   # #    # # #  #
   #       #        #       #    # #  # # #        #   # #    # #  # #
  #       #         #       #    # #   ## #    #   #   # #    # #   ##
 #       #          #        ####  #    #  ####    #   #  ####  #    #

*/

void Server::moduleStatusCallback(const ros::TimerEvent &) {
  std_stamped_msgs::StringStamped msg;
  msg.stamp = ros::Time::now();
  msg.data = "Module is running"; // hoặc dữ liệu thực tế

  module_status_pub_.publish(msg);
}

Json::Value
Server::convertRouteMsgToJson(const tuw_multi_robot_msgs::Route &route_msg) {
  Json::Value root;
  Json::Value waypoints(Json::arrayValue);

  ROS_INFO_STREAM("Converting route_msg with " << route_msg.segments.size()
                                               << " segments.");

  for (size_t i = 0; i < route_msg.segments.size(); ++i) {
    const auto &segment = route_msg.segments[i];
    ROS_INFO_STREAM("Processing segment[" << i << "]");

    if (waypoints.empty()) {
      ROS_INFO_STREAM("First waypoint, creating start and end poses.");
      // Case 0: Lần đầu tiên
      Json::Value start_wp;
      start_wp["name"] = "Pose_" + std::to_string(i);
      start_wp["position"]["position"]["x"] = segment.start.position.x;
      start_wp["position"]["position"]["y"] = segment.start.position.y;
      start_wp["position"]["position"]["z"] = segment.start.position.z;
      start_wp["position"]["orientation"]["x"] = segment.start.orientation.x;
      start_wp["position"]["orientation"]["y"] = segment.start.orientation.y;
      start_wp["position"]["orientation"]["z"] = segment.start.orientation.z;
      start_wp["position"]["orientation"]["w"] = segment.start.orientation.w;
      if (!segment.start_properties.empty()) {
        ROS_INFO_STREAM(
            "Parsing start_properties: " << segment.start_properties);
        Json::CharReaderBuilder reader;
        Json::Value props;
        std::string errs;
        std::istringstream ss(segment.start_properties);
        if (Json::parseFromStream(reader, ss, &props, &errs)) {
          for (const auto &key : props.getMemberNames())
            start_wp["data"][key] = props[key];
        } else {
          ROS_INFO_STREAM("Failed to parse start_properties: " << errs);
        }
      }
      waypoints.append(start_wp);

      Json::Value end_wp;
      end_wp["name"] = "Pose_" + std::to_string(i + 1);
      end_wp["position"]["position"]["x"] = segment.end.position.x;
      end_wp["position"]["position"]["y"] = segment.end.position.y;
      end_wp["position"]["position"]["z"] = segment.end.position.z;
      end_wp["position"]["orientation"]["x"] = segment.end.orientation.x;
      end_wp["position"]["orientation"]["y"] = segment.end.orientation.y;
      end_wp["position"]["orientation"]["z"] = segment.end.orientation.z;
      end_wp["position"]["orientation"]["w"] = segment.end.orientation.w;
      if (!segment.end_properties.empty()) {
        ROS_INFO_STREAM("Parsing end_properties: " << segment.end_properties);
        Json::CharReaderBuilder reader;
        Json::Value props;
        std::string errs;
        std::istringstream ss(segment.end_properties);
        if (Json::parseFromStream(reader, ss, &props, &errs)) {
          for (const auto &key : props.getMemberNames())
            end_wp["data"][key] = props[key];
        } else {
          ROS_INFO_STREAM("Failed to parse end_properties: " << errs);
        }
      }
      waypoints.append(end_wp);
      continue;
    }

    const auto &segment_pre = route_msg.segments[i - 1];

    auto same_point = [](const geometry_msgs::Pose &a,
                         const geometry_msgs::Pose &b) {
      return std::abs(a.position.x - b.position.x) < 1e-6 &&
             std::abs(a.position.y - b.position.y) < 1e-6;
    };

    const auto last_wp = waypoints[waypoints.size() - 1];
    double last_x = last_wp["position"]["position"]["x"].asDouble();
    double last_y = last_wp["position"]["position"]["y"].asDouble();

    auto point_not_match = [&](const geometry_msgs::Pose &p) {
      return std::abs(p.position.x - last_x) > 1e-6 ||
             std::abs(p.position.y - last_y) > 1e-6;
    };

    auto create_wp = [&](const geometry_msgs::Pose &pose,
                         const std::string &props_str) {
      Json::Value wp;
      wp["name"] = "Pose_" + std::to_string(waypoints.size());
      wp["position"]["position"]["x"] = pose.position.x;
      wp["position"]["position"]["y"] = pose.position.y;
      wp["position"]["position"]["z"] = pose.position.z;
      wp["position"]["orientation"]["x"] = pose.orientation.x;
      wp["position"]["orientation"]["y"] = pose.orientation.y;
      wp["position"]["orientation"]["z"] = pose.orientation.z;
      wp["position"]["orientation"]["w"] = pose.orientation.w;

      if (!props_str.empty()) {
        ROS_INFO_STREAM("Parsing properties: " << props_str);
        Json::CharReaderBuilder reader;
        Json::Value props;
        std::string errs;
        std::istringstream ss(props_str);
        if (Json::parseFromStream(reader, ss, &props, &errs)) {
          for (const auto &key : props.getMemberNames())
            wp["data"][key] = props[key];
        } else {
          ROS_INFO_STREAM("Failed to parse properties: " << errs);
        }
      }
      return wp;
    };

    if (same_point(segment.start, segment_pre.start)) {
      // ROS_INFO_STREAM("Case 1: start == start");
      if (point_not_match(segment.start))
        waypoints.append(create_wp(segment.start, segment.start_properties));
      waypoints.append(create_wp(segment.end, segment.end_properties));
      // ROS_INFO_STREAM("add waypoints OK");
    } else if (same_point(segment.start, segment_pre.end)) {
      // ROS_INFO_STREAM("Case 2: start == end");
      if (point_not_match(segment.start))
        waypoints.append(create_wp(segment.start, segment.start_properties));
      waypoints.append(create_wp(segment.end, segment.end_properties));
      // ROS_INFO_STREAM("add waypoints OK");
    } else if (same_point(segment.end, segment_pre.start)) {
      // ROS_INFO_STREAM("Case 3: end == start");
      if (point_not_match(segment.end))
        waypoints.append(create_wp(segment.end, segment.end_properties));
      waypoints.append(create_wp(segment.start, segment.start_properties));
      // ROS_INFO_STREAM("add waypoints OK");
    } else if (same_point(segment.end, segment_pre.end)) {
      // ROS_INFO_STREAM("Case 4: end == end");
      if (point_not_match(segment.end))
        waypoints.append(create_wp(segment.end, segment.end_properties));
      waypoints.append(create_wp(segment.start, segment.start_properties));
      // ROS_INFO_STREAM("add waypoints OK");
    } else {
      ROS_INFO_STREAM("Không khớp điều kiện nào tại segment["
                      << i << "] -> set default end");
    }

    if (waypoints.size() > 1) {
      const auto wp_last = waypoints[waypoints.size() - 1];
      const auto wp_prev = waypoints[waypoints.size() - 2];

      try {
        if (!wp_last.isMember("position") ||
            !wp_last["position"].isMember("position") ||
            !wp_last["position"]["position"].isMember("x") ||
            !wp_last["position"]["position"].isMember("y") ||
            !wp_prev.isMember("position") ||
            !wp_prev["position"].isMember("position") ||
            !wp_prev["position"]["position"].isMember("x") ||
            !wp_prev["position"]["position"].isMember("y")) {
          ROS_INFO_STREAM("Waypoint missing position/position/x/y field, "
                          "skipping duplicate check!");
          continue;
        }

        double last_x = wp_last["position"]["position"]["x"].asDouble();
        double prev_x = wp_prev["position"]["position"]["x"].asDouble();
        double last_y = wp_last["position"]["position"]["y"].asDouble();
        double prev_y = wp_prev["position"]["position"]["y"].asDouble();

        if (std::abs(last_x - prev_x) < 1e-6 &&
            std::abs(last_y - prev_y) < 1e-6) {
          ROS_INFO_STREAM("Duplicate waypoint detected at end of segment["
                          << i << "], removing.");

          if (waypoints.size() > 0) {
            Json::Value removed;
            waypoints.removeIndex(waypoints.size() - 1, &removed);
            ROS_INFO_STREAM("Removed last waypoint successfully.");
          } else {
            ROS_INFO_STREAM("Waypoints list is already empty. Cannot remove.");
          }
        }
      } catch (const std::exception &e) {
        ROS_INFO_STREAM(
            "Exception caught while accessing waypoint fields: " << e.what());
      }
    }
  }

  // remove key is_reserve in first waypoints

  // if (!waypoints.empty() && waypoints[0].isMember("data") &&
  //     waypoints[0]["data"].isMember("is_reserve")) {
  //   ROS_INFO_STREAM("Removing is_reserve from waypoints[0]");
  //   waypoints[0]["data"].removeMember("is_reserve");
  // }
  ROS_INFO_STREAM(
      "Finished converting route. Total waypoints: " << waypoints.size());
  root["waypoints"] = waypoints;
  return root;
}

Json::Value Server::getRobotRouteJson(const std::string &robot_name_req) {
  Json::Value responseJson;
  bool found = false;
  tuw_multi_robot_msgs::Route route_msg;

  for (const auto &route : route_array_msg.routes) {
    if (route.robot_name == robot_name_req) {
      route_msg = route;
      found = true;
      break;
    }
  }
  // them code xac nhan neu route moi khac route cu thi moi gui route
  if (found) {
    // Kiểm tra xem robot_name đã có trong previous_routes chưa
    if (previous_routes.find(route_msg.robot_name) == previous_routes.end()) {
      // Nếu chưa có, lưu lại route_msg
      previous_routes[route_msg.robot_name] = route_msg;

      // Chuyển route_msg thành JSON và gửi đi
      responseJson = convertRouteMsgToJson(route_msg);
    } else {
      // Nếu có, so sánh với route_msg cũ
      if (route_msg != previous_routes[route_msg.robot_name]) {
        // Nếu có sự khác biệt, cập nhật route_msg và gửi lại
        previous_routes[route_msg.robot_name] = route_msg;

        // Chuyển route_msg thành JSON và gửi đi
        responseJson = convertRouteMsgToJson(route_msg);
      } else {
        auto it = request_get_last_path.find(route_msg.robot_name);
        if (it != request_get_last_path.end() && it->second) {
          responseJson = convertRouteMsgToJson(route_msg);
        } else {
          responseJson = "DONT HAVE NEW PATH";
        }
      }
    }
    request_get_last_path[route_msg.robot_name] = false;
  } else {
    responseJson = "DONT HAVE NEW PATH";
  }

  return responseJson;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "server_traffic_control");
  signal(SIGINT, sigintHandler);
  Server server;
  server.startServer();
  std::cout << "Server started. Now listening..." << std::endl;
  ros::spin();
  // ✅ Đợi server thread kết thúc sau khi ROS shutdown
  server.joinServerThread();

  return 0;
}
