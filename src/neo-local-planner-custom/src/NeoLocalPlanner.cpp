/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "../include/NeoLocalPlanner.h"

#include <agv_msgs/DataMatrixStamped.h>
#include <base_local_planner/footprint_helper.h>
#include <base_local_planner/goal_functions.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <algorithm>

// register this planner as a BaseGlobalPlanner plugin
// (see
// http://www.ros.org/wiki/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin)
PLUGINLIB_EXPORT_CLASS(neo_local_planner::NeoLocalPlanner,
                       nav_core::BaseLocalPlanner)

namespace neo_local_planner {

/*
   ########  ########  ######   #######  ##    ## ######## ####  ######
   ##     ## ##       ##    ## ##     ## ###   ## ##        ##  ##    ##
   ##     ## ##       ##       ##     ## ####  ## ##        ##  ##
   ########  ######   ##       ##     ## ## ## ## ######    ##  ##   ####
   ##   ##   ##       ##       ##     ## ##  #### ##        ##  ##    ##
   ##    ##  ##       ##    ## ##     ## ##   ### ##        ##  ##    ##
   ##     ## ########  ######   #######  ##    ## ##       ####  ######
   */

void NeoLocalPlanner::reconfigureCB(NeoPlannerConfig &config, uint32_t level) {
  // update generic local planner params
  m_limits.max_vel_x = config.max_vel_x;
  m_limits.max_vel_trans = config.max_vel_trans;
  m_limits.acc_lim_x = config.acc_lim_x;
  m_stop_acc = config.stop_acc;
  m_emergency_acc_lim_x = config.emergency_acc_lim_x;
  m_limits.max_vel_theta = config.max_rot_vel;
  m_limits.acc_lim_theta = config.acc_lim_theta;
  m_emergency_acc_lim_theta = config.emergency_acc_lim_theta;
  publish_safety = config.publish_safety;
  slow_speed = config.slow_speed;
  stop_center_qr = config.stop_center_qr;
}

/*
   ######## ##     ## ##    ##  ######  ######## ####  #######  ##    ##
   ##       ##     ## ###   ## ##    ##    ##     ##  ##     ## ###   ##
   ##       ##     ## ####  ## ##          ##     ##  ##     ## ####  ##
   ######   ##     ## ## ## ## ##          ##     ##  ##     ## ## ## ##
   ##       ##     ## ##  #### ##          ##     ##  ##     ## ##  ####
   ##       ##     ## ##   ### ##    ##    ##     ##  ##     ## ##   ###
   ##        #######  ##    ##  ######     ##    ####  #######  ##    ##
   */

void NeoLocalPlanner::publishStateDirection(const std::string &state,
                                            const std::string &direction) {
  std_stamped_msgs::StringStamped msg;
  msg.stamp = ros::Time::now();

  nlohmann::json status_dict = {{"state", state}, {"direction", direction}};

  msg.data = status_dict.dump();
  pub_state_direction_.publish(msg);
}

double NeoLocalPlanner::GetNormaliceAngle(double angle) {
  // ---------------------------------------------------------------------
  // Normalize an angle to [-pi, pi].
  // :param angle: (double)
  // :return: (double) double in radian in [-pi, pi]
  // ---------------------------------------------------------------------
  if (angle > M_PI)
    angle -= 2 * M_PI;
  if (angle < -M_PI)
    angle += 2 * M_PI;
  return angle;
}

tf2::Quaternion createQuaternionFromYaw(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return q;
}

std::vector<tf2::Transform>::const_iterator
find_closest_point(std::vector<tf2::Transform>::const_iterator begin,
                   std::vector<tf2::Transform>::const_iterator end,
                   const tf2::Vector3 &pos, double *actual_dist = 0) {
  auto iter_short = begin;
  double dist_short = std::numeric_limits<double>::infinity();

  for (auto iter = iter_short; iter != end; ++iter) {
    const double dist = (iter->getOrigin() - pos).length();
    if (dist < dist_short) {
      dist_short = dist;
      iter_short = iter;
    }
  }
  if (actual_dist) {
    *actual_dist = dist_short;
  }
  return iter_short;
}

std::vector<tf2::Transform>::const_iterator
move_along_path(std::vector<tf2::Transform>::const_iterator begin,
                std::vector<tf2::Transform>::const_iterator end,
                const double dist, double *actual_dist = 0) {
  auto iter = begin;
  auto iter_prev = iter;
  double dist_left = dist;

  while (iter != end) {
    const double dist = (iter->getOrigin() - iter_prev->getOrigin()).length();
    dist_left -= dist;
    if (dist_left <= 0) {
      break;
    }
    iter_prev = iter;
    iter++;
  }
  if (iter == end) {
    iter = iter_prev; // targeting final pose
  }
  if (actual_dist) {
    *actual_dist = dist - dist_left;
  }
  return iter;
}

std::vector<base_local_planner::Position2DInt>
get_line_cells(costmap_2d::Costmap2D *cost_map, const tf2::Vector3 &world_pos_0,
               const tf2::Vector3 &world_pos_1) {
  int coords[2][2] = {};
  cost_map->worldToMapEnforceBounds(world_pos_0.x(), world_pos_0.y(),
                                    coords[0][0], coords[0][1]);
  cost_map->worldToMapEnforceBounds(world_pos_1.x(), world_pos_1.y(),
                                    coords[1][0], coords[1][1]);

  std::vector<base_local_planner::Position2DInt> cells;
  base_local_planner::FootprintHelper().getLineCells(
      coords[0][0], coords[1][0], coords[0][1], coords[1][1], cells);
  return cells;
}

double get_cost(costmap_2d::Costmap2DROS *cost_map_ros,
                const tf2::Vector3 &world_pos) {
  auto cost_map = cost_map_ros->getCostmap();

  int coords[2] = {};
  cost_map->worldToMapEnforceBounds(world_pos.x(), world_pos.y(), coords[0],
                                    coords[1]);

  return cost_map->getCost(coords[0], coords[1]) / 255.;
}

double compute_avg_line_cost(costmap_2d::Costmap2DROS *cost_map_ros,
                             const tf2::Vector3 &world_pos_0,
                             const tf2::Vector3 &world_pos_1) {
  auto cost_map = cost_map_ros->getCostmap();
  const std::vector<base_local_planner::Position2DInt> cells =
      get_line_cells(cost_map, world_pos_0, world_pos_1);

  double avg_cost = 0;
  for (auto cell : cells) {
    avg_cost += cost_map->getCost(cell.x, cell.y) / 255.;
  }
  return avg_cost / cells.size();
}

double compute_max_line_cost(costmap_2d::Costmap2DROS *cost_map_ros,
                             const tf2::Vector3 &world_pos_0,
                             const tf2::Vector3 &world_pos_1) {
  auto cost_map = cost_map_ros->getCostmap();
  const std::vector<base_local_planner::Position2DInt> cells =
      get_line_cells(cost_map, world_pos_0, world_pos_1);

  int max_cost = 0;
  for (auto cell : cells) {
    max_cost = std::max(max_cost, int(cost_map->getCost(cell.x, cell.y)));
  }
  return max_cost / 255.;
}

NeoLocalPlanner::NeoLocalPlanner() {}

NeoLocalPlanner::~NeoLocalPlanner() {}

bool NeoLocalPlanner::getSucceedPose(int type_stop,
                                     bool m_is_goal_reached_by_qr_code,
                                     double xy_goal_tolerance, double x_error,
                                     double y_error, double start_to_goal) {
  bool succeed_pose = false;
  if (type_stop == 0 && m_is_goal_reached_by_qr_code) {
    if (x_error < xy_goal_tolerance) {
      succeed_pose = true;
    }
  } else if (type_stop == 0 && !m_is_goal_reached_by_qr_code) {
    if (x_error < xy_goal_tolerance - max_dis_to_find_qr) {
      succeed_pose = true;
    }
  } else if (type_stop == 1 && m_is_goal_reached_by_qr_code) {
    if (-x_error < xy_goal_tolerance) {
      succeed_pose = true;
    }
  } else if (type_stop == 1 && !m_is_goal_reached_by_qr_code) {
    if (-x_error < xy_goal_tolerance - max_dis_to_find_qr) {
      succeed_pose = true;
    }
  } else if (type_stop == 2 && m_is_goal_reached_by_qr_code) {
    if (y_error < xy_goal_tolerance) {
      succeed_pose = true;
    }
  } else if (type_stop == 2 && !m_is_goal_reached_by_qr_code) {
    if (y_error < xy_goal_tolerance - max_dis_to_find_qr) {
      succeed_pose = true;
    }
  } else if (type_stop == 3 && m_is_goal_reached_by_qr_code) {
    if (-y_error < xy_goal_tolerance) {
      succeed_pose = true;
    }
  } else if (type_stop == 3 && !m_is_goal_reached_by_qr_code) {
    if (-y_error < xy_goal_tolerance - max_dis_to_find_qr) {
      succeed_pose = true;
    }
  }
  if (start_to_goal < 0.1 && m_xy_error < 0.03) {
    succeed_pose = true;
  }
  return succeed_pose;
}

/*
    ######     ###    ##               ##     ## ######## ##
   ##    ##   ## ##   ##               ##     ## ##       ##
   ##        ##   ##  ##               ##     ## ##       ##
   ##       ##     ## ##               ##     ## ######   ##
   ##       ######### ##                ##   ##  ##       ##
   ##    ## ##     ## ##                 ## ##   ##       ##
    ######  ##     ## ######## #######    ###    ######## ########
   */

bool NeoLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  boost::mutex::scoped_lock lock(m_odometry_mutex);

  // compute delta time
  const ros::WallTime time_now = ros::WallTime::now();
  const double dt = fmax(fmin((time_now - m_last_time).toSec(), 0.1), 0);

  if (m_state != m_pre_state) {
    m_pre_state = m_state;
    if (m_print_state) {
      if (m_state == state_t::STATE_IDLE)
        ROS_INFO_NAMED("NeoLocalPlanner", "Current State: STATE_IDLE");
      else if (m_state == state_t::STATE_TRANSLATING)
        ROS_INFO_NAMED("NeoLocalPlanner", "Current State: STATE_TRANSLATING");
      else if (m_state == state_t::STATE_ROTATING)
        ROS_INFO_NAMED("NeoLocalPlanner", "Current State: STATE_ROTATING");
      else if (m_state == state_t::STATE_ADJUSTING)
        ROS_INFO_NAMED("NeoLocalPlanner", "Current State: STATE_ADJUSTING");
      else if (m_state == state_t::STATE_TURNING)
        ROS_INFO_NAMED("NeoLocalPlanner", "Current State: STATE_TURNING");
      else if (m_state == state_t::STATE_STUCK)
        ROS_INFO_NAMED("NeoLocalPlanner", "Current State: STATE_STUCK");
      else
        ROS_INFO_NAMED("NeoLocalPlanner", "Current State: NONE");
    }
  }

  if (!m_odometry) {
    ROS_INFO_NAMED("NeoLocalPlanner", "Waiting for odometry ...");
    return false;
  }

  if (!m_safety_status) {
    ROS_INFO_NAMED("NeoLocalPlanner", "Waiting for safety status ...");
    return false;
  }

  if (m_global_plan.empty()) {
    ROS_INFO_NAMED("NeoLocalPlanner", "Global plan is empty!");
    return false;
  }

  if (m_set_zero_vel) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
  }
  if (count < 1) {
    if (type_move == "MIRROR") {
      m_global_frame = "odom";
    } else {
      m_global_frame = "map";
    }
    if (m_allow_reversing) {
      // auto point_1 =
      // tf2::toMsg(transformed_plan[transformed_plan.size() - 1]);

      // Estimate if the robot has travelled and then determine if the
      // path is reversed! ToDo Just checks if the goal is in the rear end
      // of the robot auto reverse_path = point_1.translation.x;
      auto reverse_path = m_global_plan.front().pose.orientation.x;
      m_robot_direction = reverse_path > 0.0 ? -1.0 : 1.0;
      if (m_robot_direction == 1) {
        ROS_INFO_NAMED("NeoLocalPlanner", "MOVE FORWARD");
        safety_job.data = "FORWARD";
      } else {
        ROS_INFO_NAMED("NeoLocalPlanner", "MOVE BACKWARD");
        safety_job.data = "BACKWARD";
      }
    } else {
      ROS_INFO_NAMED("NeoLocalPlanner", "MOVE FORWARD");
      safety_job.data = "FORWARD";
    }
    if (publish_safety) {
      m_set_safety_job_pub.publish(safety_job);
    }
    ROS_INFO_NAMED("NeoLocalPlanner", "SET FOOTPRINT DEFAULT");
    safety_footprint.data = "amr_run_alone";
    m_set_footprint_job_pub.publish(safety_footprint);
    safety_move_straight_name = safety_job.data;
    count++;
  }

  // get latest global to local transform (map to odom)
  tf2::Stamped<tf2::Transform> global_to_local;
  try {
    auto msg = m_tf->lookupTransform(m_local_frame, m_global_frame, ros::Time(),
                                     ros::Duration(0));
    tf2::fromMsg(msg, global_to_local);
  } catch (...) {
    ROS_INFO_NAMED("NeoLocalPlanner",
                   "lookupTransform(m_local_frame, m_global_frame) failed");
    return false;
  }

  tf2::Stamped<tf2::Transform> global_to_robot;
  try {
    auto msg = m_tf->lookupTransform(m_base_frame, m_global_frame, ros::Time(),
                                     ros::Duration(0));
    tf2::fromMsg(msg, global_to_robot);
  } catch (...) {
    ROS_INFO_NAMED("NeoLocalPlanner",
                   "lookupTransform(base_link, m_global_frame) failed");
    return false;
  }

  // transform plan to local frame (odom)
  std::vector<tf2::Transform> local_plan;
  std::vector<tf2::Transform> transformed_plan;

  // check reach goal
  tf2::Stamped<tf2::Transform> goal_pose_global;
  tf2::fromMsg(m_global_plan.back(), goal_pose_global);
  const auto goal_pose_local = global_to_local * goal_pose_global;

  // get robot pose in global frame
  geometry_msgs::PoseStamped robot_pose;
  m_cost_map->getRobotPose(robot_pose);

  // kiem tra xem dung theo truc y hay dung theo truc x
  // co the cai tien them la tinh goc cua diem xuat phat va diem ket thuc
  // sau do tinh ra truc tinh toan de dung
  double x_error, y_error, x_distance_to_start_pose, y_distance_to_start_pose,
      distance_robot_to_start_pose;
  if (type_move != "MIRROR") {
    x_distance_to_start_pose =
        robot_pose.pose.position.x - pose_begin_in_path_.pose.position.x;
    y_distance_to_start_pose =
        robot_pose.pose.position.y - pose_begin_in_path_.pose.position.y;
    distance_robot_to_start_pose =
        sqrt(x_distance_to_start_pose * x_distance_to_start_pose +
             y_distance_to_start_pose * y_distance_to_start_pose);
    x_error = robot_pose.pose.position.x - global_goal_.pose.position.x;
    y_error = robot_pose.pose.position.y - global_goal_.pose.position.y;
  } else {
    x_distance_to_start_pose =
        m_odometry->pose.pose.position.x - pose_begin_in_path_.pose.position.x;
    y_distance_to_start_pose =
        m_odometry->pose.pose.position.y - pose_begin_in_path_.pose.position.y;
    distance_robot_to_start_pose =
        sqrt(x_distance_to_start_pose * x_distance_to_start_pose +
             y_distance_to_start_pose * y_distance_to_start_pose);
    x_error = m_odometry->pose.pose.position.x - global_goal_.pose.position.x;
    y_error = m_odometry->pose.pose.position.y - global_goal_.pose.position.y;
  }
  if (get_state_stop) {
    start_to_goal = sqrt(x_error * x_error + y_error * y_error);
    if (abs(x_error) > abs(y_error) &&
        robot_pose.pose.position.x > global_goal_.pose.position.x) {
      type_stop = 0;
    } else if (abs(x_error) > abs(y_error) &&
               robot_pose.pose.position.x <= global_goal_.pose.position.x) {
      type_stop = 1;
    } else if (abs(x_error) < abs(y_error) &&
               robot_pose.pose.position.y > global_goal_.pose.position.y) {
      type_stop = 2;
    } else {
      type_stop = 3;
    }
    get_state_stop = false;
  }
  double yaw_error_check_stop = 0;
  double yaw_error = 0;
  m_xy_error = ::hypot(
      m_odometry->pose.pose.position.x - goal_pose_local.getOrigin().x(),
      m_odometry->pose.pose.position.y - goal_pose_local.getOrigin().y());
  ROS_WARN("m_xy_error: %f", m_xy_error);
  yaw_error_check_stop = fabs(angles::shortest_angular_distance(
      tf2::getYaw(m_odometry->pose.pose.orientation),
      tf2::getYaw(goal_pose_local.getRotation())));

  // std::cout << "m_stop_by_qr_code" << m_stop_by_qr_code << ", m_xy_error: "
  //           << m_xy_error << ", dis_check_qr_for_stop: " <<
  //           dis_check_qr_for_stop
  //           << ", m_detect_qr_code: " << m_detect_qr_code << "type_stop: "
  //           << type_stop << std::endl;
  double x_distance_last_qr_to_robot =
      last_label_qr_x - robot_pose.pose.position.x;
  double y_distance_last_qr_to_robot =
      last_label_qr_y - robot_pose.pose.position.y;
  double distance_last_qr_to_robot =
      sqrt(x_distance_last_qr_to_robot * x_distance_last_qr_to_robot +
           y_distance_last_qr_to_robot * y_distance_last_qr_to_robot);
  if (distance_last_qr_to_robot > 1) {
    decrease_vel_when_not_detect_qr = true;
  } else {
    decrease_vel_when_not_detect_qr = false;
  }
  double x_error_last_qr_to_goal =
      last_label_qr_x - global_goal_.pose.position.x;
  double y_error_last_qr_to_goal =
      last_label_qr_y - global_goal_.pose.position.y;
  double dis_error_last_qr_to_goal =
      sqrt(x_error_last_qr_to_goal * x_error_last_qr_to_goal +
           y_error_last_qr_to_goal * y_error_last_qr_to_goal);
  if (m_stop_by_qr_code) {
    if (m_xy_error < dis_check_qr_for_stop && m_detect_qr_code &&
        dis_error_last_qr_to_goal < 0.08) {
      ROS_ERROR_THROTTLE(1, "STOP BY QR");
      m_is_goal_reached_by_qr_code = true;
    }
    if (m_xy_error >= dis_check_qr_for_stop) {
      m_detect_qr_code = false;
    }
  }

  // // check success

  for (const auto &pose : m_global_plan) {
    tf2::Stamped<tf2::Transform> pose_;
    tf2::Stamped<tf2::Transform> robot_pose_;
    tf2::fromMsg(pose, pose_);
    tf2::fromMsg(pose, robot_pose_);
    local_plan.push_back(global_to_local * pose_);
    transformed_plan.push_back(global_to_robot * robot_pose_);
  }

  // get latest local pose
  tf2::Transform local_pose;
  tf2::fromMsg(m_odometry->pose.pose, local_pose);

  const double start_yaw = tf2::getYaw(local_pose.getRotation());
  const double start_vel_x = m_odometry->twist.twist.linear.x;
  const double start_vel_y = m_odometry->twist.twist.linear.y;
  const double start_yawrate = m_odometry->twist.twist.angular.z;

  // calc dynamic lookahead distances
  const double lookahead_dist =
      m_lookahead_dist + fmax(fabs(start_vel_x), 0) * m_lookahead_time;
  const double cost_y_lookahead_dist =
      m_cost_y_lookahead_dist + fmax(start_vel_x, 0) * m_cost_y_lookahead_time;

  // predict future pose (using second order midpoint method)
  tf2::Vector3 actual_pos;
  tf2::Vector3 ori_pos;
  tf2::Vector3 actual_pos_lookahead_stop;
  tf2::Vector3 actual_pos_lookahead_stop_when_near_goal;
  ori_pos = local_pose.getOrigin();
  double actual_yaw = 0;
  {
    const double midpoint_yaw =
        start_yaw + start_yawrate * m_lookahead_time / 2;
    actual_pos = local_pose.getOrigin() +
                 tf2::Matrix3x3(createQuaternionFromYaw(midpoint_yaw)) *
                     tf2::Vector3(start_vel_x, start_vel_y, 0) *
                     m_lookahead_time;
    actual_yaw = start_yaw + start_yawrate * m_lookahead_time;
  }

  {
    if (m_limits.max_vel_trans > 0.5) {
      max_goal_dist_dynamic = m_max_goal_dist;
    } else {
      max_goal_dist_dynamic = 0.2;
    }
    m_dynamic_lookahead_time_stop =
        0.3 + (std::min(1.0, std::max(0.0, fabs(m_limits.max_vel_trans)))) *
                  (0.6 - 0.3);
  }

  double actual_yaw_lookahead_stop = 0;
  {
    const double midpoint_yaw_lookahead_stop =
        start_yaw + start_yawrate * m_lookahead_time_stop / 2;
    actual_pos_lookahead_stop_when_near_goal =
        local_pose.getOrigin() +
        tf2::Matrix3x3(createQuaternionFromYaw(midpoint_yaw_lookahead_stop)) *
            tf2::Vector3(start_vel_x, start_vel_y, 0) *
            m_dynamic_lookahead_time_stop;
    actual_pos_lookahead_stop =
        local_pose.getOrigin() +
        tf2::Matrix3x3(createQuaternionFromYaw(midpoint_yaw_lookahead_stop)) *
            tf2::Vector3(start_vel_x, start_vel_y, 0) * m_lookahead_time_stop;
    actual_yaw_lookahead_stop =
        start_yaw + start_yawrate * m_lookahead_time_stop;
  }

  const bool obstacle_in_rot = 0;

  // compute cost gradients
  const double delta_x = 0.3;
  const double delta_y = 0.2;
  const double delta_yaw = 0.1;

  const double center_cost = 0;

  const double delta_cost_x = 0;

  const double delta_cost_y = 0;

  const double delta_cost_yaw = 0;

  // fill local plan later
  nav_msgs::Path::Ptr local_path = boost::make_shared<nav_msgs::Path>();
  local_path->header.frame_id = m_local_frame;
  local_path->header.stamp = m_odometry->header.stamp;

  // compute obstacle distance
  bool have_obstacle = false;
  double obstacle_dist = 10;
  double obstacle_cost = 0;

  // publish local plan
  m_local_plan_pub.publish(local_path);

  // find closest point on path to future position
  auto iter_target = find_closest_point(local_plan.cbegin(), local_plan.cend(),
                                        actual_pos_lookahead_stop);

  // check if goal target
  bool is_goal_target = false;
  {
    // check if goal is within reach
    auto iter_next =
        move_along_path(iter_target, local_plan.cend(), max_goal_dist_dynamic);
    is_goal_target = iter_next + 1 >= local_plan.cend();

    if (is_goal_target) {
      // go straight to goal
      iter_target = iter_next;
    }
  }

  // figure out target orientation
  double target_yaw = 0;

  //   if (is_goal_target) {
  //     // take goal orientation
  //     auto iter_next =
  //         move_along_path(iter_target, local_plan.cend(), lookahead_dist);
  //     if (succeed_pose) {
  //       target_yaw = tf2::getYaw(iter_target->getRotation());
  //     } else {
  //       target_yaw = tf2::getYaw(iter_next->getRotation());
  //     }
  //   }
  if (is_goal_target) {
    // take goal orientation
    target_yaw = tf2::getYaw(iter_target->getRotation());
  } else {
    // compute path based target orientation
    auto iter_next =
        move_along_path(iter_target, local_plan.cend(), lookahead_dist);
    target_yaw =
        ::atan2(iter_next->getOrigin().y() - iter_target->getOrigin().y(),
                iter_next->getOrigin().x() - iter_target->getOrigin().x());
  }

  // get target position
  const tf2::Vector3 target_pos = iter_target->getOrigin();

  double yaw_error_ori = 0;
  double yaw_error_lookahead_stop = 0;

  if (m_robot_direction == 1 or is_goal_target) {
    yaw_error = angles::shortest_angular_distance(actual_yaw, target_yaw);
    yaw_error_ori = angles::shortest_angular_distance(start_yaw, target_yaw);
    yaw_error_lookahead_stop = angles::shortest_angular_distance(
        actual_yaw_lookahead_stop, target_yaw);
  } else {
    yaw_error =
        angles::shortest_angular_distance(actual_yaw + 3.14, target_yaw);
    yaw_error_ori =
        angles::shortest_angular_distance(start_yaw + 3.14, target_yaw);
    yaw_error_lookahead_stop = angles::shortest_angular_distance(
        actual_yaw_lookahead_stop + 3.14, target_yaw);
  }

  // compute errors
  const double goal_dist =
      (local_plan.back().getOrigin() - actual_pos).length();
  const double goal_dist_origin =
      (local_plan.back().getOrigin() - ori_pos).length();
  const double goal_dist_lookahead_stop =
      (local_plan.back().getOrigin() - actual_pos_lookahead_stop_when_near_goal)
          .length();
  const double dist_to_begin =
      (local_plan.front().getOrigin() - ori_pos).length();
  const tf2::Vector3 pos_error =
      tf2::Transform(createQuaternionFromYaw(actual_yaw), actual_pos)
          .inverse() *
      target_pos;
  const tf2::Vector3 pos_error_ori =
      tf2::Transform(createQuaternionFromYaw(target_yaw), ori_pos).inverse() *
      target_pos;
  const tf2::Vector3 pos_error_lookahead_stop =
      tf2::Transform(createQuaternionFromYaw(target_yaw),
                     actual_pos_lookahead_stop)
          .inverse() *
      target_pos;

  // pub error
  error_msg.error_angle = yaw_error_ori * 180 / M_PI;
  error_msg.error_position = pos_error_ori.y();
  error_to_path_pub.publish(error_msg);

  // check success
  bool succeed_pose = getSucceedPose(type_stop, m_is_goal_reached_by_qr_code,
                                     m_limits.xy_goal_tolerance, x_error,
                                     y_error, start_to_goal);
  if (type_move == "LIDAR") {
    double pose_error_ori_x = pos_error_ori.x();
    ROS_WARN("pose_error_ori_x: %f", pose_error_ori_x);
    bool pass_goal = false;
    if (m_robot_direction == 1) {
      if (pose_error_ori_x < 0.03) {
        pass_goal = true;
      }
    } else {
      if (pose_error_ori_x > -0.03) {
        pass_goal = true;
      }
    }
    m_is_goal_reached = m_xy_error < 0.3 && pass_goal;
  } else if (type_move == "MIRROR") {
    double pose_error_ori_x = pos_error_ori.x();
    ROS_WARN("pose_error_ori_x: %f", pose_error_ori_x);
    bool pass_goal = false;
    if (m_robot_direction == 1) {
      if (pose_error_ori_x < 0.025) {
        pass_goal = true;
      }
    } else {
      if (pose_error_ori_x > -0.025) {
        pass_goal = true;
      }
    }
    // m_is_goal_reached = m_xy_error < 0.1 && pass_goal;
    m_is_goal_reached = (m_xy_error < 0.1 && pass_goal &&
                         yaw_error_check_stop < m_limits.yaw_goal_tolerance) ||
                        (start_to_goal < 0.05 &&
                         yaw_error_check_stop < m_limits.yaw_goal_tolerance);
  } else {
    m_is_goal_reached =
        (succeed_pose && yaw_error_check_stop < m_limits.yaw_goal_tolerance) ||
        (start_to_goal < 0.05 &&
         yaw_error_check_stop < m_limits.yaw_goal_tolerance) ||
        (m_is_goal_reached_by_qr_code && stop_center_qr && dis_qr_y > -0.005 &&
         start_to_goal > 0.1 && m_robot_direction == 1) ||
        (m_is_goal_reached_by_qr_code && stop_center_qr && dis_qr_y < 0.005 &&
         start_to_goal > 0.1 && m_robot_direction == -1);
  }
  if (m_is_goal_reached) {
    ROS_INFO(
        "x_error=%f, y_error=%f, m_limits.xy_goal_tolerance=%f, "
        "yaw_error=%f, m_limits.yaw_goal_tolerance=%f, "
        "m_is_goal_reached_by_qr_code=%d, dis_qr_x=%f, dis_qr_y=%f, "
        "stop_center_qr=%d, type_move=%s, m_robot_direction=%f, type_stop=%d, "
        "robot_pose=(%f, %f), global_goal=(%f, %f)",
        x_error, y_error, m_limits.xy_goal_tolerance, yaw_error_check_stop,
        m_limits.yaw_goal_tolerance, m_is_goal_reached_by_qr_code, dis_qr_x,
        dis_qr_y, stop_center_qr, type_move.c_str(), m_robot_direction,
        type_stop, robot_pose.pose.position.x, robot_pose.pose.position.y,
        global_goal_.pose.position.x, global_goal_.pose.position.y);

    if (!m_is_goal_reached_by_qr_code && receive_new_data_qr) {
      ROS_ERROR_THROTTLE(10, "STOP AT GOAL BUT NOT FOUND QR CODE");
      geometry_msgs::Pose lost_label_goal;
      lost_label_goal.position.x = global_goal_.pose.position.x;
      lost_label_goal.position.y = global_goal_.pose.position.y;

      if (current_action_type != "un_docking" && type_move == "QR") {
        lost_goal_label_pub.publish(lost_label_goal);
      }
    }
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return true;
  }

  if (check_error_angle_begin) {
    check_error_angle_begin = false;
    if (fabs(yaw_error_ori * 180 / M_PI) > 20) {
      check_have_qr_start_move = true;
    }
  }

  // compute control values
  bool stop_smoother_by_pause = false;
  bool is_emergency_brake = false;
  bool is_emergency_brake_only_vel_x = false;
  double control_vel_x = 0;
  double control_vel_y = 0;
  double control_yawrate = 0;
  double start_yaw_error = 0;

  if (is_goal_target) {
    // use term for final stopping position
    is_emergency_brake = true;
    start_yaw_error = m_start_yaw_error_when_safety;
    if (m_state != state_t::STATE_TRANSLATING &&
        m_state != state_t::STATE_ADJUSTING &&
        fabs(yaw_error_ori) > start_yaw_error) {
      control_vel_x = 0;
      time_wait_for_move_straight_first = time_now;
      if (fabs(yaw_error_ori * 180 / M_PI) > 20) {
        disable_low_pass_filter_move_rotate = true;
        // ROS_WARN("disable_low_pass_filter_move_rotate");
      }
      if (first_time_check) {
        first_time_check = false;
        is_rotation_first = true;
        if (fabs(yaw_error_ori * 180 / M_PI) > 20) {
          safety_job.data = "ROTATION";
          if (publish_safety) {
            m_set_safety_job_pub.publish(safety_job);
          }
        }
      }
    } else {
      control_vel_x = pos_error_lookahead_stop.x() * m_pos_x_gain;
      if (m_robot_direction == 1) {
        control_vel_x = fmax(min_vel_move_straight, control_vel_x);
        if (current_action_type == "un_docking") {
          control_vel_x = fmax(0.1, control_vel_x);
        }
      } else {
        control_vel_x = fmin(-min_vel_move_straight, control_vel_x);
        if (current_action_type == "un_docking") {
          control_vel_x = fmin(-0.1, control_vel_x);
        }
      }
      if (m_stop_by_qr_code && m_xy_error < m_tolerance_stop_by_qr_code) {
        control_vel_x = fmax(fabs(control_vel_x), m_min_vel_stop_by_qr_code) *
                        m_robot_direction;
      }
      disable_low_pass_filter_move_rotate = false;
      if (is_rotation_first) {
        safety_job.data = safety_move_straight_name;
        if (publish_safety) {
          m_set_safety_job_pub.publish(safety_job);
        }
        if ((time_now - time_wait_for_move_straight_first).toSec() < 0.2) {
          control_vel_x = 0;
        } else {
          is_rotation_first = false;
          succeed_rotate_first = true;
          cmd_vel.linear.x = 0.0;
          cmd_vel.angular.z = 0.0;
          return true;
        }
      }
    }
  } else {
    if (safety_field_idx == 0 || safety_camera_field_idx == 0) {
      m_last_time_stop_by_safety = time_now;
      start_yaw_error = m_start_yaw_error_when_safety;
    } else {
      start_yaw_error = m_start_yaw_error_normal;
    }
    if (safety_field_idx == 1 || safety_camera_field_idx == 1) {
      m_last_time_stop_by_safety_2 = time_now;
    }
    if (safety_field_idx == 2 || safety_camera_field_idx == 2) {
      m_last_time_stop_by_safety_3 = time_now;
    }
    control_vel_x = m_robot_direction * m_limits.max_vel_trans;
    double angel_error_allow_move;
    if (type_move == "LIDAR") {
      angel_error_allow_move = 0.1;
    } else {
      angel_error_allow_move = start_yaw_error;
    }
    // wait to start moving
    if (m_state != state_t::STATE_TRANSLATING &&
        fabs(yaw_error_ori) > angel_error_allow_move) {
      control_vel_x = 0;
      ROS_WARN("nhay vao dieu kien nay");
      time_wait_for_move_straight_first = time_now;
      if (fabs(yaw_error_ori * 180 / M_PI) > 20) {
        disable_low_pass_filter_move_rotate = true;
        // ROS_WARN("disable_low_pass_filter_move_rotate");
      }
      if (first_time_check) {
        first_time_check = false;
        is_rotation_first = true;
        if (fabs(yaw_error_ori * 180 / M_PI) > 20) {
          safety_job.data = "ROTATION";
          if (publish_safety) {
            m_set_safety_job_pub.publish(safety_job);
          }
        }
      }
    } else {
      disable_low_pass_filter_move_rotate = false;
      if (is_rotation_first) {
        safety_job.data = safety_move_straight_name;
        if ((time_now - time_wait_for_move_straight_first).toSec() < 0.2) {
          control_vel_x = 0;
        } else {
          if (publish_safety) {
            m_set_safety_job_pub.publish(safety_job);
          }
          is_rotation_first = false;
        }
      }
    }

    // limit curve velocity
    {
      // nếu muốn giảm tốc nhanh thì thêm k*yaw_error
      if (fabs(yaw_error_ori * 180 / M_PI) > 5) {
        const double max_vel_x = 0.6;
        is_emergency_brake = true;
        if (m_state == state_t::STATE_TRANSLATING) {
          is_emergency_brake_only_vel_x = true;
        }
        if (m_robot_direction == -1.0) {
          control_vel_x =
              m_robot_direction * fmin(fabs(control_vel_x), max_vel_x);
        } else {
          control_vel_x = fmin(control_vel_x, max_vel_x);
        }
      }
    }

    // limit velocity when approaching goal position
    if (fabs(start_vel_x) > 0) {
      // càng xa goal thì stop_time càng lớn
      const double stop_accel = m_stop_acc;
      const double stop_time =
          sqrt(2 * fmax(fabs(goal_dist_lookahead_stop), 0) / fabs(stop_accel));

      if (m_robot_direction == -1.0) {
        const double max_vel_x =
            m_robot_direction *
            fmax(fabs(stop_accel) * stop_time, m_limits.min_vel_trans);
        control_vel_x =
            m_robot_direction * fmin(fabs(control_vel_x), fabs(max_vel_x));
      } else {
        // nếu stop_time càng nhỏ hay càng gần đích thì max_vel_x càng
        // giảm
        const double max_vel_x =
            fmax(stop_accel * stop_time, m_limits.min_vel_trans);
        control_vel_x = fmin(control_vel_x, max_vel_x);
      }
    }

    // limit velocity when approaching an obstacle
    if (have_obstacle && fabs(start_vel_x) > 0) {
      const double stop_accel = 4 * m_limits.acc_lim_x;
      const double stop_time = sqrt(fmax(obstacle_dist / 2, 0) / stop_accel);
      const double max_vel_x = stop_accel * stop_time / 2;

      // check if it's much lower than current velocity
      if (fabs(max_vel_x) < 0.5 * fabs(start_vel_x)) {
        is_emergency_brake = true;
      }

      if (m_robot_direction == -1.0) {
        control_vel_x =
            m_robot_direction * fmin(fabs(control_vel_x), max_vel_x);
      } else {
        control_vel_x = fmin(control_vel_x, max_vel_x);
      }
    }

    // stop before hitting obstacle
    if (have_obstacle && obstacle_dist <= 0) {
      control_vel_x = 0;
    }

    // only allow forward velocity depending on the parameter setting
    if (!m_allow_reversing) {
      control_vel_x = fmax(control_vel_x, 0);
    }
  }

  if (check_have_qr_start_move && type_move == "QR") {
    if ((time_now - last_time_detect_qr_code).toSec() < 0.2) {
      if (fabs(pos_error_ori.y()) < 0.05) {
        last_angle_error_found_qr_code = fabs(yaw_error_ori);
      } else {
        last_angle_error_found_qr_code = 1000;
      }
    }
    if (fabs(yaw_error_ori) < 0.1 && control_vel_x != 0) {
      if (distance_robot_to_start_pose > 0.3) {
        if ((time_now - last_time_detect_qr_code).toSec() >= 2 &&
            last_angle_error_found_qr_code > 1) {
          // pub error
          ROS_ERROR_THROTTLE(10, "NOT FOUND QR AT START POSITION");
          geometry_msgs::Pose lost_label_goal;
          lost_label_goal.position.x = global_goal_.pose.position.x;
          lost_label_goal.position.y = global_goal_.pose.position.y;
          lost_goal_label_pub.publish(lost_label_goal);
        } else {
          check_have_qr_start_move = false;
        }
      } else {
        if ((time_now - last_time_detect_qr_code).toSec() >= 2 &&
            last_angle_error_found_qr_code > 1) {
        } else {
          check_have_qr_start_move = false;
        }
      }
    }
  }
  // limit backing up
  if (is_goal_target && m_max_backup_dist > 0 &&
      fabs(pos_error.x()) <
          (m_state == state_t::STATE_TURNING ? 0 : -1 * m_max_backup_dist)) {
    control_vel_x = 0;
    m_state = state_t::STATE_TURNING;
  } else if (m_state == state_t::STATE_TURNING) {
    m_state = state_t::STATE_IDLE;
  }

  if (m_differential_drive) {
    // /*
    //  ********** *******       **     ****     **  ******** ** **
    //  ********** ********
    // /////**/// /**////**     ****   /**/**   /** **////// /** ****
    // /////**/// /**/////
    //     /**    /**   /**    **//**  /**//**  /**/**       /** **//**
    //     /**    /**
    //     /**    /*******    **  //** /** //** /**/*********/**        **
    //     //**
    //     /**    /*******
    //     /**    /**///**   **********/**  //**/**////////**/** **********
    //     /**
    //     /**////
    //     /**    /**  //** /**//////**/**   //****       /**/** /**//////**
    //     /**
    //     /**
    //     /**    /**   //**/**     /**/**    //*** ******** /********/**
    //     /**
    //     /**    /********
    //     //     //     // //      // //      /// ////////  //////// // //
    //     //     ////////
    // */

    if (fabs(start_vel_x) > m_limits.trans_stopped_vel) {
      // we are translating, use term for lane keeping
      // if (m_xy_error < m_tolerance_stop_by_qr_code && m_detect_qr_code)
      // {
      //   control_yawrate = pos_error.y() * m_pos_y_yaw_gain *
      //                     fabs(m_dist_agv_to_qr_code) * 100;
      //   if (fabs(m_dist_agv_to_qr_code) < 0.015)
      //   {
      //     control_yawrate = 0;
      //   }
      //   control_yawrate = 0;
      // }
      if (m_xy_error < 0.02) {
        control_yawrate = 0;
        // ROS_WARN_STREAM("NeoLocalPlanner control_yawrate trans near
        // goal =  "
        // << control_yawrate);
      } else {
        double vel_tuning;
        if (start_vel_x >= 0) {
          // vel_tuning = std::min(start_vel_x, 0.5);
          vel_tuning = std::max(0.2, std::min(start_vel_x, 0.5));

          control_yawrate = pos_error.y() * m_pos_y_yaw_gain / vel_tuning;
        } else {
          // vel_tuning = std::max(start_vel_x, -0.5);
          vel_tuning = std::max(-0.5, std::min(start_vel_x, -0.2));

          control_yawrate = pos_error.y() * m_pos_y_yaw_gain / vel_tuning;
        }
        // if (is_goal_target)
        // {
        // 	if (control_yawrate >= 0) {
        // 	  control_yawrate = std::min(control_yawrate, 0.03);
        // 	} else {
        // 	  control_yawrate = std::max(control_yawrate, -0.03);
        // 	}
        // }
        // std::cout << "control_yawrate_11: " << control_yawrate <<
        // std::endl;
      }

      if (!is_goal_target && control_yawrate != 0) {
        // additional term for lane keeping
        if (fabs(start_vel_x) > 0.05) {
          // ROS_ERROR_STREAM("NeoLocalPlanner control_yawrate
          // rotation not near goal and check vel max =  " <<
          // control_yawrate);
          control_yawrate += yaw_error * m_yaw_gain;
          // std::cout << "control_yawrate_22: " << yaw_error *
          // m_yaw_gain << std::endl; std::cout << "sum control yaw: "
          // << control_yawrate << std::endl;
        }

        // add cost terms
        // check cost when obstacle in safety field
        // if (safety_field_idx < 3 || safety_camera_field_idx < 3 ||
        // slow_speed) {
        //   control_yawrate -= delta_cost_y / start_vel_x *
        //   m_cost_y_yaw_gain; control_yawrate -= delta_cost_yaw *
        //   m_cost_yaw_gain;
        // }
        // if (fabs(control_yawrate) > fabs(control_vel_x / max_r)) {
        //   if (control_yawrate > 0) {
        //     control_yawrate =
        //         fmin(fabs(control_vel_x / max_r), control_yawrate);
        //   } else if (control_yawrate < 0) {
        //     control_yawrate =
        //         fmax(-fabs(control_vel_x / max_r), control_yawrate);
        //   }
        // }
      }
      if (m_xy_error < max_dis_to_find_qr) {
        if (control_yawrate >= 0) {
          control_yawrate = std::min(control_yawrate, 0.01);
        } else {
          control_yawrate = std::max(control_yawrate, -0.01);
        }
      }

      m_state = state_t::STATE_TRANSLATING;
    } else if (m_state == state_t::STATE_TURNING) {
      // continue on current yawrate
      control_yawrate = (start_yawrate > 0 ? 1 : -1) * m_limits.max_vel_theta;
    } else if (is_goal_target &&
               (m_state == state_t::STATE_ADJUSTING ||
                fabs(yaw_error) < start_yaw_error) &&
               fabs(pos_error.y()) >
                   (m_state == state_t::STATE_ADJUSTING ? 0.1 : 0.1)) {
      // we are not translating, but we have too large y error
      control_yawrate = (pos_error.y() > 0 ? 1 : -1) * m_limits.max_vel_theta;

      m_state = state_t::STATE_ADJUSTING;
    }

    // /*
    //  *******     *******   **********     **     ********** **   *******
    //  ****
    //  **
    // /**////**   **/////** /////**///     ****   /////**/// /**  **/////**
    // /**/**   /**
    // /**   /**  **     //**    /**       **//**      /**    /** **
    // //**/**//**
    // /**
    // /*******  /**      /**    /**      **  //**     /**    /**/** /**/**
    // //** /**
    // /**///**  /**      /**    /**     **********    /**    /**/** /**/**
    // //**/**
    // /**  //** //**     **     /**    /**//////**    /**    /**//**     **
    // /**
    // //****
    // /**   //** //*******      /**    /**     /**    /**    /** //*******
    // /**
    // //***
    // //     //   ///////       //     //      //     //     //   ///////
    // //
    // ///
    // */

    else {
      // use term for static target orientation
      control_yawrate = yaw_error_lookahead_stop * m_static_yaw_gain;
      // if (fabs(yaw_error_ori) <= 0.5)
      // {
      //   control_yawrate = fmin(control_yawrate, 1);
      //   is_emergency_brake = true;
      // }
      if (fabs(yaw_error_ori) <= 0.6) {
        control_yawrate = yaw_error_lookahead_stop * m_static_yaw_gain / 2;
        is_emergency_brake = true;
      }

      // if (fabs(start_vel_x) > 0.015)
      // {
      //   control_yawrate = fmax(control_yawrate, -0.05);
      //   control_yawrate = fmin(control_yawrate, 0.05);
      // }
      if (control_yawrate > 0) {
        control_yawrate = fmax(min_vel_move_rotation, control_yawrate);
      } else if (control_yawrate < 0) {
        control_yawrate = fmin(-min_vel_move_rotation, control_yawrate);
      }
      if (fabs(control_vel_x) > 0.01) {
        if (fabs(yaw_error_ori) < start_yaw_error) {
          control_yawrate = 0;
        } else if (fabs(m_limits.max_vel_x < 0.5)) {
          if (control_yawrate > 0) {
            control_yawrate = fmin(0.01, control_yawrate);
          } else if (control_yawrate < 0) {
            control_yawrate = fmax(-0.01, control_yawrate);
          }
        } else {
          if (control_yawrate > 0) {
            control_yawrate = fmin(0.01, control_yawrate);
          } else if (control_yawrate < 0) {
            control_yawrate = fmax(-0.01, control_yawrate);
          }
        }
      }

      m_state = state_t::STATE_ROTATING;
    }
  } else {
    // simply correct y with holonomic drive
    control_vel_y = pos_error.y() * m_pos_y_gain;

    if (m_state == state_t::STATE_TURNING) {
      // continue on current yawrate
      control_yawrate = (start_yawrate > 0 ? 1 : -1) * m_limits.max_vel_theta;
    } else {
      // use term for static target orientation
      control_yawrate = yaw_error * m_static_yaw_gain;

      if (fabs(start_vel_x) > m_limits.trans_stopped_vel) {
        m_state = state_t::STATE_TRANSLATING;
      } else {
        m_state = state_t::STATE_ROTATING;
      }
    }

    // apply x cost term only when rotating
    if (m_state == state_t::STATE_ROTATING &&
        fabs(yaw_error) > start_yaw_error) {
      control_vel_x -= delta_cost_x * m_cost_x_gain;
    }

    // apply y cost term when not approaching goal or if we are rotating
    if (!is_goal_target || (m_state == state_t::STATE_ROTATING &&
                            fabs(yaw_error) > start_yaw_error)) {
      control_vel_y -= delta_cost_y * m_cost_y_gain;
    }

    // apply yaw cost term when not approaching goal
    if (!is_goal_target) {
      control_yawrate -= delta_cost_yaw * m_cost_yaw_gain;
    }
  }

  // check if we are stuck
  if (have_obstacle && obstacle_dist <= 0 && delta_cost_x > 0 &&
      m_state == state_t::STATE_ROTATING && fabs(yaw_error) < start_yaw_error) {
    // we are stuck
    m_state = state_t::STATE_STUCK;

    // ROS_INFO_NAMED("NeoLocalPlanner",
    //                "We are stuck: yaw_error=%f, obstacle_dist=%f, "
    //                "obstacle_cost=%f, delta_cost_x=%f",
    //                yaw_error, obstacle_dist, obstacle_cost,
    //                delta_cost_x);
    return false;
  }

  // handle pause
  if (m_is_pause_smoother == "PAUSE_BY_SERVER") {
    control_vel_x = 0;
    // control_yawrate = 0;
    is_emergency_brake = true;
    stop_smoother_by_pause = true;
    // control_vel_x = fmax(control_vel_x, -m_vel_max_safety_field_3);
    // control_vel_x = fmin(control_vel_x, m_vel_max_safety_field_3);
    // is_emergency_brake = true;
  } else if (m_is_pause_smoother == "PAUSE") {
    control_vel_x = 0;
    // control_yawrate = 0;
    is_emergency_brake = true;
    stop_smoother_by_pause = true;
  }
  if (pause_by_traffic) {
    control_vel_x = 0;
    // control_yawrate = 0;
    is_emergency_brake = true;
    stop_smoother_by_pause = true;
  }
  // logic check
  is_emergency_brake = is_emergency_brake && fabs(control_vel_x) >= 0;
  if (is_emergency_brake) {
    disable_low_pass_filter_move_straight = true;
  } else {
    disable_low_pass_filter_move_straight = false;
  }
  // ROS_INFO_NAMED("NeoLocalPlanner", "SPEED_BEFORE_LIM: %f", control_vel_x);
  // apply low pass filter
  if (stop_smoother_by_pause) {
    control_vel_x = control_vel_x * 0.2 + m_last_control_values[0] * (1 - 0.2);
    control_vel_y = control_vel_y * 0.2 + m_last_control_values[1] * (1 - 0.2);
  } else {
    if (disable_low_pass_filter_move_straight) {
      control_vel_x =
          control_vel_x * fmin(m_low_pass_gain * 3, 1) +
          m_last_control_values[0] * (1 - fmin(m_low_pass_gain * 3, 1));
      control_vel_y =
          control_vel_y * fmin(m_low_pass_gain * 3, 1) +
          m_last_control_values[1] * (1 - fmin(m_low_pass_gain * 3, 1));
    } else {
      control_vel_x = control_vel_x * m_low_pass_gain +
                      m_last_control_values[0] * (1 - m_low_pass_gain);
      control_vel_y = control_vel_y * m_low_pass_gain +
                      m_last_control_values[1] * (1 - m_low_pass_gain);
    }
  }
  if (disable_low_pass_filter_move_rotate) {
    control_yawrate =
        control_yawrate * fmin(m_low_pass_gain * 3, 1) +
        m_last_control_values[2] * (1 - fmin(m_low_pass_gain * 3, 1));
  } else {
    control_yawrate = control_yawrate * m_low_pass_gain +
                      m_last_control_values[2] * (1 - m_low_pass_gain);
  }

  // ROS_INFO_NAMED("NeoLocalPlanner", "SPEED_AFTER low_pass_filter: %f",
  // control_vel_x);

  // apply safety status constrain
  if (safety_field_idx == 0 || safety_camera_field_idx == 0 ||
      (time_now - m_last_time_stop_by_safety).toSec() <
          m_max_time_check_stop_by_safety) {
    // ROS_INFO_NAMED("NeoLocalPlanner", "Safety field 0");
    // control_vel_x = fmin(control_vel_x, m_vel_max_safety_field_0);
    control_vel_x = 0;
    // control_yawrate = 0;
    stop_smoother_by_pause = true;
    is_emergency_brake = true;
  } else if (safety_field_idx == 1 || safety_camera_field_idx == 1 ||
             (time_now - m_last_time_stop_by_safety_2).toSec() < 1) {
    // ROS_INFO_NAMED("NeoLocalPlanner", "Safety field 1");
    control_vel_x = fmax(control_vel_x, -m_vel_max_safety_field_1);
    control_vel_x = fmin(control_vel_x, m_vel_max_safety_field_1);
    control_yawrate = fmax(control_yawrate, -m_vel_max_safety_field_1 * 1.5);
    control_yawrate = fmin(control_yawrate, m_vel_max_safety_field_1 * 1.5);
    is_emergency_brake = true;
  } else if (safety_field_idx == 2 || safety_camera_field_idx == 2 ||
             (time_now - m_last_time_stop_by_safety_3).toSec() < 1) {
    // ROS_INFO_NAMED("NeoLocalPlanner", "Safety field 1");
    control_vel_x = fmax(control_vel_x, -m_vel_max_safety_field_2);
    control_vel_x = fmin(control_vel_x, m_vel_max_safety_field_2);
    control_yawrate = fmax(control_yawrate, -m_vel_max_safety_field_2 * 1.5);
    control_yawrate = fmin(control_yawrate, m_vel_max_safety_field_2 * 1.5);
    is_emergency_brake = true;
  } else if (safety_field_idx == 3 || safety_camera_field_idx == 3) {
    // ROS_INFO_NAMED("NeoLocalPlanner", "Safety field 3");
    control_vel_x = fmax(control_vel_x, -m_vel_max_safety_field_3);
    control_vel_x = fmin(control_vel_x, m_vel_max_safety_field_3);
    is_emergency_brake = true;
  }
  if (slow_speed) {
    // ROS_INFO_NAMED("NeoLocalPlanner", "Safety field 1");
    control_vel_x = fmax(control_vel_x, -m_vel_max_safety_field_1);
    control_vel_x = fmin(control_vel_x, m_vel_max_safety_field_1);
    is_emergency_brake = true;
  }

  // slow vel because not detect qr when move in tolerance

  if (decrease_vel_when_not_detect_qr && type_move == "QR") {
    // ROS_INFO_NAMED("NeoLocalPlanner", "Safety field 1");
    control_vel_x = fmax(control_vel_x, -max_vel_when_not_detect_qr);
    control_vel_x = fmin(control_vel_x, max_vel_when_not_detect_qr);
    is_emergency_brake = true;
  }

  // ROS_INFO_NAMED("NeoLocalPlanner", "SPEED_AFTER safety constrain: %f",
  // control_vel_x);
  // apply acceleration limits
  if (m_robot_direction == -1.0) {
    // TODO: fix bug here
    control_vel_x = fmin(fabs(control_vel_x),
                         fabs(m_last_cmd_vel.linear.x +
                              m_robot_direction * m_limits.acc_lim_x * dt));
    control_vel_x =
        m_robot_direction *
        fmax(fabs(control_vel_x),
             fabs(fmin(0, (m_last_cmd_vel.linear.x -
                           (is_emergency_brake
                                ? m_robot_direction * m_emergency_acc_lim_x
                                : m_robot_direction * m_stop_acc) *
                               dt))));
  } else {
    control_vel_x = fmax(
        fmin(control_vel_x, m_last_cmd_vel.linear.x + m_limits.acc_lim_x * dt),
        m_last_cmd_vel.linear.x -
            (is_emergency_brake ? m_emergency_acc_lim_x : m_stop_acc) * dt);
  }
  // ROS_INFO_NAMED("NeoLocalPlanner", "SPEED after acc constrain: %f",
  //   control_vel_x);

  // Calculate vel_y
  control_vel_y =
      fmin(control_vel_y, m_last_cmd_vel.linear.y + m_limits.acc_lim_y * dt);
  control_vel_y =
      fmax(control_vel_y, m_last_cmd_vel.linear.y - m_limits.acc_lim_y * dt);

  // Calculate vel_yaw
  control_yawrate = fmin(
      control_yawrate, m_last_cmd_vel.angular.z +
                           (is_emergency_brake && !is_emergency_brake_only_vel_x
                                ? m_emergency_acc_lim_theta
                                : m_limits.acc_lim_theta) *
                               dt);
  control_yawrate = fmax(
      control_yawrate, m_last_cmd_vel.angular.z -
                           (is_emergency_brake && !is_emergency_brake_only_vel_x
                                ? m_emergency_acc_lim_theta
                                : m_limits.acc_lim_theta) *
                               dt);
  if (stop_smoother_by_pause && fabs(control_vel_x) < 0.05) {
    control_yawrate = 0;
    control_vel_x = 0;
  }
  // constrain velocity after goal reached
  if (m_constrain_final && m_is_goal_reached) {
    tf2::Vector3 direction(m_last_control_values[0], m_last_control_values[1],
                           m_last_control_values[2]);
    if (direction.length() != 0) {
      direction.normalize();
      const double dist = direction.dot(
          tf2::Vector3(control_vel_x, control_vel_y, control_yawrate));
      const auto control = direction * dist;
      control_vel_x = control[0];
      control_vel_y = control[1];
      control_yawrate = control[2];
    }
  }

  // fill return data
  if (m_robot_direction == -1.0) {
    cmd_vel.linear.x =
        fmax(fmin(control_vel_x, m_robot_direction * m_limits.min_vel_x),
             m_robot_direction * m_limits.max_vel_x);
  } else {
    cmd_vel.linear.x =
        fmin(fmax(control_vel_x, m_limits.min_vel_x), m_limits.max_vel_x);
    // cmd_vel.linear.x =
    //     fmin(fmin(fmax(control_vel_x, m_limits.min_vel_x),
    //     m_limits.max_vel_x),
    //          start_vel_x + 0.2 + m_limits.acc_lim_x * dt);
  }

  cmd_vel.linear.y =
      fmin(fmax(control_vel_y, m_limits.min_vel_y), m_limits.max_vel_y);
  cmd_vel.linear.z = 0;
  cmd_vel.angular.x = 0;
  cmd_vel.angular.y = 0;
  cmd_vel.angular.z = fmin(fmax(control_yawrate, -m_limits.max_vel_theta),
                           m_limits.max_vel_theta);

  // Footprint based collision avoidance
  if (m_enable_software_stop == true) {
    if ((obstacle_in_rot == -1) &&
        (control_yawrate - start_yawrate < start_yawrate)) {
      ROS_WARN_THROTTLE(1, "During the rotation robot predicted an obstacle on "
                           "the right! Please free the robot using Joy");

      cmd_vel.angular.z = 0;
    } else if ((obstacle_in_rot == -1) &&
               (control_yawrate - start_yawrate > start_yawrate)) {
      ROS_WARN_THROTTLE(1, "During the rotation robot predicted an obstacle on "
                           "the left! Please free the robot using Joy");

      cmd_vel.angular.z = 0;
    }
  }
  // ROS_INFO_STREAM("NeoLocalPlanner r =  " << control_vel_x /
  // control_yawrate);
  if (m_print_state) {
    if (m_update_counter % 1 == 0) {
      ROS_INFO_NAMED("NeoLocalPlanner",
                     "dt=%f, pos_error=(%f, %f), pos_error_ori=(%f, %f), "
                     "yaw_error=%f, yaw_error_ori=%f, cost=%f, "
                     "obstacle_dist=%f, obstacle_cost=%f, delta_cost=(%f, %f, "
                     "%f), state=%d, cmd_vel=(%f, %f), cmd_yawrate=%f",
                     dt, pos_error.x(), pos_error.y(), pos_error_ori.x(),
                     pos_error_ori.y(), yaw_error, yaw_error_ori, center_cost,
                     obstacle_dist, obstacle_cost, delta_cost_x, delta_cost_y,
                     delta_cost_yaw, m_state, control_vel_x, control_vel_y,
                     control_yawrate);
    }
  }

  if (m_update_counter % 2 == 0) {
    std::string state;
    if (std::fabs(yaw_error_ori * 180.0 / M_PI) > 10.0) {
      state = (yaw_error_ori < 0) ? "ROTATING_CW" : "ROTATING_CCW";
    } else {
      state = "TRANSLATING";
    }
    std::string direction = (m_robot_direction == 1.0) ? "FORWARD" : "BACKWARD";

    publishStateDirection(state, direction);
  }

  m_last_time = time_now;
  m_last_control_values[0] = control_vel_x;
  m_last_control_values[1] = control_vel_y;
  m_last_control_values[2] = control_yawrate;
  m_last_cmd_vel = cmd_vel;

  m_update_counter++;

  return true;
}

bool NeoLocalPlanner::isGoalReached() {
  if (m_is_goal_reached) {
    ROS_INFO("GOAL Reached!");
    m_last_control_values[0] = 0;
    m_last_control_values[1] = 0;
    m_last_control_values[2] = 0;
    m_last_cmd_vel = geometry_msgs::Twist();
    return true;
  }
  return false;
}

// /*
//  ######  ######## ########         ########  ##          ###    ##    ##
// ##    ## ##          ##            ##     ## ##         ## ##   ###   ##
// ##       ##          ##            ##     ## ##        ##   ##  ####  ##
//  ######  ######      ##            ########  ##       ##     ## ## ## ##
//       ## ##          ##            ##        ##       ######### ##  ####
// ##    ## ##          ##            ##        ##       ##     ## ##   ###
//  ######  ########    ##    ####### ##        ######## ##     ## ##    ##
// */

bool NeoLocalPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped> &plan) {
  if (m_global_plan.empty() || plan.back().pose == m_global_plan.back().pose) {
    m_set_zero_vel = true;
  }
  m_set_zero_vel = false;
  led_turn_on = false;
  m_is_goal_reached = false;
  first_time_check = true;
  is_rotation_first = false;
  m_detect_qr_code = false;
  m_is_goal_reached_by_qr_code = false;
  m_is_force_goal_reached = false;
  succeed_rotate_first = false;
  get_state_stop = true;
  disable_low_pass_filter_move_straight = false;
  disable_low_pass_filter_move_rotate = false;

  m_state = state_t::STATE_IDLE;
  m_pre_state = state_t::NONE;
  m_last_time = ros::WallTime::now();
  m_first_goal_reached_time = ros::WallTime::now();
  time_blink_led_turn = ros::WallTime::now();
  // last_time_detect_qr_code = ros::WallTime::now();

  m_last_control_values[2] = 0;
  // m_is_pause_smoother = "RUN";
  count = 0;
  m_robot_direction = 1;
  m_limits.acc_lim_x = fabs(m_limits.acc_lim_x);
  dis_qr_x = 1000.000;
  dis_qr_y = 1000.000;
  m_xy_error = 1000.0;
  start_to_goal = 0.0;
  error_msg.error_angle = 0;
  error_msg.error_position = 0;
  global_goal_ = plan.back();
  pose_begin_in_path_ = plan.front();
  m_global_plan = plan;
  check_have_qr_start_move = false;
  check_error_angle_begin = true;
  receive_new_data_qr = false;
  last_angle_error_found_qr_code = 1000;
  ROS_INFO_NAMED("NeoLocalPlanner", "START VEL_X: %f",
                 m_last_control_values[0]);

  std_stamped_msgs::StringService srv_type_move;
  client_get_type_move_.call(srv_type_move);
  type_move = srv_type_move.response.respond;
  return true;
}

// /*
// #### ##    ## #### ######## ####    ###    ##       #### ######## ########
//  ##  ###   ##  ##     ##     ##    ## ##   ##        ##       ##  ##
//  ##  ####  ##  ##     ##     ##   ##   ##  ##        ##      ##   ##
//  ##  ## ## ##  ##     ##     ##  ##     ## ##        ##     ##    ######
//  ##  ##  ####  ##     ##     ##  ######### ##        ##    ##     ##
//  ##  ##   ###  ##     ##     ##  ##     ## ##        ##   ##      ##
// #### ##    ## ####    ##    #### ##     ## ######## #### ######## ########
// */

void NeoLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
                                 costmap_2d::Costmap2DROS *costmap_ros) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~/" + name);

  m_limits.acc_lim_x = private_nh.param<double>("acc_lim_x", 0.5);
  m_limits.acc_lim_y = private_nh.param<double>("acc_lim_y", 0.5);
  m_limits.acc_lim_theta = private_nh.param<double>("acc_lim_theta", 0.5);
  m_limits.acc_lim_trans =
      private_nh.param<double>("acc_limit_trans", m_limits.acc_lim_x);
  m_limits.min_vel_x = private_nh.param<double>("min_vel_x", -0.1);
  m_limits.max_vel_x = private_nh.param<double>("max_vel_x", 0.5);
  m_limits.min_vel_y = private_nh.param<double>("min_vel_y", -0.5);
  m_limits.max_vel_y = private_nh.param<double>("max_vel_y", 0.5);
  m_limits.min_vel_theta = private_nh.param<double>("min_rot_vel", 0.1);
  m_limits.max_vel_theta = private_nh.param<double>("max_rot_vel", 0.5);
  m_limits.min_vel_trans = private_nh.param<double>("min_trans_vel", 0.1);
  m_limits.max_vel_trans =
      private_nh.param<double>("max_trans_vel", m_limits.max_vel_x);
  m_limits.theta_stopped_vel =
      private_nh.param<double>("rot_stopped_vel", 0.5 * m_limits.min_vel_theta);
  m_limits.trans_stopped_vel = private_nh.param<double>(
      "trans_stopped_vel", 0.5 * m_limits.min_vel_trans);
  m_limits.yaw_goal_tolerance =
      private_nh.param<double>("yaw_goal_tolerance", 0.02);
  m_limits.xy_goal_tolerance =
      private_nh.param<double>("xy_goal_tolerance", 0.1);

  m_differential_drive = private_nh.param<bool>("differential_drive", true);
  m_constrain_final = private_nh.param<bool>("constrain_final", false);
  m_goal_tune_time = private_nh.param<double>("goal_tune_time", 0.5);
  m_lookahead_time = private_nh.param<double>("lookahead_time", 0.2);
  m_lookahead_time_stop = private_nh.param<double>("lookahead_time_stop", 0.2);
  m_lookahead_dist = private_nh.param<double>("lookahead_dist", 0.5);
  m_start_yaw_error_normal =
      private_nh.param<double>("start_yaw_error_normal", 0.2);
  m_start_yaw_error_when_safety =
      private_nh.param<double>("start_yaw_error_when_safety", 0.2);
  m_pos_x_gain = private_nh.param<double>("pos_x_gain", 1);
  m_pos_y_gain = private_nh.param<double>("pos_y_gain", 1);
  m_pos_y_yaw_gain = private_nh.param<double>("pos_y_yaw_gain", 1);
  m_yaw_gain = private_nh.param<double>("yaw_gain", 1);
  m_static_yaw_gain = private_nh.param<double>("static_yaw_gain", 3);
  m_cost_x_gain = private_nh.param<double>("cost_x_gain", 0.1);
  m_cost_y_gain = private_nh.param<double>("cost_y_gain", 0.1);
  m_cost_y_yaw_gain = private_nh.param<double>("cost_y_yaw_gain", 0.1);
  m_cost_y_lookahead_dist =
      private_nh.param<double>("cost_y_lookahead_dist", 0);
  m_cost_y_lookahead_time =
      private_nh.param<double>("cost_y_lookahead_time", 1);
  m_cost_yaw_gain = private_nh.param<double>("cost_yaw_gain", 1);
  m_low_pass_gain = private_nh.param<double>("low_pass_gain", 0.5);
  m_max_cost = private_nh.param<double>("max_cost", 0.9);
  m_stop_acc = private_nh.param<double>("stop_acc", 0.9);
  m_max_curve_vel = private_nh.param<double>("max_curve_vel", 0.2);
  m_max_goal_dist = private_nh.param<double>("max_goal_dist", 0.5);
  m_max_backup_dist = private_nh.param<double>(
      "max_backup_dist", m_differential_drive ? 0.1 : 0.0);
  m_min_stop_dist = private_nh.param<double>("min_stop_dist", 0.5);
  m_emergency_acc_lim_x =
      private_nh.param<double>("emergency_acc_lim_x", m_limits.acc_lim_x * 4);
  m_emergency_acc_lim_theta = private_nh.param<double>(
      "emergency_acc_lim_theta", m_limits.acc_lim_theta * 4);
  m_enable_software_stop = private_nh.param<bool>("enable_software_stop", true);
  m_allow_reversing = private_nh.param<bool>("allow_reversing", true);
  m_min_dist_obstacle_allow_rotation =
      private_nh.param<double>("min_dist_obstacle_allow_rotation", 0.5);
  m_vel_max_safety_field_0 =
      private_nh.param<double>("vel_max_safety_field_0", 0);
  m_vel_max_safety_field_1 =
      private_nh.param<double>("vel_max_safety_field_1", 0);
  m_vel_max_safety_field_2 =
      private_nh.param<double>("vel_max_safety_field_2", 0.5);
  m_vel_max_safety_field_3 =
      private_nh.param<double>("vel_max_safety_field_3", 0.5);
  max_vel_when_not_detect_qr =
      private_nh.param<double>("max_vel_when_not_detect_qr", 0.4);
  m_max_time_check_stop_by_safety =
      private_nh.param<double>("max_time_check_stop_by_safety", 1);
  m_print_state = private_nh.param<bool>("print_state", true);
  m_dist_stop_agv_to_qr_code =
      private_nh.param<double>("dist_stop_agv_to_qr_code", 0.02);
  bool m_stop_by_qr_code = private_nh.param<bool>("stop_by_qr_code", true);
  double m_tolerance_stop_by_qr_code =
      private_nh.param<double>("tolerance_stop_by_qr_code", 0.1);
  double dis_check_qr_for_stop =
      private_nh.param<double>("distance_check_qr_for_stop", 0.1);
  double m_min_vel_stop_by_qr_code =
      private_nh.param<double>("min_vel_stop_by_qr_code", 0.03);
  double min_vel_move_straight =
      private_nh.param<double>("min_vel_move_straight", 0.03);
  double min_vel_move_rotation =
      private_nh.param<double>("min_vel_move_rotation", 0.05);
  double max_r = private_nh.param<double>("max_r", 1);
  m_tf = tf;
  m_cost_map = costmap_ros;
  m_base_frame = costmap_ros->getBaseFrameID();
  disable_check_lost_qr.data = 0;

  mission_manager_module_status_subscriber_ =
      nh.subscribe("/mission_manager/module_status", 10,
                   &NeoLocalPlanner::missionStatusCb, this);

  disable_check_lost_qr_subscriber_ =
      nh.subscribe("/disable_check_error_qr_code", 1,
                   &NeoLocalPlanner::disableCheckLostQrCb, this);

  m_qr_code_data_sub = nh.subscribe<agv_msgs::DataMatrixStamped>(
      "/data_gls621", 1,
      boost::bind(&NeoLocalPlanner::dataQrCodeCallback, this, _1));
  m_pause_smoother_sub = nh.subscribe<std_stamped_msgs::StringStamped>(
      "/run_pause_req", 1,
      boost::bind(&NeoLocalPlanner::runPauseCallback, this, _1));
  m_pause_by_traffic_sub = nh.subscribe<std_msgs::Bool>(
      "/run_pause_req_by_traffic", 1,
      boost::bind(&NeoLocalPlanner::runPauseByTrafficCallback, this, _1));
  m_odom_sub = nh.subscribe<nav_msgs::Odometry>(
      "/odom", 1, boost::bind(&NeoLocalPlanner::odomCallback, this, _1));
  m_safety_status_sub = nh.subscribe<safety_msgs::SafetyStatus>(
      "/safety_status", 1,
      boost::bind(&NeoLocalPlanner::safetyCallback, this, _1));
  m_safety_camera_status_sub = nh.subscribe<safety_msgs::SafetyStatus>(
      "/safety_camera_status", 1,
      boost::bind(&NeoLocalPlanner::safetyCameraCallback, this, _1));
  pub_state_direction_ = nh.advertise<std_stamped_msgs::StringStamped>(
      "/robot_state_direction", 10);
  m_local_plan_pub = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
  m_set_safety_job_pub =
      nh.advertise<std_stamped_msgs::StringStamped>("/safety_job_name", 1);
  m_set_footprint_job_pub = nh.advertise<std_stamped_msgs::StringStamped>(
      "/safety_footprint_name", 1);
  error_to_path_pub =
      nh.advertise<agv_msgs::ErrorRobotToPath>("/error_robot_to_path", 1);

  lost_goal_label_pub =
      nh.advertise<geometry_msgs::Pose>("/lost_goal_label", 1);

  m_last_time_stop_by_safety = ros::WallTime::now();
  m_last_time_stop_by_safety_2 = ros::WallTime::now();
  m_last_time_stop_by_safety_3 = ros::WallTime::now();

  client_get_type_move_ =
      nh.serviceClient<std_stamped_msgs::StringService>("get_type_move");

  // setup dynamic reconfigure
  dsrv_ = new dynamic_reconfigure::Server<NeoPlannerConfig>(private_nh);
  dynamic_reconfigure::Server<NeoPlannerConfig>::CallbackType f;
  f = boost::bind(&NeoLocalPlanner::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(f);

  ROS_INFO_NAMED(
      "NeoLocalPlanner", "base_frame=%s, local_frame=%s, global_frame=%s",
      m_base_frame.c_str(), m_local_frame.c_str(), m_global_frame.c_str());
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

void NeoLocalPlanner::disableCheckLostQrCb(
    const std_stamped_msgs::Int8Stamped::ConstPtr &msg) {
  disable_check_lost_qr = *msg;

  // Log the value of disable_check_lost_qr.data as a warning using ROS_WARN
  ROS_WARN("disable_check_lost_qr: %d", disable_check_lost_qr.data);
}

void NeoLocalPlanner::missionStatusCb(
    const std_stamped_msgs::StringStamped::ConstPtr &msg) {
  try {
    // Parse JSON message
    auto data_dict = nlohmann::json::parse(msg->data);
    // Access the current_action_type
    current_action_type =
        data_dict.at("current_action_type").get<std::string>();
    if (pre_current_action_type != current_action_type) {
      ROS_WARN("Current Action Type: %s", current_action_type.c_str());
      pre_current_action_type = current_action_type;
    }
  } catch (const nlohmann::json::exception &e) {
    ROS_ERROR("JSON parsing error: %s", e.what());
  }
}

void NeoLocalPlanner::dataQrCodeCallback(
    const agv_msgs::DataMatrixStamped::ConstPtr &msg) {
  // ROS_ERROR("READ QR");
  dis_qr_x = double(msg->possition.x) / 1000.0;
  dis_qr_y = double(msg->possition.y) / 1000.0;
  last_label_qr_x = double(msg->lable.x) / 1000;
  last_label_qr_y = double(msg->lable.y) / 1000;
  // ROS_INFO_STREAM("GOAL reached: ERR_X XXXXXXXXXXXXX :" <<
  // m_dist_agv_to_qr_code);

  m_detect_qr_code = true;
  receive_new_data_qr = true;
  last_time_detect_qr_code = ros::WallTime::now();
}

void NeoLocalPlanner::runPauseCallback(
    const std_stamped_msgs::StringStamped::ConstPtr &msg) {
  m_is_pause_smoother = msg->data;
  ROS_INFO_STREAM("is pause smoother: " << m_is_pause_smoother);
}

void NeoLocalPlanner::runPauseByTrafficCallback(
    const std_msgs::Bool::ConstPtr &msg) {
  pause_by_traffic = msg->data;
  ROS_INFO_STREAM("is pause_by_traffic: " << pause_by_traffic);
}

void NeoLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  boost::mutex::scoped_lock lock(m_odometry_mutex);
  m_odometry = msg;
}
void NeoLocalPlanner::safetyCallback(
    const safety_msgs::SafetyStatus::ConstPtr &msg) {
  m_safety_status = msg;
  safety_field_idx = 10;
  for (auto i = 0; i < m_safety_status->fields.size(); i++) {
    if (m_safety_status->fields[i] == 1) {
      safety_field_idx = i;
      return;
    }
  }
}
void NeoLocalPlanner::safetyCameraCallback(
    const safety_msgs::SafetyStatus::ConstPtr &msg) {
  m_safety_camera_status = msg;
  safety_camera_field_idx = 10;
  for (auto i = 0; i < m_safety_camera_status->fields.size(); i++) {
    if (m_safety_camera_status->fields[i] == 1) {
      safety_camera_field_idx = i;
      return;
    }
  }
}
} // namespace neo_local_planner
