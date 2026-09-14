#include "waypoint_global_planner/waypoint_global_planner.h"

#include <agv_msgs/WaypointsPath.h>
#include <math.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>

PLUGINLIB_EXPORT_CLASS(waypoint_global_planner::WaypointGlobalPlanner,
                       nav_core::BaseGlobalPlanner)

namespace waypoint_global_planner {

WaypointGlobalPlanner::WaypointGlobalPlanner()
    : tf_buffer_(),
      tf_listener_(tf_buffer_),
      costmap_ros_(NULL),
      initialized_(false),
      clear_waypoints_(false) {}

WaypointGlobalPlanner::WaypointGlobalPlanner(
    std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    : tf_buffer_(),
      tf_listener_(tf_buffer_),
      costmap_ros_(nullptr),
      initialized_(false),
      clear_waypoints_(false) {
    initialize(name, costmap_ros);
}

WaypointGlobalPlanner::~WaypointGlobalPlanner() {}

void WaypointGlobalPlanner::initialize(std::string name,
                                       costmap_2d::Costmap2DROS *costmap_ros) {
    if (!initialized_) {
        // get the costmap
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();
        world_model_ = new base_local_planner::CostmapModel(*costmap_);

        ros::NodeHandle nh;
        ros::NodeHandle pnh("~" + name);

        // load parameters
        pnh.param("epsilon", epsilon_, 1e-1);
        pnh.param("waypoints_per_meter", waypoints_per_meter_, 20);
        pnh.param("tolerance_path", tolerance_path_, 0.03);
        // initialize publishers and subscribers
        goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
            "/move_base_simple/goal", 1);
        plan_pub_ = pnh.advertise<nav_msgs::Path>("global_plan", 1);
        client_get_waypoints_ =
            nh.serviceClient<agv_msgs::WaypointsPath>("waypoints_to_global");
        client_get_direction_ =
            nh.serviceClient<agv_msgs::DirectionMove>("get_diriction_move");
        client_get_type_move_ =
            nh.serviceClient<std_stamped_msgs::StringService>("get_type_move");
        initialized_ = true;
        ROS_INFO("Planner has been initialized");
    } else {
        ROS_WARN("This planner has already been initialized");
    }
}

geometry_msgs::PoseStamped WaypointGlobalPlanner::transformPose(
    const geometry_msgs::PoseStamped &pose_in, const std::string &target_frame,
    tf2_ros::Buffer &tf_buffer) {
    geometry_msgs::PoseStamped pose_out;
    try {
        pose_out =
            tf_buffer.transform(pose_in, target_frame, ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Transform failed: %s", ex.what());
    }
    return pose_out;
}

bool WaypointGlobalPlanner::makePlan(
    const geometry_msgs::PoseStamped &start_pose,
    const geometry_msgs::PoseStamped &goal,
    std::vector<geometry_msgs::PoseStamped> &plan) {
    agv_msgs::WaypointsPath srv_get_wps;
    agv_msgs::DirectionMove srv_direction;
    std_stamped_msgs::StringService srv_type_move;
    client_get_type_move_.call(srv_type_move);
    client_get_waypoints_.call(srv_get_wps);
    client_get_direction_.call(srv_direction);
    ROS_INFO("Call service get waypoint");
    type_move = srv_type_move.response.respond;
    array_waypoints_ = srv_get_wps.response;
    direction_ = srv_direction.response;
    ROS_INFO("use_lidar: %s", array_waypoints_.use_lidar ? "true" : "false");
    std::vector<geometry_msgs::PoseStamped> temp_waypoint;
    geometry_msgs::PoseStamped start_pose_ = start_pose;
    geometry_msgs::PoseStamped start_in_odom =
        transformPose(start_pose, "odom", tf_buffer_);
    if (!array_waypoints_.use_lidar && type_move == "MIRROR") {
        start_pose_ = start_in_odom;
    }
    if (array_waypoints_.Waypoints.size() > 1) {
        use_orient_goal_ = false;
        for (auto i = 0; i <= array_waypoints_.Waypoints.size() - 1; i++) {
            temp_waypoint.push_back(array_waypoints_.Waypoints[i].Pose);
        }
    } else {
        use_orient_goal_ = true;
        temp_waypoint.push_back(start_pose_);
        temp_waypoint.push_back(goal);
    }
    path_.poses = temp_waypoint;

    tf::Quaternion quat(
        start_pose_.pose.orientation.x, start_pose_.pose.orientation.y,
        start_pose_.pose.orientation.z, start_pose_.pose.orientation.w);
    double orient_robot_begin = quaternionToEuler(quat);

    double orient_path = atan2(
        (path_.poses[1].pose.position.y - path_.poses[0].pose.position.y),
        (path_.poses[1].pose.position.x - path_.poses[0].pose.position.x));
    double diff_angle = GetNormaliceAngle(orient_path - orient_robot_begin);
    ROS_WARN_STREAM("ORIENT ROBOT" << orient_robot_begin);
    ROS_WARN_STREAM("ORIENT PATH" << orient_path);
    ROS_WARN_STREAM("DIFF ANGLE ROBOT TO PATH" << diff_angle);
    if (direction_.direction == 0) {
        if (abs(diff_angle) > M_PI / 2 + 0.3) {
            revert_orient_ = true;
            ROS_INFO("Revert orient");
        } else {
            revert_orient_ = false;
        }
    } else if (direction_.direction == 1) {
        revert_orient_ = false;
    } else {
        revert_orient_ = true;
        ROS_INFO("Revert orient");
    }
    if (!array_waypoints_.use_lidar) {
        interpolatePath(path_);
        if (!path_.poses.empty()) {
            path_.poses.front().pose.orientation.y = 0;  // use QR
        }
        plan = path_.poses;
        path_.header.frame_id = "map";
        path_.header.stamp = ros::Time::now();
        plan_pub_.publish(path_);
        ROS_INFO("Make plan done, Published global plan");
        return true;
    } else {
        std::vector<WaypointsBezier> array_waypoints_bezier;

        makeWaypointBezier(array_waypoints_, array_waypoints_bezier);

        if (array_waypoints_bezier.size() <= 1) {
            addWaypoint(start_pose_.pose.position.x,
                        start_pose_.pose.position.y, goal.pose.position.x,
                        goal.pose.position.y, plan);
        } else {
            for (auto i = 0; i < array_waypoints_bezier.size() - 1; i++) {
                if (array_waypoints_bezier[i + 1].type != "Control_Point") {
                    addWaypoint(
                        array_waypoints_bezier[i].Waypoint.pose.position.x,
                        array_waypoints_bezier[i].Waypoint.pose.position.y,
                        array_waypoints_bezier[i + 1].Waypoint.pose.position.x,
                        array_waypoints_bezier[i + 1].Waypoint.pose.position.y,
                        plan);
                }
                if (array_waypoints_bezier[i + 1].type == "Control_Point") {
                    if (i + 2 >= array_waypoints_bezier.size()) {
                        ROS_ERROR("set path is in correct, please reset");
                        return false;
                    } else {
                        std::vector<geometry_msgs::PoseStamped> bezier_path;
                        std::vector<geometry_msgs::PoseStamped> control_point;
                        for (auto j = 0; j < 3; j++) {
                            control_point.push_back(
                                array_waypoints_bezier[i + j].Waypoint);
                        }
                        bezier_.calcBezierPath(bezier_path, control_point, 50);
                        for (auto j = 0; j < bezier_path.size(); j++) {
                            plan.push_back(bezier_path[j]);
                        }
                        i += 1;
                    }
                }
            }
        }
        if (revert_orient_ && !plan.empty()) {
            plan.front().pose.orientation.x = 1;
        }
        // Đánh dấu dùng lidar
        if (!plan.empty()) {
            plan.front().pose.orientation.y = 1;
        }
        ROS_INFO("Make plan done, Published global plan");
        // plan_pub_.publish(path_);
        path_.poses = plan;
        path_.header.frame_id = "map";
        path_.header.stamp = ros::Time::now();
        plan_pub_.publish(path_);
        return true;
    }
}

void WaypointGlobalPlanner::addWaypoint(
    double pose_1_x, double pose_1_y, double pose_2_x, double pose_2_y,
    std::vector<geometry_msgs::PoseStamped> &plan) {
    // Resolution để chia đoạn thẳng — hardcoded hoặc lấy từ ROS param
    double resolution =
        0.05;  // bạn có thể thay bằng ros::param::param nếu muốn động

    double dx = pose_2_x - pose_1_x;
    double dy = pose_2_y - pose_1_y;
    // calculate orientation of waypoints
    if (!revert_orient_) {
        orient_end_wp_ = atan2(dy, dx);
    } else {
        orient_end_wp_ = atan2(-dy, -dx);
    }

    double distance = hypot(dx, dy);

    int num_points = distance * waypoints_per_meter_;

    // Thêm các điểm nội suy
    for (int j = 0; j < num_points; ++j)  //
    {
        double ratio = static_cast<double>(j) / static_cast<double>(num_points);

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp =
            ros::Time::now();  // có thể bỏ nếu không cần timestamp

        pose.pose.position.x = pose_1_x + ratio * dx;
        pose.pose.position.y = pose_1_y + ratio * dy;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(orient_end_wp_);

        plan.push_back(pose);
    }
    // add end pose
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();  // có thể bỏ nếu không cần timestamp

    pose.pose.position.x = pose_2_x;
    pose.pose.position.y = pose_2_y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(orient_end_wp_);

    plan.push_back(pose);
}

void WaypointGlobalPlanner::makeWaypointBezier(
    const agv_msgs::WaypointsPathResponse &array_waypoints,
    std::vector<WaypointsBezier> &array_waypoints_bezier) {
    if (array_waypoints.Waypoints.size() < 1) {
        return;
    }

    for (auto i = 0; i < array_waypoints.Waypoints.size(); i++) {
        if (i == 0) {
            WaypointsBezier waypoint_bezier;
            waypoint_bezier.Waypoint = array_waypoints.Waypoints[i].Pose;
            waypoint_bezier.type = "";
            array_waypoints_bezier.push_back(waypoint_bezier);
        } else if (i == array_waypoints.Waypoints.size() - 1) {
            double diff_x =
                array_waypoints.Waypoints[i].Pose.pose.position.x -
                array_waypoints.Waypoints[i - 1].Pose.pose.position.x;
            double diff_y =
                array_waypoints.Waypoints[i].Pose.pose.position.y -
                array_waypoints.Waypoints[i - 1].Pose.pose.position.y;
            double diff_distance = sqrt(diff_x * diff_x + diff_y * diff_y);
            WaypointsBezier waypoint_bezier_1;
            double ratio_1 =
                array_waypoints.Waypoints[i - 1].radius / diff_distance;
            waypoint_bezier_1.Waypoint.pose.position.x =
                array_waypoints.Waypoints[i - 1].Pose.pose.position.x +
                ratio_1 * diff_x;
            waypoint_bezier_1.Waypoint.pose.position.y =
                array_waypoints.Waypoints[i - 1].Pose.pose.position.y +
                ratio_1 * diff_y;
            waypoint_bezier_1.type = "";
            array_waypoints_bezier.push_back(waypoint_bezier_1);

            WaypointsBezier waypoint_bezier_2;
            waypoint_bezier_2.Waypoint = array_waypoints.Waypoints[i].Pose;
            waypoint_bezier_2.type = "";
            array_waypoints_bezier.push_back(waypoint_bezier_2);
        } else {
            double diff_x =
                array_waypoints.Waypoints[i].Pose.pose.position.x -
                array_waypoints.Waypoints[i - 1].Pose.pose.position.x;
            double diff_y =
                array_waypoints.Waypoints[i].Pose.pose.position.y -
                array_waypoints.Waypoints[i - 1].Pose.pose.position.y;
            double diff_distance = sqrt(diff_x * diff_x + diff_y * diff_y);
            if (diff_distance < array_waypoints.Waypoints[i - 1].radius) {
                WaypointsBezier waypoint_bezier;
                waypoint_bezier.Waypoint = array_waypoints.Waypoints[i].Pose;
                waypoint_bezier.type = "";
                array_waypoints_bezier.push_back(waypoint_bezier);
            } else if (diff_distance <
                       array_waypoints.Waypoints[i - 1].radius +
                           array_waypoints.Waypoints[i].radius) {
                WaypointsBezier waypoint_bezier_1;
                double ratio_1 =
                    array_waypoints.Waypoints[i - 1].radius / diff_distance;
                waypoint_bezier_1.Waypoint.pose.position.x =
                    array_waypoints.Waypoints[i - 1].Pose.pose.position.x +
                    ratio_1 * diff_x;
                waypoint_bezier_1.Waypoint.pose.position.y =
                    array_waypoints.Waypoints[i - 1].Pose.pose.position.y +
                    ratio_1 * diff_y;
                waypoint_bezier_1.type = "";
                array_waypoints_bezier.push_back(waypoint_bezier_1);

                WaypointsBezier waypoint_bezier_2;
                waypoint_bezier_2.Waypoint = array_waypoints.Waypoints[i].Pose;
                waypoint_bezier_2.type = "Control_Point";
                array_waypoints_bezier.push_back(waypoint_bezier_2);
            } else {
                WaypointsBezier waypoint_bezier_1;
                double ratio_1 =
                    array_waypoints.Waypoints[i - 1].radius / diff_distance;
                waypoint_bezier_1.Waypoint.pose.position.x =
                    array_waypoints.Waypoints[i - 1].Pose.pose.position.x +
                    ratio_1 * diff_x;
                waypoint_bezier_1.Waypoint.pose.position.y =
                    array_waypoints.Waypoints[i - 1].Pose.pose.position.y +
                    ratio_1 * diff_y;
                waypoint_bezier_1.type = "";
                array_waypoints_bezier.push_back(waypoint_bezier_1);

                WaypointsBezier waypoint_bezier_2;
                double ratio_2 =
                    array_waypoints.Waypoints[i].radius / diff_distance;
                waypoint_bezier_2.Waypoint.pose.position.x =
                    array_waypoints.Waypoints[i].Pose.pose.position.x -
                    ratio_2 * diff_x;
                waypoint_bezier_2.Waypoint.pose.position.y =
                    array_waypoints.Waypoints[i].Pose.pose.position.y -
                    ratio_2 * diff_y;
                waypoint_bezier_2.type = "";
                array_waypoints_bezier.push_back(waypoint_bezier_2);

                WaypointsBezier waypoint_bezier_3;
                waypoint_bezier_3.Waypoint = array_waypoints.Waypoints[i].Pose;
                waypoint_bezier_3.type = "Control_Point";
                array_waypoints_bezier.push_back(waypoint_bezier_3);
            }
        }
    }
}

void WaypointGlobalPlanner::interpolatePath(nav_msgs::Path &path) {
    std::vector<geometry_msgs::PoseStamped> temp_path;
    for (int i = 0; i < static_cast<int>(path.poses.size() - 1); i++) {
        // calculate distance between two consecutive waypoints
        double x1 = path.poses[i].pose.position.x;
        double y1 = path.poses[i].pose.position.y;
        double x2 = path.poses[i + 1].pose.position.x;
        double y2 = path.poses[i + 1].pose.position.y;
        double dist = hypot(x1 - x2, y1 - y2);
        int num_wpts = dist * waypoints_per_meter_;
        // calculate orientation of waypoints
        if (!revert_orient_) {
            orient_end_wp_ = atan2(y2 - y1, x2 - x1);
        } else {
            orient_end_wp_ = atan2(y1 - y2, x1 - x2);
        }

        orient_two_last_wp_ = atan2(y2 - y1, x2 - x1);
        path.poses[i].pose.orientation =
            tf::createQuaternionMsgFromYaw(orient_two_last_wp_);

        temp_path.push_back(path.poses[i]);
        geometry_msgs::PoseStamped p = path.poses[i];

        for (int j = 0; j < num_wpts - 2; j++) {
            p.pose.position.x =
                x1 + static_cast<double>(j) / num_wpts * (x2 - x1);
            p.pose.position.y =
                y1 + static_cast<double>(j) / num_wpts * (y2 - y1);
            temp_path.push_back(p);
        }
    }
    // update sequence of poses
    for (size_t i = 0; i < temp_path.size(); i++)
        temp_path[i].header.seq = static_cast<int>(i);
    if (!use_orient_goal_) {
        if (revert_orient_ && !temp_path.empty()) {
            temp_path.front().pose.orientation.x = 1;
        }
        geometry_msgs::PoseStamped pose_end;
        path.poses.back().pose.orientation =
            path.poses[path.poses.size() - 2].pose.orientation;
        pose_end = path.poses.back();
        pose_end.pose.position.x = pose_end.pose.position.x +
                                   tolerance_path_ * cos(orient_two_last_wp_);
        pose_end.pose.position.y = pose_end.pose.position.y +
                                   tolerance_path_ * sin(orient_two_last_wp_);
        pose_end.pose.orientation =
            tf::createQuaternionMsgFromYaw(orient_end_wp_);
        temp_path.push_back(path.poses.back());
        temp_path.push_back(pose_end);
    } else {
        if (revert_orient_ && !temp_path.empty()) {
            temp_path.front().pose.orientation.x = 1;
        }
        temp_path.push_back(path.poses.back());
    }
    path.poses = temp_path;
}

double WaypointGlobalPlanner::GetNormaliceAngle(double angle) {
    // ---------------------------------------------------------------------
    // Normalize an angle to [-pi, pi].
    // :param angle: (double)
    // :return: (double) double in radian in [-pi, pi]
    // ---------------------------------------------------------------------
    if (angle > M_PI) angle -= 2 * M_PI;
    if (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}
double WaypointGlobalPlanner::quaternionToEuler(tf::Quaternion quat_tf) {
    // ---------------------------------------------------------------------
    // Convert quaterniton to euler angle.
    // :param quat_tf: (tf::Quaternion)
    // :return: (double) yaw in radian
    // ---------------------------------------------------------------------
    double roll, pitch, yaw;
    tf::Matrix3x3 m(quat_tf);
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

}  // namespace waypoint_global_planner