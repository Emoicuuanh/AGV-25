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

#ifndef INCLUDE_NEOLOCALPLANNER_H_
#define INCLUDE_NEOLOCALPLANNER_H_

#include <agv_msgs/DataMatrixStamped.h>
#include <agv_msgs/ErrorRobotToPath.h>
#include <angles/angles.h>
#include <base_local_planner/Position2DInt.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/world_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dynamic_reconfigure/server.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <neo_local_planner/NeoPlannerConfig.h>
#include <safety_msgs/SafetyStatus.h>
#include <std_msgs/Bool.h>
#include <std_stamped_msgs/EmptyStamped.h>
#include <std_stamped_msgs/Int16MultiArrayStamped.h>
#include <std_stamped_msgs/Int8Stamped.h>
#include <std_stamped_msgs/StringService.h>
#include <std_stamped_msgs/StringStamped.h>
#include <tf2_ros/buffer.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <nlohmann/json.hpp>
namespace neo_local_planner {

class NeoLocalPlanner : public nav_core::BaseLocalPlanner {
   public:
    NeoLocalPlanner();

    ~NeoLocalPlanner();

    bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel) override;

    bool isGoalReached() override;

    bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) override;

    void initialize(std::string name, tf2_ros::Buffer *tf,
                    costmap_2d::Costmap2DROS *costmap_ros) override;

    double GetNormaliceAngle(double angle);

    bool getSucceedPose(int type_stop, bool m_is_goal_reached_by_qr_code,
                        double xy_goal_tolerance, double x_error,
                        double y_error, double start_to_goal);

   private:
    void disableCheckLostQrCb(
        const std_stamped_msgs::Int8Stamped::ConstPtr &msg);
    void missionStatusCb(const std_stamped_msgs::StringStamped::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void safetyCallback(const safety_msgs::SafetyStatus::ConstPtr &msg);
    void safetyCameraCallback(const safety_msgs::SafetyStatus::ConstPtr &msg);
    void reconfigureCB(NeoPlannerConfig &config, uint32_t level);
    void runPauseByTrafficCallback(const std_msgs::Bool::ConstPtr &msg);
    void runPauseCallback(const std_stamped_msgs::StringStamped::ConstPtr &msg);
    void dataQrCodeCallback(const agv_msgs::DataMatrixStamped::ConstPtr &msg);
    void publishStateDirection(const std::string &state,
                               const std::string &direction);

   private:
    tf2_ros::Buffer *m_tf = 0;
    costmap_2d::Costmap2DROS *m_cost_map = 0;
    std::vector<geometry_msgs::PoseStamped> m_global_plan;

    boost::mutex m_odometry_mutex;
    base_local_planner::LocalPlannerLimits m_limits = {};
    nav_msgs::Odometry::ConstPtr m_odometry;
    safety_msgs::SafetyStatus::ConstPtr m_safety_status;
    safety_msgs::SafetyStatus::ConstPtr m_safety_camera_status;

    ros::Subscriber disable_check_lost_qr_subscriber_;
    ros::Subscriber mission_manager_module_status_subscriber_;
    ros::Subscriber m_qr_code_data_sub;
    ros::Subscriber m_pause_smoother_sub;
    ros::Subscriber m_pause_by_traffic_sub;
    ros::Subscriber m_odom_sub;
    ros::Subscriber m_safety_status_sub;
    ros::Subscriber m_safety_camera_status_sub;

    ros::Publisher m_local_plan_pub;
    ros::Publisher m_set_safety_job_pub;
    ros::Publisher m_set_footprint_job_pub;
    ros::Publisher error_to_path_pub;

    ros::Publisher lost_goal_label_pub;

    ros::Publisher pub_state_direction_;

    std::string m_global_frame = "map";
    std::string m_local_frame = "odom";
    std::string m_base_frame = "base_link";
    std::string m_is_pause_smoother = "";

    uint64_t m_update_counter = 0;

    int count = 0;
    int safety_field_idx = 10;
    int safety_camera_field_idx = 10;
    int type_stop = 0;

    double dis_qr_x = 1000.000;
    double dis_qr_y = 1000.000;
    double last_label_qr_x = 100000000.0;
    double last_label_qr_y = 100000000.0;
    double m_goal_tune_time = 0;               // [s]
    double m_lookahead_time = 0;               // [s]
    double m_lookahead_dist = 0;               // [m]
    double m_start_yaw_error_normal = 0;       // [rad]
    double m_start_yaw_error_when_safety = 0;  // [rad]
    double m_pos_x_gain = 0;                   // [1/s]
    double m_pos_y_gain = 0;                   // [1/s]
    double m_pos_y_yaw_gain = 0;               // [rad/s^2]
    double m_yaw_gain = 0;                     // [1/s]
    double m_static_yaw_gain = 0;              // [1/s]
    double m_cost_x_gain = 0;
    double m_cost_y_gain = 0;
    double m_cost_y_yaw_gain = 0;
    double m_cost_y_lookahead_dist = 0;  // [m]
    double m_cost_y_lookahead_time = 0;  // [s]
    double m_cost_yaw_gain = 0;
    double m_low_pass_gain = 0;
    double low_pass_gain_dynamic = 0;
    double m_max_curve_vel = 0;        // [rad/s]
    double m_max_goal_dist = 0;        // [m]
    double max_goal_dist_dynamic = 0;  // [m]
    double m_max_backup_dist = 0;      // [m]
    double m_max_cost = 0;             // [1]
    double m_stop_acc = 0;
    double m_min_stop_dist = 0;            // [m]
    double m_emergency_acc_lim_x = 0;      // [m/s^2]
    double m_emergency_acc_lim_theta = 0;  // [m/s^2]
    double m_robot_direction = 1.0;
    double m_vel_max_safety_field_0 = 0;
    double m_vel_max_safety_field_1 = 0;
    double m_vel_max_safety_field_2 = 0;
    double m_vel_max_safety_field_3 = 0;
    double time_begin_rotation = 0;  // [s]
    double m_max_time_check_stop_by_safety = 0;
    double m_tolerance_stop_by_qr_code = 0.05;
    double m_min_vel_stop_by_qr_code = 0.03;
    double min_vel_move_straight = 0.03;
    double min_vel_move_rotation = 0.05;
    double m_dist_stop_agv_to_qr_code = 0.02;
    double m_xy_error = 1000;
    double m_last_control_values[3] = {};
    double max_r = 1.5;
    double dis_check_qr_for_stop = 0.15;
    double max_dis_to_find_qr = 0.08;
    double start_to_goal = 0.0;
    double m_lookahead_time_stop = 0.0;
    double m_dynamic_lookahead_time_stop = 0.0;
    bool disable_low_pass_filter_move_straight = false;
    bool disable_low_pass_filter_move_rotate = false;

    bool pause_by_traffic = false;
    bool slow_speed = false;
    bool publish_safety = true;
    bool stop_center_qr = true;
    bool m_enable_software_stop = true;
    bool m_differential_drive = false;
    bool m_constrain_final = false;
    bool m_allow_reversing = false;
    bool m_set_zero_vel = true;
    bool m_min_dist_obstacle_allow_rotation = 0.5;
    bool is_stop_vel_x = false;
    bool is_top_vel_yaw = false;
    bool enable_rotate = false;
    bool m_print_state = false;
    bool m_is_goal_reached = false;
    bool m_is_goal_reached_by_qr_code = false;
    bool m_is_force_goal_reached = false;
    bool m_detect_qr_code = false;
    bool m_stop_by_qr_code = true;
    bool is_rotation_first = false;
    bool first_time_check;
    bool succeed_rotate_first;
    bool led_turn_on = true;
    bool get_state_stop = true;
    bool receive_new_data_qr = false;
    bool check_error_angle_begin = false;
    double last_angle_error_found_qr_code = 1000;
    double max_vel_when_not_detect_qr = 0.4;
    bool decrease_vel_when_not_detect_qr = false;

    enum state_t {
        NONE,
        STATE_IDLE,
        STATE_TRANSLATING,
        STATE_ROTATING,
        STATE_ADJUSTING,
        STATE_TURNING,
        STATE_STUCK
    };

    std::string current_action_type = "";
    std::string pre_current_action_type = "";
    std::string pre_action_type = "";
    bool check_have_qr_start_move = false;

    state_t m_state = state_t::STATE_IDLE;
    state_t m_pre_state = state_t::NONE;

    ros::WallTime m_last_time_stop_by_safety;
    ros::WallTime m_last_time_stop_by_safety_2;
    ros::WallTime m_last_time_stop_by_safety_3;
    ros::WallTime m_last_time;
    ros::WallTime m_first_goal_reached_time;
    ros::WallTime time_wait_for_move_straight_first;
    ros::WallTime last_time_detect_qr_code;
    ros::WallTime time_blink_led_turn;

    geometry_msgs::Twist m_last_cmd_vel;
    std_stamped_msgs::Int8Stamped disable_check_lost_qr;
    std_stamped_msgs::StringStamped safety_job;
    std_stamped_msgs::StringStamped safety_footprint;
    std::string safety_move_straight_name = "";
    agv_msgs::ErrorRobotToPath error_msg;
    geometry_msgs::PoseStamped global_goal_;
    geometry_msgs::PoseStamped pose_begin_in_path_;

    dynamic_reconfigure::Server<NeoPlannerConfig> *dsrv_;
    dynamic_reconfigure::Server<NeoPlannerConfig>::CallbackType f;

    ros::ServiceClient client_get_type_move_;
    std::string type_move;
};

}  // namespace neo_local_planner

#endif /* INCLUDE_NEOLOCALPLANNER_H_ */
