#ifndef WAYPOINT_GLOBAL_PLANNER_H
#define WAYPOINT_GLOBAL_PLANNER_H

#include <agv_msgs/DirectionMove.h>
#include <agv_msgs/WaypointsPath.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/world_model.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_stamped_msgs/StringService.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "waypoint_global_planner/bezier_path.h"

namespace waypoint_global_planner {
class WaypointsBezier {
   public:
    geometry_msgs::PoseStamped Waypoint;
    std::string type;
};

/**
 * @class CarrotPlanner
 * @brief Provides a simple global planner for producing boustrophedon paths
 * without taking into account obstacles
 */
class WaypointGlobalPlanner : public nav_core::BaseGlobalPlanner {
   public:
    /**
     * @brief Default Constructor
     */
    WaypointGlobalPlanner();

    /**
     * @brief Constructor for the planner
     * @param name The name of this planner
     * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for
     * planning
     */
    WaypointGlobalPlanner(std::string name,
                          costmap_2d::Costmap2DROS *costmap_ros);

    /**
     * @brief Default destructor
     */
    ~WaypointGlobalPlanner();

    /**
     * @brief Initialization function for the planner
     * @param name The name of this planner
     * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for
     * planning
     */
    void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

    /**
     * @brief Given a goal pose in the world, compute a plan
     * @param start_pose The starting pose of the robot
     * @param goal The goal pose
     * @param plan The plan filled by the planner
     * @return True if a valid plan was found, false otherwise
     */
    bool makePlan(const geometry_msgs::PoseStamped &start_pose,
                  const geometry_msgs::PoseStamped &goal,
                  std::vector<geometry_msgs::PoseStamped> &plan);
    /**
     * @brief Interpolates a path (position and orientation) using a fixed
     * number of points per meter
     * @param path The input path to be interpolated
     */
    void interpolatePath(nav_msgs::Path &path);
    double GetNormaliceAngle(double angle);
    double quaternionToEuler(tf::Quaternion quat_tf);

    void addWaypoint(double pose_1_x, double pose_1_y, double pose_2_x,
                     double pose_2_y,
                     std::vector<geometry_msgs::PoseStamped> &plan);

    void makeWaypointBezier(
        const agv_msgs::WaypointsPathResponse &array_waypoints,
        std::vector<WaypointsBezier> &array_waypoints_bezier);
    geometry_msgs::PoseStamped transformPose(
        const geometry_msgs::PoseStamped &pose_in,
        const std::string &target_frame, tf2_ros::Buffer &tf_buffer);

   private:
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    bool initialized_;  //!< flag indicating the planner has been initialized
    costmap_2d::Costmap2DROS *costmap_ros_;        //!< costmap ros wrapper
    costmap_2d::Costmap2D *costmap_;               //!< costmap container
    base_local_planner::WorldModel *world_model_;  //!< world model

    // subscribers and publishers
    ros::Publisher
        goal_pub_;  //!< publisher of goal corresponding to the final waypoint
    ros::Publisher plan_pub_;  //!< publisher of the global plan
    ros::ServiceClient client_get_waypoints_;
    ros::ServiceClient client_get_direction_;
    ros::ServiceClient client_get_type_move_;

    // configuration parameters
    double epsilon_;  //!< distance threshold between two waypoints that
                      //!< signifies the last waypoint
    int waypoints_per_meter_;  //!< number of waypoints per meter of generated
                               //!< path used for interpolation
    double tolerance_path_;
    double orient_two_last_wp_;
    double orient_end_wp_;
    // containers
    std::vector<geometry_msgs::PoseStamped>
        waypoints_;        //!< container for the manually inserted waypoints
    nav_msgs::Path path_;  //!< container for the generated interpolated path
    // service get waypoint
    agv_msgs::WaypointsPathResponse array_waypoints_;
    bezier_path::Bezier bezier_;
    // service get direction
    agv_msgs::DirectionMoveResponse direction_;
    std::string type_move;

    // flags
    bool clear_waypoints_;  //!< flag indicating that the waypoint container
                            //!< must be cleared to start anew
    bool use_orient_goal_;
    bool revert_orient_;
};

}  // namespace waypoint_global_planner

#endif  // WAYPOINT_GLOBAL_PLANNER_H