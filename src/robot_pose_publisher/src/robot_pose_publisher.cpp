#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_pose_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // Parameters
  std::string map_frame;
  std::string base_frame, laser_safety_frame, laser_map_frame;
  double publish_frequency;
  bool is_stamped;

  nh_priv.param<std::string>("map_frame", map_frame, "/map");
  nh_priv.param<std::string>("base_frame", base_frame, "");
  nh_priv.param<std::string>("laser_safety_frame", laser_safety_frame, "");
  nh_priv.param<std::string>("laser_map_frame", laser_map_frame, "");
  nh_priv.param<double>("publish_frequency", publish_frequency, 10);
  nh_priv.param<bool>("is_stamped", is_stamped, false);

  // Publishers (optional depending on input frame)
  ros::Publisher p_pub0, p_pub1, p_pub2;
  if (!base_frame.empty())
  {
    if (is_stamped)
      p_pub0 = nh.advertise<geometry_msgs::PoseStamped>("robot_pose", 1);
    else
      p_pub0 = nh.advertise<geometry_msgs::Pose>("robot_pose", 1);
  }

  if (!laser_safety_frame.empty())
  {
    if (is_stamped)
      p_pub1 = nh.advertise<geometry_msgs::PoseStamped>("laser_safety_pose", 1);
    else
      p_pub1 = nh.advertise<geometry_msgs::Pose>("laser_safety_pose", 1);
  }

  if (!laser_map_frame.empty())
  {
    if (is_stamped)
      p_pub2 = nh.advertise<geometry_msgs::PoseStamped>("laser_map_pose", 1);
    else
      p_pub2 = nh.advertise<geometry_msgs::Pose>("laser_map_pose", 1);
  }

  tf::TransformListener listener;
  ros::Rate rate(publish_frequency);

  std::map<std::string, ros::Time> last_failed_time;

  while (ros::ok())
  {
    ros::Time now = ros::Time::now();

    if (!base_frame.empty())
    {
      bool can_try = (last_failed_time.count(base_frame) == 0 ||
                      (now - last_failed_time[base_frame]).toSec() > 2.0);
      if (can_try)
      {
        try
        {
          tf::StampedTransform tf_robot;
          listener.lookupTransform(map_frame, base_frame, ros::Time(0),
                                   tf_robot);

          geometry_msgs::PoseStamped pose;
          pose.header.frame_id = map_frame;
          pose.header.stamp = now;
          pose.pose.position.x = tf_robot.getOrigin().x();
          pose.pose.position.y = tf_robot.getOrigin().y();
          pose.pose.position.z = tf_robot.getOrigin().z();
          pose.pose.orientation.x = tf_robot.getRotation().x();
          pose.pose.orientation.y = tf_robot.getRotation().y();
          pose.pose.orientation.z = tf_robot.getRotation().z();
          pose.pose.orientation.w = tf_robot.getRotation().w();

          if (is_stamped)
            p_pub0.publish(pose);
          else
            p_pub0.publish(pose.pose);

          last_failed_time.erase(base_frame);  // success
        }
        catch (tf::TransformException& ex)
        {
          last_failed_time[base_frame] = now;
        }
      }
    }

    if (!laser_safety_frame.empty())
    {
      bool can_try =
          (last_failed_time.count(laser_safety_frame) == 0 ||
           (now - last_failed_time[laser_safety_frame]).toSec() > 2.0);
      if (can_try)
      {
        try
        {
          tf::StampedTransform tf_laser;
          listener.lookupTransform(map_frame, laser_safety_frame, ros::Time(0),
                                   tf_laser);

          geometry_msgs::PoseStamped pose;
          pose.header.frame_id = map_frame;
          pose.header.stamp = now;
          pose.pose.position.x = tf_laser.getOrigin().x();
          pose.pose.position.y = tf_laser.getOrigin().y();
          pose.pose.position.z = tf_laser.getOrigin().z();
          pose.pose.orientation.x = tf_laser.getRotation().x();
          pose.pose.orientation.y = tf_laser.getRotation().y();
          pose.pose.orientation.z = tf_laser.getRotation().z();
          pose.pose.orientation.w = tf_laser.getRotation().w();

          if (is_stamped)
            p_pub1.publish(pose);
          else
            p_pub1.publish(pose.pose);

          last_failed_time.erase(laser_safety_frame);
        }
        catch (tf::TransformException& ex)
        {
          last_failed_time[laser_safety_frame] = now;
        }
      }
    }

    if (!laser_map_frame.empty())
    {
      bool can_try = (last_failed_time.count(laser_map_frame) == 0 ||
                      (now - last_failed_time[laser_map_frame]).toSec() > 2.0);
      if (can_try)
      {
        try
        {
          tf::StampedTransform tf_laser_map;
          listener.lookupTransform(map_frame, laser_map_frame, ros::Time(0),
                                   tf_laser_map);

          geometry_msgs::PoseStamped pose;
          pose.header.frame_id = map_frame;
          pose.header.stamp = now;
          pose.pose.position.x = tf_laser_map.getOrigin().x();
          pose.pose.position.y = tf_laser_map.getOrigin().y();
          pose.pose.position.z = tf_laser_map.getOrigin().z();
          pose.pose.orientation.x = tf_laser_map.getRotation().x();
          pose.pose.orientation.y = tf_laser_map.getRotation().y();
          pose.pose.orientation.z = tf_laser_map.getRotation().z();
          pose.pose.orientation.w = tf_laser_map.getRotation().w();

          if (is_stamped)
            p_pub2.publish(pose);
          else
            p_pub2.publish(pose.pose);

          last_failed_time.erase(laser_map_frame);
        }
        catch (tf::TransformException& ex)
        {
          last_failed_time[laser_map_frame] = now;
        }
      }
    }

    rate.sleep();
  }

  return EXIT_SUCCESS;
}
