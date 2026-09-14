#ifndef COMMON_FUNCTION_H
#define COMMON_FUNCTION_H

#include <algorithm>
#include <any>
#include <cstdlib>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <map>
#include <ros/ros.h>
#include <string>
#include <tf/transform_listener.h>
#include <vector>
namespace common_function
{
class CommonFunction
{
public:
  // CommonFunction();
  // ~CommonFunction();
   geometry_msgs::Pose lockupPose(tf::TransformListener& tf_listener,
                                        const std::string& base_frame,
                                        const std::string& robot_frame);
   double distanceTwoPose(const geometry_msgs::Pose& src,
                                const geometry_msgs::Pose& des);
  // static std::vector<std::string>
  // objToDictRecursive(void* obj, std::map<std::string, std::any>& base_dict,
  //                    const std::string& str = "", bool clear_list = false);
  // static std::map<std::string, std::any>
  // objToDict(void* obj, std::map<std::string, std::any>& base_dict);
};
}  // namespace common_functioncmo
#endif  // COMMON_FUNCTION_H