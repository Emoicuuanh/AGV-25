#include "util.h"

geometry_msgs::Pose
PoseConverter::lockupPose(tf::TransformListener& tf_listener,
                          const std::string& base_frame,
                          const std::string& robot_frame)
{
  try
  {
    tf_listener.waitForTransform(base_frame, robot_frame, ros::Time(0),
                                 ros::Duration(1.0));
    tf::StampedTransform transform;
    tf_listener.lookupTransform(base_frame, robot_frame, ros::Time(0),
                                transform);
    geometry_msgs::Pose ret;
    ret.position.x = transform.getOrigin().x();
    ret.position.y = transform.getOrigin().y();
    ret.position.z = transform.getOrigin().z();
    ret.orientation.x = transform.getRotation().x();
    ret.orientation.y = transform.getRotation().y();
    ret.orientation.z = transform.getRotation().z();
    ret.orientation.w = transform.getRotation().w();
    return ret;
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN_STREAM("lockupPose: " << ex.what());
    return geometry_msgs::Pose();
  }
}

double PoseConverter::distanceTwoPose(const geometry_msgs::Pose& src,
                                      const geometry_msgs::Pose& des)
{
  double x = des.position.x - src.position.x;
  double y = des.position.y - src.position.y;
  return std::sqrt(x * x + y * y);
}

std::vector<std::string>
ObjToDict::objToDictRecursive(void* obj,
                              std::map<std::string, std::any>& base_dict,
                              const std::string& str, bool clear_list)
{
  std::vector<std::string> cmd_list;
  if (clear_list)
  {
    cmd_list.clear();
  }

  for (auto& [key, value] : base_dict)
  {
    if (value.type() == typeid(std::map<std::string, std::any>))
    {
      if (obj)
      {
        auto it = std::any_cast<std::map<std::string, std::any>>(value);
        if (it.find(key) != it.end())
        {
          objToDictRecursive(obj, it, str + key + ".");
        }
        else
        {
          std::cout << "objToDictRecursive: object has no attr \"" << key
                    << "\"" << std::endl;
        }
      }
    }
    else
    {
      std::string att = "_obj." + str + key;
      std::replace(att.begin(), att.end(), '.', '[');
      att += "]";
      std::string cmd =
          base_dict[key].type().name() + " " + att + " = " + att + ";";
      cmd_list.push_back(cmd);
    }
  }

  return cmd_list;
}

std::map<std::string, std::any>
ObjToDict::objToDict(void* obj, std::map<std::string, std::any>& base_dict)
{
  std::vector<std::string> cmd_list =
      objToDictRecursive(obj, base_dict, "", true);
  for (const auto& cmd : cmd_list)
  {
    system(cmd.c_str());
  }
  return base_dict;
}