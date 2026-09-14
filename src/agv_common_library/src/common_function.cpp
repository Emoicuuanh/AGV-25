#include <agv_common_library/common_function.h>
namespace common_function
{
geometry_msgs::Pose
CommonFunction::lockupPose(tf::TransformListener& tf_listener,
                           const std::string& base_frame,
                           const std::string& robot_frame)
{
  try
  {
    ros::Time now = ros::Time::now();
    tf_listener.waitForTransform(base_frame, robot_frame, now,
                                 ros::Duration(1.0));
    tf::StampedTransform transform;

    tf_listener.lookupTransform(base_frame, robot_frame, now, transform);
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

double CommonFunction::distanceTwoPose(const geometry_msgs::Pose& src,
                                       const geometry_msgs::Pose& des)
{
  double x = des.position.x - src.position.x;
  double y = des.position.y - src.position.y;
  return std::sqrt(x * x + y * y);
}

// std::vector<std::string>
// CommonFunction::objToDictRecursive(void* obj,
//                                    std::map<std::string, std::any>& base_dict,
//                                    const std::string& str, bool clear_list)
// {
//   static std::vector<std::string> cmd_list;

//   if (clear_list)
//   {
//     cmd_list.clear();
//   }

//   if (typeid(base_dict) == typeid(std::map<std::string, std::any>))
//   {
//     for (auto& [key, value] : base_dict)
//     {
//       if (value.type() == typeid(std::map<std::string, std::any>))
//       {
//         auto& nested_map =
//             std::any_cast<std::map<std::string, std::any>&>(value);
//         objToDictRecursive(obj, nested_map, str + key + ".", false);
//       }
//       else
//       {
//         std::string att = "_obj." + str + key;
//         std::vector<std::string> att_arr;
//         std::stringstream ss(att);
//         std::string item;
//         while (std::getline(ss, item, '.'))
//         {
//           if (!item.empty())
//             att_arr.push_back(item);
//         }

//         std::string att_str = "_base_dict";
//         for (const auto& a : att_arr)
//         {
//           att_str += "[\"" + a + "\"]";
//         }
//         std::string cmd = att_str + " = " + att;
//         cmd_list.push_back(cmd);
//       }
//     }
//   }
//   else
//   {
//     throw std::runtime_error("objToDictRecursive: input is not a dict");
//   }
//   return cmd_list;
// }

// std::map<std::string, std::any>
// CommonFunction::objToDict(void* obj, std::map<std::string, std::any>& base_dict)
// {
//   std::vector<std::string> cmd_list =
//       objToDictRecursive(obj, base_dict, "", true);
//   for (const auto& cmd : cmd_list)
//   {
//     std::cout << cmd << std::endl;  // Replace exec with a safer operation
//   }
//   return base_dict;
// }
}  // namespace common_function