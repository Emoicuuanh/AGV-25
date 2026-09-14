#include "waypoint_global_planner/bezier_path.h"

namespace bezier_path
{
Bezier::Bezier() {}

Bezier::~Bezier() {}

void Bezier::calcBezierPath(
    std::vector<geometry_msgs::PoseStamped>& bezier_path,
    const std::vector<geometry_msgs::PoseStamped>& control_points,
    int number_points)
{
  if (number_points == 0)
  {
    ROS_ERROR("error numble point input");
  }
  for (auto i = 0; i <= number_points; i++)
  {
    double t = 0 + double(i) / double(number_points);
    bezier_path.push_back(bezier(t, control_points));
    // std::cout << t << std::endl;
  }
}

geometry_msgs::PoseStamped
Bezier::bezier(double t,
               const std::vector<geometry_msgs::PoseStamped>& control_points)
{
  int n = control_points.size() - 1;
  geometry_msgs::PoseStamped point_bezier;
  point_bezier.header.frame_id = "map";
  point_bezier.pose.orientation.w = 1;
  point_bezier.pose.position.x = 0;
  point_bezier.pose.position.y = 0;
  for (auto i = 0; i <= n; i++)
  {
    point_bezier.pose.position.x +=
        bernsteinPoly(n, i, t) * control_points[i].pose.position.x;
    point_bezier.pose.position.y +=
        bernsteinPoly(n, i, t) * control_points[i].pose.position.y;
  }
  return point_bezier;
}

double Bezier::bernsteinPoly(int n, int i, double t)
{
  return combination(i, n) * pow(t, i) * pow((1 - t), (n - i));
}

int Bezier::combination(int k, int n)
{
  if (k == 0 || k == n)
    return 1;
  if (k == 1)
    return n;
  return combination(k - 1, n - 1) + combination(k, n - 1);
}

}  // namespace bezier_path