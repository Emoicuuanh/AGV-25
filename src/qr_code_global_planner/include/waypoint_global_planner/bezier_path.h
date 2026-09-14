#ifndef BEZIER_PATH_H_
#define BEZIER_PATH_H_

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace bezier_path
{
class Bezier
{
public:
  // constructor
  Bezier();
  // destructor
  ~Bezier();

  void
  calcBezierPath(std::vector<geometry_msgs::PoseStamped>& bezier_path,
                 const std::vector<geometry_msgs::PoseStamped>& control_points,
                 int number_points);

  /**
   * @brief Return one point on the bezier curve.
   * @param t: (double) number in [0, 1]
   * @param control_points: (const std::vector<geometry_msgs::PoseStamped>& )
   * @return: PoseStamped of the point
   */
  geometry_msgs::PoseStamped
  bezier(double t,
         const std::vector<geometry_msgs::PoseStamped>& control_points);

  /**
   * @brief Bernstein polynomial
   * @param n: (int) polynom degree
   * @param i: (int)
   * @param t: (double)
   * @return: (double)
   */
  double bernsteinPoly(int n, int i, double t);

  // computer combination
  int combination(int k, int n);
};

};  // namespace bezier_path

#endif
