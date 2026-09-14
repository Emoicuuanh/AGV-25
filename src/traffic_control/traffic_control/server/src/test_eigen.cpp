#include <eigen3/Eigen/Dense>
#include <iostream>

int main() {
  Eigen::Vector2d a(1.0, 2.0);
  Eigen::Vector2d b(3.0, 4.0);
  std::cout << "Dot product: " << a.dot(b) << std::endl;
  return 0;
}
