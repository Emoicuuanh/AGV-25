#ifndef RANSAC_CIRCLE_H
#define RANSAC_CIRCLE_H
#include <mirror_detect/Headers.h>
#include <random>

struct PointIntensity {
  double x;
  double y;
  float intensity;
};

struct Point2D {
  float x;
  float y;
};
class RANSAC {
public:
    RANSAC(const std::vector<Point2D>& points, int n) : points(points), n(n), d_min(std::numeric_limits<double>::max()), best_model{0, 0, 0} {}

    void execute_ransac() {
        for (int i = 0; i < n; ++i) {
            std::vector<Point2D> sample = random_sampling();
            std::array<double, 3> model = make_model(sample);
            double d_temp = eval_model(model);

            if (d_min > d_temp) {
                best_model = model;
                d_min = d_temp;
            }
        }
    }

    std::array<double, 3> get_best_model() const {
        return best_model;
    }

private:
    std::vector<Point2D> random_sampling() {
        std::vector<Point2D> sample;
        std::vector<int> save_ran;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, points.size() - 1);

        while (sample.size() < 3) {
            int ran = dis(gen);

            if (std::find(save_ran.begin(), save_ran.end(), ran) == save_ran.end()) {
                sample.push_back(points[ran]);
                save_ran.push_back(ran);
            }
        }

        return sample;
    }

    std::array<double, 3> make_model(const std::vector<Point2D>& sample) {
        const Point2D& pt1 = sample[0];
        const Point2D& pt2 = sample[1];
        const Point2D& pt3 = sample[2];

        Eigen::Matrix2d A;
        A << pt2.x - pt1.x, pt2.y - pt1.y,
             pt3.x - pt2.x, pt3.y - pt2.y;

        Eigen::Vector2d B;
        B << (pt2.x * pt2.x - pt1.x * pt1.x + pt2.y * pt2.y - pt1.y * pt1.y),
             (pt3.x * pt3.x - pt2.x * pt2.x + pt3.y * pt3.y - pt2.y * pt2.y);

        Eigen::Vector2d C = A.inverse() * B;
        double c_x = C(0) / 2.0;
        double c_y = C(1) / 2.0;
        double r = std::sqrt((c_x - pt1.x) * (c_x - pt1.x) + (c_y - pt1.y) * (c_y - pt1.y));

        return {c_x, c_y, r};
    }

    double eval_model(const std::array<double, 3>& model) {
        double d = 0;
        double c_x = model[0];
        double c_y = model[1];
        double r = model[2];

        for (const auto& point : points) {
            double dis = std::sqrt((point.x - c_x) * (point.x - c_x) + (point.y - c_y) * (point.y - c_y));

            if (dis >= r) {
                d += dis - r;
            } else {
                d += r - dis;
            }
        }

        return d;
    }
    
    const std::vector<Point2D>& points;
    int n;
    double d_min;
    std::array<double, 3> best_model; // x y radius
};

#endif //