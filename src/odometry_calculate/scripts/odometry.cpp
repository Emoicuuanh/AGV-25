#include <agv_msgs/EncoderDifferential.h>
#include <boost/array.hpp>
#include <cmath>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

using namespace std;

// Class KalmanFilter1D tách riêng để tái sử dụng dễ dàng
class KalmanFilter1D {
public:
  double x; // trạng thái ước lượng
  double p; // hiệp phương sai ước lượng
  double q; // hiệp phương sai nhiễu quá trình
  double r; // hiệp phương sai nhiễu đo
  double k; // hệ số Kalman

  KalmanFilter1D(double q_, double r_, double p_, double initial_x) {
    q = q_;
    r = r_;
    p = p_;
    x = initial_x;
  }

  void reset(double initial_x = 0) {
    x = initial_x;
    p = 1.0;
    k = 0;
  }

  double update(double measurement) {
    // Dự đoán
    p = p + q;

    // Cập nhật đo đạc
    k = p / (p + r);
    x = x + k * (measurement - x);
    p = (1 - k) * p;

    return x;
  }
};

class Odometry_cal {
private:
  const boost::array<double, 36UL> ODOM_POSE_COVARIANCE = {{
      0.25, 0, 0, 0,   0, 0, 0, 0.25, 0, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
      0,    0, 0, 1e6, 0, 0, 0, 0,    0, 0, 1e6, 0, 0, 0, 0,   0, 0, 0.1,
  }};
  const boost::array<double, 36UL> ODOM_TWIST_COVARIANCE = {{
      1e-2, 0, 0, 0,   0, 0, 0, 1e-2, 0, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
      0,    0, 0, 1e6, 0, 0, 0, 0,    0, 0, 1e6, 0, 0, 0, 0,   0, 0, 0.1,
  }};
  int PULLING_FREQ = 100;
  int LEFT = 0;
  int RIGHT = 1;

  float WHEEL_RADIUS = 0.0625;
  float WHEEL_SEPARATION = 0.427;
  bool MOTOR_DIRECTION = true;

  float ENCODER_RESOLUTION = 16384;
  float TICK_PER_ROUND = ENCODER_RESOLUTION;
  float RAD_PER_TICK = (2 * M_PI) / TICK_PER_ROUND;
  float TICK_PER_RAD = TICK_PER_ROUND / (2 * M_PI);

  geometry_msgs::Pose odom_pose;
  vector<double> odom_vel = {0.0, 0.0, 0.0};
  vector<float> last_diff_tick = {0, 0};
  vector<double> last_velocity = {0.0, 0.0};
  vector<double> last_rad = {0.0, 0.0};
  vector<double> last_tick = {0.0, 0.0};

  float theta = 0.0; // Dùng biến góc duy nhất
  float rad = 0.0;

  agv_msgs::EncoderDifferential motor_encoder_msg;
  nav_msgs::Odometry odom_msg;

  ros::Publisher odom_pub;
  ros::Publisher reset_encoder_pub;
  ros::Publisher motor_vel_pub;

  ros::Subscriber motor_encoder_sub;
  ros::Subscriber reset_odom_sub;
  ros::NodeHandle pnh;
  ros::NodeHandle nh;

  bool use_absolute_odom = false;
  bool use_encoder_odom = true;
  bool fake_map_odom_tf = false;
  float vel_filter_threshold = 1;

  int64_t arduino_last_time = 0;

  tf2_ros::TransformBroadcaster br;

  KalmanFilter1D linear_vel_kalman;
  KalmanFilter1D angular_vel_kalman;

  // Tham số giới hạn vận tốc
  float linear_vel_limit_max = 2.0f;
  float linear_vel_limit_min = -2.0f;
  float angular_vel_limit_max = 3.0f;
  float angular_vel_limit_min = -3.0f;

  ros::Timer tf_publish_timer;

  float deg_to_rad(float deg);
  float rad_to_deg(float rad);
  float tick_to_rad(float tick);
  void update_tf(const ros::TimerEvent &);
  void updateMotorInfo(float left_tick, float right_tick);
  void calcOdometry(double diff_time, bool update_pose);

  inline float clamp(float val, float min_val, float max_val) {
    return std::max(min_val, std::min(val, max_val));
  }

  float normalizeAngle(float angle);

  void motor_encoder_cb(const agv_msgs::EncoderDifferential &msg);
  void reset_odom_cb(const std_msgs::Empty &msg);

  void resetKalmanFilters();

public:
  Odometry_cal();
};

Odometry_cal::Odometry_cal()
    : nh(), pnh("~"), linear_vel_kalman(0.05, 0.1, 1, 0),
      angular_vel_kalman(0.05, 0.1, 1, 0) {
  pnh.param<bool>("use_absolute_odom", use_absolute_odom, false);
  pnh.param<bool>("use_encoder_odom", use_encoder_odom, true);
  pnh.param<bool>("fake_map_odom_tf", fake_map_odom_tf, false);
  pnh.param<float>("vel_filter_threshold", vel_filter_threshold, 1.0);
  pnh.param<float>("wheel_radius", WHEEL_RADIUS, 0.075);
  pnh.param<float>("wheel_separation", WHEEL_SEPARATION, 0.523);
  pnh.param<float>("encoder_resolution", ENCODER_RESOLUTION, 1024);
  pnh.param<bool>("motor_direction", MOTOR_DIRECTION, false);

  // Tham số Kalman filter có thể cấu hình qua param server
  double q_linear, r_linear, q_angular, r_angular;
  pnh.param<double>("kalman_q_linear", q_linear, 0.05);
  pnh.param<double>("kalman_r_linear", r_linear, 0.1);
  pnh.param<double>("kalman_q_angular", q_angular, 0.001);
  pnh.param<double>("kalman_r_angular", r_angular, 0.1);

  linear_vel_kalman.q = q_linear;
  linear_vel_kalman.r = r_linear;
  angular_vel_kalman.q = q_angular;
  angular_vel_kalman.r = r_angular;

  // Giới hạn vận tốc cấu hình được
  pnh.param<float>("linear_vel_limit_max", linear_vel_limit_max, 2.0f);
  pnh.param<float>("linear_vel_limit_min", linear_vel_limit_min, -2.0f);
  pnh.param<float>("angular_vel_limit_max", angular_vel_limit_max, 3.0f);
  pnh.param<float>("angular_vel_limit_min", angular_vel_limit_min, -3.0f);

  TICK_PER_ROUND = ENCODER_RESOLUTION;
  RAD_PER_TICK = (2 * M_PI) / TICK_PER_ROUND;
  TICK_PER_RAD = TICK_PER_ROUND / (2 * M_PI);

  odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);
  reset_encoder_pub = nh.advertise<std_msgs::Empty>("/reset_encoder", 1);
  motor_vel_pub = nh.advertise<std_msgs::String>("/motor_vel_pub", 1);

  motor_encoder_sub =
      nh.subscribe("/motor_encoder", 1, &Odometry_cal::motor_encoder_cb, this);
  reset_odom_sub =
      nh.subscribe("/reset_odom", 1, &Odometry_cal::reset_odom_cb, this);

  // Khởi tạo thời gian last_time bằng thời điểm hiện tại
  arduino_last_time = ros::Time::now().toNSec();

  reset_encoder_pub.publish(std_msgs::Empty());

  // Timer để publish tf với tần số 50Hz
  tf_publish_timer =
      nh.createTimer(ros::Duration(0.02), &Odometry_cal::update_tf, this);

  ROS_INFO_STREAM("TICK_PER_ROUND: " << TICK_PER_ROUND);
  ROS_INFO_STREAM("RAD_PER_TICK: " << RAD_PER_TICK);
  ROS_INFO_STREAM("ENCODER_RESOLUTION: " << ENCODER_RESOLUTION);
}

void Odometry_cal::motor_encoder_cb(const agv_msgs::EncoderDifferential &msg) {
  motor_encoder_msg = msg;

  if ((use_absolute_odom && false /*abs_odom_received*/) ||
      (!use_absolute_odom)) {
    if (use_encoder_odom) {
      int64_t arduino_current_time = motor_encoder_msg.header.stamp.toNSec();

      // Kiểm tra tràn số hoặc thời gian ngược
      if (arduino_current_time <= arduino_last_time) {
        ROS_WARN("Timestamp đi ngược hoặc trùng, bỏ qua update odometry");
        return;
      }

      double arduino_diff_time =
          (arduino_current_time - arduino_last_time) / 1e9;

      // Bỏ qua nếu thời gian quá nhỏ hoặc quá lớn
      if (arduino_diff_time < 1e-4 || arduino_diff_time > 1.0) {
        ROS_WARN("Ignoring odometry update due to invalid diff_time: %f",
                 arduino_diff_time);
      } else {
        calcOdometry(arduino_diff_time, !use_absolute_odom);
      }
      arduino_last_time = arduino_current_time;
    }
  }

  // Publish odometry message
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_footprint";
  odom_pub.publish(odom_msg);
}

void Odometry_cal::reset_odom_cb(const std_msgs::Empty &msg) {
  // Reset vị trí odometry
  odom_msg.pose.pose.position.x = 0.0;
  odom_msg.pose.pose.position.y = 0.0;
  odom_msg.pose.pose.position.z = 0.0;

  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = 0.0;
  odom_msg.pose.pose.orientation.w = 1.0;

  theta = 0.0;
  rad = 0.0;

  // Reset Kalman filter để tránh sai số tích tụ
  resetKalmanFilters();

  // Reset tick encoder để tránh nhảy tick lớn do reset
  last_tick[LEFT] = 0;
  last_tick[RIGHT] = 0;
  last_diff_tick[LEFT] = 0;
  last_diff_tick[RIGHT] = 0;

  ROS_INFO("Odometry and Kalman filters reset.");
}

float Odometry_cal::deg_to_rad(float deg) { return (deg * M_PI / 180); }

float Odometry_cal::rad_to_deg(float rad) { return (rad * 180 / M_PI); }

float Odometry_cal::tick_to_rad(float tick) { return (tick * RAD_PER_TICK); }

float Odometry_cal::normalizeAngle(float angle) {
  while (angle > M_PI)
    angle -= 2 * M_PI;
  while (angle < -M_PI)
    angle += 2 * M_PI;
  return angle;
}

void Odometry_cal::update_tf(const ros::TimerEvent &) {
  if (fake_map_odom_tf) {
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0.0, 0.0, 0.0);
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "odom";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;

    transformStamped.transform.rotation.x = myQuaternion.x();
    transformStamped.transform.rotation.y = myQuaternion.y();
    transformStamped.transform.rotation.z = myQuaternion.z();
    transformStamped.transform.rotation.w = myQuaternion.w();
    br.sendTransform(transformStamped);
  }

  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = "base_footprint";
  transformStamped.transform.translation.x = odom_msg.pose.pose.position.x;
  transformStamped.transform.translation.y = odom_msg.pose.pose.position.y;
  transformStamped.transform.translation.z = odom_msg.pose.pose.position.z;

  transformStamped.transform.rotation.x = odom_msg.pose.pose.orientation.x;
  transformStamped.transform.rotation.y = odom_msg.pose.pose.orientation.y;
  transformStamped.transform.rotation.z = odom_msg.pose.pose.orientation.z;
  transformStamped.transform.rotation.w = odom_msg.pose.pose.orientation.w;
  br.sendTransform(transformStamped);
}

void Odometry_cal::updateMotorInfo(float left_tick, float right_tick) {
  // Xử lý hướng động cơ
  if (!MOTOR_DIRECTION) {
    left_tick = -left_tick;
    right_tick = -right_tick;
  }

  // Xử lý reset hoặc tràn tick encoder
  if (fabs(left_tick - last_tick[LEFT]) > (TICK_PER_ROUND * 0.9)) {
    ROS_WARN("Left encoder tick jump detected, resetting last_tick");
    last_tick[LEFT] = left_tick;
    last_diff_tick[LEFT] = 0;
  } else {
    last_diff_tick[LEFT] = left_tick - last_tick[LEFT];
    last_tick[LEFT] = left_tick;
  }
  last_rad[LEFT] += tick_to_rad(last_diff_tick[LEFT]);

  if (fabs(right_tick - last_tick[RIGHT]) > (TICK_PER_ROUND * 0.9)) {
    ROS_WARN("Right encoder tick jump detected, resetting last_tick");
    last_tick[RIGHT] = right_tick;
    last_diff_tick[RIGHT] = 0;
  } else {
    last_diff_tick[RIGHT] = right_tick - last_tick[RIGHT];
    last_tick[RIGHT] = right_tick;
  }
  last_rad[RIGHT] += tick_to_rad(last_diff_tick[RIGHT]);
}

void Odometry_cal::calcOdometry(double diff_time, bool update_pose) {
  if (diff_time < 1e-4 || diff_time > 1.0)
    return;

  updateMotorInfo(motor_encoder_msg.left, motor_encoder_msg.right);

  float wheel_l_raw = tick_to_rad(last_diff_tick[LEFT]);
  float wheel_r_raw = tick_to_rad(last_diff_tick[RIGHT]);

  if (fabs(wheel_l_raw) > 2 || fabs(wheel_r_raw) > 2) {
    ROS_ERROR("Odometry wheel rad out of range: left=%.3f, right=%.3f",
              wheel_l_raw, wheel_r_raw);
    return;
  }

  float delta_s = WHEEL_RADIUS * (wheel_r_raw + wheel_l_raw) / 2.0;
  float delta_theta =
      WHEEL_RADIUS * (wheel_r_raw - wheel_l_raw) / WHEEL_SEPARATION;

  float theta_old = theta;
  theta += delta_theta;
  theta = normalizeAngle(theta);

  if (update_pose) {
    float delta_theta_half = delta_theta / 2.0;
    float theta_mid = theta_old + delta_theta_half;
    float cos_term = cos(theta_mid);
    float sin_term = sin(theta_mid);

    odom_msg.pose.pose.position.x += delta_s * cos_term;
    odom_msg.pose.pose.position.y += delta_s * sin_term;
  }

  // Tính vận tốc thô
  float linear_vel_raw = delta_s / diff_time;
  float angular_vel_raw = delta_theta / diff_time;

  linear_vel_raw =
      clamp(linear_vel_raw, linear_vel_limit_min, linear_vel_limit_max);
  angular_vel_raw =
      clamp(angular_vel_raw, angular_vel_limit_min, angular_vel_limit_max);

  if (fabs(linear_vel_raw - linear_vel_kalman.x) > vel_filter_threshold) {
    ROS_WARN_THROTTLE(1,
                      "Linear velocity jump detected: raw=%.3f, filtered=%.3f",
                      linear_vel_raw, linear_vel_kalman.x);
  }
  if (fabs(angular_vel_raw - angular_vel_kalman.x) > vel_filter_threshold) {
    ROS_WARN_THROTTLE(1,
                      "Angular velocity jump detected: raw=%.3f, filtered=%.3f",
                      angular_vel_raw, angular_vel_kalman.x);
  }

  float linear_vel_filtered = linear_vel_kalman.update(linear_vel_raw);
  float angular_vel_filtered = angular_vel_kalman.update(angular_vel_raw);

  odom_msg.twist.twist.linear.x = linear_vel_filtered;
  odom_msg.twist.twist.angular.z = angular_vel_filtered;

  last_velocity[LEFT] = wheel_l_raw / diff_time;
  last_velocity[RIGHT] = wheel_r_raw / diff_time;

  if (update_pose) {
    odom_msg.pose.pose.position.z = 0.0;
    tf2::Quaternion abc;
    abc.setRPY(0.0, 0.0, theta);
    odom_msg.pose.pose.orientation.x = abc.x();
    odom_msg.pose.pose.orientation.y = abc.y();
    odom_msg.pose.pose.orientation.z = abc.z();
    odom_msg.pose.pose.orientation.w = abc.w();
  }

  odom_msg.pose.covariance = ODOM_POSE_COVARIANCE;
  odom_msg.twist.covariance = ODOM_TWIST_COVARIANCE;
}

void Odometry_cal::resetKalmanFilters() {
  linear_vel_kalman.reset(0.0);
  angular_vel_kalman.reset(0.0);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry");
  Odometry_cal odom_cal;
  ros::spin();
  return 0;
}
