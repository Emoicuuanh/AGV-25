#! /usr/bin/env python
# -*- coding: utf-8 -*-
import yaml
import os
import sys
import rospy
import rospkg
import math
import numpy as np
from math import *
from nav_msgs.msg import Odometry
from agv_msgs.msg import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import (
    Twist,
    Pose,
    PoseStamped,
    PoseWithCovarianceStamped,
)
from std_msgs.msg import Bool, Int16, String, Int16MultiArray, Int8
from math import pi, sqrt
from tf.listener import TransformListener, Transformer
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from std_stamped_msgs.msg import (
    StringAction,
    StringStamped,
    StringResult,
    StringFeedback,
    StringGoal,
    Int8Stamped,
    EmptyStamped,
    Float32Stamped,
    Int16MultiArrayStamped,
)

pkg_path = rospkg.RosPack().get_path("test_io_agv")
results_path = os.path.join(pkg_path, "results")

class CALIB_CAM(object):
    def __init__(self, name, *args, **kwargs):
        # """
        # ..######..##.....##.########...######..########..####.########..########.########.
        # .##....##.##.....##.##.....##.##....##.##.....##..##..##.....##.##.......##.....##
        # .##.......##.....##.##.....##.##.......##.....##..##..##.....##.##.......##.....##
        # ..######..##.....##.########..##.......########...##..########..######...########.
        # .......##.##.....##.##.....##.##.......##...##....##..##.....##.##.......##...##..
        # .##....##.##.....##.##.....##.##....##.##....##...##..##.....##.##.......##....##.
        # ..######...#######..########...######..##.....##.####.########..########.##.....##
        # """

        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/data_gls621", DataMatrixStamped, self.data_gls_cb)


        # """
        # .########..##.....##.########..##.......####..######..##.....##.########.########.
        # .##.....##.##.....##.##.....##.##........##..##....##.##.....##.##.......##.....##
        # .##.....##.##.....##.##.....##.##........##..##.......##.....##.##.......##.....##
        # .########..##.....##.########..##........##...######..#########.######...########.
        # .##........##.....##.##.....##.##........##........##.##.....##.##.......##...##..
        # .##........##.....##.##.....##.##........##..##....##.##.....##.##.......##....##.
        # .##.........#######..########..########.####..######..##.....##.########.##.....##
        # """

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)

        self.init_variable(*args, **kwargs)
        self.loop()

    def yaml_load(self, filepath):
        try:
            with open(filepath, "r") as read_file:
                data = yaml.load(read_file, Loader=yaml.FullLoader)
            return data
        except Exception as e:
            rospy.logerr("Load yaml file error: {}".format(e))

    def init_variable(self, *args, **kwargs):
        self.rate = rospy.Rate(10)
        self.pose_odom2robot = None
        self.qr_angle = None
        self.dis_qr_x = None
        self.dis_qr_y = None
        self.last_label_qr_x = None
        self.last_label_qr_y = None

    def shutdown(self):
        rospy.loginfo("Shuting down")

    """
    .########.##.....##.##....##..######..########.####..#######..##....##
    .##.......##.....##.###...##.##....##....##.....##..##.....##.###...##
    .##.......##.....##.####..##.##..........##.....##..##.....##.####..##
    .######...##.....##.##.##.##.##..........##.....##..##.....##.##.##.##
    .##.......##.....##.##..####.##..........##.....##..##.....##.##..####
    .##.......##.....##.##...###.##....##....##.....##..##.....##.##...###
    .##........#######..##....##..######.....##....####..#######..##....##
    """
    def normalize_angle(self, angle):
        """Đưa góc về khoảng [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi
    def rotate_to_reduce_angle(self, target_yaw=0.0, angular_speed=0.05, threshold=math.radians(0.2)):
        """
        Xoay robot cho đến khi qr_angle gần bằng target_yaw với sai số cho phép.
        Publish tốc độ góc qua cmd_vel.

        Args:
            target_yaw (float): Góc mục tiêu (rad), mặc định 0.0
            angular_speed (float): Vận tốc xoay (rad/s)
            threshold (float): Sai số cho phép (rad)
        """
        rate = rospy.Rate(20)  # 20 Hz
        twist = Twist()

        while not rospy.is_shutdown():
            if self.qr_angle is None:
                rate.sleep()
                continue

            yaw_diff = self.normalize_angle(target_yaw - self.qr_angle)

            if abs(yaw_diff) < threshold:
                # Dừng robot khi đã đạt góc
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                rospy.loginfo("Đã đạt góc mục tiêu!")
                break

            # Xác định hướng xoay (trái hoặc phải)
            direction = 1.0 if yaw_diff > 0 else -1.0

            # Publish vận tốc góc
            twist.angular.z = direction * angular_speed
            self.cmd_vel_pub.publish(twist)

            rospy.loginfo(f"Đang xoay: qr_angle={math.degrees(self.qr_angle):.2f}°, Sai số={math.degrees(yaw_diff):.2f}°")

            rate.sleep()

        return self.qr_angle

    def calibrate_angle(self, save_path="/tmp/offset_angle.yaml"):

        # rotation accuracy
        max_retries = 10   # số lần thử tối đa
        retry_count = 0
        success = False

        self.reset_data()
        while not rospy.is_shutdown() and (self.qr_angle is None or self.pose_odom2robot is None):
            rospy.sleep(0.1)

        while not rospy.is_shutdown() and retry_count < max_retries:
            # Chuyển radian sang độ
            qr_angle_deg = math.degrees(self.qr_angle)

            # Kiểm tra điều kiện nhỏ hơn 0.1 độ
            if abs(qr_angle_deg) < 0.2:
                rospy.sleep(1.0)  # đợi 1 giây để ổn định

                # Kiểm tra lại sau khi đợi
                qr_angle_deg = math.degrees(self.qr_angle)
                if abs(qr_angle_deg) < 0.2:
                    print("Góc đã được hiệu chỉnh thành công")
                    success = True
                    break
                else:
                    print("Sau khi đợi 1s, góc vẫn chưa đạt. Thử lại...")
            else:
                print("Góc chưa đạt, tiếp tục xoay...")

            # Gửi lệnh xoay để giảm self.qr_angle về gần 0
            self.rotate_to_reduce_angle(target_yaw=0.0, angular_speed=0.03)  # hàm cần được định nghĩa để thực hiện xoay

            retry_count += 1
            rospy.sleep(0.5)

        # calib here
        if not success:
            rospy.loginfo("Cannot rotation correct angle 0 do")
            return
        rospy.loginfo("BEGIN calibrate angle -> Waiting for initial QR data...")
        self.reset_data()
        while not rospy.is_shutdown() and (self.qr_angle is None or self.pose_odom2robot is None):
            rospy.sleep(0.1)

        # Lưu dữ liệu ban đầu
        init_qr_x = self.dis_qr_x
        init_qr_y = self.dis_qr_y
        init_label_x = self.last_label_qr_x
        init_label_y = self.last_label_qr_y
        start_angle_begin = self.qr_angle
        init_pose = self.pose_odom2robot

        start_x = init_pose.position.x
        start_y = init_pose.position.y
        init_yaw = self.get_yaw(init_pose)

        twist = Twist()
        twist.linear.x = 0.05  # m/s
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

            # Khoảng cách di chuyển từ odom
            cur_pose = self.pose_odom2robot
            dist_odom = sqrt((cur_pose.position.x - start_x) ** 2 +
                            (cur_pose.position.y - start_y) ** 2)

            # Điều kiện dừng: đọc QR khác hoặc di chuyển quá 1m
            if (self.last_label_qr_x != init_label_x or
                    self.last_label_qr_y != init_label_y) and dist_odom >= 0.1:
                break
            if (self.last_label_qr_x == init_label_x and
                    self.last_label_qr_y == init_label_y) and dist_odom < 0.05:
                start_angle_begin = self.qr_angle
            if dist_odom > 0.6:
                rospy.logwarn("Exceeded maximum movement (0.6m), stopping!")
                return

        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1.0)
        self.reset_data()
        while not rospy.is_shutdown() and (self.qr_angle is None or self.pose_odom2robot is None):
            rospy.sleep(0.1)


        after_angle = self.qr_angle
        after_qr_x = self.dis_qr_x
        after_qr_y = self.dis_qr_y
        cur_pose = self.pose_odom2robot
        dist_odom = sqrt((cur_pose.position.x - start_x) ** 2 +
                        (cur_pose.position.y - start_y) ** 2)
        after_yaw = self.get_yaw(cur_pose)

        # Tính offset_angle
        delta_x = abs(after_qr_x - init_qr_x)
        if dist_odom < 0.3:
            rospy.logwarn("Not enough movement to calculate offset_angle!")
            return
        if abs(after_angle - start_angle_begin) > math.radians(1):
            rospy.logerr("error angel before and after: {} rad".format(abs(after_angle - start_angle_begin)))
            rospy.logwarn("Robot not move straigth -> fail to calib")
            return

        offset_angle = asin(delta_x / dist_odom)
        if after_qr_x <= init_qr_x:
            offset_angle = -offset_angle

        #final
        offset_angle += start_angle_begin

        # Lưu dữ liệu vào YAML
        data_to_save = {
            "before": {
                "qr_x": float(init_qr_x),
                "qr_y": float(init_qr_y),
                "odom": {
                    "x": float(init_pose.position.x),
                    "y": float(init_pose.position.y),
                    "yaw": float(init_yaw)
                }
            },
            "after": {
                "qr_x": float(after_qr_x),
                "qr_y": float(after_qr_y),
                "odom": {
                    "x": float(cur_pose.position.x),
                    "y": float(cur_pose.position.y),
                    "yaw": float(after_yaw)
                }
            },
            "dist_odom": float(dist_odom),
            "offset_angle": float(offset_angle)
        }

        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        with open(save_path, "a") as f:
            yaml.dump(data_to_save, f)

        rospy.loginfo(f"Calibration offset angle saved to {save_path}")
        rospy.loginfo(f"offset_angle={offset_angle}")
        return offset_angle

    def calibrate_position(self, save_path="/tmp/offset_position.yaml"):
        rospy.loginfo("BEGIN calibrate_position x,y -> Waiting for initial data...")
        self.reset_data()
        while not rospy.is_shutdown() and ( self.qr_angle is None or self.pose_odom2robot is None):
            rospy.sleep(0.1)

        # Lưu giá trị ban đầu
        init_pose = self.pose_odom2robot
        init_yaw = self.get_yaw(init_pose)
        init_dis_x = self.dis_qr_x
        init_dis_y = self.dis_qr_y
        rospy.loginfo(f"Initial QR data: x={init_dis_x}, y={init_dis_y}")

        # Quay robot 180 độ với vận tốc 0.1 rad/s
        twist = Twist()
        twist.angular.z = 0.1
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            current_yaw = self.get_yaw(self.pose_odom2robot)
            yaw_diff = self.angle_diff(current_yaw, init_yaw)
            rospy.logwarn("yaw_diff: {}".format(yaw_diff))
            if abs(yaw_diff) >= math.radians(179):  # quay đủ 180 độ
                break
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        # Dừng quay
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1.0)
        self.reset_data()
        while not rospy.is_shutdown() and (self.qr_angle is None or self.pose_odom2robot is None):
            rospy.sleep(0.1)

        # Lấy giá trị sau khi quay
        after_pose = self.pose_odom2robot
        after_dis_x = self.dis_qr_x
        after_dis_y = self.dis_qr_y
        after_yaw = self.get_yaw(after_pose)
        rospy.loginfo(f"After rotation QR data: x={after_dis_x}, y={after_dis_y}")

        # Tính offset
        offset_x, offset_y = None, None
        if after_dis_x < -init_dis_x:
            offset_x = abs(init_dis_x + after_dis_x) / 2.0
        else:
            offset_x = -abs(init_dis_x + after_dis_x) / 2.0
        if after_dis_y < -init_dis_y:
            offset_y = abs(init_dis_y + after_dis_y) / 2.0
        else:
            offset_y = -abs(init_dis_y + after_dis_y) / 2.0

        # Lưu dữ liệu vào YAML
        data_to_save = {
            "before": {
                "qr_x": float(init_dis_x),
                "qr_y": float(init_dis_y),
                "odom": {
                    "x": float(init_pose.position.x),
                    "y": float(init_pose.position.y),
                    "yaw": float(init_yaw)
                }
            },
            "after": {
                "qr_x": float(after_dis_x),
                "qr_y": float(after_dis_y),
                "odom": {
                    "x": float(after_pose.position.x),
                    "y": float(after_pose.position.y),
                    "yaw": float(after_yaw)
                }
            },
            "offset_x": offset_x,
            "offset_y": offset_y
        }

        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        with open(save_path, "a") as f:
            yaml.dump(data_to_save, f)

        rospy.loginfo(f"Calibration offset positon data saved to {save_path}")
        rospy.loginfo(f"Offset_x={offset_x}, Offset_y={offset_y}")
        return offset_x, offset_y

    def get_yaw(self, pose):
        quat = pose.orientation
        (_, _, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return yaw

    def angle_diff(self, a, b):
        d = a - b
        while d > pi:
            d -= 2 * pi
        while d < -pi:
            d += 2 * pi
        return d

    def loop(self):
        while not rospy.is_shutdown():
            user_input = input("Chọn hàm offset [1=calibrate_position, 2=calibrate_angle, q=quit]: ")

            if user_input.lower() == "q":
                rospy.loginfo("Exit loop.")
                break

            # Reset các biến trước khi chạy hàm
            self.reset_data()
            if user_input == "1":
                rospy.loginfo("Running calibrate_position...")
                save_file = os.path.join(results_path, "offset_position.yaml")
                self.calibrate_position(save_path=save_file)
            elif user_input == "2":
                rospy.loginfo("Running calibrate_angle...")
                save_file = os.path.join(results_path, "offset_angle.yaml")
                self.calibrate_angle(save_path=save_file)
            else:
                rospy.logwarn("Invalid input. Please enter 1, 2 or q.")


    def reset_data(self):
        self.pose_odom2robot = None
        self.qr_angle = None
        self.dis_qr_x = None
        self.dis_qr_y = None
        self.last_label_qr_x = None
        self.last_label_qr_y = None


    """
     ######     ###    ##       ##       ########     ###     ######  ##    ##
    ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
    ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
    ##       ##     ## ##       ##       ########  ##     ## ##       #####
    ##       ######### ##       ##       ##     ## ######### ##       ##  ##
    ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
     ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
    """

    def odom_cb(self, msg):
        self.pose_odom2robot = msg.pose.pose

    def data_gls_cb(self, msg):
        self.qr_angle = msg.possition.angle
        self.dis_qr_x = float(msg.possition.x) / 1000.0
        self.dis_qr_y = float(msg.possition.y) / 1000.0
        self.last_label_qr_x = float(msg.lable.x) / 1000
        self.last_label_qr_y = float(msg.lable.y) / 1000



def parse_opts():
    from optparse import OptionParser

    parser = OptionParser()
    parser.add_option(
        "-d",
        "--ros_debug",
        action="store_true",
        dest="log_debug",
        default=False,
        help="log_level=rospy.DEBUG",
    )

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)


def main():
    (options, args) = parse_opts()
    log_level = None
    if options.log_debug:
        log_level = rospy.DEBUG
    rospy.init_node("calib_cam", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    CALIB_CAM(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
