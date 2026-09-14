#include "ros/ros.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point32.h"
#include "std_msgs/Header.h"
#include "safety_msgs/SafetyJob.h"

int main(int argc, char **argv)
{
    // Khởi tạo node và tạo một publisher để gửi thông tin về mảng polygon
    ros::init(argc, argv, "polygon_array_publisher");
    ROS_INFO("Init node polygon_array_publisher");
    ros::NodeHandle n;
    ros::Publisher polygon_array_pub = n.advertise<safety_msgs::SafetyJob>("polygon_array_topic", 10);

    // Tạo một message polygon mới và thêm các điểm vào danh sách
    geometry_msgs::Polygon polygon1, polygon2, polygon3, footprint;
    geometry_msgs::Point32 point1, point2, point3, point4, point5, point6, point7, point8, point9, point10, point11;
    point1.x = 0.0;
    point1.y = 0.3;
    point2.x = 0.0;
    point2.y = -0.3;

    point3.x = 0.8;
    point3.y = -0.3;
    point4.x = 0.8;
    point4.y = 0.3;

    polygon1.points.push_back(point1);
    polygon1.points.push_back(point2);
    polygon1.points.push_back(point3);
    polygon1.points.push_back(point4);
    polygon1.points.push_back(point1);
    point6.x = 1.2;
    point6.y = -0.4;
    point7.x = 1.2;
    point7.y = 0.4;
    polygon2.points.push_back(point1);
    polygon2.points.push_back(point2);
    polygon2.points.push_back(point6);
    polygon2.points.push_back(point7);
    polygon2.points.push_back(point1);
    point8.x = 1.6;
    point8.y = -0.5;
    point9.x = 1.6;
    point9.y = 0.5;
    polygon3.points.push_back(point1);
    polygon3.points.push_back(point2);
    polygon3.points.push_back(point8);
    polygon3.points.push_back(point9);
    polygon3.points.push_back(point1);
    point10.x = 0.4;
    point10.y = -0.3;
    point11.x = 0.4;
    point11.y = 0.3;
    footprint.points.push_back(point1);
    footprint.points.push_back(point2);
    footprint.points.push_back(point10);
    footprint.points.push_back(point11);
    footprint.points.push_back(point1);


    // polygon2.points.push_back(point8);

    // Tạo một message polygon stamped và thêm các polygon vào danh sách
    safety_msgs::SafetyJob polygon_array_msg;
    std_msgs::Header header;
    header.seq = 0;
    header.stamp = ros::Time::now();
    header.frame_id = "base_link";
    polygon_array_msg.header = header;
    polygon_array_msg.footprint = footprint;
    polygon_array_msg.jobs = {polygon1, polygon2, polygon3};
    // Gửi message đến topic
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        polygon_array_pub.publish(polygon_array_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}