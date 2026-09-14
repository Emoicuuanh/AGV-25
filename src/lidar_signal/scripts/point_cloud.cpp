#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

class LaserToPointCloudConverter
{
public:
    LaserToPointCloudConverter()
    {
        laser_scan_subscriber_ = nh_.subscribe("/scan", 1, &LaserToPointCloudConverter::laserScanCallback, this);
        point_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/pointcloud", 1);
    }

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
    {
        sensor_msgs::PointCloud2 point_cloud_msg;

        point_cloud_msg.header = scan_msg->header;
        point_cloud_msg.height = 1;
        point_cloud_msg.width = scan_msg->ranges.size();
        point_cloud_msg.is_bigendian = false;
        point_cloud_msg.is_dense = true;

        sensor_msgs::PointCloud2Modifier modifier(point_cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(point_cloud_msg.width * point_cloud_msg.height);

        sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg, "z");

        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {
            float range = scan_msg->ranges[i];
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

            *iter_x = range * std::cos(angle);
            *iter_y = range * std::sin(angle);
            *iter_z = 0.0;

            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        point_cloud_publisher_.publish(point_cloud_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber laser_scan_subscriber_;
    ros::Publisher point_cloud_publisher_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_to_pointcloud");

    LaserToPointCloudConverter converter;

    ros::spin();

    return 0;
}