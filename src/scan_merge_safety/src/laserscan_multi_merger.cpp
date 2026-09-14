#include <ros/ros.h>
#include <string.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "sensor_msgs/LaserScan.h"
#include "pcl_ros/point_cloud.h"
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <scan_merge_safety/laserscan_multi_mergerConfig.h>
#include <safety_msgs/SafetyStatus.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <safety_msgs/SafetyJob.h>

using namespace std;
using namespace pcl;
using namespace laserscan_multi_merger;

class LaserscanMerger
{
public:
	typedef boost::geometry::model::d2::point_xy<double> point_type;
	typedef boost::geometry::model::polygon<point_type> polygon_type;
	LaserscanMerger();
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan, std::string topic);
	void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud);
	void reconfigureCallback(laserscan_multi_mergerConfig &config, uint32_t level);
	void safetyCallback(const safety_msgs::SafetyJob::ConstPtr &msg);
	point_type from_geometry_msgs_point(const geometry_msgs::Point32 &p);
	polygon_type from_geometry_msgs_polygon(const geometry_msgs::Polygon &poly_msg);
	~LaserscanMerger()
	{
		delete polygon_prt;
	}

private:
	ros::NodeHandle node_;
	laser_geometry::LaserProjection projector_;
	tf::TransformListener tfListener_;

	ros::Publisher point_cloud_publisher_;
	ros::Publisher scan_safety_publisher_;
	ros::Publisher laser_scan_publisher_;
	vector<ros::Subscriber> scan_subscribers;
	ros::Subscriber safety_job_sub_;

	vector<bool> clouds_modified;

	vector<pcl::PCLPointCloud2> clouds;
	vector<string> input_topics;

	void laserscan_topic_parser();

	double angle_min;
	double angle_max;
	double angle_increment;
	double time_increment;
	double scan_time;
	double range_min;
	double range_max;
	double safety_range_max;

	string destination_frame;
	string cloud_destination_topic;
	string scan_destination_topic;
	string laserscan_topics;
	string safety_status_topic;
	string safety_job_topic;
	bool check_safety_job;
	int num_job;
	int div_factor_count;	// Biến đếm số lần laser callback
	int div_factor_pub;		// Hệ số chia tần số quét vật cản từ tần số phát laser
	int min_laser_to_merge; // Số topic laser tối thiểu nhận được để publish laser merge
	int min_point_safety; // Số topic laser tối thiểu nhận được để publish laser merge
	double min_dist_two_point;
	int min_point_safety_field_end;

	// Khai báo footprint, safety job
	polygon_type footprint_pol, *polygon_prt;
	// Khai báo điểm
	point_type point_laser;
};

// Hàm chuyển đổi từ geometry_msgs::Point32 sang point_type
LaserscanMerger::point_type LaserscanMerger::from_geometry_msgs_point(const geometry_msgs::Point32 &p)
{
	return point_type(p.x, p.y);
}

// Hàm chuyển đổi từ geometry_msgs::Polygon sang polygon
LaserscanMerger::polygon_type LaserscanMerger::from_geometry_msgs_polygon(const geometry_msgs::Polygon &poly_msg)
{
	polygon_type poly;

	// Lặp qua danh sách các điểm và thêm chúng vào outer ring của polygon
	for (const auto &p : poly_msg.points)
	{
		boost::geometry::append(poly.outer(), LaserscanMerger::from_geometry_msgs_point(p));
	}
	// Sắp xếp lại thứ tự các đỉnh cho đúng thứ tự thuận chiều kim đồng hồ
	boost::geometry::correct(poly);

	return poly;
}

void LaserscanMerger::reconfigureCallback(laserscan_multi_mergerConfig &config, uint32_t level)
{
	this->angle_min = config.angle_min;
	this->angle_max = config.angle_max;
	this->angle_increment = config.angle_increment;
	this->time_increment = config.time_increment;
	this->scan_time = config.scan_time;
	this->range_min = config.range_min;
	this->range_max = config.range_max;
}

void LaserscanMerger::laserscan_topic_parser()
{
	// LaserScan topics to subscribe
	ros::master::V_TopicInfo topics;

	istringstream iss(laserscan_topics);
	set<string> tokens;
	copy(istream_iterator<string>(iss), istream_iterator<string>(), inserter<set<string>>(tokens, tokens.begin()));
	vector<string> tmp_input_topics;

	while (!tokens.empty())
	{
		ROS_INFO("Waiting for topics ...");
		ros::master::getTopics(topics);
		sleep(1);

		for (int i = 0; i < topics.size(); i++)
		{
			if (topics[i].datatype == "sensor_msgs/LaserScan" && tokens.erase(topics[i].name) > 0)
			{
				tmp_input_topics.push_back(topics[i].name);
			}
		}
	}

	sort(tmp_input_topics.begin(), tmp_input_topics.end());
	std::vector<string>::iterator last = std::unique(tmp_input_topics.begin(), tmp_input_topics.end());
	tmp_input_topics.erase(last, tmp_input_topics.end());

	// Do not re-subscribe if the topics are the same
	if ((tmp_input_topics.size() != input_topics.size()) || !equal(tmp_input_topics.begin(), tmp_input_topics.end(), input_topics.begin()))
	{

		// Unsubscribe from previous topics
		for (int i = 0; i < scan_subscribers.size(); i++)
			scan_subscribers[i].shutdown();

		input_topics = tmp_input_topics;

		if (input_topics.size() > 0)
		{
			scan_subscribers.resize(input_topics.size());
			clouds_modified.resize(input_topics.size());
			clouds.resize(input_topics.size());
			ROS_INFO("Subscribing to topics\t%ld", scan_subscribers.size());
			for (int i = 0; i < input_topics.size(); ++i)
			{
				scan_subscribers[i] = node_.subscribe<sensor_msgs::LaserScan>(input_topics[i].c_str(), 1, boost::bind(&LaserscanMerger::scanCallback, this, _1, input_topics[i]));
				clouds_modified[i] = false;
				std::cout << input_topics[i] << " ";
			}
		}
		else
			ROS_INFO("Not subscribed to any topic.");
	}
}

LaserscanMerger::LaserscanMerger()
{
	ros::NodeHandle nh("~");

	nh.param<std::string>("destination_frame", destination_frame, "cart_frame");
	nh.param<std::string>("cloud_destination_topic", cloud_destination_topic, "/merged_cloud");
	nh.param<std::string>("scan_destination_topic", scan_destination_topic, "/scan_multi");
	nh.param<std::string>("laserscan_topics", laserscan_topics, "");
	nh.param("angle_min", angle_min, -2.36);
	nh.param("angle_max", angle_max, 2.36);
	nh.param("angle_increment", angle_increment, 0.0058);
	nh.param("scan_time", scan_time, 0.0333333);
	nh.param("range_min", range_min, 0.45);
	nh.param("range_max", range_max, 25.0);
	nh.param("min_laser_to_merge", min_laser_to_merge, 0); // default = 0 is publish when all subscribed scans have arrived
	nh.param("min_point_safety", min_point_safety, 2); // min number of point to confirm have object in safety zone
	nh.param("min_point_safety_field_end", min_point_safety_field_end, 2); // min number of point to confirm have object in safety zone
	nh.param("min_dist_two_point", min_dist_two_point, 0.05);


	// Add safety params
	nh.param("check_safety_job", check_safety_job, false);
	nh.param("div_factor_pub", div_factor_pub, 2);
	nh.param("safety_range_max", safety_range_max, 5.0);
	nh.param<std::string>("safety_job_topic", safety_job_topic, "/safety_job_topic");
	nh.param<std::string>("safety_status_topic", safety_status_topic, "/safety_status_topic");

	point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(cloud_destination_topic.c_str(), 1, false);
	laser_scan_publisher_ = node_.advertise<sensor_msgs::LaserScan>(scan_destination_topic.c_str(), 1, false);
	// add scan safety
	scan_safety_publisher_ = node_.advertise<safety_msgs::SafetyStatus>(safety_status_topic.c_str(), 1, false);
	safety_job_sub_ = node_.subscribe(safety_job_topic.c_str(), 10, &LaserscanMerger::safetyCallback, this);
	// Khởi tạo footprint
	boost::geometry::read_wkt("POLYGON((-0.3 -0.3,0.3 -0.3,0.3 0.3,-0.3 0.3,-0.3 -0.3))", this->footprint_pol);
	num_job = 0;
	div_factor_count = 0;
	if (check_safety_job)
	{
		ROS_INFO("Check safety job is enable.");
	}
	else
	{
		ROS_INFO("Check safety job is disable.");
	}
	this->laserscan_topic_parser();
}

void LaserscanMerger::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan, std::string topic)
{
	sensor_msgs::PointCloud tmpCloud1, tmpCloud2;
	sensor_msgs::PointCloud2 tmpCloud3;

	// refer to http://wiki.ros.org/tf/Tutorials/tf%20and%20Time%20%28C%2B%2B%29
	try
	{
		// Verify that TF knows how to transform from the received scan to the destination scan frame
		tfListener_.waitForTransform(scan->header.frame_id.c_str(), destination_frame.c_str(), scan->header.stamp, ros::Duration(1));
		projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan, tmpCloud1, tfListener_, laser_geometry::channel_option::Distance);
		tfListener_.transformPointCloud(destination_frame.c_str(), tmpCloud1, tmpCloud2);
	}
	catch (tf::TransformException ex)
	{
		return;
	}
	// Compare topic subcribe with list topic
	for (int i = 0; i < input_topics.size(); i++)
	{
		// if topic in list topic then set clouds_modified to true and save point clound to vector clouds
		if (topic.compare(input_topics[i]) == 0)
		{
			sensor_msgs::convertPointCloudToPointCloud2(tmpCloud2, tmpCloud3);
			pcl_conversions::toPCL(tmpCloud3, clouds[i]);
			clouds_modified[i] = true;
		}
	}

	// Count how many scans we have
	int totalClouds = 0;
	for (int i = 0; i < clouds_modified.size(); i++)
		if (clouds_modified[i])
			totalClouds++;

	// constrain min_laser_to_merge
	if ((min_laser_to_merge <= 0) || (min_laser_to_merge >= clouds_modified.size()))
	{
		min_laser_to_merge = clouds_modified.size();
	}

	// Go ahead only if all subscribed scans have arrived
	// Go ahead when topic arrived over min_laser_to_merge
	if (totalClouds >= min_laser_to_merge)
	{
		pcl::PCLPointCloud2 merged_cloud = clouds[0];
		clouds_modified[0] = false;

		for (int i = 1; i < clouds_modified.size(); i++)
		{
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
			pcl::concatenate(merged_cloud, clouds[i], merged_cloud);
#else
			pcl::concatenatePointCloud(merged_cloud, clouds[i], merged_cloud);
#endif
			clouds_modified[i] = false;
		}

		point_cloud_publisher_.publish(merged_cloud);

		Eigen::MatrixXf points;
		getPointCloudAsEigen(merged_cloud, points);
		// ROS_INFO("getPointCloudAsEigen");
		pointcloud_to_laserscan(points, &merged_cloud);
	}
}

void LaserscanMerger::safetyCallback(const safety_msgs::SafetyJob::ConstPtr &msg)
{
	if (check_safety_job == false)
		return;

	footprint_pol = from_geometry_msgs_polygon(msg->footprint); // chuyển đổi sang polygon
	std::cout << "Footprint field: " << boost::geometry::wkt(footprint_pol);
	num_job = msg->jobs.size();
	if (num_job > 0)
	{
		ROS_INFO("Num Job: %d", num_job);
		polygon_prt = new polygon_type[num_job]; // tao đối tượng chứa các job theo polygon_type
		// Truyền giá trị của các polygon từ message sang đối tượng vừa tạo
		for (int i = 0; i < num_job; i++)
		{
			polygon_prt[i] = from_geometry_msgs_polygon(msg->jobs[i]); // chuyển đổi sang polygon
			std::cout << "polygon field" << i << ":" << boost::geometry::wkt(polygon_prt[i]);

			ROS_INFO("Creat Polygon field: %d", i);
		}
	}
	else
	{
		ROS_WARN("No job data");
	}
}

void LaserscanMerger::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud) {
    safety_msgs::SafetyStatusPtr field_data(new safety_msgs::SafetyStatus());
    if (check_safety_job) {
        div_factor_count++;
        if (div_factor_count >= div_factor_pub) {
            field_data->header = pcl_conversions::fromPCL(merged_cloud->header);
            field_data->job = 0;
            field_data->fields.assign(num_job, 0); // Set all fields to empty
        }
    }

    sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
    output->header = pcl_conversions::fromPCL(merged_cloud->header);
    output->angle_min = this->angle_min;
    output->angle_max = this->angle_max;
    output->angle_increment = this->angle_increment;
    output->time_increment = this->time_increment;
    output->scan_time = this->scan_time;
    output->range_min = this->range_min;
    output->range_max = this->range_max;

    uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
    // Initialize all laser values to range_max + 1
    output->ranges.assign(ranges_size, output->range_max + 1.0);

    // Initialize array to store the count of points in safety fields
    std::vector<int> pointCount(num_job, 0);

    // Variables to store the previous point's coordinates for each field
    std::vector<bool> has_previous_point(num_job, false);
    std::vector<Eigen::Vector2f> previous_points(num_job);

    // Iterate through each point in the point cloud
    for (int i = 0; i < points.cols(); i++) {
        const float &x = points(0, i);
        const float &y = points(1, i);
        const float &z = points(2, i);

        if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
            ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
            continue;
        }

        double range_sq = std::pow(y, 2) + std::pow(x, 2);
        double range_min_sq_ = std::pow(output->range_min, 2);
        if (range_sq < range_min_sq_) {
            ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
            continue;
        }

        double angle = std::atan2(y, x);
        if (angle < output->angle_min || angle > output->angle_max) {
            ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
            continue;
        }
        int index = (angle - output->angle_min) / output->angle_increment;
        // Update the range if the current point is closer
        if (std::pow(output->ranges[index], 2) > range_sq)
            output->ranges[index] = std::sqrt(range_sq);

        /* Check scan safety */
        if ((div_factor_count >= div_factor_pub) && (check_safety_job)) {
            if (num_job == 0) {
                ROS_DEBUG("No job or footprint");
                continue;
            }
            if ((range_sq > std::pow(safety_range_max, 2)) || (field_data->fields[0] == 1)) {
                ROS_DEBUG("range max or have job in field 0");
                continue;
            }

            try {
                point_laser.set<0>(x); // Set X coordinate
                point_laser.set<1>(y); // Set Y coordinate

                if (boost::geometry::within(point_laser, footprint_pol)) {
                    ROS_DEBUG("Point is in footprint");
                    continue;
                }
            } catch (...) {
                ROS_WARN("ERROR when find footprint.");
            }

            // Check each field from outer to inner
            for (int id = num_job; id > 0; id--) {
                if (field_data->fields[id - 1] == 1) {
                    continue;
                }
                if (boost::geometry::within(point_laser, polygon_prt[id - 1])) {
                    // Check the distance between the previous and current points in the same field
                    if (has_previous_point[id - 1]) {
                        float distance = std::sqrt(std::pow(previous_points[id - 1].x() - x, 2) + std::pow(previous_points[id - 1].y() - y, 2));
						previous_points[id - 1] = Eigen::Vector2f(x, y);
						if (distance > min_dist_two_point) { // 0.05 units distance threshold
                            ROS_DEBUG("Point too close to the previous one in field %d", id);
                            continue;
                        }
                    }

                    // Update the previous point coordinates for the current field
                    previous_points[id - 1] = Eigen::Vector2f(x, y);
                    has_previous_point[id - 1] = true;

                    pointCount[id - 1]++;
                    if (pointCount[id - 1] >= min_point_safety) {
                        field_data->fields[id - 1] = 1;
                        ROS_DEBUG("Have object in field %d", id);
                    }
					if (id == 1 && pointCount[id - 1] >= min_point_safety_field_end) {
                        field_data->fields[id - 1] = 1;
                        ROS_DEBUG("Have object in field %d", id);
                    }
                }
            }
        }
    }

    laser_scan_publisher_.publish(output);
    if (div_factor_count >= div_factor_pub) {
        scan_safety_publisher_.publish(field_data);
        div_factor_count = 0;
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_multi_merger");

	LaserscanMerger _laser_merger;

	dynamic_reconfigure::Server<laserscan_multi_mergerConfig> server;
	dynamic_reconfigure::Server<laserscan_multi_mergerConfig>::CallbackType f;

	f = boost::bind(&LaserscanMerger::reconfigureCallback, &_laser_merger, _1, _2);
	server.setCallback(f);

	ros::spin();

	return 0;
}
