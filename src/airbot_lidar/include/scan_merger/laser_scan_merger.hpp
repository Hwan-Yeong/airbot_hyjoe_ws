#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/time.hpp"

#include <cmath>
#include <functional>
#include <string>
#include <vector>
#include <array>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace message_filters;
using SyncPolicy = sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>;

class scanMerger : public rclcpp::Node
{
public:
    scanMerger();
    ~scanMerger();
    
private:
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> laser1_sub_, laser2_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    
    sensor_msgs::msg::LaserScan laser1_, laser2_;
    std::string topic1_, topic2_, cloudTopic_, cloudFrameId_;
    bool flip1_, flip2_;
    float laser1XOff_, laser1YOff_, laser1ZOff_, laser1Alpha_, laser1AngleMin_, laser1AngleMax_;
    float laser2XOff_, laser2YOff_, laser2ZOff_, laser2Alpha_, laser2AngleMin_, laser2AngleMax_;

private:
    void initialize_params();
    void refresh_params();
    void synchronized_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser1_msg, const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser2_msg);
    void update_point_cloud_rgb();
    void removePointsWithinRadius(pcl::PointCloud<pcl::PointXYZ> &cloud, float radius);
};