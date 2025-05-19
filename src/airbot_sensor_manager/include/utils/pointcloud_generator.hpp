#ifndef __POINTCLOUD_GENERATOR__
#define __POINTCLOUD_GENERATOR__

#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "vision_msgs/msg/bounding_box2_d_array.hpp"
#include "utils/common_struct.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

class PointCloudGenerator
{
public:
    PointCloudGenerator();
    ~PointCloudGenerator();

    sensor_msgs::msg::PointCloud2 mergePointCloud2Vector(const std::vector<sensor_msgs::msg::PointCloud2>& pc_msgs, std::string frame);
    sensor_msgs::msg::PointCloud2 generatePointCloud2Message(const std::vector<tPoint> &points, std::string frame);
    sensor_msgs::msg::PointCloud2 generatePointCloud2Message(const vision_msgs::msg::BoundingBox2DArray input_bbox_array, float resolution);
    sensor_msgs::msg::PointCloud2 generatePointCloud2EmptyMessage(const std::string &frame);
private:
};

#endif // POINTCLOUD_GENERATOR