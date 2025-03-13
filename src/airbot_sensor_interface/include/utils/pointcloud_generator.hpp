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

/**
 * @brief: input 데이터를 pointcloud2 데이터 형태로 변환시켜주는 클래스입니다.
 * @input: std::vector<tPoint>, vision_msgs::msg::BoundingBox2DArray
 */
class PointCloudGenerator
{
public:
    PointCloudGenerator();
    ~PointCloudGenerator();

    sensor_msgs::msg::PointCloud2 generatePointCloud2Message(const std::vector<tPoint> &points, std::string frame);
    sensor_msgs::msg::PointCloud2 generatePointCloud2Message(const vision_msgs::msg::BoundingBox2DArray input_bbox_array, float resolution);
private:
};

#endif // POINTCLOUD_GENERATOR