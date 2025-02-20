#ifndef __POINTCLOUD_LIDAR_HPP__
#define __POINTCLOUD_LIDAR_HPP__

#include <cmath>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "utils/common_struct.hpp"

class PointCloudLidar
{
public:
    PointCloudLidar();
    ~PointCloudLidar();

    void updateParams(tLidarParam front, tLidarParam back);
    sensor_msgs::msg::PointCloud2 updateLidarPointCloudMsg(sensor_msgs::msg::LaserScan front_lidar_msg, sensor_msgs::msg::LaserScan back_lidar_msg);

private:
    std::string target_frame_;
    float front_offset_x_, front_offset_y_, front_offset_z_, front_alpha_, front_angle_min_, front_angle_max_,
        back_offset_x_, back_offset_y_, back_offset_z_, back_alpha_, back_angle_min_, back_angle_max_;

    void removePointsWithinRadius(pcl::PointCloud<pcl::PointXYZ>& cloud, float radius, float center_x = 0.0f, float center_y = 0.0f);
};

#endif //POINTCLOUD_LIDAR_HPP