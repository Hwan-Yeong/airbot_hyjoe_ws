#ifndef __POINTCLOUD_CAMERA_HPP__
#define __POINTCLOUD_CAMERA_HPP__

#include <cmath>
#include <string>
#include "robot_custom_msgs/msg/camera_data.hpp"
#include "robot_custom_msgs/msg/camera_data_array.hpp"
#include "utils/pointcloud_generator.hpp"
#include "utils/boundingbox_generator.hpp"

class PointCloudCamera
{
public:
    PointCloudCamera();
    ~PointCloudCamera();

    sensor_msgs::msg::PointCloud2 updateCameraPointCloudMsg(vision_msgs::msg::BoundingBox2DArray msg, float pc_resolution);

private:
    std::shared_ptr<PointCloudGenerator> pointcloud_generator_;
};

#endif //POINTCLOUD_CAMERA_HPP