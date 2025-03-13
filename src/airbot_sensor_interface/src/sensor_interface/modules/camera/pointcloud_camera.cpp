#include "sensor_interface/modules/camera/pointcloud_camera.hpp"

PointCloudCamera::PointCloudCamera()
{
}

PointCloudCamera::~PointCloudCamera()
{
}

sensor_msgs::msg::PointCloud2 PointCloudCamera::updateCameraPointCloudMsg(vision_msgs::msg::BoundingBox2DArray msg, float pc_resolution)
{
    return pointcloud_generator_->generatePointCloud2Message(msg, pc_resolution);
}
