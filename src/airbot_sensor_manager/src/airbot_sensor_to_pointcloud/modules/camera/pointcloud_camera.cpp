#include "airbot_sensor_to_pointcloud/modules/camera/pointcloud_camera.hpp"

PointCloudCamera::PointCloudCamera()
{
}

PointCloudCamera::~PointCloudCamera()
{
}

sensor_msgs::msg::PointCloud2 PointCloudCamera::updateCameraPointCloudMsg(vision_msgs::msg::BoundingBox2DArray msg, float pc_resolution, const std::vector<int> bbox_ai_ids, const geometry_msgs::msg::PoseWithCovarianceStamped init_pose_msg)
{
    return pointcloud_generator_->generatePointCloud2Message(msg, pc_resolution, bbox_ai_ids, init_pose_msg);
}
