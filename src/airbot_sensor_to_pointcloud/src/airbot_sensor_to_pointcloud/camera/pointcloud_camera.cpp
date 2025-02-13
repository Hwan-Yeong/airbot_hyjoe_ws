#include "airbot_sensor_to_pointcloud/camera/pointcloud_camera.hpp"

PointCloudCamera::PointCloudCamera(double camera_sensor_frame_x_translate = 0.15473,
                                   double camera_sensor_frame_y_translate = 0.0,
                                   double camera_sensor_frame_z_translate = 0.5331)
    : camera_translation_(camera_sensor_frame_x_translate,
                          camera_sensor_frame_y_translate,
                          camera_sensor_frame_z_translate)
{
    camera_bbox_array_ = vision_msgs::msg::BoundingBox2DArray();
}

PointCloudCamera::~PointCloudCamera()
{
}

/**
 * @ Description
 * ### 노드 초기화 시점에, 파라미터로 받은 frame_id로 업데이트
 */
void PointCloudCamera::updateTargetFrame(std::string &updated_frame)
{
    target_frame_ = updated_frame;
}

/**
 * @ Description
 * ### Callback 받은 시점에, topic에 찍힌 robot_pose로 업데이트
 */
void PointCloudCamera::updateRobotPose(tPose &pose)
{
    robot_pose_ = pose;
}

vision_msgs::msg::BoundingBox2DArray PointCloudCamera::updateCameraBoundingBoxMsg(const robot_custom_msgs::msg::AIDataArray::SharedPtr msg, std::vector<long int> class_id_list, int th_confidence, bool direction)
{
    camera_bbox_array_ = boundingbox_generator_->generateBoundingBoxMessage(msg,
                                                                           target_frame_,
                                                                           robot_pose_,
                                                                           camera_translation_,
                                                                           class_id_list,
                                                                           th_confidence,
                                                                           direction);
    return camera_bbox_array_;
}

sensor_msgs::msg::PointCloud2 PointCloudCamera::updateCameraPointCloudMsg(vision_msgs::msg::BoundingBox2DArray msg, float pc_resolution)
{
    return pointcloud_generator_->generateCameraPointCloud2Message(msg,
                                                                   pc_resolution);
}
