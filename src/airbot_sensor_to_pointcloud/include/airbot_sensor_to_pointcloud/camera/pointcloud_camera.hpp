#ifndef __POINTCLOUD_CAMERA_HPP__
#define __POINTCLOUD_CAMERA_HPP__

#include <cmath>
#include <string>
#include "robot_custom_msgs/msg/ai_data.hpp"
#include "robot_custom_msgs/msg/ai_data_array.hpp"
#include "utils/pointcloud_generator.hpp"
#include "utils/boundingbox_generator.hpp"

class PointCloudCamera
{
public:
    PointCloudCamera(double camera_sensor_frame_x_translate,
                     double camera_sensor_frame_y_translate,
                     double camera_sensor_frame_z_translate);
    ~PointCloudCamera();

    void updateTargetFrame(std::string &updated_frame);
    void updateRobotPose(tPose &pose);
    vision_msgs::msg::BoundingBox2DArray updateCameraBoundingBoxMsg(const robot_custom_msgs::msg::AIDataArray::SharedPtr msg, std::vector<long int> class_id_list, int th_confidence, bool direction);
    sensor_msgs::msg::PointCloud2 updateCameraPointCloudMsg(vision_msgs::msg::BoundingBox2DArray msg, float pc_resolution);

private:
    std::shared_ptr<PointCloudGenerator> pointcloud_generator_;
    std::shared_ptr<BoundingBoxGenerator> boundingbox_generator_;

    tPose robot_pose_;
    std::string target_frame_;
    tPoint camera_translation_;
    vision_msgs::msg::BoundingBox2DArray camera_bbox_array_;
};

#endif //POINTCLOUD_CAMERA_HPP