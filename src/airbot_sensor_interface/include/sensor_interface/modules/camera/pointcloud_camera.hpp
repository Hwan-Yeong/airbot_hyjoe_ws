#ifndef __POINTCLOUD_CAMERA_HPP__
#define __POINTCLOUD_CAMERA_HPP__

#include <cmath>
#include <string>
#include "robot_custom_msgs/msg/camera_data.hpp"
#include "robot_custom_msgs/msg/camera_data_array.hpp"
#include "utils/pointcloud_generator.hpp"
#include "utils/boundingbox_generator.hpp"

/**
 * @brief: Camera 객체인식 데이터를 pointcloud로 변환시키는 클래스입니다.
 * (정확히는 boundingbox 타입의 메시지로 처리된 객체인식 데이터가 input으로 들어옵니다)
 */
class PointCloudCamera
{
public:
    PointCloudCamera();
    ~PointCloudCamera();

    sensor_msgs::msg::PointCloud2 updateCameraPointCloudMsg(vision_msgs::msg::BoundingBox2DArray msg, float pc_resolution);

private:
    std::shared_ptr<PointCloudGenerator> pointcloud_generator_;
};

#endif // __POINTCLOUD_CAMERA_HPP__