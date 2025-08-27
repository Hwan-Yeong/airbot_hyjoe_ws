#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "vision_msgs/msg/bounding_box2_d_array.hpp"
#include "robot_custom_msgs/msg/tof_data.hpp"
#include "robot_custom_msgs/msg/camera_data.hpp"
#include "robot_custom_msgs/msg/camera_data_array.hpp"
#include "yaml-cpp/yaml.h"

#include "utils/common_struct.hpp"
#include "utils/frame_converter.hpp"
#include "utils/pointcloud_generator.hpp"


namespace sensor_to_pointcloud {

class SensorToPointcloudNode;

using PointCloudMsg = sensor_msgs::msg::PointCloud2;

/**
 * sensor -> PointCloud2  Strategy interface
 * Customize conversion method by sensor
 */
class CloudConverterStrategy
{
public:
    CloudConverterStrategy(std::shared_ptr<SensorToPointcloudNode> node_ptr_);

    virtual ~CloudConverterStrategy() = default;

    virtual PointCloudMsg pc_convert(const void* sensor_msg) = 0;

    /**
     * @brief 필터가 참조하는 SensorToPointcloudNode 스마트 포인터를 반환합니다.
     *
     * @return std::shared_ptr<SensorToPointcloudNode> SensorToPointcloudNode 스마트 포인터
     */
    std::shared_ptr<SensorToPointcloudNode> getNodePtr() const;

protected:
    std::shared_ptr<SensorToPointcloudNode> node_ptr{};
    std::string target_frame_;

    FrameConverter frame_converter_;
    PointCloudGenerator pointcloud_generator_;
};
using CloudConverterPtr = std::shared_ptr<CloudConverterStrategy>;

/**
 * @brief 1D ToF 센서 데이터 -> PointCloud2 변환
 */
class TofMonoCloudConverter : public CloudConverterStrategy
{
public:
    TofMonoCloudConverter(std::shared_ptr<SensorToPointcloudNode> node_ptr_, const YAML::Node& config);

private:
    PointCloudMsg pc_convert(const void* sensor_msg) override;
    sensor_msgs::msg::PointCloud2 generateTofMonoPointCloudMsg(const robot_custom_msgs::msg::TofData* input_msg, tPose &robot_pose);

    bool use_tof_mono_ = true;
    tPose sensor_frame_pose_ = tPose(tPoint(0.0942, 0.0, 0.56513),tOrientation(0.0, -DEG2RAD(39.0), 0.0));
    double tof_mono_sensor_frame_pitch_cosine;
    double tof_mono_sensor_frame_pitch_sine;
};

/**
 * @brief Camera 센서 데이터 -> PointCloud2 변환
 */
class CameraCloudConverter : public CloudConverterStrategy
{
public:
    CameraCloudConverter(std::shared_ptr<SensorToPointcloudNode> node_ptr_, const YAML::Node& config);

private:
    PointCloudMsg pc_convert(const void* sensor_msg) override;
    vision_msgs::msg::BoundingBox2DArray generateObjectBBoxArray(const robot_custom_msgs::msg::CameraDataArray* msg, tPose &robot_pose,std::map<int, int> class_id_confidence_th, bool direction, double object_max_distance);

    // set default: yaml 파일이 정상이 아닌 경우를 대비하여
    bool use_camera_ = true;
    bool object_direction_ = true;
    double pointcloud_resolution_ = 0.05;
    double object_max_dist_ = 1.5;
    tPose sensor_frame_pose_ = tPose(tPoint(0.15473, 0.0, 0.5331),tOrientation(0.0, 0.0, 0.0));
    std::map<int, int> camera_class_id_confidence_th_ = {};
};

/**
 * @brief Empty PointCloud2 변환
 */
class EmptyCloudConverter : public CloudConverterStrategy
{
public:
    EmptyCloudConverter(std::shared_ptr<SensorToPointcloudNode> node_ptr_, const YAML::Node& config);

private:
    PointCloudMsg pc_convert(const void* sensor_msg) override;

    // set default: yaml 파일이 정상이 아닌 경우를 대비하여
    bool use_empty_msg_ = true;
};

} // namespace sensor_to_pointcloud
