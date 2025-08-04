#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "vision_msgs/msg/bounding_box2_d_array.hpp"
#include "robot_custom_msgs/msg/camera_data.hpp"
#include "robot_custom_msgs/msg/camera_data_array.hpp"
#include "yaml-cpp/yaml.h"

#include "utils/common_struct.hpp"


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
};
using CloudConverterPtr = std::shared_ptr<CloudConverterStrategy>;

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
    sensor_msgs::msg::PointCloud2 generateCameraPointCloudMsg(const vision_msgs::msg::BoundingBox2DArray input_bbox_array, float resolution);

    bool use_camera_;
    bool object_direction_;
    double pointcloud_resolution_;
    double object_max_dist_;
    tPoint sensor_frame_translation_;
    std::map<int, int> camera_class_id_confidence_th_;
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
    sensor_msgs::msg::PointCloud2 generateEmptyPointCloudMsg();

    bool use_empty_msg_;
};

} // namespace sensor_to_pointcloud
