#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "sensor_types.hpp"

#include "robot_custom_msgs/msg/camera_data.hpp"
#include "robot_custom_msgs/msg/camera_data_array.hpp"
#include "yaml-cpp/yaml.h"


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
};
using CloudConverterPtr = std::shared_ptr<CloudConverterStrategy>;

/**
 * @brief Camera 센서 데이터 -> PointCloud2 변환
 */
class CameraCloudConverter : public CloudConverterStrategy
{
  public:
    CameraCloudConverter(std::shared_ptr<SensorToPointcloudNode> node_ptr_) : CloudConverterStrategy(node_ptr_) {}

  private:
    PointCloudMsg pc_convert(const void* sensor_msg) override;
};

} // namespace sensor_to_pointcloud
