#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "cloud_converter/cloud_converter.hpp"

namespace sensor_to_pointcloud {

class SensorToPointcloudNode;

/**
 * 센서 종류(SensorType)별로 적절한 전략 객체를 생성해주는 팩토리 역할
 */
class CloudConverterFactory
{
  public:
    CloudConverterFactory() = default;
    ~CloudConverterFactory() = default;

    static CloudConverterPtr create(const std::string& type, std::shared_ptr<SensorToPointcloudNode> node_ptr);
};

} // namespace sensor_to_pointcloud
