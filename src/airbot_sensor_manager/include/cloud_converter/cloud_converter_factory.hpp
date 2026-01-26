#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "cloud_converter/cloud_converter.hpp"

namespace sensor_manager {

class SensorManagerNode;

/**
 * 센서 종류(SensorType)별로 적절한 전략 객체를 생성해주는 팩토리
 */
class CloudConverterFactory
{
public:
    CloudConverterFactory() = default;
    ~CloudConverterFactory() = default;

    static CloudConverterPtr create(std::shared_ptr<SensorManagerNode> node_ptr, const std::string& type, const YAML::Node& config);
    static CloudConverterPtr create(std::shared_ptr<SensorManagerNode> node_ptr, const YAML::Node& config);
};

} // namespace sensor_manager
