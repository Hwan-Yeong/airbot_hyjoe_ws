#pragma once

#include <memory>

#include "cloud_converter/cloud_converter_strategy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace sensor_manager {

class SensorManagerNode;

/**
 * @brief Factory class that creates strategy objects for each SensorType.
 */
class CloudConverterFactory {
 public:
  CloudConverterFactory() = default;
  ~CloudConverterFactory() = default;

  static CloudConverterPtr Create(SensorManagerNode* node_ptr,
                                  const std::string& type,
                                  const YAML::Node& config);
  static CloudConverterPtr Create(SensorManagerNode* node_ptr,
                                  const YAML::Node& config);
};

}  // namespace sensor_manager
