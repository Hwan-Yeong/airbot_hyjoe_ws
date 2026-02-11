#ifndef AIRBOT_SENSOR_MANAGER_CLOUD_CONVERTER_CLOUD_CONVERTER_FACTORY_HPP_
#define AIRBOT_SENSOR_MANAGER_CLOUD_CONVERTER_CLOUD_CONVERTER_FACTORY_HPP_

#include <memory>

#include "cloud_converter/cloud_converter.hpp"
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

  static CloudConverterPtr Create(std::shared_ptr<SensorManagerNode> node_ptr,
                                  const std::string& type,
                                  const YAML::Node& config);
  static CloudConverterPtr Create(std::shared_ptr<SensorManagerNode> node_ptr,
                                  const YAML::Node& config);
};

}  // namespace sensor_manager

#endif  // AIRBOT_SENSOR_MANAGER_CLOUD_CONVERTER_CLOUD_CONVERTER_FACTORY_HPP_
