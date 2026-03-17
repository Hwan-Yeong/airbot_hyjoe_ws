#pragma once

#include "cloud_converter/cloud_converter_strategy.hpp"
#include <yaml-cpp/yaml.h>
#include <memory>

namespace sensor_manager {

/**
 * @brief Collision event occurrences -> PointCloud2 conversion.
 */
class CollisionCloudConverter : public CloudConverterStrategy {
 public:
  CollisionCloudConverter(SensorManagerNode* node_ptr,
                          const YAML::Node& config);

 private:
  void ResetInternalVariables() override {
    // Do nothing
  }
  ConverterOutput PcConvertImpl(const void* sensor_msg) override;
};

}  // namespace sensor_manager