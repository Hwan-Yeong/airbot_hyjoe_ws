#pragma once

#include "cloud_converter/cloud_converter_strategy.hpp"
#include <yaml-cpp/yaml.h>
#include <memory>

namespace sensor_manager {

/**
 * @brief Bottom IR sensor data -> PointCloud2 conversion.
 */
class BottomIrCloudConverter : public CloudConverterStrategy {
 public:
  BottomIrCloudConverter(SensorManagerNode* node_ptr,
                         const YAML::Node& config);

 private:
  void ResetInternalVariables() override {
    // Do nothing
  }
  ConverterOutput PcConvertImpl(const void* sensor_msg) override;

  double ir_dist_center_to_sensor_;
  double ir_angle_sensor_to_next_sensor_;
};

}  // namespace sensor_manager