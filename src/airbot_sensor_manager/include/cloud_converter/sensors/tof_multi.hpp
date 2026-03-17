#pragma once

#include "cloud_converter/cloud_converter_strategy.hpp"
#include <yaml-cpp/yaml.h>
#include <memory>

namespace sensor_manager {

/**
 * @brief Multi ToF Left sensor data -> PointCloud2 conversion, Calibration coordinate conversion.
 */
class TofMultiLeftCloudConverter : public CloudConverterStrategy {
 public:
  TofMultiLeftCloudConverter(SensorManagerNode* node_ptr,
                             const YAML::Node& config);

 private:
  sensor_manager::TofUtils tof_utils_;

  void ResetInternalVariables() override {
    // Do nothing
  }
  ConverterOutput PcConvertImpl(const void* sensor_msg) override;
  std_msgs::msg::Float32MultiArray CalibrationConvert(
      const void* sensor_msg) override;
  double tof_multi_left_fov_;
  std::vector<int> tof_multi_left_sub_cell_idx_array_;
  std::vector<double> tof_multi_left_y_tan_array_;
  std::vector<double> tof_multi_left_z_tan_array_;
};

/**
 * @brief Multi ToF Right sensor data -> PointCloud2 conversion, Calibration coordinate conversion.
 */
class TofMultiRightCloudConverter : public CloudConverterStrategy {
 public:
  TofMultiRightCloudConverter(SensorManagerNode* node_ptr,
                              const YAML::Node& config);

 private:
  sensor_manager::TofUtils tof_utils_;

  void ResetInternalVariables() override {
    // Do nothing
  }
  ConverterOutput PcConvertImpl(const void* sensor_msg) override;
  std_msgs::msg::Float32MultiArray CalibrationConvert(
      const void* sensor_msg) override;

  double tof_multi_right_fov_;
  std::vector<int> tof_multi_right_sub_cell_idx_array_;
  std::vector<double> tof_multi_right_y_tan_array_;
  std::vector<double> tof_multi_right_z_tan_array_;
};

}  // namespace sensor_manager