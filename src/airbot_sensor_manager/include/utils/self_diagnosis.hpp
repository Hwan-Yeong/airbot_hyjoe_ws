#ifndef AIRBOT_SENSOR_MANAGER_UTILS_SELF_DIAGNOSIS_HPP_
#define AIRBOT_SENSOR_MANAGER_UTILS_SELF_DIAGNOSIS_HPP_

#include <map>
#include <memory>
#include <string>

#include "cloud_converter/cloud_converter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils/common_struct.hpp"
#include "yaml-cpp/yaml.h"

namespace sensor_manager {

class SelfDiagnosis {
 public:
  SelfDiagnosis(
      rclcpp::Node* node,
      const std::unordered_map<std::string, CloudConverterPtr>& converters);
  ~SelfDiagnosis();

  /**
   * @brief Checks latency between data reception and current time
   *
   * @param sensor_type Type of the sensor
   * @param receive_time Time when data was received (rclcpp::Time)
   * @param publish_rate_ms Configured publish rate in ms
   */
  void CheckLatency(SensorType sensor_type, const rclcpp::Time& receive_time,
                    unsigned int publish_rate_ms);

  /**
   * @brief Runs a startup self-diagnosis by feeding dummy data to converters
   *
   * @param config The full sensor configuration YAML node
   */
  void RunStartupDiagnosis(const YAML::Node& config);

 private:
  rclcpp::Node* node_;
  std::unordered_map<std::string, CloudConverterPtr> converters_;

  // Helper functions for dummy data generation
  void CheckSingleSensor(const std::string& sensor_name,
                         const YAML::Node& sensor_config);

  // Virtual Sensor Data Generators
  std::shared_ptr<void> CreateDummyTofData();
  std::shared_ptr<void> CreateDummyCameraData();
  std::shared_ptr<void> CreateDummyBottomIrData();
  std::shared_ptr<void> CreateDummyCollisionData();
};

}  // namespace sensor_manager

#endif  // AIRBOT_SENSOR_MANAGER_UTILS_SELF_DIAGNOSIS_HPP_
