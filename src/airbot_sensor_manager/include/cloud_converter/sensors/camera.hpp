#pragma once

#include "cloud_converter/cloud_converter_strategy.hpp"
#include <yaml-cpp/yaml.h>
#include <memory>

namespace sensor_manager {

/**
 * @brief Camera sensor data -> PointCloud2 conversion.
 */
class CameraCloudConverter : public CloudConverterStrategy {
 public:
  CameraCloudConverter(SensorManagerNode* node_ptr,
                       const YAML::Node& config);

 private:
  void ResetInternalVariables() override {
    is_ramp_detection_ = false;
    ramp_release_cnt = 0;
    logged_objects_.clear();
  }
  ConverterOutput PcConvertImpl(const void* sensor_msg) override;

  void LogNewObjects(const std::vector<CameraObject>& objects);

  // 경사로 감지 시 Camera 데이터 변환을 수행하지 않기 위해 추가된 플래그
  void SetupImuSubscription();
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  bool is_ramp_detection_ = false;
  int ramp_release_cnt = 0;

  bool object_direction_;
  bool use_object_logger_;
  double pointcloud_resolution_;
  double object_max_dist_;
  double object_ignore_pitch_th_;
  double object_logger_margin_distance_diff_m_;
  std::map<int, int> camera_class_id_confidence_th_ = {};

  std::map<uint32_t, std::vector<vision_msgs::msg::BoundingBox2D>>
      logged_objects_;
};

}  // namespace sensor_manager