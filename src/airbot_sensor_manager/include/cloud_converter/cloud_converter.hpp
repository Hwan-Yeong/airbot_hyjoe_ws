#ifndef AIRBOT_SENSOR_MANAGER_CLOUD_CONVERTER_CLOUD_CONVERTER_HPP_
#define AIRBOT_SENSOR_MANAGER_CLOUD_CONVERTER_CLOUD_CONVERTER_HPP_

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rclcpp/rclcpp.hpp"
#include "utils/common_struct.hpp"
#include "utils/frame_converter.hpp"
#include "utils/pointcloud_generator.hpp"
#include "utils/tof_utils.hpp"

namespace sensor_manager {

class SensorManagerNode;  // forward declaration

using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using PointCloudMsgVector = std::vector<PointCloudMsg>;
struct ConverterOutput {
  PointCloudMsgVector target_frame_clouds;
  PointCloudMsgVector local_frame_clouds;
  std::string local_topic_suffix;
};

/**
 * sensor -> PointCloud2  Strategy interface
 * Customize conversion method by sensor
 */
class CloudConverterStrategy {
 public:
  CloudConverterStrategy(std::shared_ptr<SensorManagerNode> node_ptr);

  virtual ~CloudConverterStrategy() = default;

  /**
   * @brief Pointcloud convert interface
   *
   * @note To maintain the clean state of the converter independently
   *       when conversion is not performed periodically (function call continuity is broken),
   *       all internal variables are initialized.
   * @note Disable: set reset_timeout_sec parameter to -1.0.
   */
  ConverterOutput PcConvert(const void* sensor_msg);

  /**
   * @brief Virtual function dedicated to Multizone ToF calibration (default implementation provided).
   *
   * @note Inherits this default behavior if not overridden.
   */
  virtual std_msgs::msg::Float32MultiArray CalibrationConvert(
      const void* sensor_msg) {
    (void)sensor_msg;
    throw std::runtime_error(
        "This converter does not support calibration_convert.");
    return std_msgs::msg::Float32MultiArray{};
  }

  /**
   * @brief Virtual function to provide TF for each sensor
   *
   * @note default: no TF - nullptr
   */
  virtual std::optional<geometry_msgs::msg::TransformStamped> GetStaticTf();

  /**
   * @brief Returns the SensorManagerNode smart pointer referenced by the filter.
   *
   * @return std::shared_ptr<SensorManagerNode>
   */
  std::shared_ptr<SensorManagerNode> GetNodePtr() const;

 protected:
  /**
   * @brief Virtual function provided for children to initialize their own variables.
   */
  virtual void ResetInternalVariables() = 0;

  /**
   * @brief Interface for PointCloud2 data conversion function.
   */
  virtual ConverterOutput PcConvertImpl(const void* sensor_msg) = 0;

  /**
   * @brief Common config parsing function for child classes.
   *
   * @note Includes common extrinsic and basic flags of sensor modules,
   *       and logic to prevent segfault due to missing parameters
   */
  void LoadCommonConfig(const YAML::Node& config);

  /**
   * @brief Return string type common config variables.
   */
  std::string GetCommonConfigInfo(const std::string& sensor_type);

  std::shared_ptr<SensorManagerNode> node_ptr_{};
  std::chrono::steady_clock::time_point last_call_time_;
  double timeout_limit_sec_ = -1.0;  // converter reset timeout time (default: -1, disabled)
  bool is_already_reset_ = true;
  bool use_converter_ = true;
  bool enable_target_frame_cloud_ = true;
  bool enable_sensor_tf_cloud_ = false;
  std::string target_frame_ = "map";
  std::string parent_frame_ = "base_link";
  std::string child_frame_ = "";
  Pose sensor_extrinsic_;

  FrameConverter frame_converter_;
  PointCloudGenerator pointcloud_generator_;
};
using CloudConverterPtr = std::shared_ptr<CloudConverterStrategy>;

/**
 * @brief 1D ToF sensor data -> PointCloud2 conversion.
 */
class TofMonoCloudConverter : public CloudConverterStrategy {
 public:
  TofMonoCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr,
                        const YAML::Node& config);

 private:
  void ResetInternalVariables() override {
    // Do nothing
  }
  ConverterOutput PcConvertImpl(const void* sensor_msg) override;
};

/**
 * @brief Multi ToF Left sensor data -> PointCloud2 conversion, Calibration coordinate conversion.
 */
class TofMultiLeftCloudConverter : public CloudConverterStrategy {
 public:
  TofMultiLeftCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr,
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
  TofMultiRightCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr,
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

/**
 * @brief Camera sensor data -> PointCloud2 conversion.
 */
class CameraCloudConverter : public CloudConverterStrategy {
 public:
  CameraCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr,
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

/**
 * @brief Bottom IR sensor data -> PointCloud2 conversion.
 */
class BottomIrCloudConverter : public CloudConverterStrategy {
 public:
  BottomIrCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr,
                         const YAML::Node& config);

 private:
  void ResetInternalVariables() override {
    // Do nothing
  }
  ConverterOutput PcConvertImpl(const void* sensor_msg) override;

  double ir_dist_center_to_sensor_;
  double ir_angle_sensor_to_next_sensor_;
};

/**
 * @brief Collision event occurrences -> PointCloud2 conversion.
 */
class CollisionCloudConverter : public CloudConverterStrategy {
 public:
  CollisionCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr,
                          const YAML::Node& config);

 private:
  void ResetInternalVariables() override {
    // Do nothing
  }
  ConverterOutput PcConvertImpl(const void* sensor_msg) override;
};

/**
 * @brief Empty PointCloud2 conversion.
 */
class EmptyCloudConverter : public CloudConverterStrategy {
 public:
  EmptyCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr,
                      const YAML::Node& config);

 private:
  void ResetInternalVariables() override {
    // Do nothing
  }
  ConverterOutput PcConvertImpl(const void* sensor_msg) override;
};

}  // namespace sensor_manager

#endif  // AIRBOT_SENSOR_MANAGER_CLOUD_CONVERTER_CLOUD_CONVERTER_HPP_
