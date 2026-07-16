#pragma once

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
  std::string local_topic_suffix = "/local";
};

/**
 * sensor -> PointCloud2  Strategy interface
 * Customize conversion method by sensor
 */
class CloudConverterStrategy {
 public:
  CloudConverterStrategy(SensorManagerNode* node_ptr);

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
   * @brief Create empty pointcloud with specific frame id
   * 
   * @param frame_id Target frame id
   */
  ConverterOutput PcConvertEmpty(const std::string& frame_id);

  /**
   * @brief Virtual function to provide TF for each sensor
   *
   * @note default: no TF - nullptr
   */
  virtual std::optional<geometry_msgs::msg::TransformStamped> GetStaticTf();

  /**
   * @brief Returns the SensorManagerNode raw pointer by the filter.
   *
   * @return SensorManagerNode*
   */
SensorManagerNode* GetNodePtr() const;

  /**
   * @brief Get target frame id
   */
  std::string GetTargetFrame() const { return target_frame_; }

  /**
   * @brief Get child frame id (local frame)
   */
  std::string GetChildFrame() const { return child_frame_; }

  /**
   * @brief Check if sensor tf cloud is enabled
   */
  bool IsEnableSensorTfCloud() const { return enable_sensor_tf_cloud_; }

  /**
   * @brief Check if target frame cloud is enabled
   */
  bool IsEnableTargetFrameCloud() const { return enable_target_frame_cloud_; }

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

  SensorManagerNode* node_ptr_{nullptr};
  std::chrono::steady_clock::time_point last_call_time_;
  double timeout_limit_sec_ =
      -1.0;  // converter reset timeout time (default: -1, disabled)
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

}  // namespace sensor_manager
