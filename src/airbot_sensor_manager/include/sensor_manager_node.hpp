#pragma once

#include <ctime>
#include <deque>
#include <iomanip>
#include <memory>
#include <unordered_map>

#include "robot_custom_msgs/msg/camera_data_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "cloud_converter/cloud_converter.hpp"
#include "cloud_converter/cloud_converter_factory.hpp"
#include "utils/multizone_tof_calibrator.hpp"
#include "utils/self_diagnosis.hpp"

namespace sensor_manager {

using PC2PublisherPtr =
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;

class SensorManagerNode : public rclcpp::Node {
 public:
  SensorManagerNode();
  void Init();
  std::string GetTargetFrame() const { return node_target_frame_; }
  std::shared_ptr<tf2_ros::Buffer> GetTfBuffer() const { return tf_buffer_; }

 private:
  /**
   * @brief Loads contents of the yaml file into the `config_` member variable.
   */
  void LoadConfig();

  /**
   * @brief Initializes variables during node runtime.
   */
  void InitializeRuntime();

  /**
   * @brief Initializes publishers.
   *
   * @note Topic format based on 'target frame': /sensor_manager/pointcloud/{sensor_name}
   * @note Topic format based on 'tf2'         : /sensor_manager/pointcloud/{sensor_name}/local
   */
  void InitPublisher(const YAML::Node& config);

  /**
   * @brief Initializes Converters and Static TF.
   */
  void InitConverters(const YAML::Node& config);

  /**
   * @brief Main timer of the node that periodically performs pointcloud publishing.
   */
  void PublishPointcloudTimer();

  /**
   * @brief Converts sensor data and publishes the message.
   *
   * @param sensor_type Type of the sensor (Enum).
   * @param msg_copy Raw sensor data to be converted.
   * @param receive_time Time when the data was received.
   */
  void PublishPointcloud(SensorType sensor_type,
                         const std::shared_ptr<void> msg_copy,
                         const rclcpp::Time& receive_time);

  /**
   * @brief Publishes empty pointclouds for all converter topics.
   *
   * @note Safety mechanism to ensure that the last pointcloud from when the node
   * was inactive does not affect the costmap after the next activation.
   */
  void PublishEmptyMsg();

  /**
   * @brief Publishes topics by index for Multizone ToF.
   *
   * @param output Converter output containing vector set of PointCloud2.
   * @param topic_key Topic name (string).
   */
  void PublishMultiTofIdxPointcloud(const ConverterOutput& output,
                                    const std::string& topic_key);

  /**
   * @brief Loads parameters used for Multizone ToF Calibration.
   */
  TofCalibrationParam LoadMultizoneTofCalibrationParams();

  /**
   * @brief Executes the Multizone ToF Calibration function.
   */
  void RunMultizoneToFCalibration(
      robot_custom_msgs::msg::TofData::SharedPtr tof_msg);

  YAML::Node config_;
  std::string node_target_frame_;

  std::unordered_map<std::string, CloudConverterPtr> converters_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sensor_manager_cmd_sub_;
  rclcpp::Subscription<robot_custom_msgs::msg::TofData>::SharedPtr tof_sub_;
  rclcpp::Subscription<robot_custom_msgs::msg::BottomIrData>::SharedPtr
      bottom_ir_sub_;
  rclcpp::Subscription<robot_custom_msgs::msg::CameraDataArray>::SharedPtr
      camera_sub_;
  rclcpp::Subscription<robot_custom_msgs::msg::AbnormalEventData>::SharedPtr
      collision_sub_;

  std::unordered_map<std::string, PC2PublisherPtr> pointcloud_pubs_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      node_active_cmd_response_pub_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle>
      target_frame_callback_handle_;

  rclcpp::TimerBase::SharedPtr timer_;

  bool node_active_cmd_;

  std::unordered_map<std::string, unsigned int> pointcloud_publishing_rate_map_;

  std::vector<int> multi_tof_left_sub_cell_idx_array_;
  std::vector<int> multi_tof_right_sub_cell_idx_array_;

  /*
    Sensor Data Buffers
  */
  SensorBuffer<robot_custom_msgs::msg::TofData> tof_buffer_;
  SensorBuffer<robot_custom_msgs::msg::CameraDataArray> camera_buffer_;
  SensorBuffer<robot_custom_msgs::msg::BottomIrData> bottom_ir_buffer_;
  SensorBuffer<robot_custom_msgs::msg::AbnormalEventData> collision_buffer_;

  /*
    Multizone ToF Calibration
  */
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr
      mtof_calibration_cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      mtof_calibration_complete_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      mtof_calibration_data_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mtof_calibration_state_pub_;
  std::unique_ptr<MultizoneTofCalibrator> mtof_calibrator_;

  struct SensorTopicConfig {
    std::string converter_key;
    std::string topic_key;
  };

  std::unordered_map<SensorType, SensorTopicConfig> sensor_topic_registry_;

  std::shared_ptr<SelfDiagnosis> self_diagnosis_;
};

}  // namespace sensor_manager
