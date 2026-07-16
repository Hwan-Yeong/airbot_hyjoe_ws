#pragma once

#include <ctime>
#include <deque>
#include <iomanip>
#include <memory>
#include <unordered_map>

#include "robot_custom_msgs/msg/camera_data_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "cloud_converter/cloud_converter_strategy.hpp"
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
   * @brief input_topics[stream_key] 구독 토픽을 반환한다. 없으면 default_topic.
   */
  std::string ResolveInputTopic(const std::string& stream_key,
                                const std::string& default_topic);

  /**
   * @brief sensors[sensor_key].use 조회. 블록이 없으면 false,
   *        use 키가 생략되면 true(LoadCommonConfig 의 기본값과 동일).
   */
  bool IsSensorUsed(const std::string& sensor_key);

  /**
   * @brief 주어진 소비 센서들 중 하나라도 use:true 이면 true.
   *        하나의 입력 스트림을 공유하는 converter 들의 사용 여부 판정용.
   */
  bool IsAnySensorUsed(const std::vector<std::string>& sensor_keys);

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
   * @brief Helper function to process sensor buffers safely
   */
  template <typename BufferT, typename MsgT>
  bool ProcessBuffer(BufferT& buffer, MsgT& msg_copied_out, rclcpp::Time& recv_time_out, rclcpp::Time& last_time) {
    std::lock_guard<std::mutex> lock(buffer.mtx);
    if (buffer.latest_msg != nullptr && buffer.receive_time > last_time) {
      msg_copied_out = buffer.latest_msg;
      recv_time_out = buffer.receive_time;
      last_time = buffer.receive_time;
      return true;
    }
    return false;
  }

  /**
   * @brief Individual timers of the node that periodically perform pointcloud publishing.
   */
  void PublishTofMonoTimerCallback();
  void PublishTofMultiTimerCallback();
  void PublishCameraTimerCallback();
  void PublishBottomIrTimerCallback();
  void PublishCollisionFrontTimerCallback();
  void PublishCollisionRearTimerCallback();
  void PublishDepthCameraTimerCallback();

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
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;

  std::unordered_map<std::string, PC2PublisherPtr> pointcloud_pubs_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      node_active_cmd_response_pub_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle>
      target_frame_callback_handle_;

  std::unordered_map<std::string, rclcpp::TimerBase::SharedPtr> timers_;

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
  SensorBuffer<sensor_msgs::msg::Image> depth_img_buffer_;
  SensorBuffer<sensor_msgs::msg::CameraInfo> depth_info_buffer_;

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
