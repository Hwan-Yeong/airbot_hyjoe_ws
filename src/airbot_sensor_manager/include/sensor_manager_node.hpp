#pragma once

#include <memory>
#include <unordered_map>

#include <deque>
#include <ctime>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "robot_custom_msgs/msg/camera_data_array.hpp"

#include "cloud_converter/cloud_converter.hpp"
#include "cloud_converter/cloud_converter_factory.hpp"
#include "utils/multizone_tof_calibrator.hpp"

namespace sensor_manager {

using PC2PublisherPtr = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;

class SensorManagerNode : public rclcpp::Node
{
public:
    SensorManagerNode();
    void init();
    std::string getTargetFrame() const { return node_target_frame_; }

private:
    /**
     * @brief yaml 파일 내용을 config_ 멤버변수에 저장하는 함수
     */
    void loadConfig();

    /**
     * @brief 노드 실행 중 초기화 할 변수 모아둔 함수
     */
    void initializeRuntime();

    /**
     * @brief 퍼블리셔 초기화 함수
     * 
     * @note target frame 기준 토픽 형태 : /sensor_manager/pointcloud/{sensor_name}
     * @note tf2 기준 토픽 형태 : /sensor_manager/pointcloud/{sensor_name}/local
     */
    void initPublisher(const YAML::Node& config);

    /**
     * @brief Converter 및 Static TF 초기화 함수
     */
    void initConverters(const YAML::Node &config);

    /**
     * @brief pointcloud 퍼블리싱을 주기적으로 수행하는 노드의 메인 타이머
     */
    void publishPointcloudTimer();

    /**
     * @brief 센서 데이터 변환 후 메시지 퍼블리싱하는 함수
     * 
     * @param sensor_type: 센서 타입 (Enum)
     * @param msg_copied: 변환할 센서 raw data
     */
    void publishPointcloud(SensorType sensor_type, const std::shared_ptr<void> msg_copy);

    /**
     * @brief 모든 converter 토픽들의 empty pointcloud 발행 함수
     * 
     * @note 노드 비활성화 시점의 마지막 pointcloud 가 다음 활성화 후 주행 costmap에 영향을 끼치지 않게 하기 위한 안전장치
     */
    void publishEmptyMsg();

    /**
     * @brief Multizone ToF의 index 별 토픽 발행을 위한 함수
     * 
     * @param clouds: 발행할 pointCloud2의 벡터 집합 (std::vector<sensor_msgs::msg::PointCloud2>)
     * @param topic_key: 발행할 토픽명 (string)
     */
    void publishMultiTofIdxPointcloud(const ConverterOutput& output, const std::string& topic_key);

    /**
     * @brief Multizone ToF 보정(Calibration)에 사용되는 파라미터 로딩 함수
     */
    tTofCalibrationParam loadMultizoneTofCalibrationParams();

    /**
     * @brief Multizone ToF의 보정(Calibration) 기능 동작 함수
     */
    void runMultizoneToFCalibration(robot_custom_msgs::msg::TofData::SharedPtr tof_msg);

    YAML::Node config_;
    std::string node_target_frame_;

    std::unordered_map<std::string, CloudConverterPtr> converters_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sensor_manager_cmd_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::TofData>::SharedPtr tof_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::BottomIrData>::SharedPtr bottom_ir_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::CameraDataArray>::SharedPtr camera_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::AbnormalEventData>::SharedPtr collision_sub_;

    std::unordered_map<std::string, PC2PublisherPtr> pointcloud_pubs_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr node_active_cmd_response_pub_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> target_frame_callback_handle_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool node_active_cmd_;

    std::unordered_map<std::string, unsigned int> pointcloud_publishing_rate_map_;

    std::vector<int> multi_tof_left_sub_cell_idx_array_;
    std::vector<int> multi_tof_right_sub_cell_idx_array_;

    /*
      Sensor Data Buffers
    */
    tSensorBuffer<robot_custom_msgs::msg::TofData> tof_buffer_;
    tSensorBuffer<robot_custom_msgs::msg::CameraDataArray> camera_buffer_;
    tSensorBuffer<robot_custom_msgs::msg::BottomIrData> bottom_ir_buffer_;
    tSensorBuffer<robot_custom_msgs::msg::AbnormalEventData> collision_buffer_;

    /*
      Multizone ToF Calibration
    */
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mtof_calibration_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mtof_calibration_complete_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mtof_calibration_data_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mtof_calibration_state_pub_;
    std::unique_ptr<MultizoneTofCalibrator> mtof_calibrator_;

    struct SensorTopicConfig {
        std::string converter_key;
        std::string topic_key;
    };

    std::unordered_map<SensorType, SensorTopicConfig> sensor_topic_registry_;
};

} // namespace sensor_manager