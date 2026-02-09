#pragma once

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "yaml-cpp/yaml.h"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "utils/common_struct.hpp"
#include "utils/frame_converter.hpp"
#include "utils/pointcloud_generator.hpp"
#include "utils/tof_utils.hpp"


namespace sensor_manager {

class SensorManagerNode; // forward declaration

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
class CloudConverterStrategy
{
  public:
    CloudConverterStrategy(std::shared_ptr<SensorManagerNode> node_ptr);

    virtual ~CloudConverterStrategy() = default;

    /**
     * @brief 사용자에게 공개되는 pointcloud convert 인터페이스
     *
     * @note 변환이 주기적이로 이루어지지 않았을 때 (함수 호출 연속성이 훼손되었을 때),
     *       converter 독립적으로 상태를 깔끔하게 유지하기 위하여 내부 변수를 모두 초기화
     *       초기화 기능 비활성화 : reset_timeout_sec 파라미터 -1.0 으로 설정
     */
    ConverterOutput pc_convert(const void* sensor_msg);

    /**
     * @brief Multizone ToF 캘리브레이션 전용 가상 함수 (기본 구현 제공)
     *
     * @note 이 함수는 오버라이드 하지 않으면 이 기본 동작을 상속받습니다.
     */
    virtual std_msgs::msg::Float32MultiArray calibration_convert(const void* sensor_msg)
    {
      (void)sensor_msg;
      throw std::runtime_error("This converter does not support calibration_convert.");
      return std_msgs::msg::Float32MultiArray{};
    }

    /**
     * @brief 각 센서의 TF 를 제공하기 위한 가상 함수 (default : tf 없음 - nullptr)
     */
    virtual std::optional<geometry_msgs::msg::TransformStamped> get_static_tf();

    /**
     * @brief 필터가 참조하는 SensorManagerNode 스마트 포인터를 반환합니다.
     *
     * @return std::shared_ptr<SensorManagerNode> SensorManagerNode 스마트 포인터
     */
    std::shared_ptr<SensorManagerNode> get_node_ptr() const;

  protected:
    /**
     * @brief 자식들이 각자의 변수를 초기화할 수 있도록 제공한 순수 가상 함수
     */
    virtual void reset_internal_variables() = 0;

    /**
     * @brief PointCloud2 데이터 변환 함수 인터페이스
     */
    virtual ConverterOutput pc_convert_impl(const void* sensor_msg) = 0;

    /**
     * @brief 자식 클래스들의 공통 config 파싱 함수
     *
     * @note 센서 모듈의 공통적인 extrinsic 및 기본 flag 등 / 파라미터 누락으로 인한 segfault 방지 로직 포함
     */
    void load_common_config(const YAML::Node& config);

    /**
     * @brief load 된 공통 config 변수 string 타입으로 전달받기 위한 함수
     *
     * @note converter 생성 시 공통 config 변수 및 converter 개별 변수 더하여 print
     */
    std::string get_common_config_info(const std::string& sensor_type);

    std::shared_ptr<SensorManagerNode> node_ptr_{};
    std::chrono::steady_clock::time_point last_call_time_;
    double timeout_limit_sec_ = -1.0; // 자식 클래스마다 다르게 가질 converter 초기화 타임아웃 시간 (default: -1, 비활성화)
    bool is_already_reset_ = true;
    bool use_converter_ = true;
    bool enable_target_frame_cloud_ = true;
    bool enable_sensor_tf_cloud_ = false;
    std::string target_frame_ = "map";
    std::string parent_frame_ = "base_link";
    std::string child_frame_ = "";
    tPose sensor_extrinsic_;

    FrameConverter frame_converter_;
    PointCloudGenerator pointcloud_generator_;
};
using CloudConverterPtr = std::shared_ptr<CloudConverterStrategy>;

/**
 * @brief 1D ToF 센서 데이터 -> PointCloud2 변환
 */
class TofMonoCloudConverter : public CloudConverterStrategy
{
  public:
    TofMonoCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr, const YAML::Node& config);

  private:
    void reset_internal_variables() override {
      // Do nothing
    }
    ConverterOutput pc_convert_impl(const void* sensor_msg) override;
};

/**
 * @brief Multi ToF Left 센서 데이터 -> PointCloud2 변환, Calibration 좌표 변환
 */
class TofMultiLeftCloudConverter : public CloudConverterStrategy
{
  public:
    TofMultiLeftCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr, const YAML::Node& config);

  private:
    sensor_manager::TofUtils tof_utils_;

    void reset_internal_variables() override {
      // Do nothing
    }
    ConverterOutput pc_convert_impl(const void* sensor_msg) override;
    std_msgs::msg::Float32MultiArray calibration_convert(const void* sensor_msg) override;
    double tof_multi_left_fov_;
    std::vector<int> tof_multi_left_sub_cell_idx_array_;
    std::vector<double> tof_multi_left_y_tan_array_;
    std::vector<double> tof_multi_left_z_tan_array_;
};

/**
 * @brief Multi ToF Right 센서 데이터 -> PointCloud2 변환, Calibration 좌표 변환
 */
class TofMultiRightCloudConverter : public CloudConverterStrategy
{
  public:
    TofMultiRightCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr, const YAML::Node& config);

  private:
    sensor_manager::TofUtils tof_utils_;

    void reset_internal_variables() override {
      // Do nothing
    }
    ConverterOutput pc_convert_impl(const void* sensor_msg) override;
    std_msgs::msg::Float32MultiArray calibration_convert(const void* sensor_msg) override;

    double tof_multi_right_fov_;
    std::vector<int> tof_multi_right_sub_cell_idx_array_;
    std::vector<double> tof_multi_right_y_tan_array_;
    std::vector<double> tof_multi_right_z_tan_array_;
};

/**
 * @brief Camera 센서 데이터 -> PointCloud2 변환
 */
class CameraCloudConverter : public CloudConverterStrategy
{
  public:
    CameraCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr, const YAML::Node& config);

  private:
    void reset_internal_variables() override {
      is_ramp_detection_ = false;
      ramp_release_cnt = 0;
      logged_objects_.clear();
    }
    ConverterOutput pc_convert_impl(const void* sensor_msg) override;

    void logNewObjects(const std::vector<CameraObject>& objects);

    // 경사로 감지 시 Camera 데이터 변환을 수행하지 않기 위해 추가된 플래그
    void setup_imu_subscription();
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

    std::map<uint32_t, std::vector<vision_msgs::msg::BoundingBox2D>> logged_objects_;
};

/**
 * @brief Bottom IR 센서 데이터 -> PointCloud2 변환
 */
class BottomIrCloudConverter : public CloudConverterStrategy
{
  public:
    BottomIrCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr, const YAML::Node& config);

  private:
    void reset_internal_variables() override {
      // Do nothing
    }
    ConverterOutput pc_convert_impl(const void* sensor_msg) override;

    double ir_dist_center_to_sensor_;
    double ir_angle_sensor_to_next_sensor_;
};

/**
 * @brief Collision 이벤트 발생 -> PointCloud2 변환
 */
class CollisionCloudConverter : public CloudConverterStrategy
{
  public:
    CollisionCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr, const YAML::Node& config);

  private:
    void reset_internal_variables() override {
      // Do nothing
    }
    ConverterOutput pc_convert_impl(const void* sensor_msg) override;
};

/**
 * @brief Empty PointCloud2 변환
 */
class EmptyCloudConverter : public CloudConverterStrategy
{
  public:
    EmptyCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr, const YAML::Node& config);

  private:
    void reset_internal_variables() override {
      // Do nothing
    }
    ConverterOutput pc_convert_impl(const void* sensor_msg) override;
};

} // namespace sensor_manager
