#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "yaml-cpp/yaml.h"

#include "utils/common_struct.hpp"
#include "utils/frame_converter.hpp"
#include "utils/pointcloud_generator.hpp"
#include "utils/tof_utils.hpp"


namespace sensor_manager {

class SensorManagerNode;

using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using PointCloudMsgVector = std::vector<PointCloudMsg>;

/**
 * sensor -> PointCloud2  Strategy interface
 * Customize conversion method by sensor
 */
class CloudConverterStrategy
{
  public:
    CloudConverterStrategy(std::shared_ptr<SensorManagerNode> node_ptr_);

    virtual ~CloudConverterStrategy() = default;

    virtual PointCloudMsgVector pc_convert(const void* sensor_msg) = 0;

    // 캘리브레이션 전용 가상 함수 (기본 구현 제공), 이 함수는 오버라이드 하지 않으면 이 기본 동작을 상속받습니다.
    virtual std_msgs::msg::Float32MultiArray calibration_convert(const void* sensor_msg)
    {
      (void)sensor_msg;
      throw std::runtime_error("This converter does not support calibration_convert.");
      return std_msgs::msg::Float32MultiArray{};
    }

    /**
     * @brief 필터가 참조하는 SensorManagerNode 스마트 포인터를 반환합니다.
     *
     * @return std::shared_ptr<SensorManagerNode> SensorManagerNode 스마트 포인터
     */
    std::shared_ptr<SensorManagerNode> getNodePtr() const;

  protected:
    std::shared_ptr<SensorManagerNode> node_ptr{};
    std::string target_frame_;

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
    TofMonoCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr_, const YAML::Node& config);

  private:
    PointCloudMsgVector pc_convert(const void* sensor_msg) override;

    bool use_tof_mono_ = true;
    tPose tof_mono_sensor_frame_pose_ = tPose(tPoint(0.0942, 0.0, 0.56513),tOrientation(0.0, -DEG2RAD(39.0), 0.0));
};

/**
 * @brief Multi ToF Left 센서 데이터 -> PointCloud2 변환
 */
class TofMultiLeftCloudConverter : public CloudConverterStrategy
{
  public:
    TofMultiLeftCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr_, const YAML::Node& config);

  private:
    tof_utils::TofUtils tof_utils_;

    PointCloudMsgVector pc_convert(const void* sensor_msg) override;
    std_msgs::msg::Float32MultiArray calibration_convert(const void* sensor_msg) override;

    // set default: yaml 파일이 정상이 아닌 경우를 대비하여
    bool use_tof_multi_left_ = true;
    double tof_multi_left_fov_ = DEG2RAD(45.0);
    tPose tof_multi_left_sensor_frame_pose_ = tPose(tPoint(0.14316, 0.075446, 0.03),tOrientation(0.0, -DEG2RAD(5.0), DEG2RAD(15.0)));
    std::vector<int> tof_multi_left_sub_cell_idx_array_;
    std::vector<double> tof_multi_left_y_tan_array_;
    std::vector<double> tof_multi_left_z_tan_array_;
};

/**
 * @brief Multi ToF Right 센서 데이터 -> PointCloud2 변환
 */
class TofMultiRightCloudConverter : public CloudConverterStrategy
{
  public:
    TofMultiRightCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr_, const YAML::Node& config);

  private:
    tof_utils::TofUtils tof_utils_;

    PointCloudMsgVector pc_convert(const void* sensor_msg) override;
    std_msgs::msg::Float32MultiArray calibration_convert(const void* sensor_msg) override;

    // set default: yaml 파일이 정상이 아닌 경우를 대비하여
    bool use_tof_multi_right_ = true;
    double tof_multi_right_fov_ = DEG2RAD(45.0);
    tPose tof_multi_right_sensor_frame_pose_ = tPose(tPoint(0.14316, -0.075446, 0.03),tOrientation(0.0, -DEG2RAD(5.0), -DEG2RAD(15.0)));
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
    CameraCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr_, const YAML::Node& config);

  private:
    PointCloudMsgVector pc_convert(const void* sensor_msg) override;

    // set default: yaml 파일이 정상이 아닌 경우를 대비하여
    bool use_camera_ = true;
    bool object_direction_ = true;
    double pointcloud_resolution_ = 0.05;
    double object_max_dist_ = 1.5;
    tPose camera_sensor_frame_pose_ = tPose(tPoint(0.15473, 0.0, 0.5331),tOrientation(0.0, 0.0, 0.0));
    std::map<int, int> camera_class_id_confidence_th_ = {};
};

/**
 * @brief Bottom IR 센서 데이터 -> PointCloud2 변환
 */
class BottomIrCloudConverter : public CloudConverterStrategy
{
  public:
    BottomIrCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr_, const YAML::Node& config);

  private:
    PointCloudMsgVector pc_convert(const void* sensor_msg) override;

    // set default: yaml 파일이 정상이 아닌 경우를 대비하여
    bool use_bottom_ir_ = true;
    double ir_dist_center_to_sensor = 0.15;
    double ir_angle_sensor_to_next_sensor = 50.0;
};

/**
 * @brief Collision 이벤트 발생 -> PointCloud2 변환
 */
class CollisionCloudConverter : public CloudConverterStrategy
{
  public:
    CollisionCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr_, const YAML::Node& config);

  private:
    PointCloudMsgVector pc_convert(const void* sensor_msg) override;

    // set default: yaml 파일이 정상이 아닌 경우를 대비하여
    bool use_collision_ = true;
    tPose collision_sensor_frame_pose_ = tPose(tPoint(0.19, 0.0, 0.0),tOrientation(0.0, 0.0, 0.0));
};

/**
 * @brief Empty PointCloud2 변환
 */
class EmptyCloudConverter : public CloudConverterStrategy
{
  public:
    EmptyCloudConverter(std::shared_ptr<SensorManagerNode> node_ptr_, const YAML::Node& config);

  private:
    PointCloudMsgVector pc_convert(const void* sensor_msg) override;

    // set default: yaml 파일이 정상이 아닌 경우를 대비하여
    bool use_empty_msg_ = true;
};

} // namespace sensor_manager
