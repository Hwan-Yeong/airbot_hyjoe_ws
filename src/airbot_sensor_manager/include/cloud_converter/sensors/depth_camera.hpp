#pragma once

#include <yaml-cpp/yaml.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <robot_custom_msgs/msg/depth_camera_data.hpp>

#include "cloud_converter/cloud_converter_strategy.hpp"

namespace sensor_manager {

/**
 * @brief Inuitive Depth Camera (M4.51s) data -> PointCloud2 conversion.
 */
class DepthCameraCloudConverter : public CloudConverterStrategy {
 public:
  DepthCameraCloudConverter(SensorManagerNode* node_ptr,
                            const YAML::Node& config);

 private:
  void ResetInternalVariables() override {
  }

  ConverterOutput PcConvertImpl(const void* sensor_msg) override;

  /**
   * @brief depth image + camera_info -> 광학 좌표계 raw PointCloud2 생성.
   *
   * @note frame_id 는 depth image header 의 frame_id 를 그대로 사용하되,
   *       stamp 는 수신 시각(input->timestamp, ROS time)으로 재스탬프한다.
   *       (드라이버 stamp 는 별도 클럭이라 TF lookup 이 항상 timeout 됨)
   */
  bool BuildPointCloud(const robot_custom_msgs::msg::DepthCameraData* input,
                       sensor_msgs::msg::PointCloud2& cloud_out) const;

  /**
   * @brief target_frame 기준 cloud 에 filter pipeline 적용.
   *
   * @param sensor_tf sensor(optical) -> target_frame 변환.
   *                  translation 이 target_frame 기준 센서 위치이며,
   *                  RANSAC 바닥 평면 검증(높이 체크)에 사용된다.
   */
  void ApplyFilterPipeline(
      sensor_msgs::msg::PointCloud2& cloud,
      const geometry_msgs::msg::TransformStamped& sensor_tf) const;

  /**
   * @brief RANSAC 으로 바닥 평면을 추정하고, 평면 근방/아래 점을 제거.
   *
   * 검출된 평면이 "바닥" 검증(기울기, 센서 위치에서의 높이, inlier 비율)을
   * 통과하면 평면 위쪽 distance_threshold 초과 점만 남긴다.
   * 검증 실패 시 fallback_min_z 기반 단순 z-crop 으로 폴백한다.
   */
  void RemoveGroundPlane(
      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      const geometry_msgs::msg::TransformStamped& sensor_tf) const;

  // ---- depth image -> pointcloud 변환 ----
  double depth_scale_ = 0.001;  // 16UC1(mm) -> m
  double range_min_ = 0.16;     // [m]
  double range_max_ = 2.0;      // [m]
  int row_step_ = 6;            // down sampling (행)
  int col_step_ = 6;            // down sampling (열)

  // ---- height(z) crop ----
  bool crop_enable_ = true;
  double crop_min_z_ = 0.00;   // 지면 위 최소 높이 [m]
  double crop_max_z_ = 0.60;   // 지면 위 최대 높이 [m] (로봇 높이 0.55 + 여유)

  // ---- RANSAC 바닥(평면) 제거 ----
  bool ransac_enable_ = false;
  double ransac_distance_threshold_ = 0.02;  // [m] 평면 근방 제거 두께
  int ransac_max_iterations_ = 100;
  double ransac_eps_angle_deg_ = 10.0;      // [deg] 수평 대비 허용 기울기
  double ransac_max_ground_offset_ = 0.05;  // [m] 센서 xy 위치에서의 평면 높이 허용치
  double ransac_min_inlier_ratio_ = 0.05;   // 바닥 평면 최소 inlier 비율 (전체 대비)
  double ransac_fallback_min_z_ = 0.02;     // [m] 평면 미검출 시 z-crop 폴백 (<=0 비활성)

  double tf_timeout_sec_ = 0.05;
};

}  // namespace sensor_manager
