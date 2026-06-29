#pragma once

#include <yaml-cpp/yaml.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "cloud_converter/cloud_converter_strategy.hpp"

namespace sensor_manager {

/**
 * @brief Depth-camera PointCloud2 -> (TF to target frame) -> PointCloud2 변환.
 *
 * 입력: depth_pointcloud_converter(dpc)가 발행한 PointCloud2 (광학 좌표계, raw).
 *       dpc 단계에서 range 게이트 + decimation 만 적용된 상태.
 *
 * 이 converter 의 책임(현재):
 *   1) 입력 클라우드를 target_frame_(=map) 으로 TF 변환
 *   2) 높이(z) crop (지면 위 높이 범위만 통과)
 *
 * @note 추후 필터(voxel grid, statistical/radius outlier 등)는
 *       ApplyFilterPipeline() 안에 단계로 추가하면 된다.
 *       (현재는 height crop 한 단계만 존재)
 */
class DepthCameraCloudConverter : public CloudConverterStrategy {
 public:
  DepthCameraCloudConverter(SensorManagerNode* node_ptr,
                            const YAML::Node& config);

 private:
  void ResetInternalVariables() override {
    // 내부 누적 상태 없음.
  }

  ConverterOutput PcConvertImpl(const void* sensor_msg) override;

  /**
   * @brief target_frame 기준으로 필터 파이프라인을 적용한다.
   *
   * @note 현재는 height crop 만. 향후 voxel/outlier 등을 순차 추가하는 지점.
   */
  void ApplyFilterPipeline(sensor_msgs::msg::PointCloud2& cloud) const;

  // ---- 높이(z) crop ----
  // target_frame=map 의 z = 지면 위 높이(map 원점이 바닥일 때).
  // x/y crop 은 map 프레임에서 world 좌표라 의미가 없어 두지 않는다.
  // (robot-relative 전방 코리도는 A1_perception 의 depth_camera_low_obstacle 필터가 담당)
  bool crop_enable_ = true;
  double crop_min_z_ = 0.03;   // 지면 위 최소 높이 [m]
  double crop_max_z_ = 0.60;   // 지면 위 최대 높이 [m] (로봇 높이 0.55 + 여유)

  // TF lookup timeout [s] (입력 stamp 기준 변환 실패 시 latest 로 폴백)
  double tf_timeout_sec_ = 0.05;
};

}  // namespace sensor_manager
