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
 *   1) 입력 클라우드를 target_frame_ 으로 TF 변환
 *   2) target_frame 기준 x/y/z 박스 크롭 (단순 크롭만)
 *
 * @note 추후 필터(voxel grid, statistical/radius outlier 등)는
 *       ApplyFilterPipeline() 안에 단계로 추가하면 된다.
 *       (현재는 crop_box 한 단계만 존재)
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
   * @note 현재는 crop_box 만. 향후 voxel/outlier 등을 순차 추가하는 지점.
   */
  void ApplyFilterPipeline(sensor_msgs::msg::PointCloud2& cloud) const;

  // ---- crop box (target_frame 기준) ----
  bool crop_enable_ = true;
  double crop_min_x_ = -3.0;
  double crop_max_x_ = 3.0;
  double crop_min_y_ = -3.0;
  double crop_max_y_ = 3.0;
  double crop_min_z_ = 0.03;
  double crop_max_z_ = 1.5;

  // TF lookup timeout [s] (입력 stamp 기준 변환 실패 시 latest 로 폴백)
  double tf_timeout_sec_ = 0.05;
};

}  // namespace sensor_manager
