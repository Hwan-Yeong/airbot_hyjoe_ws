// 2D 스캔 → 분할 → 다리·덩어리 판정 (ROS 비의존)
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "kebi_follow_tracker/geometry.hpp"
#include "kebi_follow_tracker/params.hpp"

namespace kebi_follow {
namespace tracker {

// LaserScan 에서 쓰는 부분만
struct ScanView {
  const float* ranges{nullptr};
  std::size_t count{0};
  double angle_min{0.0};
  double angle_increment{0.0};
  double range_min{0.0};
  double range_max{0.0};
};

struct ScanPoint {
  Vec2 p;             // base_link
  double range{0.0};  // 센서 기준 거리
  int ray{0};         // 빔 번호
};

struct Segment {
  int first{0};  // points() 안의 시작 위치
  int count{0};
  Vec2 centroid;  // base_link, 점 평균
  Vec2 center;  // base_link, 보이는 면 뒤쪽으로 보정한 물체 중심
  double range{0.0};  // 센서 기준 평균 거리
  double chord{0.0};  // 양끝 점 거리
  double width{0.0};  // 추정 폭 = chord + 빔 간격 보정
  // 안쪽 점들이 양끝 직선에서 센서 쪽으로 나온 평균 거리 (+ 볼록)
  double bulge{0.0};
  Vec2 arc_center;  // base_link, 원 적합 중심 (arc_valid 일 때)
  double arc_radius{0.0};
  double arc_residual{0.0};
  bool arc_valid{false};
  // 첫 빔 쪽 이웃이 더 가깝다 (앞 물체에 가려 잘림)
  // 이웃 빔이 이만큼 더 가깝다 (앞 물체에 가려 잘림). 0 이면 가리지 않음
  double occluder_gap_first{0.0};
  double occluder_gap_last{0.0};
  bool occluded_first{false};
  bool occluded_last{false};  // 끝 빔 쪽 이웃이 더 가깝다
  bool is_leg{false};
  bool is_blob{false};
  double leg_score{0.0};  // 0..1, 다리 모양 신뢰도
};

class LegDetector {
 public:
  LegDetector(const ScanParams& scan_params, const LegParams& leg_params);

  // sensor_in_base: base_link <- 스캔 프레임
  void detect(const ScanView& scan, const Pose2& sensor_in_base);

  const std::vector<ScanPoint>& points() const { return points_; }
  const std::vector<Segment>& segments() const { return segments_; }

  // 점 기반 계산 (시험용으로 공개)
  void computeFeatures(Segment& seg) const;
  void classify(Segment& seg) const;

 private:
  double jumpThreshold(double range, int steps) const;
  void updateRayTable(const ScanView& scan, int ray_count);
  void mergeWrapAround(bool circular, int ray_count);
  double occluderGap(int ray, int direction, double range) const;

  ScanParams scan_params_;
  LegParams leg_params_;
  Pose2 sensor_in_base_;
  double angle_increment_{0.0};
  // 빔 방향 cos/sin 표 (스캔 각도 구성이 바뀔 때만 다시 계산)
  std::vector<double> ray_cos_;
  std::vector<double> ray_sin_;
  double table_angle_min_{0.0};
  double table_angle_increment_{0.0};
  std::vector<double> ray_range_;  // 빔별 유효 거리 (무효 = +inf), 가림 판정용
  int ray_count_{0};
  bool circular_{false};
  std::vector<ScanPoint> points_;
  std::vector<Segment> segments_;
};

}  // namespace tracker
}  // namespace kebi_follow
