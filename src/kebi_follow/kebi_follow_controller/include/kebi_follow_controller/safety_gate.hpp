// 진행 경로 장애물·회전 여유 기하 계산 (SPEC S3·S4). ROS 의존 없음.
//
// 진행 경로(통로) = 직진 경로 ∪ 현재 곡률(w/v)의 원호 경로, 반폭 half_width.
// 거리 d 는 로봇 앞면(원형 몸체)에서 경로를 따라 점까지의 거리다.
//   - 점의 경로 위 위치 s(중심 기준 호 길이), 경로에서 옆으로 떨어진 거리 lat
//   - lat < 반경 r 이면 몸체가 닿는 위치: d = s - sqrt(r^2 - lat^2)
//   - r <= lat <= half_width (여유 띠)면 d = s
//   - 중심 뒤(s <= 0)는 보지 않는다 (후진 없음)
#pragma once

#include <cstddef>

#include "kebi_follow_controller/types.hpp"

namespace kebi_follow {

struct CorridorParams {
  double robot_radius{0.22};            // [m]
  double half_width{0.32};              // 통로 반폭 (반경 + 0.10) [m]
  double lookahead{2.5};                // 경로 길이 상한 [m]
  double max_curvature{2.0};            // 원호 곡률 상한 [1/m]
  double target_exclusion_radius{0.4};  // 타깃 주변 점 제외 [m]
};

// 한 센서의 장애물 점들. 점은 측정 시점 base 좌표, to_now 로 현재 base 로 옮긴다
struct ObstacleSet {
  const Vec2* points{nullptr};
  std::size_t count{0};
  Pose2D to_now;
};

// 장애물로 보지 않을 타깃 주변: 점 1 개면 원, 2 개 이상이면 꺾은선(타깃이 지나온 경로)에서 반경 안
struct Exclusion {
  const Vec2* points{nullptr};  // 현재 base 좌표
  std::size_t count{0};
};

class SafetyGate {
 public:
  explicit SafetyGate(const CorridorParams& p = CorridorParams{}) : p_(p) {}

  // 점 하나의 경로 거리 (통로 밖이면 kInf)
  double pathDistance(const Vec2& p, double curvature) const;

  // 통로 안 최근접 장애물 거리. include_straight 면 직진 경로도 함께 본다
  double corridorDistance(const ObstacleSet& set, double curvature,
                          bool include_straight,
                          const Exclusion& exclusion = Exclusion{}) const;

  // 점이 타깃 제외 구역(target_exclusion_radius) 안인가
  bool excluded(const Vec2& p, const Exclusion& exclusion) const;

  // 로봇 중심에서 가장 가까운 점까지 거리 (S4 회전 여유)
  double minRadius(const ObstacleSet& set) const;

  // 선분 a-b (현재 base 좌표) 에서 가장 가까운 장애물 점까지 거리. 원 (excl_center, excl_radius) 안 점은 뺀다.
  // 발자취 따라가기의 시야 검사: 이 값이 통로 반폭보다 작으면 그 직선으로는 못 간다
  double segmentClearance(const ObstacleSet& set, const Vec2& a, const Vec2& b,
                          const Vec2& excl_center, double excl_radius) const;

  const CorridorParams& params() const { return p_; }
  void setParams(const CorridorParams& p) { p_ = p; }  // 실행 중 조정

 private:
  CorridorParams p_;
};

// (v, w) 명령의 경로 곡률. v 가 작으면 0 (직진으로 본다)
double pathCurvature(double v, double w, double min_speed,
                     double max_curvature);

// S3 속도 상한: slow_length 구간 선형 감속 + 물리적 제동 곡선.
// 제동 곡선은 v·t_r + v²/(2a) = d - margin 의 근 (t_r = reaction_time, 0 이면 sqrt(2 a (d - margin)))
double obstacleSpeedCap(double d, double stop_margin, double slow_length,
                        double decel, double v_max,
                        double reaction_time = 0.0);

// S3 즉시 정지 조건: d <= margin + v·t_r + 제동거리(v)
bool obstacleStopRequired(double d, double stop_margin, double v_now,
                          double decel, double reaction_time = 0.0);

}  // namespace kebi_follow
