// depth 영상 → 바닥 위 장애물 점 (SPEC 6.3 S3). ROS 의존 없음.
//
// 1. 간격 stride 로 픽셀을 골라 base 좌표 점으로 바꾼다.
//    카메라 TF(pitch 0) 에 camera_pitch_correction 을 더한 "기준 자세"를 쓴다.
// 2. 기준 바닥(z=0) 근처 점으로 바닥 평면 z = a x + b y + c 를 RANSAC + 최소제곱으로 맞춘다.
//    - RANSAC 가설은 가까운 띠(로봇 앞 0.6~1.0 m) 점에서만 뽑고, 그 띠에 inlier 가 충분해야 한다
//      (멀리 있는 넓고 낮은 물체 윗면이 바닥으로 뽑히지 않게).
//    - 시작·복구: 기준 자세에서 3°·3 cm 안이고 서로 맞는 추정(가까운 띠만으로 맞춤)이
//      bootstrap_frames 번 이어지면 그 평균을 기준 평면(anchor)으로 고정한다.
//    - 추적: anchor 에서 1°·1.5 cm 안의 추정만 받고, 출력 평면의 프레임당 변화를 제한한다.
//      anchor 는 추적 중 움직이지 않는다 (물체가 오래 보여도 바닥이 그쪽으로 기어가지 않게).
//      받은 추정이 hold_s 동안 없으면 다시 시작 절차로 anchor 를 새로 잡는다.
//    - 받은 평면이 hold_s 안이고 가까운 영역 유효 픽셀 비율이 충분해야 healthy.
//      아니면 degraded (노드는 카메라 상실로 처리). 기준 평면으로 조용히 대신하지 않는다.
// 3. 평면 위 높이 [min_obstacle_height - height_tolerance, max_obstacle_height] 인 점을
//    방위 bin 으로 모으고, bin 마다 k 번째로 가까운 점(잡음 k-1 개까지 무시)을 장애물 점으로 낸다.
//    로봇 몸체(body_exclusion_radius 안)와 max_obstacle_distance 밖은 버린다.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "kebi_follow_controller/types.hpp"

namespace kebi_follow {

struct CameraIntrinsics {
  double fx{424.0};
  double fy{340.0};
  double cx{423.9};
  double cy{239.8};
  int width{848};
  int height{480};
};

// 광학 좌표(x 오른쪽, y 아래, z 정면) → base 좌표. R 은 행 우선 3x3
struct CameraExtrinsics {
  double R[9]{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  double t[3]{0.0, 0.0, 0.0};
};

struct DepthCheckerParams {
  int stride{4};              // 픽셀 간격 (가로·세로)
  int max_row{-1};            // 이 행부터 아래는 안 본다 (<0: 전부)
  double depth_scale{0.001};  // raw(16UC1) → m
  double min_depth{0.25};     // z-depth 유효 범위 [m]
  double max_depth{4.0};
  double max_obstacle_distance{2.0};  // base 중심 수평 거리 상한 [m]
  double min_obstacle_height{0.05};  // 이 높이 이상 물체를 장애물로 본다 [m]
  double max_obstacle_height{1.2};
  double height_tolerance{0.015};      // 바닥 추정·depth 잡음 여유 [m]
  double body_exclusion_radius{0.25};  // 이 안은 로봇 몸체 [m]
  double bin_deg{1.0};
  int min_points_per_bin{3};
  // 바닥 평면 추정
  bool floor_fit{true};
  double floor_band{0.10};  // 기준 바닥에서 이 높이 안 점만 후보 [m]
  double floor_min_distance{0.6};  // 후보 수평 거리 [m]
  double floor_max_distance{3.0};
  double floor_inlier_threshold{0.015};
  int floor_iterations{40};
  int floor_max_samples{1500};
  int floor_min_inliers{150};
  // 시작·복구 때 기준 자세 대비 상한 (세션마다 숙임이 8.6~10.5° 로 달라 넓게)
  double floor_max_tilt_deg{3.0};
  double floor_max_offset{0.03};  // [m]
  // 추적 중 직전 평면 대비 상한 (프레임 잡음 약 0.45°)
  double floor_max_tilt_change_deg{1.0};
  double floor_max_offset_change{0.015};  // [m]
  // 프레임당 평면 변화 제한
  double floor_rate_tilt_deg{0.5};
  double floor_rate_offset{0.005};  // [m]
  // 가까운 띠: 로봇 앞 x 범위 [m] 와 그 안 inlier 최소 개수
  double floor_near_min{0.6};
  double floor_near_max{1.0};
  int floor_near_min_inliers{40};
  int floor_bootstrap_frames{3};  // 시작·복구 때 연속으로 맞아야 하는 프레임 수
  double floor_hold_s{2.0};  // 받은 평면을 healthy 로 쓰는 시간
  // 가까운 영역(기준 바닥이 보여야 하는 곳) 유효 depth 비율 하한
  double near_region_min{0.6};  // 로봇 앞 x [m]
  double near_region_max{1.2};
  double near_region_half_width{0.8};  // |y| [m]
  double min_near_valid_ratio{0.15};
};

enum class FloorSource : uint8_t {
  kNone = 0,    // 받은 평면 없음 → 장애물 점 없음 (degraded)
  kFitted = 1,  // 이번 프레임 추정을 받음 (healthy)
  kHeld = 2,    // hold_s 안의 직전 평면 (healthy)
  kStale = 3,  // hold_s 를 넘긴 직전 평면: 장애물 점은 내지만 degraded
};

// 기준 base 좌표에서 z = a x + b y + c
struct FloorPlane {
  double a{0.0};
  double b{0.0};
  double c{0.0};
};

struct DepthResult {
  std::vector<Vec2> points;  // 장애물 점 (base 좌표, 프레임 수신 시점)
  FloorSource floor_source{FloorSource::kNone};
  FloorPlane floor;
  double pitch_deg{0.0};  // 바닥으로 본 카메라 숙임 = 기준 + atan(a)
  int floor_inliers{0};
  int floor_near_inliers{0};
  int valid_samples{0};
  int obstacle_samples{0};
  double near_valid_ratio{0.0};  // 가까운 영역 표본 중 유효 depth 비율
  // 받은 평면이 hold_s 안이고 유효 비율 충분 (아니면 degraded)
  bool healthy{false};
  double proc_ms{0.0};
};

enum class FrameConvention : uint8_t {
  kAuto = 0,     // 전방(base x) 성분이 큰 축으로 판단
  kOptical = 1,  // z 정면, x 오른쪽, y 아래 (REP 103 광학)
  kBody = 2,     // x 정면, y 왼쪽, z 위
};

// base ← depth 프레임 TF(R 행 우선, t)를 광학 기준 외부 파라미터로. *optical 에 판단 결과
CameraExtrinsics toOpticalExtrinsics(const double R[9], const double t[3],
                                     FrameConvention conv, bool* optical);

class DepthObstacleChecker {
 public:
  explicit DepthObstacleChecker(
      const DepthCheckerParams& p = DepthCheckerParams{});

  // pitch_correction_rad: TF 위에 더하는 숙임 (+ 가 아래)
  void configure(const CameraIntrinsics& K, const CameraExtrinsics& T,
                 double pitch_correction_rad);
  bool configured() const { return configured_; }
  const CameraIntrinsics& intrinsics() const { return K_; }

  // data: 16UC1 행 우선, step: 행 바이트 수. now: 수신 시각 [s] (hold 판단용)
  bool process(const uint8_t* data, std::size_t step, int width, int height,
               bool big_endian, double now, DepthResult* out);

  const DepthCheckerParams& params() const { return p_; }

 private:
  // ref 평면에서 tilt·offset 안의 평면만 찾는다. near_only 면 최소제곱도 가까운 띠 점만
  bool fitFloor(const FloorPlane& ref, double max_tilt_rad, double max_offset,
                bool near_only, FloorPlane* plane, int* inliers,
                int* near_inliers);
  uint32_t nextRandom();

  struct Bin {
    static constexpr int kMaxK = 8;
    float r[kMaxK];
    float x[kMaxK];
    float y[kMaxK];
    int count;
  };

  DepthCheckerParams p_;
  CameraIntrinsics K_;
  bool configured_{false};
  double nominal_pitch_deg_{0.0};
  float t_[3]{0.0f, 0.0f, 0.0f};
  int cols_{0};
  int rows_{0};
  // 표본 픽셀마다 base 좌표 광선 (z-depth 1 m 당)
  std::vector<float> ray_x_;
  std::vector<float> ray_y_;
  std::vector<float> ray_z_;
  // 프레임마다 쓰는 버퍼 (미리 할당)
  std::vector<float> px_;
  std::vector<float> py_;
  std::vector<float> pz_;
  std::vector<int> cand_;
  std::vector<int> near_cand_;  // cand_ 중 가까운 띠
  std::vector<int> subset_;
  std::vector<Bin> bins_;
  double bin_rad_{0.0};
  int k_{3};
  std::vector<uint8_t> near_mask_;  // 표본이 가까운 영역(기준 바닥)인지
  int near_total_{0};
  bool have_last_{false};
  FloorPlane last_plane_;
  double last_plane_time_{0.0};
  FloorPlane anchor_;  // 시작 때 잡은 기준 바닥 (추적 중 고정)
  int boot_count_{0};  // 시작·복구 중 연속으로 맞은 추정 수
  FloorPlane boot_plane_;
  FloorPlane boot_sum_;
  uint32_t rng_{1u};
};

}  // namespace kebi_follow
