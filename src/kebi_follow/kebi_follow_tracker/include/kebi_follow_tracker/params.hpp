// leg_tracker 파라미터. 기본값 = docs/SPEC.md 6.2 + config/leg_tracker.yaml (둘을 같이 고친다)
#pragma once

namespace kebi_follow {
namespace tracker {

// 스캔 전처리·분할
struct ScanParams {
  double robot_radius{0.22};  // [m] 로봇 반경 (SPEC 3절)
  double body_margin{0.05};  // [m] 반경+여유 안의 점은 로봇 몸체로 보고 버린다
  double max_range{5.0};  // [m] base_link 기준 이보다 먼 점은 쓰지 않는다
  double jump_min{0.10};  // [m] 분할 거리 점프 임계의 하한
  double jump_ratio{3.0};  // 임계 = max(jump_min, jump_ratio * r * dphi * 빔수)
  int max_invalid_gap{1};  // 한 분할 안에서 건너뛸 수 있는 연속 무효 빔 수
};

// 다리·덩어리 판정 (SPEC: 폭 0.05~0.25 m, 점 수, 원호 적합)
struct LegParams {
  double min_width{0.05};  // [m] 추정 폭 = 양끝 거리 + 빔 간격 보정
  double max_width{0.25};
  int min_points{2};
  // 거리별 최소 점 수 = round(min_point_width / (r * dphi)). 빔 간격보다 가는 물체(의자 다리 등)는
  // 점 2개로 잡혀도 추정 폭이 2 r dphi 가 되어 폭 조건을 통과하므로 점 수로 거른다
  double min_point_width{0.07};  // [m]
  double concave_tol{0.015};  // [m] 평균 볼록량이 -tol 보다 작으면(오목) 버린다
  // [m] 한쪽 끝을 가린 앞 물체가 이보다 앞에 있으면 배경 조각 (사람 뒤 벽이 그림자 경계에서 잘린 것).
  // 걸을 때 앞다리에 가린 뒷다리는 이 안이다
  double max_occluder_gap{0.8};
  int arc_min_points{5};  // 이 이상이면 원 적합으로 모양을 본다
  double arc_max_radius{0.25};    // [m]
  double arc_max_residual{0.03};  // [m] RMS
  double arc_fail_score{0.5};  // 원 적합이 안 맞는 다리의 leg_score
  // [m] 두 다리가 붙은 덩어리 (치마·넓은 바지 포함)
  double blob_min_width{0.18};
  double blob_max_width{0.60};
  int blob_min_points{4};
  // [m] 다리 쌍 간격 (SPEC 0.1~0.6). 새 트랙 생성에 쓴다
  double pair_min_sep{0.10};
  double pair_max_sep{0.60};
  // [m] 기존 트랙에 다리 둘을 붙일 때 간격 상한 (빠른 보행의 보폭)
  double max_leg_spread{0.80};
  // 보이는 면의 중심 → 다리 중심 보정 (반경 비율)
  double center_offset_ratio{0.65};
  double center_offset_max_radius{0.07};  // [m] 보정에 쓰는 반경 상한
};

// 등속(CV) 칼만 필터, odom 좌표
struct KalmanParams {
  double accel_std{1.2};  // [m/s^2] 가속 잡음 (사람의 출발·정지·회전)
  double init_vel_std{0.8};      // [m/s]
  double meas_std_pair{0.06};    // [m] 다리 쌍 중점
  double meas_std_merged{0.08};  // [m] 붙은 덩어리 중심
  // [m] 다리 하나, 걷는 중 (보폭만큼 사람 중심에서 벗어남)
  double meas_std_single{0.18};
  // [m] 다리 하나, 서 있을 때 (다리는 중심 ±0.1 m 안)
  double meas_std_single_standing{0.10};
  // [m/s] 이 속력부터 meas_std_single, 그 아래는 선형 보간
  double single_std_speed{1.0};
  double max_speed{2.5};  // [m/s] 잘못된 연관으로 인한 발산 방지
  // [s] 측정 없이 예측만 할 때 속도 감쇠 시상수 (0: 감쇠 없음)
  double coast_tau_s{0.0};
  // 정지 트랙(움직인 적 없이 오래 보인 물체): 정지 물체 모델로 게이트를 작게 해 지나가는 다리를 못 가져가게 한다
  double static_meas_std{0.04};   // [m]
  double static_accel_std{0.05};  // [m/s^2]
};

// 연관 (Mahalanobis 게이트 + GNN)
struct AssocParams {
  double gate_chi2{9.21};  // 자유도 2, 99%
  // [m] 불확실성이 커져도 이보다 먼 측정은 붙이지 않는다
  double max_gate_dist{1.0};
  // 후보가 설명하는 분할 하나당 비용 감소 (설명하지 못한 분할은 잡음으로 버리는 비용).
  // 다리 쌍이 다리 하나보다 유리해져 보폭이 큰 보행·회전에서 트랙이 디딤발에 끌려가지 않는다.
  double segment_bonus{4.0};
};

// 트랙 생성·확정·삭제
struct TrackParams {
  int confirm_hits{3};
  int tentative_max_misses{2};  // 미확정 트랙의 연속 미측정 한도
  // [s] 확정 트랙을 지우는 미측정 시간 (>= target.lost_s)
  double delete_s{2.5};
  double delete_std{1.0};  // [m]
  // [m] 사람 트랙 주변에서는 새 트랙을 만들지 않는다. 그 트랙이 다리 하나만 받았으면
  // 그 다리에서 보폭(leg.max_leg_spread) 안의 다리로도 만들지 않는다
  double birth_suppress_radius{0.5};
  double duplicate_radius{0.2};  // [m] 모든 트랙 주변의 중복 생성 방지
  bool allow_single_birth{true};  // 다리 하나로도 트랙 생성 (형상 점수는 낮음)
  double shape_alpha{0.15};  // 형상 점수 EMA 계수 (측정마다)
  double pair_shape{1.0};
  double merged_shape{0.8};
  double single_shape{0.3};
  // 락 후보 판정에서, 움직인 적도 카메라 확인도 없는 트랙(기둥+의자 다리 쌍, 상자, 통)의 형상 점수 비율.
  // 1.0 이면 끈다. 카메라를 쓰면(require_camera) 카메라 확인 트랙에는 적용되지 않는다.
  double static_shape_factor{0.4};
  double moved_dist{0.3};  // [m] 생성 위치에서 이만큼 움직였으면 "움직인 트랙"
  // 생성 위치에서 moved_dist 밖에 이만큼 연속으로 있어야 "움직인 트랙" (튐 무시)
  int moved_scans{5};
  // 정지 트랙: 움직인 적 없이 static_min_s 이상, static_min_hits 이상 갱신된 트랙
  // (움직임이 확인되면 해제, 락 타깃에는 정지 물체 모델을 쓰지 않는다)
  double static_min_s{3.0};  // [s]
  int static_min_hits{15};
  // 한 사람에 붙은 중복 트랙 병합: 움직이는 확정 트랙끼리 merge_radius 안에 merge_scans 연속이면
  // 우선순위(락 타깃 > 카메라 확인 > 갱신 수)가 낮은 쪽을 지운다
  double merge_radius{0.35};  // [m]
  int merge_scans{5};
  int max_tracks{64};
};

// 카메라 융합
struct CameraParams {
  double gate{0.4};              // [m] SPEC 6.2
  double confirm_window_s{1.0};  // [s] 이 시간 안에 연관됐으면 camera_confirmed
  double max_age_s{1.0};  // [s] 최신 scan 보다 이만큼 오래된 검출은 버린다
  double min_score{0.3};
  double person_weight{0.6};  // person_score = 1 - (1 - shape) * (1 - w*camera)
  // [s] 마지막 카메라 프레임이 이 안이면 카메라가 살아 있다고 본다
  double alive_s{1.0};
  // 카메라로 타깃을 확인할 수 있는 영역 (target.camera_lost_s 판정에 쓴다)
  double check_half_fov_deg{35.0};
  // [m] 실 bag 에서 0.5~0.8 m 카메라 검출률 0.99 (무릎 100% 보임). 더 가까우면 데이터 없음
  double check_min_range{0.6};
  double check_max_range{3.5};
};

// 획득 (mode ACQUIRE), SPEC 6.2
struct AcquireParams {
  bool require_camera{true};
  double min_range{0.5};  // [m]
  double max_range{1.5};
  double max_bearing_deg{20.0};
  double confirm_s{1.0};  // [s] 조건 유지 시간
  double min_shape_score{0.5};  // 사람 형상 (다리 쌍·덩어리로 보인 비율)
  double max_coast_s{0.3};  // [s] 후보는 최근 이 시간 안에 측정돼야 한다
  // [m] 경쟁 후보를 세는 영역 = 획득 영역 + 여유
  double competitor_margin{0.3};
  double competitor_margin_deg{10.0};
};

// 유지·상실·재획득 (mode TRACK), SPEC 6.2
struct TargetParams {
  double lost_s{2.0};               // [s]
  double max_std{0.5};              // [m] 위치 불확실성 상한
  double reacquire_radius{1.0};     // [m] LOST 직전 예측 위치 기준
  double reacquire_confirm_s{0.5};  // [s]
  // true: 재획득 영역을 "마지막 관측 위치 ~ 마지막 예측 위치" 선분에서 반경 안으로 넓힌다
  // (가려진 뒤 멈춘 사람은 등속 예측보다 사라진 곳 근처에서 다시 나온다). false: SPEC 그대로
  bool reacquire_from_last_seen{true};
  // 카메라가 살아 있고 타깃이 카메라 확인 영역에 이 시간 넘게 있는데 확인이 없으면 LOST (0: 끔).
  // 사람이 떠난 뒤 타깃이 사람 모양 가구(다리 쌍·덩어리)에 붙는 것을 막는다
  double camera_lost_s{3.0};
};

// 모호 판정, SPEC 6.2
struct AmbiguityParams {
  double radius{0.6};  // [m] 타깃 주변의 다른 사람 트랙
  double cost_margin{2.0};  // 연관 비용 차이가 이보다 작으면 비슷하다고 본다
  double min_person_score{0.5};  // "사람"으로 셀 트랙의 person_score 하한
  // 모호가 이 시간 넘게 계속되면 LOST 로 바꾸고 재획득은 모호 시작 직전 타깃 위치에서 한다 (0: 끔).
  // 가까이 지나간 다른 사람에게 타깃이 끌려갔을 때를 위한 선택 기능. 가구 옆 거짓 모호에서도 LOST 가
  // 되므로 기본은 끈다 (합성 시험: 가구 사이 추종 커버리지 1.0 → 0.55)
  double lost_s{0.0};
};

struct TrackerParams {
  ScanParams scan;
  LegParams leg;
  KalmanParams kf;
  AssocParams assoc;
  TrackParams track;
  CameraParams camera;
  AcquireParams acquire;
  TargetParams target;
  AmbiguityParams ambiguity;
};

}  // namespace tracker
}  // namespace kebi_follow
