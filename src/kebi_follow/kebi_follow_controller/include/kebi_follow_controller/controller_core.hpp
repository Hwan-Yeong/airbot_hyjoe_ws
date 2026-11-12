// follow_controller 의 주기 판단 전체 (상태기계 + 추종 법칙 + 안전 게이트 + 가감속).
// ROS 의존 없음: 노드는 입력을 모아 step() 을 부르고 결과를 발행만 한다.
#pragma once

#include <cstdint>

#include "kebi_follow_controller/follow_fsm.hpp"
#include "kebi_follow_controller/follow_law.hpp"
#include "kebi_follow_controller/safety_gate.hpp"
#include "kebi_follow_controller/types.hpp"
#include "kebi_follow_controller/velocity_limiter.hpp"

namespace kebi_follow {

struct ControllerParams {
  FollowLawParams law;
  LimiterParams limiter;
  CorridorParams corridor;
  FsmParams fsm;
  // S3 장애물 (거리는 로봇 앞면 기준)
  double slow_distance{1.0};  // 이 거리부터 감속 [m]
  double stop_margin{0.25};   // 제동거리 + 이 값 이내 정지 [m]
  // 장애물 감속·정지 계산에 넣는 지연: 스캔 주기·처리·명령 주기·베이스 응답 [s]
  double obstacle_reaction_time{0.25};
  // depth 점 정지 여유 (카메라 근거리 사각 전에 선다) [m]
  double depth_stop_margin{0.5};
  // 이보다 느리면 원호 대신 직진 경로만 [m/s]
  double curvature_min_speed{0.05};
  // S4 회전
  // 중심에서 이 안에 물체가 있으면 제자리 회전 금지 [m]
  double rotation_clearance{0.30};
  double rotation_limit_s{3.0};  // 제자리 회전 연속 상한 [s]
  double rotation_rearm_s{2.0};  // 상한 정지 후 다시 회전을 허용하기까지 [s]
  // 이보다 느린 선속도는 제자리 회전으로 본다 [m/s]
  double in_place_linear_eps{0.01};
  // 이보다 빠른 각속도만 회전으로 센다 [rad/s]
  double in_place_angular_eps{0.05};
  // 서 있을 때 이보다 작은 선속도 요구로는 출발하지 않는다 [m/s] (0: 끔)
  double start_speed{0.0};
  // 추종 법칙 출력 저역 통과 시간 상수 [s] (0: 끔). 다리 흔들림·속도 추정 잡음이 가가속도로 번지지 않게.
  // 안전 상한(사람 앞 제동·장애물·근접)은 걸러진 값 뒤에 그대로 건다
  double v_filter_s{0.0};
  double w_filter_s{0.0};
  // S6 센서
  double camera_missing_speed_cap{0.3};  // depth 가 없을 때 선속도 상한 [m/s]
  // S7 타깃
  double target_lost_s{0.7};  // 측정 없는 시간이 이를 넘으면 정지 → WAITING [s]
  double occluded_speed_factor{0.5};  // 측정 없는 동안 선속도 배율
  double target_fresh_s{0.3};  // 이 나이 안의 LOCKED 만 "측정 있음" [s]
  // S10
  // 제자리 회전 중 선속도 (베이스 워치독용) [m/s]
  double rotation_creep_speed{0.002};
  double max_dt{0.2};  // 주기 지연이 커도 가감속 계산에 쓰는 dt 상한 [s]
  // /follow/detectuser
  double moving_linear{0.02};   // |v| 가 이를 넘으면 "이동 중" [m/s]
  double moving_angular{0.05};  // |w| 가 이를 넘으면 "이동 중" [rad/s]
  double detect_debounce_s{0.3};  // 1·2·3 사이 값은 이 시간 유지돼야 바꾼다 [s]
};

struct TargetInput {
  bool received{false};
  TargetStatus status{TargetStatus::kNone};
  bool ambiguous{false};
  double msg_age{kInf};       // 지금 - 수신 시각 [s]
  double occluded_time{0.0};  // 메시지의 occluded_time [s]
  Vec2 position;              // 현재 base 좌표 (외삽·이동 보정 끝)
  Vec2 velocity;              // 지면 속도, 현재 base 축 [m/s]
};

struct SensorStatus {
  bool scan_ok{false};
  bool odom_ok{false};
  bool tf_ok{false};
  bool bottom_ir_ok{false};
  bool lifted_ok{false};     // /error/s_code/lifted 가 끊기지 않음
  bool fall_down_ok{false};  // /error/s_code/fall_down 이 끊기지 않음
  // depth 가 신선하고 healthy (바닥 평면 확보·유효 비율 충분). false 면 선속도 상한
  bool camera_ok{false};
  // depth 장애물 점을 쓸 수 있음 (degraded 여도 직전 평면으로 낸 점은 쓴다)
  bool depth_points_ok{false};
};

struct CycleInput {
  double now{0.0};
  double dt{0.0};
  TargetInput target;
  SensorStatus sensors;
  ObstacleSet scan;
  ObstacleSet depth;
  // depth 영상은 늦게 온다: 그 사이 타깃이 지나온 경로 (현재 base 좌표, 끝점 = 현재 타깃).
  // 비어 있으면 현재 타깃 위치만 제외
  Exclusion depth_exclusion;
  uint32_t halt_active{0};  // 지금 켜져 있는 HALT 사유 (레벨 신호)
  double measured_speed{0.0};  // odom 선속도 [m/s]
};

struct CycleOutput {
  bool publish{false};  // 이번 주기 /cmd_vel 발행 여부
  double v{0.0};
  double w{0.0};
  FsmState state{FsmState::kIdle};
  ModeValue mode{ModeValue::kOff};
  bool state_changed{false};
  uint32_t stop_reasons{0};
  double v_limit{0.0};
  double obstacle_distance{kInf};
  double scan_obstacle_distance{kInf};
  double depth_obstacle_distance{kInf};
  double target_range{-1.0};
  double target_bearing{0.0};
  int16_t detect_user{detect_user::kNone};
};

// 1·2·3 사이 잦은 변화를 누른다. 0(종료)·없음(-1)으로/에서의 변화는 바로 반영
class DetectUserFilter {
 public:
  explicit DetectUserFilter(double debounce_s = 0.3)
      : debounce_s_(debounce_s) {}
  int16_t update(int16_t raw, double now);
  void reset() {
    reported_ = detect_user::kNone;
    pending_ = detect_user::kNone;
  }

 private:
  double debounce_s_;
  int16_t reported_{detect_user::kNone};
  int16_t pending_{detect_user::kNone};
  double pending_since_{0.0};
};

// 값이 바뀌거나 period 가 지나면 발행
class ChangeOrPeriodic {
 public:
  explicit ChangeOrPeriodic(double period_s = 1.0) : period_s_(period_s) {}
  bool shouldPublish(int value, double now);
  void reset() { have_ = false; }

 private:
  double period_s_;
  bool have_{false};
  int last_value_{0};
  double last_time_{0.0};
};

class ControllerCore {
 public:
  explicit ControllerCore(const ControllerParams& p = ControllerParams{});

  // 이벤트. 반환값이 true 면 노드는 지금 바로 0 을 한 번 발행한다
  bool onStart(double now);
  bool onStop(double now);
  bool onHalt(double now, uint32_t reasons);

  CycleOutput step(const CycleInput& in);

  FsmState state() const { return fsm_.state(); }
  const ControllerParams& params() const { return p_; }
  const VelocityLimiter& limiter() const { return limiter_; }  // 시험·진단용 (현재 속도·가속도)
  // 실행 중 조정: FSM·속도·회전 상태는 유지하고 파라미터만 바꾼다 (다음 step 부터 적용)
  void updateParams(const ControllerParams& p);

 private:
  void resetMotion();

  ControllerParams p_;
  FollowFsm fsm_;
  FollowLaw law_;
  VelocityLimiter limiter_;
  SafetyGate gate_;
  DetectUserFilter detect_filter_;
  double rotation_since_{-1.0};  // 제자리 회전 시작 시각 (<0: 회전 안 함)
  bool rotation_latched_{false};  // S4 연속 회전 상한 걸림
  double rotation_latched_at_{0.0};
  double v_law_f_{0.0};  // 걸러진 추종 법칙 출력
  double w_law_f_{0.0};
  bool law_f_init_{false};
  bool too_close_latched_{false};  // S2 근접 히스테리시스
  bool cap_active_{false};  // 안전 상한 감속(kCap) 중: 상한에 닿을 때까지 유지 (중간에 편안으로 바꾸면 출렁인다)
};

}  // namespace kebi_follow
