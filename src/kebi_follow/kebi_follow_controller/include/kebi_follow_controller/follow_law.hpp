// 추종 법칙 (SPEC 6.3, S2). ROS 의존 없음.
#pragma once

#include "kebi_follow_controller/types.hpp"

namespace kebi_follow {

struct FollowLawParams {
  double follow_distance{1.0};  // 목표 거리 [m]
  double stop_distance{0.8};    // 이 거리 이내면 정지 (S2) [m]
  // 사람이 갑자기 서도 stop_distance+이 값 앞에서 서도록 [m]
  double approach_margin{0.1};
  double reaction_time{0.2};  // 측정·명령 지연 [s] (제동 상한 계산용)
  double k_v{1.0};            // 거리 오차 이득 [1/s]
  double k_w{1.2};            // 방위 이득 [1/s]
  double ff_gain{1.0};        // 타깃 전방 속도 feed-forward 이득
  // 이보다 느린 타깃 속도는 잡음으로 보고 무시 [m/s]
  double ff_min_target_speed{0.1};
  double bearing_deadband{0.05};        // [rad]
  double slow_bearing{degToRad(15.0)};  // 여기서부터 v 감소 [rad]
  // 여기서 v=0, 넘으면 제자리 회전 [rad]
  double turn_in_place_bearing{degToRad(45.0)};
  double max_linear_speed{0.5};   // [m/s]
  double max_angular_speed{0.8};  // [rad/s]
  double brake_decel{0.8};        // 제동 상한 계산용 감속 [m/s^2]
};

struct FollowCommand {
  double v{0.0};
  double w{0.0};
  bool too_close{false};  // range <= stop_distance (S2)
  bool in_place{false};   // 제자리 회전 요청 (v=0, w≠0)
  double v_cap{kInf};     // 사람 앞 제동 상한 [m/s]
};

// 남은 거리 D 안에서 반응 시간 t_r 후 감속 a 로 설 수 있는 최대 속도
double stoppingSpeed(double D, double decel, double reaction_time);

class FollowLaw {
 public:
  explicit FollowLaw(const FollowLawParams& p = FollowLawParams{}) : p_(p) {}

  // range·bearing: 현재 base_link 기준 타깃, target_forward_speed: 타깃 지면 속도의 로봇 전방 성분
  FollowCommand compute(double range, double bearing,
                        double target_forward_speed) const;

  const FollowLawParams& params() const { return p_; }

 private:
  FollowLawParams p_;
};

}  // namespace kebi_follow
