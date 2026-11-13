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
  // 타깃 속도 feed-forward 는 이 값의 0.5 배에서 0, 1.5 배에서 전부가 되도록 부드럽게 섞는다
  // (그보다 느린 속도는 잡음으로 본다. 계단으로 켜고 끄면 출발·정지가 덜컥거린다) [m/s]
  double ff_min_target_speed{0.1};
  double bearing_deadband{0.05};        // [rad]
  double slow_bearing{degToRad(15.0)};  // 여기서부터 v 감소 [rad]
  // 여기서 v=0, 넘으면 제자리 회전 [rad]
  double turn_in_place_bearing{degToRad(45.0)};
  double max_linear_speed{0.5};   // [m/s]
  double max_angular_speed{0.8};  // [rad/s]
  double brake_decel{0.8};        // 제동 상한 계산용 감속 [m/s^2]
  double brake_jerk{0.0};  // 제동 상한 계산용 jerk [m/s^3] (감속이 차오르는 시간만큼 반응이 늦다), 0 이하 끔
  // 제동 상한에서 사람이 서는 감속 가정 [m/s^2]. 걷던 사람은 바로 서지 못하고 v²/(2a) 를 더 간다
  // (RSS 의 앞차 제동과 같은 생각). 그만큼 가까이 따라간다. 0 이하면 끔 (사람이 그 자리에 바로 선다고 봄)
  double person_stop_decel{0.0};
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

  // range·bearing: 현재 base_link 기준 타깃, target_forward_speed: 타깃 지면 속도의 로봇 전방 성분.
  // brake_person_speed: 제동 상한(person_stop_decel)에 쓰는 사람 전방 속도 [m/s]. 음수면 target_forward_speed
  // (호출하는 쪽이 걸러진·보수적인 값을 넘긴다)
  FollowCommand compute(double range, double bearing, double target_forward_speed,
                        double brake_person_speed = -1.0) const;

  const FollowLawParams& params() const { return p_; }
  void setParams(const FollowLawParams& p) { p_ = p; }  // 실행 중 조정

 private:
  FollowLawParams p_;
};

}  // namespace kebi_follow
