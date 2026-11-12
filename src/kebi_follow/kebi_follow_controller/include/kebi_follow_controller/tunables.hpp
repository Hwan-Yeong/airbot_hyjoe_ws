// 실행 중 조정 가능한 follow_controller 파라미터 표 (노드·시뮬레이터 공용). ROS 의존 없음.
// 조정 UI 등에서 ros2 param set 으로 바꿀 수 있다. 범위 밖이거나 서로 모순되면 거부하고,
// 받은 값은 다음 제어 주기 시작에 한꺼번에 적용한다 (SPEC 6.3 "실행 중 조정"). 나머지는 기동 때만 읽는다.
#pragma once

#include <string>

#include "kebi_follow_controller/controller_core.hpp"
#include "kebi_follow_controller/types.hpp"

namespace kebi_follow {

struct TunableSpec {
  const char* name;
  double lo;
  double hi;
  const char* desc;
  double (*get)(const ControllerParams&);
  void (*set)(ControllerParams&, double);
};

constexpr double kMinStopToFollowGap = 0.1;  // stop_distance + 이 값 <= follow_distance [m]

inline const TunableSpec kTunables[] = {
    // S1 속도·가감속
    {"max_linear_speed", 0.0, 1.0, "선속도 상한 [m/s]",
     [](const ControllerParams& p) { return p.limiter.max_linear_speed; },
     [](ControllerParams& p, double v) { p.limiter.max_linear_speed = v; }},
    {"max_angular_speed", 0.0, 2.0, "각속도 상한 [rad/s]",
     [](const ControllerParams& p) { return p.limiter.max_angular_speed; },
     [](ControllerParams& p, double v) { p.limiter.max_angular_speed = v; }},
    {"max_linear_accel", 0.1, 1.0, "선가속 [m/s^2]",
     [](const ControllerParams& p) { return p.limiter.max_linear_accel; },
     [](ControllerParams& p, double v) { p.limiter.max_linear_accel = v; }},
    {"max_linear_decel", 0.3, 1.5, "선감속 (일반 정지) [m/s^2]",
     [](const ControllerParams& p) { return p.limiter.max_linear_decel; },
     [](ControllerParams& p, double v) { p.limiter.max_linear_decel = v; }},
    {"max_linear_jerk", 0.0, 10.0, "선 jerk, 가속·감속 모두 [m/s^3] (S 곡선, 0: 끔)",
     [](const ControllerParams& p) { return p.limiter.max_linear_jerk; },
     [](ControllerParams& p, double v) { p.limiter.max_linear_jerk = v; }},
    {"max_angular_accel", 0.3, 3.0, "각가속 [rad/s^2]",
     [](const ControllerParams& p) { return p.limiter.max_angular_accel; },
     [](ControllerParams& p, double v) { p.limiter.max_angular_accel = v; }},
    {"max_angular_decel", 0.3, 3.0, "각감속 [rad/s^2]",
     [](const ControllerParams& p) { return p.limiter.max_angular_decel; },
     [](ControllerParams& p, double v) { p.limiter.max_angular_decel = v; }},
    {"max_angular_jerk", 0.0, 50.0, "각 jerk [rad/s^3] (S 곡선, 0: 끔)",
     [](const ControllerParams& p) { return p.limiter.max_angular_jerk; },
     [](ControllerParams& p, double v) { p.limiter.max_angular_jerk = v; }},
    {"firm_linear_decel", 0.8, 3.0, "단호 정지 선감속 (근접·장애물·센서) [m/s^2]",
     [](const ControllerParams& p) { return p.limiter.firm_linear_decel; },
     [](ControllerParams& p, double v) { p.limiter.firm_linear_decel = v; }},
    {"firm_linear_jerk", 2.0, 100.0, "단호 정지 선 jerk [m/s^3]",
     [](const ControllerParams& p) { return p.limiter.firm_linear_jerk; },
     [](ControllerParams& p, double v) { p.limiter.firm_linear_jerk = v; }},
    {"firm_angular_decel", 1.0, 10.0, "단호 정지 각감속 (회전 금지·센서) [rad/s^2]",
     [](const ControllerParams& p) { return p.limiter.firm_angular_decel; },
     [](ControllerParams& p, double v) { p.limiter.firm_angular_decel = v; }},
    {"firm_angular_jerk", 5.0, 200.0, "단호 정지 각 jerk [rad/s^3]",
     [](const ControllerParams& p) { return p.limiter.firm_angular_jerk; },
     [](ControllerParams& p, double v) { p.limiter.firm_angular_jerk = v; }},
    // S2 추종 법칙
    {"follow_distance", 0.8, 2.5, "추종 거리 [m]",
     [](const ControllerParams& p) { return p.law.follow_distance; },
     [](ControllerParams& p, double v) { p.law.follow_distance = v; }},
    {"stop_distance", 0.5, 1.5, "이 거리 이내면 즉시 정지 [m]",
     [](const ControllerParams& p) { return p.law.stop_distance; },
     [](ControllerParams& p, double v) { p.law.stop_distance = v; }},
    {"approach_margin", 0.0, 0.5, "사람이 갑자기 서도 정지 거리 + 이 값 앞에서 선다 [m]",
     [](const ControllerParams& p) { return p.law.approach_margin; },
     [](ControllerParams& p, double v) { p.law.approach_margin = v; }},
    {"reaction_time", 0.0, 0.5, "사람 앞 제동 상한 계산 지연 [s]",
     [](const ControllerParams& p) { return p.law.reaction_time; },
     [](ControllerParams& p, double v) { p.law.reaction_time = v; }},
    {"k_v", 0.1, 3.0, "거리 오차 → 선속도 게인 [1/s]",
     [](const ControllerParams& p) { return p.law.k_v; },
     [](ControllerParams& p, double v) { p.law.k_v = v; }},
    {"k_w", 0.2, 3.0, "방향각 → 각속도 게인 [1/s]",
     [](const ControllerParams& p) { return p.law.k_w; },
     [](ControllerParams& p, double v) { p.law.k_w = v; }},
    {"ff_gain", 0.0, 1.5, "사람 속도 피드포워드 배율",
     [](const ControllerParams& p) { return p.law.ff_gain; },
     [](ControllerParams& p, double v) { p.law.ff_gain = v; }},
    {"ff_min_target_speed", 0.0, 0.4, "피드포워드를 섞기 시작하는 사람 속도 중심 [m/s]",
     [](const ControllerParams& p) { return p.law.ff_min_target_speed; },
     [](ControllerParams& p, double v) { p.law.ff_min_target_speed = v; }},
    {"start_speed", 0.0, 0.3, "서 있을 때 이보다 작은 요구로는 출발 안 함 [m/s]",
     [](const ControllerParams& p) { return p.start_speed; },
     [](ControllerParams& p, double v) { p.start_speed = v; }},
    {"v_filter_s", 0.0, 1.5, "추종 선속도 요구 저역 통과 [s] (0: 끔)",
     [](const ControllerParams& p) { return p.v_filter_s; },
     [](ControllerParams& p, double v) { p.v_filter_s = v; }},
    {"w_filter_s", 0.0, 1.0, "추종 각속도 요구 저역 통과 [s] (0: 끔)",
     [](const ControllerParams& p) { return p.w_filter_s; },
     [](ControllerParams& p, double v) { p.w_filter_s = v; }},
    {"bearing_deadband", 0.0, 0.3, "방향각 불감대 [rad]",
     [](const ControllerParams& p) { return p.law.bearing_deadband; },
     [](ControllerParams& p, double v) { p.law.bearing_deadband = v; }},
    {"slow_bearing_deg", 0.0, 60.0, "이 방향각부터 선속도를 줄임 [deg]",
     [](const ControllerParams& p) { return radToDeg(p.law.slow_bearing); },
     [](ControllerParams& p, double v) { p.law.slow_bearing = degToRad(v); }},
    {"turn_in_place_deg", 10.0, 90.0, "이 방향각부터 제자리 회전 [deg]",
     [](const ControllerParams& p) {
       return radToDeg(p.law.turn_in_place_bearing);
     },
     [](ControllerParams& p, double v) {
       p.law.turn_in_place_bearing = degToRad(v);
     }},
    // S3 장애물
    {"slow_distance", 0.5, 3.0, "장애물 감속 시작 거리 [m]",
     [](const ControllerParams& p) { return p.slow_distance; },
     [](ControllerParams& p, double v) { p.slow_distance = v; }},
    {"stop_margin", 0.15, 0.6, "라이다 장애물 정지 여유 [m]",
     [](const ControllerParams& p) { return p.stop_margin; },
     [](ControllerParams& p, double v) { p.stop_margin = v; }},
    {"depth_stop_margin", 0.3, 1.0, "depth 장애물 정지 여유 [m]",
     [](const ControllerParams& p) { return p.depth_stop_margin; },
     [](ControllerParams& p, double v) { p.depth_stop_margin = v; }},
    {"obstacle_reaction_time", 0.0, 0.5, "장애물 제동 계산 지연 [s]",
     [](const ControllerParams& p) { return p.obstacle_reaction_time; },
     [](ControllerParams& p, double v) { p.obstacle_reaction_time = v; }},
    // S4 회전, S6·S7
    {"rotation_limit_s", 1.0, 10.0, "제자리 회전 연속 상한 [s]",
     [](const ControllerParams& p) { return p.rotation_limit_s; },
     [](ControllerParams& p, double v) { p.rotation_limit_s = v; }},
    {"rotation_rearm_s", 0.5, 5.0, "회전 상한 뒤 다시 허용까지 [s]",
     [](const ControllerParams& p) { return p.rotation_rearm_s; },
     [](ControllerParams& p, double v) { p.rotation_rearm_s = v; }},
    {"target_lost_s", 0.3, 2.0, "측정 없는 시간이 이를 넘으면 정지 [s]",
     [](const ControllerParams& p) { return p.target_lost_s; },
     [](ControllerParams& p, double v) { p.target_lost_s = v; }},
    {"occluded_speed_factor", 0.0, 1.0, "가림 중 선속도 배율",
     [](const ControllerParams& p) { return p.occluded_speed_factor; },
     [](ControllerParams& p, double v) { p.occluded_speed_factor = v; }},
    {"camera_missing_speed_cap", 0.0, 0.5, "카메라·depth 비정상 때 선속도 상한 [m/s]",
     [](const ControllerParams& p) { return p.camera_missing_speed_cap; },
     [](ControllerParams& p, double v) { p.camera_missing_speed_cap = v; }},
};

inline const TunableSpec* findTunable(const std::string& name) {
  for (const auto& s : kTunables) {
    if (name == s.name) {
      return &s;
    }
  }
  return nullptr;
}

// 추종 법칙이 함께 쓰는 값 (limiter 와 같아야 한다)
inline void syncDerived(ControllerParams* p) {
  p->law.max_linear_speed = p->limiter.max_linear_speed;
  p->law.max_angular_speed = p->limiter.max_angular_speed;
  p->law.brake_decel = p->limiter.max_linear_decel;
  p->law.brake_jerk = p->limiter.max_linear_jerk;
}

// 서로 모순되는 조합이면 이유, 아니면 빈 문자열
inline std::string inconsistency(const ControllerParams& p) {
  if (p.law.stop_distance + kMinStopToFollowGap > p.law.follow_distance) {
    return "stop_distance (" + std::to_string(p.law.stop_distance) +
           ") must be at least 0.1 m below follow_distance (" +
           std::to_string(p.law.follow_distance) + ")";
  }
  if (p.law.slow_bearing >= p.law.turn_in_place_bearing) {
    return "slow_bearing_deg must be below turn_in_place_deg";
  }
  return "";
}

}  // namespace kebi_follow
