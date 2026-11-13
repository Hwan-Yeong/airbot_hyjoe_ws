#include "kebi_follow_controller/follow_law.hpp"

#include <algorithm>
#include <cmath>

namespace kebi_follow {

namespace {
constexpr double kMinSpan = 1e-6;  // 0 나눗셈 방지
}  // namespace

double stoppingSpeed(double D, double decel, double reaction_time) {
  if (D <= 0.0 || decel <= 0.0) {
    return 0.0;
  }
  // v*t_r + v^2/(2a) = D 의 양의 근
  const double at = decel * std::max(0.0, reaction_time);
  return -at + std::sqrt(at * at + 2.0 * decel * D);
}

FollowCommand FollowLaw::compute(double range, double bearing,
                                 double target_forward_speed,
                                 double brake_person_speed) const {
  FollowCommand c;

  // 각속도: 불감대만큼 빼서 연속으로 만든다 (경계 떨림 방지)
  const double ab = std::abs(bearing);
  if (ab > p_.bearing_deadband) {
    const double b = std::copysign(ab - p_.bearing_deadband, bearing);
    c.w = std::clamp(p_.k_w * b, -p_.max_angular_speed, p_.max_angular_speed);
  }

  // 물러서지 않고 멈춘다 (회전은 허용)
  if (range <= p_.stop_distance) {
    c.too_close = true;
    c.v_cap = 0.0;
    c.in_place = (c.w != 0.0);
    return c;
  }

  const double e = range - p_.follow_distance;
  // 타깃 속도 feed-forward: 0.5·ff_min 에서 0, 1.5·ff_min 에서 1 이 되도록 섞는다
  const double s = std::max(0.0, target_forward_speed);
  const double lo = 0.5 * p_.ff_min_target_speed;
  const double hi = 1.5 * p_.ff_min_target_speed;
  const double blend =
      (hi > lo) ? std::clamp((s - lo) / (hi - lo), 0.0, 1.0) : (s > 0.0 ? 1.0 : 0.0);
  const double v_ff = p_.ff_gain * s * blend;
  double v = std::max(0.0, p_.k_v * e + v_ff);

  // 사람이 지금 서기 시작해도 stop_distance + approach_margin 앞에서 설 수 있는 속도까지만.
  // 감속이 jerk 제한으로 차오르는 동안(a/2j 만큼) 반응이 늦은 것으로 본다.
  // 걷던 사람은 person_stop_decel 로 서므로 그 정지 거리만큼 더 쓸 수 있다 (0 이면 바로 선다고 봄)
  const double t_jerk =
      (p_.brake_jerk > 0.0) ? p_.brake_decel / (2.0 * p_.brake_jerk) : 0.0;
  const double vp = (brake_person_speed >= 0.0) ? brake_person_speed : s;
  const double person_stop =
      (p_.person_stop_decel > 0.0) ? vp * vp / (2.0 * p_.person_stop_decel) : 0.0;
  c.v_cap = stoppingSpeed(range - p_.stop_distance - p_.approach_margin + person_stop,
                          p_.brake_decel, p_.reaction_time + t_jerk);
  v = std::min({v, c.v_cap, p_.max_linear_speed});

  // 방위 오차가 크면 감속, turn_in_place_bearing 부터 제자리 회전
  if (ab >= p_.turn_in_place_bearing) {
    v = 0.0;
  } else if (ab > p_.slow_bearing) {
    const double span =
        std::max(p_.turn_in_place_bearing - p_.slow_bearing, kMinSpan);
    v *= (p_.turn_in_place_bearing - ab) / span;
  }
  c.v = v;
  c.in_place = (c.v <= 0.0 && c.w != 0.0);
  return c;
}

}  // namespace kebi_follow
