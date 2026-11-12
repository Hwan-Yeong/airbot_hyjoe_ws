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
                                 double target_forward_speed) const {
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
  const double v_ff = (target_forward_speed > p_.ff_min_target_speed)
                          ? p_.ff_gain * target_forward_speed
                          : 0.0;
  double v = std::max(0.0, p_.k_v * e + v_ff);

  // 사람이 지금 멈춰도 stop_distance + approach_margin 앞에서 설 수 있는 속도까지만
  c.v_cap = stoppingSpeed(range - p_.stop_distance - p_.approach_margin,
                          p_.brake_decel, p_.reaction_time);
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
