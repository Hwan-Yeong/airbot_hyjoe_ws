#include "kebi_follow_controller/velocity_limiter.hpp"

#include <algorithm>
#include <cmath>

namespace kebi_follow {

void VelocityLimiter::step(double v_target, double w_target, double dt) {
  if (!(dt > 0.0)) {
    return;
  }
  v_target = std::clamp(v_target, 0.0, p_.max_linear_speed);
  w_target = std::clamp(w_target, -p_.max_angular_speed, p_.max_angular_speed);

  // 선속도
  double v_new = v_;
  if (v_target > v_) {
    double a = p_.max_linear_accel;
    if (p_.max_linear_jerk > 0.0) {
      // jerk 제한 + 목표에 닿을 때 가속도가 0 이 되도록 (S 곡선)
      const double a_prev = std::max(a_, 0.0);
      a = std::min(a, a_prev + p_.max_linear_jerk * dt);
      a = std::min(a, std::sqrt(2.0 * p_.max_linear_jerk * (v_target - v_)));
    }
    v_new = std::min(v_ + a * dt, v_target);
  } else if (v_target < v_) {
    v_new = std::max(v_ - p_.max_linear_decel * dt, v_target);
  }
  a_ = (v_new - v_) / dt;
  v_ = v_new;

  // 각속도: 크기가 커지면 accel, 줄거나 부호가 바뀌면 decel
  const bool speeding_up =
      std::abs(w_target) > std::abs(w_) && w_ * w_target >= 0.0;
  const double rate =
      (speeding_up ? p_.max_angular_accel : p_.max_angular_decel) * dt;
  w_ += std::clamp(w_target - w_, -rate, rate);
}

}  // namespace kebi_follow
