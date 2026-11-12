#include "kebi_follow_controller/velocity_limiter.hpp"

#include <algorithm>
#include <cmath>

namespace kebi_follow {

namespace {
constexpr double kEps = 1e-9;
}  // namespace

void sCurveStep(double target, double a_up, double a_dn, double jerk, double dt,
                double* v, double* a) {
  const double e = target - *v;
  if (std::abs(e) < kEps && std::abs(*a) < kEps) {
    *v = target;
    *a = 0.0;
    return;
  }
  if (jerk > 0.0) {
    // 착지: 이번 걸음에 목표에 정확히 닿고, 그다음 가속도 0 으로 내려도 jerk 한계 안이면 붙인다
    const double a_exact = e / dt;
    if (std::abs(a_exact) <= jerk * dt + kEps &&
        std::abs(a_exact - *a) <= jerk * dt + kEps) {
      *v = target;
      *a = 0.0;
      return;
    }
  }
  // |v| 가 커지는 쪽(같은 부호로 더 큰 목표)이면 가속 한계, 아니면 감속 한계
  const bool growing =
      (target * *v >= 0.0) && (std::abs(target) > std::abs(*v));
  const double a_lim = std::max(growing ? a_up : a_dn, 0.0);
  if (jerk > 0.0) {
    // 목표에 닿을 때 가속도가 0 이 되도록: |a| <= sqrt(2 j |e|) 의 이산 시간판
    // (한 걸음 jerk·dt 만큼 늦게 줄어드는 것을 빼서 지나치지 않게)
    const double h = 0.5 * jerk * dt;
    const double a_stop = -h + std::sqrt(h * h + 2.0 * jerk * std::abs(e));
    const double a_des = std::copysign(std::min(a_lim, a_stop), e);
    *a += std::clamp(a_des - *a, -jerk * dt, jerk * dt);
  } else {
    *a = std::copysign(a_lim, e);
  }
  if (jerk <= 0.0) {
    // jerk 제한이 없으면 사다리꼴: 목표를 지나치지 않게 붙인다
    const double v_new = *v + *a * dt;
    if ((e > 0.0 && v_new >= target) || (e < 0.0 && v_new <= target)) {
      *v = target;
      *a = 0.0;
      return;
    }
    *v = v_new;
    return;
  }
  // jerk 제한: 가속도를 끊지 않는다 (목표가 갑자기 바뀌면 조금 지나쳤다가 jerk 한계 안에서 돌아온다)
  *v += *a * dt;
}

void VelocityLimiter::step(double v_target, double w_target, double dt,
                           StopProfile linear, StopProfile angular) {
  if (!(dt > 0.0)) {
    return;
  }
  v_target = std::clamp(v_target, 0.0, p_.max_linear_speed);
  w_target = std::clamp(w_target, -p_.max_angular_speed, p_.max_angular_speed);

  // 선속도
  const bool firm_v = (linear == StopProfile::kFirm);
  const double v_dn = firm_v ? std::max(p_.firm_linear_decel, p_.max_linear_decel)
                             : p_.max_linear_decel;
  double j_v = p_.max_linear_jerk;
  if (firm_v && j_v > 0.0) {
    j_v = std::max(p_.firm_linear_jerk, j_v);
  }
  const bool v_within = v_ <= p_.max_linear_speed + 1e-9;
  sCurveStep(v_target, p_.max_linear_accel, v_dn, j_v, dt, &v_, &a_);
  if (v_ < 0.0) {  // 후진 없음
    v_ = 0.0;
    a_ = std::max(a_, 0.0);
  } else if (v_within && v_ > p_.max_linear_speed) {
    // 상한 안에서 가속하다 살짝 넘친 것만 자른다 (실행 중 상한을 낮춘 경우는 S 곡선으로 내려간다)
    v_ = p_.max_linear_speed;
    a_ = std::min(a_, 0.0);
  }

  // 각속도
  const bool firm_w = (angular == StopProfile::kFirm);
  const double w_dn =
      firm_w ? std::max(p_.firm_angular_decel, p_.max_angular_decel)
             : p_.max_angular_decel;
  double j_w = p_.max_angular_jerk;
  if (firm_w && j_w > 0.0) {
    j_w = std::max(p_.firm_angular_jerk, j_w);
  }
  const bool w_within = std::abs(w_) <= p_.max_angular_speed + 1e-9;
  sCurveStep(w_target, p_.max_angular_accel, w_dn, j_w, dt, &w_, &alpha_);
  if (w_within && std::abs(w_) > p_.max_angular_speed) {
    w_ = std::copysign(p_.max_angular_speed, w_);
    alpha_ = (alpha_ * w_ > 0.0) ? 0.0 : alpha_;
  }
}

}  // namespace kebi_follow
