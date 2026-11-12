// 가감속 제한 (SPEC 6.3, S1). ROS 의존 없음.
// - 선속도: 가속은 max_linear_accel + jerk 제한, 감속은 max_linear_decel (jerk 제한 없음: 제동을 늦추지 않는다)
// - 후진 없음: 목표 선속도는 [0, max_linear_speed] 로 자른다
// - 즉시 정지(stopLinear/stopAngular)는 제한 없이 0 으로 만든다 (HALT·장애물 정지)
#pragma once

namespace kebi_follow {

struct LimiterParams {
  double max_linear_speed{0.5};   // [m/s]
  double max_angular_speed{0.8};  // [rad/s]
  double max_linear_accel{0.4};   // [m/s^2]
  double max_linear_decel{0.8};   // [m/s^2]
  double max_linear_jerk{2.0};    // [m/s^3], 0 이하면 끔 (가속 쪽만)
  double max_angular_accel{1.5};  // [rad/s^2]
  double max_angular_decel{1.5};  // [rad/s^2]
};

class VelocityLimiter {
 public:
  explicit VelocityLimiter(const LimiterParams& p = LimiterParams{}) : p_(p) {}

  void step(double v_target, double w_target, double dt);
  void stopLinear() {
    v_ = 0.0;
    a_ = 0.0;
  }
  void stopAngular() { w_ = 0.0; }
  void reset() {
    stopLinear();
    stopAngular();
  }

  double v() const { return v_; }
  double w() const { return w_; }
  double linearAccel() const { return a_; }
  const LimiterParams& params() const { return p_; }

 private:
  LimiterParams p_;
  double v_{0.0};
  double w_{0.0};
  double a_{0.0};  // 직전 주기 선가속도
};

}  // namespace kebi_follow
