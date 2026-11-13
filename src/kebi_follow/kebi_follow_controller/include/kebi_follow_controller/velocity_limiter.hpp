// 가감속 제한 (SPEC 6.3, S1). ROS 의존 없음.
// - S 곡선: 선속도·각속도 모두 가속과 감속에 jerk 를 제한한다.
//   목표에 닿을 때 가속도가 0 이 되도록 미리 줄인다 (|a| <= sqrt(2 j |v_target - v|)).
// - 정지 종류 (StopProfile)
//   kComfort: 평소 값 (max_*_accel/decel/jerk). 추종, 놓침, 모호, 회전 상한
//   kFirm:    단호한 감속 (firm_*). 근접(S2), 장애물(S3), 센서 끊김(S6), 회전 금지(S4)
//   kCap:     안전 상한(사람 앞 제동·장애물·카메라 이상)이 지금 궤적 아래로 갑자기 내려갔을 때 (선속도만).
//             감속 크기는 편안 값, jerk 는 단호 값: 가속을 바로 멈추고 감속을 빨리 건다 (상한을 지나치지 않게)
//   즉시 0:   stopLinear()/stopAngular()/reset(). HALT(낭떠러지·충돌·들림·비상정지)
// - 후진 없음: 목표 선속도는 [0, max_linear_speed] 로 자른다
#pragma once

namespace kebi_follow {

enum class StopProfile { kComfort, kFirm, kCap };

struct LimiterParams {
  double max_linear_speed{0.5};   // [m/s]
  double max_angular_speed{0.8};  // [rad/s]
  double max_linear_accel{0.4};   // [m/s^2]
  double max_linear_decel{0.8};   // [m/s^2] 편안 정지
  double max_linear_jerk{2.0};    // [m/s^3] 가속·감속 모두, 0 이하면 끔
  double max_angular_accel{1.5};  // [rad/s^2]
  double max_angular_decel{1.5};  // [rad/s^2]
  double max_angular_jerk{0.0};   // [rad/s^3], 0 이하면 끔
  double firm_linear_decel{1.2};  // [m/s^2] 단호 정지 (편안 값보다 작으면 편안 값을 쓴다). 기본값은 config 와 같게
  double firm_linear_jerk{5.0};   // [m/s^3]
  double firm_angular_decel{4.0};  // [rad/s^2]
  double firm_angular_jerk{40.0};  // [rad/s^3]
};

class VelocityLimiter {
 public:
  explicit VelocityLimiter(const LimiterParams& p = LimiterParams{}) : p_(p) {}

  void step(double v_target, double w_target, double dt,
            StopProfile linear = StopProfile::kComfort,
            StopProfile angular = StopProfile::kComfort);
  void stopLinear() {
    v_ = 0.0;
    a_ = 0.0;
  }
  void stopAngular() {
    w_ = 0.0;
    alpha_ = 0.0;
  }
  void reset() {
    stopLinear();
    stopAngular();
  }

  double v() const { return v_; }
  double w() const { return w_; }
  double linearAccel() const { return a_; }
  double angularAccel() const { return alpha_; }
  const LimiterParams& params() const { return p_; }
  // 실행 중 조정: 현재 속도 상태는 그대로 두고 상한·가감속만 바꾼다
  void setParams(const LimiterParams& p) { p_ = p; }

 private:
  LimiterParams p_;
  double v_{0.0};
  double w_{0.0};
  double a_{0.0};      // 선가속도 [m/s^2]
  double alpha_{0.0};  // 각가속도 [rad/s^2]
};

// 한 축 S 곡선 한 걸음 (시험·시뮬레이터에서도 쓴다).
// a_up: |v| 가 커질 때 한계, a_dn: 줄어들 때 한계, jerk <= 0 이면 가속도를 바로 바꾼다
void sCurveStep(double target, double a_up, double a_dn, double jerk, double dt,
                double* v, double* a);

}  // namespace kebi_follow
