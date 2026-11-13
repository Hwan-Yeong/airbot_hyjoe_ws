// 가감속·jerk 제한 (S1)
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "kebi_follow_controller/velocity_limiter.hpp"

using namespace kebi_follow;

namespace {
constexpr double kDt = 0.05;  // 20 Hz
constexpr double kTol = 1e-9;
}  // namespace

TEST(Limiter, AccelLimitWithoutJerk) {
  LimiterParams p;
  p.max_linear_jerk = 0.0;
  VelocityLimiter lim(p);
  for (int i = 0; i < 10; ++i)
    lim.step(0.5, 0.0, kDt);  // 0.5 s
  EXPECT_NEAR(lim.v(), 0.2, kTol);
  for (int i = 0; i < 100; ++i)
    lim.step(0.5, 0.0, kDt);
  EXPECT_NEAR(lim.v(), 0.5, kTol);
}

TEST(Limiter, DecelLimitWithoutJerk) {
  LimiterParams p;
  p.max_linear_jerk = 0.0;  // jerk 끔: 사다리꼴 (예전 감속 동작)
  VelocityLimiter lim(p);
  for (int i = 0; i < 200; ++i)
    lim.step(0.5, 0.0, kDt);
  ASSERT_NEAR(lim.v(), 0.5, kTol);
  lim.step(0.0, 0.0, kDt);  // jerk 끄면 바로 0.8 m/s^2
  EXPECT_NEAR(lim.v(), 0.5 - 0.8 * kDt, kTol);
  for (int i = 0; i < 4; ++i)
    lim.step(0.0, 0.0, kDt);
  EXPECT_NEAR(lim.v(), 0.5 - 0.8 * 0.25, kTol);
  for (int i = 0; i < 20; ++i)
    lim.step(0.0, 0.0, kDt);
  EXPECT_DOUBLE_EQ(lim.v(), 0.0);
}

TEST(Limiter, JerkLimitedAccelerationNoOvershoot) {
  LimiterParams p;  // jerk 2.0 m/s^3
  VelocityLimiter lim(p);
  double a_prev = 0.0;
  double v_prev = 0.0;
  double max_a = 0.0;
  for (int i = 0; i < 100; ++i) {
    lim.step(0.3, 0.0, kDt);
    const double a = (lim.v() - v_prev) / kDt;
    EXPECT_LE(a, p.max_linear_accel + kTol);
    EXPECT_LE(a - a_prev, p.max_linear_jerk * kDt + kTol) << "step " << i;
    EXPECT_LE(lim.v(), 0.3 + kTol);
    max_a = std::max(max_a, a);
    a_prev = a;
    v_prev = lim.v();
  }
  EXPECT_NEAR(lim.v(), 0.3, kTol);
  EXPECT_NEAR(max_a, 0.4, 0.05);  // 가속 상한까지는 간다
}

TEST(Limiter, JerkRampsAccelerationFromRest) {
  LimiterParams p;
  VelocityLimiter lim(p);
  lim.step(0.5, 0.0, kDt);
  EXPECT_NEAR(lim.linearAccel(), p.max_linear_jerk * kDt, kTol);  // 0.1 m/s^2
  lim.step(0.5, 0.0, kDt);
  EXPECT_NEAR(lim.linearAccel(), 2.0 * p.max_linear_jerk * kDt, kTol);
}

TEST(Limiter, NoReverseAndSpeedClamp) {
  VelocityLimiter lim;
  for (int i = 0; i < 100; ++i)
    lim.step(-1.0, 0.0, kDt);
  EXPECT_DOUBLE_EQ(lim.v(), 0.0);
  for (int i = 0; i < 200; ++i)
    lim.step(2.0, 5.0, kDt);
  EXPECT_NEAR(lim.v(), 0.5, kTol);
  EXPECT_NEAR(lim.w(), 0.8, kTol);
}

TEST(Limiter, AngularAccelAndDecel) {
  VelocityLimiter lim;
  lim.step(0.0, 0.8, kDt);
  EXPECT_NEAR(lim.w(), 1.5 * kDt, kTol);
  for (int i = 0; i < 20; ++i)
    lim.step(0.0, 0.8, kDt);
  EXPECT_NEAR(lim.w(), 0.8, kTol);
  lim.step(0.0, -0.8, kDt);  // 반대 방향은 감속으로 제한
  EXPECT_NEAR(lim.w(), 0.8 - 1.5 * kDt, kTol);
}

TEST(Limiter, ImmediateStops) {
  VelocityLimiter lim;
  for (int i = 0; i < 100; ++i)
    lim.step(0.5, 0.5, kDt);
  lim.stopLinear();
  EXPECT_DOUBLE_EQ(lim.v(), 0.0);
  EXPECT_DOUBLE_EQ(lim.linearAccel(), 0.0);
  EXPECT_NEAR(lim.w(), 0.5, kTol);
  lim.reset();
  EXPECT_DOUBLE_EQ(lim.w(), 0.0);
}

TEST(Limiter, ZeroOrNegativeDtIgnored) {
  VelocityLimiter lim;
  lim.step(0.5, 0.5, 0.0);
  lim.step(0.5, 0.5, -1.0);
  EXPECT_DOUBLE_EQ(lim.v(), 0.0);
  EXPECT_DOUBLE_EQ(lim.w(), 0.0);
}

TEST(Limiter, SCurveDecelerationBothWays) {
  // 2026-11-13: 감속도 jerk 제한 (S 곡선). 가속도 변화 <= jerk·dt, 목표 아래로 내려가지 않고 0 에서 선다
  LimiterParams p;  // jerk 2.0, 감속 0.8
  VelocityLimiter lim(p);
  for (int i = 0; i < 200; ++i)
    lim.step(0.5, 0.0, kDt);
  ASSERT_NEAR(lim.v(), 0.5, kTol);
  double a_prev = 0.0;
  double v_prev = lim.v();
  double min_a = 0.0;
  for (int i = 0; i < 100; ++i) {
    lim.step(0.0, 0.0, kDt);
    const double a = (lim.v() - v_prev) / kDt;
    EXPECT_GE(a, -p.max_linear_decel - kTol) << i;
    // 착지 마지막 한 걸음(속도 0.01 m/s 미만)은 이산 시간이라 jerk 한계를 15 % 까지 넘을 수 있다
    EXPECT_LE(std::abs(a - a_prev), 1.15 * p.max_linear_jerk * kDt) << i;
    EXPECT_GE(lim.v(), 0.0);
    EXPECT_LE(lim.v(), v_prev + kTol);
    min_a = std::min(min_a, a);
    a_prev = a;
    v_prev = lim.v();
  }
  EXPECT_DOUBLE_EQ(lim.v(), 0.0);
  EXPECT_LT(min_a, -0.5);  // 감속 상한 가까이까지는 쓴다
}

TEST(Limiter, FirmStopIsFasterButContinuous) {
  LimiterParams p;
  VelocityLimiter comfort(p), firm(p);
  for (int i = 0; i < 200; ++i) {
    comfort.step(0.5, 0.0, kDt);
    firm.step(0.5, 0.0, kDt);
  }
  int n_comfort = 0, n_firm = 0;
  while (comfort.v() > 0.0 && n_comfort < 200) {
    comfort.step(0.0, 0.0, kDt);
    ++n_comfort;
  }
  double v_prev = firm.v();
  while (firm.v() > 0.0 && n_firm < 200) {
    firm.step(0.0, 0.0, kDt, StopProfile::kFirm);
    EXPECT_LE(v_prev - firm.v(), p.firm_linear_decel * kDt + kTol);  // 단호 감속 한계 안
    v_prev = firm.v();
    ++n_firm;
  }
  EXPECT_LT(n_firm, n_comfort);
  // 0.5 m/s 에서 단호 S 곡선 정지 시간 (v/a + a/j) + 두 주기 안 (기본값 1.2·5: 0.66 s)
  const double a = p.firm_linear_decel;
  const double j = p.firm_linear_jerk;
  EXPECT_LE(n_firm * kDt, 0.5 / a + a / j + 2.0 * kDt);
}

TEST(Limiter, LoweringCapAtRuntimeDeceleratesSmoothly) {
  // 실행 중 상한을 낮추면 잘라 내지 않고 S 곡선으로 내려간다
  LimiterParams p;
  VelocityLimiter lim(p);
  for (int i = 0; i < 200; ++i)
    lim.step(0.5, 0.0, kDt);
  p.max_linear_speed = 0.2;
  lim.setParams(p);
  lim.step(0.5, 0.0, kDt);
  EXPECT_GT(lim.v(), 0.45);
  for (int i = 0; i < 100; ++i)
    lim.step(0.5, 0.0, kDt);
  EXPECT_NEAR(lim.v(), 0.2, kTol);
}

TEST(Limiter, AngularSCurve) {
  LimiterParams p;
  p.max_angular_jerk = 4.0;
  VelocityLimiter lim(p);
  double al_prev = 0.0;
  double w_prev = 0.0;
  for (int i = 0; i < 60; ++i) {
    lim.step(0.0, 0.8, kDt);
    const double al = (lim.w() - w_prev) / kDt;
    EXPECT_LE(std::abs(al - al_prev), 1.15 * p.max_angular_jerk * kDt) << i;
    EXPECT_LE(lim.w(), 0.8 + kTol);
    al_prev = al;
    w_prev = lim.w();
  }
  EXPECT_NEAR(lim.w(), 0.8, kTol);
}

namespace {
// 야간 기본값 (follow_controller.yaml): 선 0.8 m/s, 가속 0.5, 감속 0.8, jerk 0.6, 단호 jerk 15
LimiterParams nightParams() {
  LimiterParams p;
  p.max_linear_speed = 0.8;
  p.max_linear_accel = 0.5;
  p.max_linear_decel = 0.8;
  p.max_linear_jerk = 0.6;
  p.firm_linear_jerk = 15.0;
  return p;
}

// 0.8 을 향해 가속도가 0.45 이상 붙을 때까지 가속한다
void accelerateHard(VelocityLimiter& lim) {
  for (int i = 0; i < 200 && lim.linearAccel() < 0.45; ++i)
    lim.step(0.8, 0.0, kDt);
  ASSERT_GE(lim.linearAccel(), 0.45);
}
}  // namespace

TEST(Limiter, ComfortSCurveOvershootsSuddenLowerTarget) {
  // 리뷰 2026-11-13: 편안 S 곡선은 가속 중 목표가 갑자기 내려가면 가속도를 jerk 로 줄이는 동안 지나친다
  VelocityLimiter lim(nightParams());
  accelerateHard(lim);
  const double v0 = lim.v();
  double v_max = v0;
  for (int i = 0; i < 40; ++i) {
    lim.step(v0 - 0.1, 0.0, kDt, StopProfile::kComfort);
    v_max = std::max(v_max, lim.v());
  }
  EXPECT_GT(v_max - (v0 - 0.1), 0.2);  // 이래서 안전 상한에는 kCap 을 쓴다
}

TEST(Limiter, CapProfileStopsAccelerationAndBrakesQuickly) {
  VelocityLimiter lim(nightParams());
  accelerateHard(lim);
  const double v0 = lim.v();
  const double cap = v0 - 0.1;
  double v_max = v0;
  double t_reach = -1.0;
  for (int i = 0; i < 40; ++i) {
    lim.step(cap, 0.0, kDt, StopProfile::kCap);
    v_max = std::max(v_max, lim.v());
    EXPECT_GE(lim.linearAccel(), -0.8 - kTol);  // 감속 크기는 편안 값
    if (t_reach < 0.0 && lim.v() <= cap + 1e-6)
      t_reach = (i + 1) * kDt;
  }
  EXPECT_LE(v_max, v0 + 0.03);  // 가속을 바로 멈춘다 (편안이면 +0.2 m/s 넘게)
  ASSERT_GT(t_reach, 0.0);
  EXPECT_LE(t_reach, 0.6);
  EXPECT_NEAR(lim.v(), cap, 1e-6);
}

TEST(Limiter, JerkSwitchedOffAtTargetSnapsInsteadOfOvershooting) {
  // 실행 중 jerk 를 0 으로 바꾼 직후: 목표에 있는데 S 곡선의 가속도가 남아 있다
  double v = 0.5;
  double a = 0.3;
  sCurveStep(0.5, 0.5, 0.8, 0.0, kDt, &v, &a);
  EXPECT_DOUBLE_EQ(v, 0.5);
  EXPECT_DOUBLE_EQ(a, 0.0);
}
