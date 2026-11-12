// 가감속·jerk 제한 (S1)
#include <gtest/gtest.h>

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

TEST(Limiter, DecelLimit) {
  LimiterParams p;
  VelocityLimiter lim(p);
  for (int i = 0; i < 200; ++i)
    lim.step(0.5, 0.0, kDt);
  ASSERT_NEAR(lim.v(), 0.5, kTol);
  lim.step(0.0, 0.0, kDt);  // 제동은 jerk 제한 없이 바로 0.8 m/s^2
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
