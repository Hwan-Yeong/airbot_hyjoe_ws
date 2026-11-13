// 추종 법칙 (SPEC 6.3, S2)
#include <gtest/gtest.h>

#include <cmath>

#include "kebi_follow_controller/follow_law.hpp"

using namespace kebi_follow;

TEST(FollowLaw, StationaryTargetAtFollowDistance) {
  FollowLaw law;
  const auto c = law.compute(1.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(c.v, 0.0);
  EXPECT_DOUBLE_EQ(c.w, 0.0);
  EXPECT_FALSE(c.too_close);
}

TEST(FollowLaw, FarTargetCappedAtMaxSpeed) {
  FollowLaw law;
  const auto c = law.compute(3.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(c.v, 0.5);
  EXPECT_GT(law.compute(1.3, 0.0, 0.0).v, 0.0);
  EXPECT_LT(law.compute(1.3, 0.0, 0.0).v, 0.5);
}

TEST(FollowLaw, NeverReverses) {
  FollowLaw law;
  for (double r : {0.3, 0.79, 0.85, 0.95, 1.0}) {
    for (double vt : {-1.0, -0.3, 0.0}) {
      EXPECT_GE(law.compute(r, 0.0, vt).v, 0.0) << r << " " << vt;
      EXPECT_DOUBLE_EQ(law.compute(r, 0.0, vt).v, 0.0) << r << " " << vt;
    }
  }
}

TEST(FollowLaw, TooCloseStopsButMayTurn) {
  FollowLaw law;
  const auto c = law.compute(0.8, 0.5, 1.0);
  EXPECT_TRUE(c.too_close);
  EXPECT_DOUBLE_EQ(c.v, 0.0);
  EXPECT_GT(c.w, 0.0);
  EXPECT_TRUE(c.in_place);
  EXPECT_FALSE(law.compute(0.81, 0.0, 0.0).too_close);
}

TEST(FollowLaw, FeedForwardWithDeadband) {
  FollowLaw law;
  // 목표 거리 1.0 m 보다 0.15 m 멀 때, 타깃이 0.2 m/s 로 멀어지면 v = k_v e + v_ff
  const auto c = law.compute(1.15, 0.0, 0.2);
  EXPECT_NEAR(c.v, 1.0 * 0.15 + 0.2, 1e-9);
  // 느린 타깃 속도는 부드럽게 섞는다: 0.05 m/s 이하 0, 0.15 m/s 이상 전부 (ff_min 0.1 기준)
  EXPECT_NEAR(law.compute(1.15, 0.0, 0.04).v, 0.15, 1e-9);
  EXPECT_NEAR(law.compute(1.15, 0.0, 0.1).v, 0.15 + 0.5 * 0.1, 1e-9);
  EXPECT_NEAR(law.compute(1.15, 0.0, 0.15).v, 0.15 + 0.15, 1e-9);
  // 다가오는 성분은 쓰지 않는다
  EXPECT_NEAR(law.compute(1.15, 0.0, -0.4).v, 0.15, 1e-9);
}

TEST(FollowLaw, BrakingCapKeepsStopDistance) {
  FollowLaw law;
  // 사람이 1 m 에서 빠르게 걸어도 지금 멈추면 0.9 m 앞에서 설 수 있는 속도까지만
  const auto c = law.compute(1.0, 0.0, 1.0);
  const double D = 1.0 - 0.8 - 0.1;
  EXPECT_NEAR(c.v * 0.2 + c.v * c.v / (2.0 * 0.8), D, 1e-9);
  EXPECT_LT(c.v, 0.5);
  EXPECT_NEAR(c.v_cap, c.v, 1e-12);
}

TEST(FollowLaw, StoppingSpeedFormula) {
  EXPECT_DOUBLE_EQ(stoppingSpeed(0.0, 0.8, 0.2), 0.0);
  EXPECT_DOUBLE_EQ(stoppingSpeed(-1.0, 0.8, 0.2), 0.0);
  const double v = stoppingSpeed(0.5, 0.8, 0.2);
  EXPECT_NEAR(v * 0.2 + v * v / 1.6, 0.5, 1e-12);
  EXPECT_NEAR(stoppingSpeed(0.5, 0.8, 0.0), std::sqrt(0.8), 1e-12);
}

TEST(FollowLaw, BearingDeadbandContinuous) {
  FollowLaw law;
  EXPECT_DOUBLE_EQ(law.compute(2.0, 0.04, 0.0).w, 0.0);
  EXPECT_DOUBLE_EQ(law.compute(2.0, -0.05, 0.0).w, 0.0);
  EXPECT_NEAR(law.compute(2.0, 0.06, 0.0).w, 1.2 * 0.01, 1e-12);
  EXPECT_NEAR(law.compute(2.0, -0.15, 0.0).w, -1.2 * 0.10, 1e-12);
  // 포화
  EXPECT_DOUBLE_EQ(law.compute(2.0, 1.5, 0.0).w, 0.8);
  EXPECT_DOUBLE_EQ(law.compute(2.0, -1.5, 0.0).w, -0.8);
}

TEST(FollowLaw, BearingSlowdownAndTurnInPlace) {
  FollowLaw law;
  const double v0 = law.compute(3.0, degToRad(15.0), 0.0).v;
  EXPECT_DOUBLE_EQ(v0, 0.5);
  EXPECT_NEAR(law.compute(3.0, degToRad(30.0), 0.0).v, 0.25, 1e-9);
  const auto at45 = law.compute(3.0, degToRad(45.0), 0.0);
  EXPECT_DOUBLE_EQ(at45.v, 0.0);
  EXPECT_TRUE(at45.in_place);
  const auto at90 = law.compute(3.0, degToRad(-90.0), 0.0);
  EXPECT_DOUBLE_EQ(at90.v, 0.0);
  EXPECT_DOUBLE_EQ(at90.w, -0.8);
  EXPECT_TRUE(at90.in_place);
  EXPECT_FALSE(law.compute(3.0, degToRad(10.0), 0.0).in_place);
}

TEST(FollowLaw, PersonStopDecelExtendsBrakingCap) {
  // 걷던 사람은 바로 서지 못한다: 사람 정지 거리 v²/(2a) 만큼 제동 상한에 여유를 더한다 (0 이면 끔)
  FollowLawParams p;
  p.follow_distance = 1.0;
  p.stop_distance = 0.5;
  p.approach_margin = 0.3;
  p.reaction_time = 0.2;
  p.brake_decel = 1.0;
  p.brake_jerk = 1.0;  // + a/2j = 0.5 s
  p.max_linear_speed = 1.5;
  const double t_r = 0.2 + 0.5;
  FollowLaw off(p);
  EXPECT_NEAR(off.compute(1.5, 0.0, 1.0).v_cap, stoppingSpeed(1.5 - 0.8, 1.0, t_r), 1e-12);
  p.person_stop_decel = 1.5;
  FollowLaw on(p);
  const double extra = 1.0 * 1.0 / (2.0 * 1.5);
  EXPECT_NEAR(on.compute(1.5, 0.0, 1.0).v_cap, stoppingSpeed(1.5 - 0.8 + extra, 1.0, t_r), 1e-12);
  // 호출한 쪽이 넘긴 (걸러진·보수적) 속도를 쓴다
  EXPECT_NEAR(on.compute(1.5, 0.0, 1.0, 0.0).v_cap, off.compute(1.5, 0.0, 1.0).v_cap, 1e-12);
  // 사람이 서 있으면 차이 없음
  EXPECT_NEAR(on.compute(1.5, 0.0, 0.0).v_cap, off.compute(1.5, 0.0, 0.0).v_cap, 1e-12);
}
