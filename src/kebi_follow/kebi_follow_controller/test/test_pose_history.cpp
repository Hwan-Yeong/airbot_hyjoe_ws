#include <gtest/gtest.h>

#include <cmath>

#include "kebi_follow_controller/pose_history.hpp"

using namespace kebi_follow;

TEST(PoseHistory, InterpolatesAndWrapsYaw) {
  PoseHistory h(16);
  h.add(1.0, {0.0, 0.0, 3.1});
  h.add(2.0, {1.0, 2.0, -3.1});  // yaw 가 ±pi 를 넘어감
  Pose2D p;
  ASSERT_TRUE(h.at(1.5, 0.1, &p));
  EXPECT_NEAR(p.x, 0.5, 1e-12);
  EXPECT_NEAR(p.y, 1.0, 1e-12);
  EXPECT_NEAR(std::abs(p.yaw), M_PI, 1e-6);
}

TEST(PoseHistory, RangeChecks) {
  PoseHistory h(16);
  Pose2D p;
  EXPECT_FALSE(h.at(1.0, 0.1, &p));
  h.add(1.0, {1.0, 0.0, 0.0});
  h.add(2.0, {2.0, 0.0, 0.0});
  EXPECT_FALSE(h.at(0.5, 0.1, &p));  // 이력보다 오래됨
  EXPECT_TRUE(h.at(2.05, 0.1, &p));  // 조금 늦으면 최신
  EXPECT_NEAR(p.x, 2.0, 1e-12);
  EXPECT_FALSE(h.at(2.5, 0.1, &p));
}

TEST(PoseHistory, RingOverflowKeepsNewest) {
  PoseHistory h(4);
  for (int i = 0; i < 10; ++i)
    h.add(i, {static_cast<double>(i), 0.0, 0.0});
  EXPECT_EQ(h.size(), 4u);
  Pose2D p;
  EXPECT_FALSE(h.at(5.5, 0.1, &p));
  ASSERT_TRUE(h.at(7.5, 0.1, &p));
  EXPECT_NEAR(p.x, 7.5, 1e-12);
  double t;
  ASSERT_TRUE(h.latest(&t, &p));
  EXPECT_DOUBLE_EQ(t, 9.0);
}

TEST(PoseHistory, TimeJumpBackClears) {
  PoseHistory h(8);
  h.add(10.0, {1.0, 0.0, 0.0});
  h.add(11.0, {2.0, 0.0, 0.0});
  h.add(1.0, {5.0, 0.0, 0.0});  // bag 재시작
  EXPECT_EQ(h.size(), 1u);
}
