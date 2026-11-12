// 진행 경로·회전 여유 기하 (S3·S4)
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "kebi_follow_controller/safety_gate.hpp"

using namespace kebi_follow;

namespace {
constexpr double kTol = 1e-9;

double dist(const SafetyGate& g, std::vector<Vec2> pts, double kappa = 0.0,
            bool straight = true, const Vec2* excl = nullptr,
            Pose2D to_now = Pose2D{}) {
  ObstacleSet s{pts.data(), pts.size(), to_now};
  Exclusion ex;
  if (excl != nullptr) {
    ex = Exclusion{excl, 1};
  }
  return g.corridorDistance(s, kappa, straight, ex);
}
}  // namespace

TEST(SafetyGate, StraightCorridorGeometry) {
  SafetyGate g;
  EXPECT_NEAR(g.pathDistance({1.0, 0.0}, 0.0), 1.0 - 0.22, kTol);
  // 몸체 폭 안: 둥근 앞면이 닿는 위치
  EXPECT_NEAR(g.pathDistance({0.5, 0.2}, 0.0),
              0.5 - std::sqrt(0.22 * 0.22 - 0.04), kTol);
  // 여유 띠 (0.22..0.32)
  EXPECT_NEAR(g.pathDistance({1.0, 0.30}, 0.0), 1.0, kTol);
  EXPECT_NEAR(g.pathDistance({1.0, -0.32}, 0.0), 1.0, kTol);
  // 통로 밖·뒤·너무 멀리
  EXPECT_FALSE(std::isfinite(g.pathDistance({1.0, 0.33}, 0.0)));
  EXPECT_FALSE(std::isfinite(g.pathDistance({-0.5, 0.0}, 0.0)));
  EXPECT_FALSE(std::isfinite(g.pathDistance({2.6, 0.0}, 0.0)));
  // 이미 닿은 점은 0
  EXPECT_DOUBLE_EQ(g.pathDistance({0.1, 0.0}, 0.0), 0.0);
}

TEST(SafetyGate, ArcCorridorLeftAndRight) {
  SafetyGate g;
  // 왼쪽 반경 1 m 원호 위 90° 지점
  EXPECT_NEAR(g.pathDistance({1.0, 1.0}, 1.0), M_PI / 2.0 - 0.22, kTol);
  EXPECT_NEAR(g.pathDistance({1.0, -1.0}, -1.0), M_PI / 2.0 - 0.22, kTol);
  // 원호에서 0.3 m 바깥(여유 띠)
  EXPECT_NEAR(g.pathDistance({1.3, 1.0}, 1.0), M_PI / 2.0, kTol);
  // 정면 1.5 m 는 원호 통로 밖
  EXPECT_FALSE(std::isfinite(g.pathDistance({1.5, 0.0}, 1.0)));
  // 직진 경로도 함께 보면 잡힌다
  EXPECT_NEAR(dist(g, {{1.5, 0.0}}, 1.0, true), 1.5 - 0.22, kTol);
  EXPECT_FALSE(std::isfinite(dist(g, {{1.5, 0.0}}, 1.0, false)));
  // 회전 안쪽 모서리 (원호 경로엔 있고 직진 경로엔 없다)
  const Vec2 corner{0.6, 0.45};
  EXPECT_FALSE(std::isfinite(g.pathDistance(corner, 0.0)));
  EXPECT_TRUE(std::isfinite(g.pathDistance(corner, 1.0)));
}

TEST(SafetyGate, TargetExclusion) {
  SafetyGate g;
  const Vec2 target{1.0, 0.0};
  EXPECT_FALSE(
      std::isfinite(dist(g, {{1.0, 0.1}, {1.2, -0.1}}, 0.0, true, &target)));
  EXPECT_NEAR(dist(g, {{0.55, 0.0}}, 0.0, true, &target), 0.55 - 0.22, kTol);
  EXPECT_NEAR(dist(g, {{1.0, 0.1}, {0.55, 0.0}}, 0.0, true, nullptr),
              0.55 - 0.22, kTol);
}

TEST(SafetyGate, MotionCompensation) {
  SafetyGate g;
  // 측정 뒤 로봇이 0.2 m 전진: 1.0 m 점은 지금 0.8 m
  const Pose2D moved = relativePose(Pose2D{0.2, 0.0, 0.0}, Pose2D{});
  EXPECT_NEAR(dist(g, {{1.0, 0.0}}, 0.0, true, nullptr, moved), 0.8 - 0.22,
              kTol);
  // 측정 뒤 왼쪽으로 90° 회전: 정면이던 점이 오른쪽으로
  const Pose2D turned = relativePose(Pose2D{0.0, 0.0, M_PI / 2.0}, Pose2D{});
  EXPECT_FALSE(
      std::isfinite(dist(g, {{1.0, 0.0}}, 0.0, true, nullptr, turned)));
}

TEST(SafetyGate, MinRadiusForRotation) {
  SafetyGate g;
  std::vector<Vec2> pts{{0.0, 0.29}, {1.0, 1.0}};
  ObstacleSet s{pts.data(), pts.size(), Pose2D{}};
  EXPECT_NEAR(g.minRadius(s), 0.29, kTol);
  ObstacleSet empty{nullptr, 0, Pose2D{}};
  EXPECT_FALSE(std::isfinite(g.minRadius(empty)));
}

TEST(SafetyGate, SpeedCapAndStop) {
  const double m = 0.25, L = 0.75, a = 0.8, vmax = 0.5;
  EXPECT_DOUBLE_EQ(obstacleSpeedCap(kInf, m, L, a, vmax), vmax);
  // 1.0 m 부터 감속
  EXPECT_DOUBLE_EQ(obstacleSpeedCap(1.0, m, L, a, vmax), vmax);
  EXPECT_NEAR(obstacleSpeedCap(0.625, m, L, a, vmax), 0.25, kTol);
  EXPECT_DOUBLE_EQ(obstacleSpeedCap(0.25, m, L, a, vmax), 0.0);
  EXPECT_DOUBLE_EQ(obstacleSpeedCap(0.1, m, L, a, vmax), 0.0);
  // 제동 곡선이 더 낮은 경우
  EXPECT_NEAR(obstacleSpeedCap(0.3, m, 0.0, a, vmax), std::sqrt(2 * a * 0.05),
              kTol);
  // 제동거리 + 0.25 이내 즉시 정지
  EXPECT_TRUE(obstacleStopRequired(0.4, m, 0.5, a));  // 0.156 + 0.25 = 0.406
  EXPECT_FALSE(obstacleStopRequired(0.41, m, 0.5, a));
  EXPECT_TRUE(obstacleStopRequired(0.25, m, 0.0, a));
  EXPECT_FALSE(obstacleStopRequired(kInf, m, 0.5, a));
}

TEST(SafetyGate, ReactionTime) {
  const double m = 0.25, a = 0.8, vmax = 2.0, tr = 0.25;
  // 제동 곡선: v·t_r + v²/(2a) = d - m. d = 1.0 이면 v = -0.2 + sqrt(0.04 + 1.2)
  EXPECT_NEAR(obstacleSpeedCap(1.0, m, 0.0, a, vmax, tr),
              -0.2 + std::sqrt(0.04 + 1.2), kTol);
  // t_r = 0 이면 기존 곡선과 같다
  EXPECT_NEAR(obstacleSpeedCap(1.0, m, 0.0, a, vmax, 0.0),
              std::sqrt(2 * a * 0.75), kTol);
  // 1.0 m/s: 0.25 + 1.0·0.25 + 1.0²/1.6 = 1.125 m 이내 즉시 정지
  EXPECT_TRUE(obstacleStopRequired(1.12, m, 1.0, a, tr));
  EXPECT_FALSE(obstacleStopRequired(1.13, m, 1.0, a, tr));
  // 상한 곡선과 정지 경계가 같은 식이다: 상한보다 조금 느리면 정지하지 않는다
  const double v = obstacleSpeedCap(1.5, m, 0.0, a, vmax, tr);
  EXPECT_FALSE(obstacleStopRequired(1.5, m, v - 1e-3, a, tr));
  EXPECT_TRUE(obstacleStopRequired(1.5, m, v + 1e-3, a, tr));
}

TEST(SafetyGate, PathCurvature) {
  EXPECT_DOUBLE_EQ(pathCurvature(0.01, 0.5, 0.05, 2.0), 0.0);
  EXPECT_NEAR(pathCurvature(0.5, 0.25, 0.05, 2.0), 0.5, kTol);
  EXPECT_DOUBLE_EQ(pathCurvature(0.1, 0.8, 0.05, 2.0), 2.0);
  EXPECT_DOUBLE_EQ(pathCurvature(0.1, -0.8, 0.05, 2.0), -2.0);
}

TEST(SafetyGate, PolylineExclusionCoversTargetPath) {
  SafetyGate g;
  // 타깃이 0.6 m 에서 1.4 m 로 걸어간 경로: 늦게 온 depth 에 남은 사람 몸도 제외
  const std::vector<Vec2> path{{0.6, 0.0}, {1.0, 0.05}, {1.4, 0.0}};
  const Exclusion ex{path.data(), path.size()};
  EXPECT_TRUE(g.excluded({0.75, 0.2}, ex));
  EXPECT_TRUE(g.excluded({1.2, -0.35}, ex));
  EXPECT_FALSE(g.excluded({1.0, 0.5}, ex));
  EXPECT_FALSE(g.excluded({0.15, 0.0}, ex));
  const Exclusion single{&path[2], 1};
  EXPECT_FALSE(g.excluded({0.75, 0.2}, single));
  std::vector<Vec2> pts{{0.75, 0.0}};
  ObstacleSet set{pts.data(), pts.size(), Pose2D{}};
  EXPECT_FALSE(std::isfinite(g.corridorDistance(set, 0.0, true, ex)));
  EXPECT_NEAR(g.corridorDistance(set, 0.0, true, single), 0.75 - 0.22, kTol);
}
