// 등속 칼만 필터 시험
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include <Eigen/Eigenvalues>

#include "kebi_follow_tracker/kalman_cv.hpp"

namespace kt = kebi_follow::tracker;

TEST(KalmanCV, ConvergesToConstantVelocity) {
  kt::KalmanCV kf;
  kf.init({1.0, 0.0}, 0.06, 0.8);
  std::mt19937 rng(3);
  std::normal_distribution<double> noise(0.0, 0.05);
  const double vx = 0.8;
  const double vy = -0.3;
  const double dt = 0.1;
  for (int k = 1; k <= 40; ++k) {
    kf.predict(dt, 1.2);
    kf.update({1.0 + vx * k * dt + noise(rng), vy * k * dt + noise(rng)}, 0.06);
  }
  EXPECT_NEAR(kf.velocity().x, vx, 0.15);
  EXPECT_NEAR(kf.velocity().y, vy, 0.15);
  EXPECT_NEAR(kf.position().x, 1.0 + vx * 4.0, 0.08);
  EXPECT_LT(kf.positionStd(), 0.06);
}

TEST(KalmanCV, UncertaintyGrowsWithoutMeasurements) {
  kt::KalmanCV kf;
  kf.init({0.0, 0.0}, 0.06, 0.3);
  double prev = kf.positionStd();
  for (int k = 0; k < 20; ++k) {
    kf.predict(0.1, 1.2);
    const double s = kf.positionStd();
    EXPECT_GT(s, prev);
    prev = s;
  }
  EXPECT_GT(prev, 0.5);  // 2 s 가림이면 0.5 m 를 넘는다 (LOST 기준)
}

TEST(KalmanCV, MahalanobisGrowsWithDistance) {
  kt::KalmanCV kf;
  kf.init({1.0, 1.0}, 0.1, 0.5);
  double d_near = 0.0;
  double d_far = 0.0;
  double ld_near = 0.0;
  double ld_far = 0.0;
  kf.innovation({1.05, 1.0}, 0.1, d_near, ld_near);
  kf.innovation({1.4, 1.0}, 0.1, d_far, ld_far);
  EXPECT_LT(d_near, d_far);
  EXPECT_DOUBLE_EQ(ld_near, ld_far);
  // S = (0.1^2 + 0.1^2) I → d2 = 0.4^2 / 0.02 = 8
  EXPECT_NEAR(d_far, 8.0, 1e-9);
  // 측정 잡음이 크면 거리는 작고 ln|S| 는 크다
  double d_wide = 0.0;
  double ld_wide = 0.0;
  kf.innovation({1.4, 1.0}, 0.3, d_wide, ld_wide);
  EXPECT_LT(d_wide, d_far);
  EXPECT_GT(ld_wide, ld_far);
}

TEST(KalmanCV, CovarianceStaysSymmetricPositiveDefinite) {
  kt::KalmanCV kf;
  kf.init({0.0, 0.0}, 0.05, 1.0);
  for (int k = 0; k < 200; ++k) {
    kf.predict(0.1, 1.5);
    kf.update({0.01 * k, 0.0}, k % 3 == 0 ? 0.18 : 0.06);
  }
  const Eigen::Matrix4d& P = kf.covariance();
  EXPECT_LT((P - P.transpose()).cwiseAbs().maxCoeff(), 1e-12);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(P);
  EXPECT_GT(es.eigenvalues().minCoeff(), 0.0);
}

TEST(KalmanCV, ClampSpeedLimitsVelocity) {
  kt::KalmanCV kf;
  kf.init({0.0, 0.0}, 0.05, 1.0);
  kf.predict(0.1, 1.0);
  kf.update({1.0, 0.0}, 0.01);  // 0.1 s 에 1 m (10 m/s 로 보임)
  kf.clampSpeed(2.5);
  EXPECT_LE(std::hypot(kf.velocity().x, kf.velocity().y), 2.5 + 1e-9);
}

TEST(KalmanCV, NonPositiveDtIsIgnored) {
  kt::KalmanCV kf;
  kf.init({2.0, 3.0}, 0.05, 1.0);
  const Eigen::Matrix4d before = kf.covariance();
  kf.predict(0.0, 1.0);
  kf.predict(-0.5, 1.0);
  EXPECT_EQ((kf.covariance() - before).cwiseAbs().maxCoeff(), 0.0);
}
