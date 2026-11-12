#include <gtest/gtest.h>

#include <algorithm>
#include <deque>
#include <random>
#include <utility>

#include "kebi_follow_detector/time_offset.hpp"

using kebi_follow::CameraTimeOffset;

namespace
{

constexpr double kClockOffset = 32396.478;  // AMR clock - Jetson clock (about +9 h)
constexpr double kFramePeriod = 0.1;
constexpr double kOdomPeriod = 0.01;

// latest odom stamp at the moment an image (camera stamp t_cam) arrives with transport delay `delay`
double odom_at_arrival(double t_cam, double delay)
{
  const double arrival_robot = t_cam + kClockOffset + delay;
  return std::floor(arrival_robot / kOdomPeriod) * kOdomPeriod;  // odom published every 10 ms
}

}  // namespace

TEST(CameraTimeOffset, InvalidUntilFirstSample)
{
  CameraTimeOffset est(5.0);
  EXPECT_FALSE(est.valid());
  EXPECT_DOUBLE_EQ(est.offset(), 0.0);
  est.add(100.5, 100.0);
  EXPECT_TRUE(est.valid());
  EXPECT_DOUBLE_EQ(est.offset(), 0.5);
  EXPECT_DOUBLE_EQ(est.to_robot_time(101.0, 0.05), 101.45);
}

TEST(CameraTimeOffset, ConvergesToOffsetPlusMinimumDelay)
{
  CameraTimeOffset est(5.0);
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> delay(0.03, 0.12);
  const double t0 = 1.8e9;  // realistic epoch magnitude (double precision check)
  for (int k = 0; k < 200; ++k) {
    const double t_cam = t0 + k * kFramePeriod;
    est.add(odom_at_arrival(t_cam, delay(rng)), t_cam);
  }
  // min over the window: offset + ~0.03 s delay - < 10 ms odom quantization
  EXPECT_NEAR(est.offset(), kClockOffset + 0.03, 0.012);
  const double t_cam = t0 + 200 * kFramePeriod;
  EXPECT_NEAR(est.to_robot_time(t_cam, 0.05), t_cam + kClockOffset - 0.02, 0.012);
}

TEST(CameraTimeOffset, OldMinimumLeavesTheWindow)
{
  CameraTimeOffset est(5.0);
  est.add(1000.0 + 0.01, 1000.0);  // unusually fast sample
  for (int k = 1; k <= 40; ++k) {  // 4 s later: still inside the window
    est.add(1000.0 + k * kFramePeriod + 0.08, 1000.0 + k * kFramePeriod);
  }
  EXPECT_NEAR(est.offset(), 0.01, 1e-9);
  for (int k = 41; k <= 70; ++k) {  // > 5 s after the fast sample
    est.add(1000.0 + k * kFramePeriod + 0.08, 1000.0 + k * kFramePeriod);
  }
  EXPECT_NEAR(est.offset(), 0.08, 1e-9);
}

TEST(CameraTimeOffset, ResetsWhenReferenceJumpsBack)
{
  CameraTimeOffset est(5.0, 1.0);
  for (int k = 0; k < 20; ++k) {
    est.add(5000.0 + k * kFramePeriod + 0.05, 2000.0 + k * kFramePeriod);  // offset 3000.05
  }
  EXPECT_NEAR(est.offset(), 3000.05, 1e-9);
  // bag restarted: reference time goes back by > 1 s, new offset 2999.0
  est.add(4000.0, 1001.0);
  EXPECT_EQ(est.size(), 1u);
  EXPECT_NEAR(est.offset(), 2999.0, 1e-9);
}

TEST(CameraTimeOffset, MatchesBruteForceSlidingMinimum)
{
  CameraTimeOffset est(2.0, 1e9);
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> jitter(0.0, 0.2);
  std::deque<std::pair<double, double>> all;  // (ref, value)
  double ref = 10.0;
  for (int k = 0; k < 500; ++k) {
    ref += 0.05 + jitter(rng) * 0.1;
    const double value = 1.0 + jitter(rng);
    est.add(ref, ref - value);
    all.emplace_back(ref, value);
    double mn = 1e9;
    for (const auto & s : all) {
      if (s.first >= ref - 2.0) {
        mn = std::min(mn, s.second);
      }
    }
    ASSERT_NEAR(est.offset(), mn, 1e-9) << "step " << k;
  }
}
