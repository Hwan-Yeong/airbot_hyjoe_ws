// 분할·다리 판정 시험
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "kebi_follow_tracker/leg_detector.hpp"
#include "sim/scan_sim.hpp"

namespace kt = kebi_follow::tracker;
using kt::Vec2;

namespace {

struct DetectorFixture {
  DetectorFixture() : detector(params.scan, params.leg), rng(7) {
    lidar.edge_mixed_prob = 0.0;
    lidar.dropout_prob = 0.0;
    lidar.noise_std = 0.0;
    lidar.noise_per_m = 0.0;
  }

  const std::vector<kt::Segment>& detect() {
    kt::sim::simulateScan(world, kt::Pose2{}, lidar, rng, ranges);
    kt::ScanView view;
    view.ranges = ranges.data();
    view.count = ranges.size();
    view.angle_min = lidar.angle_min;
    view.angle_increment = lidar.angle_increment;
    view.range_min = lidar.range_min;
    view.range_max = lidar.range_max;
    detector.detect(view, lidar.mount);
    return detector.segments();
  }

  // c 에서 가장 가까운 분할
  const kt::Segment* nearest(const Vec2& c, double max_dist = 0.3) {
    const kt::Segment* best = nullptr;
    double best_d = max_dist;
    for (const auto& s : detector.segments()) {
      const double d = kt::distance(s.center, c);
      if (d < best_d) {
        best_d = d;
        best = &s;
      }
    }
    return best;
  }

  kt::TrackerParams params;
  kt::LegDetector detector;
  kt::sim::World world;
  kt::sim::LidarModel lidar;
  std::mt19937 rng;
  std::vector<float> ranges;
};

}  // namespace

TEST(LegDetector, SingleLegAtOneMeterIsLegWithCorrectCenter) {
  DetectorFixture f;
  const Vec2 c{1.0, 0.1};
  f.world.circles.push_back({c, 0.06});
  const auto& segs = f.detect();
  ASSERT_EQ(segs.size(), 1u);
  EXPECT_TRUE(segs[0].is_leg);
  EXPECT_FALSE(segs[0].is_blob);
  EXPECT_GE(segs[0].count, 6);
  EXPECT_NEAR(segs[0].width, 0.12, 0.03);
  EXPECT_LT(kt::distance(segs[0].center, c), 0.025);
  EXPECT_DOUBLE_EQ(segs[0].leg_score, 1.0);
}

TEST(LegDetector, LegsAtLongRangeStillDetectedWithFewPoints) {
  for (double r : {2.0, 3.0}) {
    DetectorFixture f;
    f.world.circles.push_back({{r, 0.0}, 0.06});
    f.detect();
    const kt::Segment* s = f.nearest({r, 0.0});
    ASSERT_NE(s, nullptr) << "range " << r;
    EXPECT_TRUE(s->is_leg) << "range " << r << " width " << s->width << " n "
                           << s->count;
    EXPECT_LE(s->count, 5);
  }
}

TEST(LegDetector, ThinChairLegNearbyIsNotLeg) {
  DetectorFixture f;
  f.world.circles.push_back({{1.0, 0.0}, 0.015});
  f.detect();
  const kt::Segment* s = f.nearest({1.0, 0.0});
  ASSERT_NE(s, nullptr);
  EXPECT_FALSE(s->is_leg) << "width " << s->width;
  EXPECT_FALSE(s->is_blob);
}

TEST(LegDetector, WallIsNotLegOrBlob) {
  DetectorFixture f;
  f.world.walls.push_back({{2.0, -1.5}, {2.0, 1.5}});
  for (const auto& s : f.detect()) {
    EXPECT_FALSE(s.is_leg);
    EXPECT_FALSE(s.is_blob);
  }
}

TEST(LegDetector, FlatBoxFaceIsOnlyBlobCandidate) {
  // 붙은 다리와 상자 면은 모양으로 구별되지 않아 덩어리 후보가 된다 (정적 덩어리는 추적에서 낮춘다)
  DetectorFixture f;
  kt::sim::addBox(f.world, {-1.2, 0.0}, 0.0, 0.3, 0.3);
  f.detect();
  const kt::Segment* s = f.nearest({-1.05, 0.0});
  ASSERT_NE(s, nullptr);
  EXPECT_FALSE(s->is_leg);
  EXPECT_TRUE(s->is_blob);
  EXPECT_NEAR(s->bulge, 0.0, 0.005);
}

TEST(LegDetector, TwoTouchingLegsFormBlob) {
  DetectorFixture f;
  f.world.circles.push_back({{1.0, 0.06}, 0.065});
  f.world.circles.push_back({{1.0, -0.06}, 0.065});
  f.detect();
  const kt::Segment* s = f.nearest({1.0, 0.0});
  ASSERT_NE(s, nullptr);
  EXPECT_TRUE(s->is_blob) << "width " << s->width << " bulge " << s->bulge;
}

TEST(LegDetector, LegPairIsTwoLegsAndWallBetweenIsNot) {
  // 두 다리 사이로 보이는 벽 조각(양쪽이 다리에 가림)은 다리가 아니다
  DetectorFixture f;
  f.world.circles.push_back({{1.2, 0.1}, 0.06});
  f.world.circles.push_back({{1.2, -0.1}, 0.06});
  f.world.walls.push_back({{3.0, -2.0}, {3.0, 2.0}});
  f.detect();
  int legs = 0;
  for (const auto& s : f.detector.segments()) {
    legs += s.is_leg ? 1 : 0;
  }
  EXPECT_EQ(legs, 2);
}

TEST(LegDetector, LegInFrontOfWallIsSplitByRangeJump) {
  DetectorFixture f;
  f.world.circles.push_back({{1.5, 0.0}, 0.06});
  f.world.walls.push_back({{1.85, -1.0}, {1.85, 1.0}});
  f.detect();
  const kt::Segment* s = f.nearest({1.5, 0.0});
  ASSERT_NE(s, nullptr);
  EXPECT_TRUE(s->is_leg);
  EXPECT_LT(s->chord, 0.15);
}

TEST(LegDetector, RobotBodyReturnsAreIgnored) {
  DetectorFixture f;
  f.world.circles.push_back({{0.20, 0.0}, 0.03});  // 몸체 안 (반경 0.22 + 여유)
  EXPECT_TRUE(f.detect().empty());
}

TEST(LegDetector, OneDropoutInsideLegDoesNotSplit) {
  DetectorFixture f;
  f.world.circles.push_back({{1.0, 0.0}, 0.07});
  kt::sim::simulateScan(f.world, kt::Pose2{}, f.lidar, f.rng, f.ranges);
  // 다리 가운데 빔 하나를 무반사로
  const int mid = static_cast<int>(
      std::round((0.0 - f.lidar.angle_min) / f.lidar.angle_increment));
  f.ranges[mid] = std::numeric_limits<float>::infinity();
  kt::ScanView view;
  view.ranges = f.ranges.data();
  view.count = f.ranges.size();
  view.angle_min = f.lidar.angle_min;
  view.angle_increment = f.lidar.angle_increment;
  view.range_min = f.lidar.range_min;
  view.range_max = f.lidar.range_max;
  f.detector.detect(view, f.lidar.mount);
  ASSERT_EQ(f.detector.segments().size(), 1u);
  EXPECT_TRUE(f.detector.segments()[0].is_leg);
}

TEST(LegDetector, SegmentAcrossScanSeamIsMerged) {
  DetectorFixture f;
  // 바로 뒤(±180°) 다리: 첫 빔과 끝 빔에 걸친다
  f.world.circles.push_back({{-1.0, 0.0}, 0.06});
  f.detect();
  int near = 0;
  for (const auto& s : f.detector.segments()) {
    if (kt::distance(s.center, {-1.0, 0.0}) < 0.3) {
      ++near;
      EXPECT_TRUE(s.is_leg);
    }
  }
  EXPECT_EQ(near, 1);
}

TEST(LegDetector, ConcaveCornerIsRejected) {
  DetectorFixture f;
  // 센서 쪽으로 열린 V (안쪽 모서리)
  f.world.walls.push_back({{1.0, 0.1}, {1.1, 0.0}});
  f.world.walls.push_back({{1.1, 0.0}, {1.0, -0.1}});
  f.detect();
  const kt::Segment* s = f.nearest({1.05, 0.0});
  ASSERT_NE(s, nullptr);
  EXPECT_LT(s->bulge, 0.0);
  EXPECT_FALSE(s->is_leg);
  EXPECT_FALSE(s->is_blob);
}

TEST(LegDetector, SensorMountOffsetIsApplied) {
  DetectorFixture f;
  f.lidar.mount = {0.1, 0.0, kt::kPi};  // 뒤를 보는 센서를 앞으로 0.1 m
  const Vec2 c{1.0, 0.0};               // base_link
  f.world.circles.push_back({c, 0.06});
  // simulateScan 은 base_link 원점 로봇 자세에 mount 를 합성한다
  f.detect();
  const kt::Segment* s = f.nearest(c);
  ASSERT_NE(s, nullptr);
  EXPECT_TRUE(s->is_leg);
  EXPECT_LT(kt::distance(s->center, c), 0.03);
}

TEST(LegDetector, WallFragmentAtPersonShadowEdgeIsNotLeg) {
  // 사람 뒤 벽이 사람 그림자 경계와 벽 끝 사이에서 짧게 잘리면 다리처럼 보인다 → 버린다
  DetectorFixture f;
  f.world.circles.push_back({{1.2, 0.0}, 0.07});  // 사람 다리
  f.world.walls.push_back(
      {{2.6, -0.02}, {2.6, 0.14}});  // 짧은 벽 조각 (다리 그림자 옆)
  f.detect();
  for (const auto& s : f.detector.segments()) {
    if (s.center.x > 2.0) {
      EXPECT_FALSE(s.is_leg)
          << "width " << s.width << " gap " << s.occluder_gap_first << ","
          << s.occluder_gap_last;
    }
  }
}
