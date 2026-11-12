// 다중 추적 시험: 쌍 만들기, 보행, 넓은 보폭, 트랙 수명, 카메라 연관, 로봇 회전
#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <vector>

#include "kebi_follow_tracker/people_tracker.hpp"
#include "sim/harness.hpp"

namespace kt = kebi_follow::tracker;
namespace ks = kebi_follow::tracker::sim;
using kt::Vec2;

namespace {

int confirmedNear(const kt::PeopleTracker& tracker, const Vec2& p,
                  double radius, int* id = nullptr) {
  int n = 0;
  for (const auto& tr : tracker.tracks()) {
    if (tr.confirmed && kt::distance(tr.kf.position(), p) < radius) {
      ++n;
      if (id != nullptr) {
        *id = tr.id;
      }
    }
  }
  return n;
}

void addPair(ks::Harness& h, const Vec2& c) {
  h.world.circles.push_back({{c.x, c.y + 0.1}, 0.06});
  h.world.circles.push_back({{c.x, c.y - 0.1}, 0.06});
}

int confirmedCount(const kt::PeopleTracker& tracker) {
  int n = 0;
  for (const auto& tr : tracker.tracks()) {
    n += tr.confirmed ? 1 : 0;
  }
  return n;
}

}  // namespace

TEST(PeopleTracker, TwoStandingPeopleGiveTwoPairTracks) {
  ks::Harness h(ks::lidarOnlyParams());
  h.world.walkers.emplace_back(ks::standingPerson({1.2, 0.5}));
  h.world.walkers.emplace_back(ks::standingPerson({1.5, -0.6}));
  h.run(3);
  const auto& tr = h.core.tracker();
  EXPECT_EQ(confirmedCount(tr), 2);
  for (const auto& t : tr.tracks()) {
    EXPECT_EQ(t.kind, kt::CandidateKind::PAIR);
    EXPECT_GT(t.shape_score, 0.9);
  }
  EXPECT_EQ(confirmedNear(tr, {1.2, 0.5}, 0.1), 1);
  EXPECT_EQ(confirmedNear(tr, {1.5, -0.6}, 0.1), 1);
}

TEST(PeopleTracker, WalkingPersonKeepsOneTrackAndVelocity) {
  ks::Harness h(ks::lidarOnlyParams());
  ks::WalkerSpec w;
  w.start = {1.0, -1.5};
  w.heading = kt::kPi / 2.0;
  w.path = {{{1.0, 2.0}, 1.2, 0.0}};
  h.world.walkers.emplace_back(w);
  std::set<int> ids;
  for (int k = 0; k < 30; ++k) {
    h.scan();
    if (k >= 5) {
      int id = -1;
      ASSERT_EQ(confirmedNear(h.core.tracker(), h.world.walkers[0].center(),
                              0.3, &id),
                1)
          << "scan " << k;
      ids.insert(id);
    }
  }
  EXPECT_EQ(ids.size(), 1u);
  const kt::PersonTrack* tr = h.core.tracker().findTrack(*ids.begin());
  ASSERT_NE(tr, nullptr);
  EXPECT_NEAR(tr->kf.velocity().y, h.world.walkers[0].velocity().y, 0.3);
  EXPECT_LT(kt::distance(tr->kf.position(), h.world.walkers[0].center()), 0.15);
}

TEST(PeopleTracker, WideStrideBeyondPairSeparationStaysOneTrack) {
  // 기존 트랙은 pair_max_sep(0.6) 보다 넓은 두 다리(0.75 m)도 한 사람으로 붙인다
  ks::Harness h(ks::lidarOnlyParams());
  h.world.circles.push_back({{1.5, 0.1}, 0.06});
  h.world.circles.push_back({{1.5, -0.1}, 0.06});
  h.run(5);
  ASSERT_EQ(confirmedCount(h.core.tracker()), 1);
  const int id = h.core.tracker().tracks().front().id;
  h.world.circles.clear();
  h.world.circles.push_back({{1.88, 0.1}, 0.06});
  h.world.circles.push_back({{1.13, -0.1}, 0.06});
  h.run(3);
  EXPECT_EQ(confirmedCount(h.core.tracker()), 1);
  EXPECT_EQ(h.core.tracker().tracks().size(),
            1u);  // 다리 하나가 새 트랙이 되지 않는다
  EXPECT_EQ(h.core.tracker().tracks().front().id, id);
  EXPECT_EQ(h.core.tracker().tracks().front().kind, kt::CandidateKind::PAIR);
}

TEST(PeopleTracker, TentativeTrackDiesAndConfirmedTrackExpires) {
  kt::TrackerParams p = ks::lidarOnlyParams();
  ks::Harness h(p);
  h.world.circles.push_back({{2.0, 0.0}, 0.06});
  h.scan();
  ASSERT_EQ(h.core.tracker().tracks().size(), 1u);
  EXPECT_FALSE(h.core.tracker().tracks().front().confirmed);
  h.world.circles.clear();
  h.run(p.track.tentative_max_misses + 1);
  EXPECT_TRUE(h.core.tracker().tracks().empty());

  addPair(h, {2.0, 0.0});
  h.run(30);  // 속도 추정이 안정될 때까지
  ASSERT_EQ(confirmedCount(h.core.tracker()), 1);
  h.world.circles.clear();
  h.run(10);
  EXPECT_EQ(h.core.tracker().tracks().size(), 1u);  // 1 s 가림: 아직 예측 중
  h.run(static_cast<int>(p.track.delete_s / ks::kScanPeriod));
  EXPECT_TRUE(h.core.tracker().tracks().empty());
}

TEST(PeopleTracker, CameraAssociationConfirmsNearestTrackOnly) {
  ks::Harness h(ks::lidarOnlyParams());
  h.world.circles.push_back(
      {{1.2, 0.0}, 0.06});  // 다리 하나만 보이는 사람 (형상 점수 낮음)
  h.world.walkers.emplace_back(ks::standingPerson({1.2, 1.0}));
  h.run(4);
  ASSERT_EQ(confirmedCount(h.core.tracker()), 2);
  const kt::CameraParams& cp = h.core.params().camera;
  std::vector<kt::CameraDetection> det(1);
  det[0].position = {1.2, 0.03};
  det[0].score = 0.9;
  EXPECT_EQ(h.core.processCamera(h.t - 0.15, h.robot, det),
            1);  // 0.15 s 전 검출
  for (const auto& tr : h.core.tracker().tracks()) {
    const bool near0 = kt::distance(tr.kf.position(), {1.2, 0.0}) < 0.2;
    EXPECT_EQ(tr.cameraConfirmed(h.t, cp.confirm_window_s), near0);
    if (near0) {
      EXPECT_LT(tr.shape_score, 0.5);
      EXPECT_GT(tr.personScore(h.t, cp), 0.6);
    }
  }
  // 게이트(0.4 m) 밖 검출은 연관하지 않는다
  std::vector<kt::CameraDetection> far(1);
  far[0].position = {1.2, 0.5};
  far[0].score = 0.9;
  EXPECT_EQ(h.core.processCamera(h.t, h.robot, far), 0);
}

TEST(PeopleTracker, StaticPersonStaysPutInOdomWhileRobotRotates) {
  ks::Harness h(ks::lidarOnlyParams());
  h.world.walkers.emplace_back(ks::standingPerson({1.3, 0.2}));
  h.run(5);
  int id = -1;
  ASSERT_EQ(confirmedNear(h.core.tracker(), {1.3, 0.2}, 0.1, &id), 1);
  h.yaw_rate = 0.8;  // 제자리 회전 (스캔은 scan 시각 자세로 변환)
  h.run(20);
  const kt::PersonTrack* tr = h.core.tracker().findTrack(id);
  ASSERT_NE(tr, nullptr);
  EXPECT_LT(kt::distance(tr->kf.position(), {1.3, 0.2}), 0.08);
  EXPECT_LT(std::hypot(tr->kf.velocity().x, tr->kf.velocity().y), 0.15);
}

TEST(PeopleTracker, PositionAtInterpolatesHistory) {
  ks::Harness h(ks::lidarOnlyParams());
  ks::WalkerSpec w;
  w.start = {1.5, -1.0};
  w.heading = kt::kPi / 2.0;
  w.path = {{{1.5, 2.0}, 1.0, 0.0}};
  h.world.walkers.emplace_back(w);
  h.run(15);
  int id = -1;
  ASSERT_EQ(
      confirmedNear(h.core.tracker(), h.world.walkers[0].center(), 0.3, &id),
      1);
  const kt::PersonTrack* tr = h.core.tracker().findTrack(id);
  const Vec2 now = tr->positionAt(h.t);
  const Vec2 before = tr->positionAt(h.t - 0.3);
  EXPECT_NEAR(now.y - before.y, 0.3, 0.12);  // 1 m/s 로 0.3 s
}

TEST(PeopleTracker, TimeJumpBackwardResets) {
  ks::Harness h(ks::lidarOnlyParams());
  h.world.walkers.emplace_back(ks::standingPerson({1.2, 0.0}));
  h.run(4);
  ASSERT_EQ(confirmedCount(h.core.tracker()), 1);
  h.t -= 10.0;  // bag 반복 재생
  h.scan();
  EXPECT_EQ(confirmedCount(h.core.tracker()), 0);
  EXPECT_EQ(h.core.tracker().tracks().size(), 1u);
}

TEST(PeopleTracker, StaticBlobIsNotLockableButWalkingMergedPersonIs) {
  ks::Harness h(ks::lidarOnlyParams());
  ks::addBox(h.world, {1.6, -0.8}, 0.0, 0.3, 0.3);  // 상자 (덩어리)
  ks::WalkerSpec skirt;
  skirt.start = {1.2, 1.5};
  skirt.heading = -kt::kPi / 2.0;
  skirt.skirt_radius = 0.22;
  skirt.path = {{{1.2, 0.6}, 0.8, 10.0}};
  h.world.walkers.emplace_back(skirt);
  h.run(40);
  const kt::CameraParams& cp = h.core.params().camera;
  int box_tracks = 0;
  int skirt_tracks = 0;
  for (const auto& tr : h.core.tracker().tracks()) {
    if (!tr.confirmed) {
      continue;
    }
    if (kt::distance(tr.kf.position(), {1.45, -0.8}) < 0.3) {
      ++box_tracks;
      EXPECT_FALSE(tr.moved);
      EXPECT_LT(tr.lockShape(h.t, h.core.params()), 0.5);  // 락 후보가 아니다
      EXPECT_GE(tr.personScore(h.t, cp),
                0.5);  // 경쟁자·모호 판정에는 보수적으로 센다
    }
    if (kt::distance(tr.kf.position(), {1.2, 0.6}) < 0.3) {
      ++skirt_tracks;
      EXPECT_TRUE(tr.moved);
      EXPECT_EQ(tr.kind, kt::CandidateKind::MERGED);
      EXPECT_GE(tr.lockShape(h.t, h.core.params()), 0.5);
    }
  }
  EXPECT_EQ(box_tracks, 1);
  EXPECT_EQ(skirt_tracks, 1);
}

TEST(PeopleTracker, DuplicateTracksOnOnePersonAreMerged) {
  // 한 사람에 트랙이 둘 생기면(다리를 하나씩 가져감) merge_scans 뒤 하나만 남는다
  ks::Harness h(ks::lidarOnlyParams());
  h.world.walkers.emplace_back(ks::standingPerson({1.5, 0.0}));
  h.run(5);
  ASSERT_EQ(confirmedCount(h.core.tracker()), 1);
  // 다리 사이가 크게 벌어진 순간을 흉내: 두 다리를 0.7 m 로 벌려 각 다리가 따로 트랙이 될 수 있게 한다
  h.world.walkers.clear();
  addPair(h, {1.5, 0.0});
  h.world.circles[0].c.y = 0.35;
  h.world.circles[1].c.y = -0.35;
  h.run(10);
  h.world.circles[0].c.y = 0.1;
  h.world.circles[1].c.y = -0.1;
  h.run(h.core.params().track.merge_scans + 3);
  EXPECT_EQ(confirmedNear(h.core.tracker(), {1.5, 0.0}, 0.4), 1);
}

TEST(PeopleTracker, StationaryTrackKeepsSmallGateAndStaysPut) {
  // 오래 서 있던 물체(정지 트랙)는 옆을 지나는 사람에게 끌려가지 않는다
  ks::Harness h(ks::lidarOnlyParams());
  h.world.circles.push_back(
      {{1.2, 0.6}, 0.04});  // 지름 8 cm 기둥 (다리처럼 보임)
  h.run(40);
  int id = -1;
  ASSERT_EQ(confirmedNear(h.core.tracker(), {1.2, 0.6}, 0.1, &id), 1);
  ASSERT_TRUE(h.core.tracker().findTrack(id)->stationary);
  ks::WalkerSpec w;
  w.start = {1.4, 2.0};
  w.heading = -kt::kPi / 2.0;
  w.path = {{{1.4, -1.5}, 1.0, 0.0}};  // 물체 옆 0.2 m 를 지나간다
  h.world.walkers.emplace_back(w);
  h.run(40);
  const kt::PersonTrack* tr = h.core.tracker().findTrack(id);
  ASSERT_NE(tr, nullptr);
  EXPECT_LT(kt::distance(tr->kf.position(), {1.2, 0.6}), 0.05);
  EXPECT_TRUE(tr->stationary);
}
