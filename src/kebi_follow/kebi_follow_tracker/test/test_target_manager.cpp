// 타깃 상태기계 시험: 획득(후보 1명/2명, 카메라 조건, 영역), 가림 → OCCLUDED → LOST → 재획득,
// 모호, mode 전환
#include <gtest/gtest.h>

#include <vector>

#include "sim/harness.hpp"

namespace kt = kebi_follow::tracker;
namespace ks = kebi_follow::tracker::sim;
using kt::Mode;
using kt::TargetStatus;
using kt::Vec2;

namespace {

// 상태기계 시험: 정적 물체 감점을 끈다 (감점 자체는 StaticObjectIsNotLockedWithoutCamera 에서 본다)
kt::TrackerParams stateParams() {
  kt::TrackerParams p = ks::lidarOnlyParams();
  p.track.static_shape_factor = 1.0;
  return p;
}

void addPerson(ks::Harness& h, const Vec2& c, double half_sep = 0.1) {
  h.world.circles.push_back({{c.x, c.y + half_sep}, 0.06});
  h.world.circles.push_back({{c.x, c.y - half_sep}, 0.06});
}

// 추적을 먼저 안정시킨 뒤 ACQUIRE 를 켜고 락까지 scan 수를 센다 (-1: 락 없음)
int acquireScans(ks::Harness& h, int max_scans, bool with_camera = false) {
  h.run(5, with_camera);
  h.core.setMode(Mode::ACQUIRE);
  for (int k = 0; k < max_scans; ++k) {
    h.run(1, with_camera);
    if (h.core.target().status == TargetStatus::LOCKED) {
      return k;
    }
  }
  return -1;
}

}  // namespace

TEST(TargetManager, SingleCandidateLocksAfterConfirmTime) {
  ks::Harness h(stateParams());
  addPerson(h, {1.0, 0.0});
  h.run(5);
  h.core.setMode(Mode::ACQUIRE);
  for (int k = 0; k < 10; ++k) {
    const auto& out = h.run(1);
    EXPECT_EQ(out.status, TargetStatus::CANDIDATE) << "scan " << k;
    EXPECT_FALSE(out.ambiguous);
  }
  const auto& out = h.run(1);  // 11 번째 scan = 첫 후보 뒤 1.0 s
  EXPECT_EQ(out.status, TargetStatus::LOCKED);
  EXPECT_GE(out.track_id, 0);
  EXPECT_EQ(h.core.lockedId(), out.track_id);
  EXPECT_LT(kt::distance(out.position, {1.0, 0.0}), 0.08);
}

TEST(TargetManager, TwoCandidatesAreAmbiguousUntilOneLeaves) {
  ks::Harness h(stateParams());
  addPerson(h, {1.0, 0.2});
  addPerson(h, {1.2, -0.3});
  h.run(5);
  h.core.setMode(Mode::ACQUIRE);
  for (int k = 0; k < 30; ++k) {
    const auto& out = h.run(1);
    EXPECT_EQ(out.status, TargetStatus::CANDIDATE);
    EXPECT_TRUE(out.ambiguous);
    EXPECT_EQ(out.zone_candidates, 2);
  }
  h.world.circles.erase(h.world.circles.begin() + 2,
                        h.world.circles.end());  // 두 번째 사람이 떠남
  const int scans = [&]() {
    for (int k = 0; k < 60; ++k) {
      if (h.run(1).status == TargetStatus::LOCKED) {
        return k;
      }
    }
    return -1;
  }();
  ASSERT_GE(scans, 0);
  EXPECT_LT(kt::distance(h.core.target().position, {1.0, 0.2}), 0.08);
}

TEST(TargetManager, CompetitorJustOutsideZoneBlocksLock) {
  // 획득 영역(±20°) 밖이지만 경쟁 영역(+10°) 안의 사람도 "후보 1명" 조건을 깬다
  ks::Harness h(stateParams());
  addPerson(h, {1.0, 0.0});
  ks::WalkerSpec
      other;  // 걸어 들어와 약 24° 에 선 사람 (움직인 이력이 있어 정지 트랙이 아니다)
  other.start = {1.8, 1.2};
  other.heading = -2.4;
  other.path = {{{1.0, 0.45}, 0.8, 100.0}};
  h.world.walkers.emplace_back(other);
  EXPECT_EQ(acquireScans(h, 30), -1);
  EXPECT_TRUE(h.core.target().ambiguous);
}

TEST(TargetManager, RequireCameraBlocksLockUntilCameraConfirms) {
  kt::TrackerParams p;  // require_camera = true (기본)
  ks::Harness h(p);
  h.world.walkers.emplace_back(ks::standingPerson({1.1, 0.0}));
  EXPECT_EQ(acquireScans(h, 25, false), -1);
  EXPECT_EQ(h.core.target().status, TargetStatus::NONE);
  h.core.setMode(Mode::OFF);
  const int scans = acquireScans(h, 25, true);
  EXPECT_EQ(scans, 10);
  EXPECT_TRUE(h.core.target().camera_confirmed);
}

TEST(TargetManager, CandidatesOutsideZoneAreIgnored) {
  for (const Vec2 c : {Vec2{2.0, 0.0}, Vec2{0.9, 0.55}, Vec2{0.35, 0.0}}) {
    ks::Harness h(stateParams());
    addPerson(h, c);
    EXPECT_EQ(acquireScans(h, 20), -1) << c.x << "," << c.y;
    EXPECT_EQ(h.core.target().status, TargetStatus::NONE);
  }
}

TEST(TargetManager, OcclusionThenLostThenReacquire) {
  kt::TrackerParams p = stateParams();
  ks::Harness h(p);
  addPerson(h, {1.2, 0.0});
  ASSERT_GE(acquireScans(h, 15), 0);
  h.core.setMode(Mode::TRACK);
  const int first_id = h.core.target().track_id;
  h.run(3);
  EXPECT_EQ(h.core.target().status, TargetStatus::LOCKED);

  h.world.circles.clear();  // 가림
  const auto& occ = h.run(1);
  EXPECT_EQ(occ.status, TargetStatus::OCCLUDED);
  EXPECT_EQ(occ.track_id, first_id);
  EXPECT_NEAR(occ.occluded_time, 0.1, 1e-6);
  EXPECT_LT(kt::distance(occ.position, {1.2, 0.0}), 0.1);

  int lost_at = -1;
  for (int k = 0; k < 30; ++k) {
    const auto& out = h.run(1);
    if (out.status == TargetStatus::LOST) {
      lost_at = k;
      break;
    }
    EXPECT_EQ(out.status, TargetStatus::OCCLUDED);
    EXPECT_LE(out.position_std, p.target.max_std);
  }
  ASSERT_GE(lost_at, 0);
  EXPECT_LE((lost_at + 2) * ks::kScanPeriod, p.target.lost_s + 0.1);
  EXPECT_TRUE(h.core.target().has_lost_position);
  h.run(10);
  EXPECT_EQ(h.core.target().status, TargetStatus::LOST);
  EXPECT_EQ(h.core.lockedId(), -1);

  // 마지막 예측 위치에서 0.6 m 떨어진 곳에 다시 나타남 → 확정 0.3 s + 유지 0.5 s 뒤 LOCKED
  addPerson(h, {1.2, 0.6});
  int relock = -1;
  for (int k = 0; k < 20; ++k) {
    if (h.run(1).status == TargetStatus::LOCKED) {
      relock = k;
      break;
    }
    EXPECT_EQ(h.core.target().status, TargetStatus::LOST);
  }
  EXPECT_GE(relock, 6);
  EXPECT_LE(relock, 10);
  EXPECT_LT(kt::distance(h.core.target().position, {1.2, 0.6}), 0.08);
}

TEST(TargetManager, NoReacquireOutsideRadiusOrWithTwoPeople) {
  ks::Harness h(stateParams());
  addPerson(h, {1.2, 0.0});
  ASSERT_GE(acquireScans(h, 15), 0);
  h.core.setMode(Mode::TRACK);
  h.world.circles.clear();
  h.run(25);
  ASSERT_EQ(h.core.target().status, TargetStatus::LOST);
  addPerson(h, {2.4, 0.0});  // 1.2 m 밖
  h.run(20);
  EXPECT_EQ(h.core.target().status, TargetStatus::LOST);
  h.world.circles.clear();
  addPerson(h, {1.2, 0.35});
  addPerson(h, {1.2, -0.35});  // 반경 안에 두 명
  h.run(20);
  EXPECT_EQ(h.core.target().status, TargetStatus::LOST);
  EXPECT_TRUE(h.core.target().ambiguous);
}

TEST(TargetManager, AmbiguousWhenAnotherPersonComesClose) {
  ks::Harness h(stateParams());
  h.world.walkers.emplace_back(ks::standingPerson({1.2, 0.0}));
  ASSERT_GE(acquireScans(h, 15), 0);
  h.core.setMode(Mode::TRACK);
  EXPECT_FALSE(h.run(3).ambiguous);
  ks::WalkerSpec other;
  other.start = {1.3, 2.0};
  other.heading = -kt::kPi / 2.0;
  other.path = {{{1.3, 0.45}, 0.8, 5.0}};  // 타깃 옆 0.45 m 에 와서 선다
  h.world.walkers.emplace_back(other);
  bool seen = false;
  for (int k = 0; k < 40; ++k) {
    const auto& out = h.run(1);
    seen = seen || out.ambiguous;
    EXPECT_NE(out.status, TargetStatus::LOST);
  }
  EXPECT_TRUE(seen);
  EXPECT_TRUE(h.core.target().ambiguous);  // 서 있는 동안 계속
  EXPECT_LT(kt::distance(h.core.target().position, {1.2, 0.0}),
            0.1);  // 타깃은 그대로
}

TEST(TargetManager, ModeOffReleasesAndAcquireRestarts) {
  ks::Harness h(stateParams());
  addPerson(h, {1.0, 0.0});
  ASSERT_GE(acquireScans(h, 15), 0);
  h.core.setMode(Mode::TRACK);
  h.run(2);
  ASSERT_EQ(h.core.target().status, TargetStatus::LOCKED);
  h.core.setMode(Mode::OFF);
  EXPECT_EQ(h.run(1).status, TargetStatus::NONE);
  EXPECT_EQ(h.core.lockedId(), -1);
  EXPECT_EQ(h.core.target().track_id, -1);
  // TRACK 에서 ACQUIRE 로 오면 락을 버리고 다시 획득한다 (1 s 유지 필요)
  h.core.setMode(Mode::ACQUIRE);
  ASSERT_GE(
      [&]() {
        for (int k = 0; k < 15; ++k) {
          if (h.run(1).status == TargetStatus::LOCKED) {
            return k;
          }
        }
        return -1;
      }(),
      9);
  h.core.setMode(Mode::TRACK);
  h.run(1);
  h.core.setMode(Mode::ACQUIRE);
  EXPECT_EQ(h.run(1).status, TargetStatus::CANDIDATE);
}

TEST(TargetManager, TrackModeWithoutLockGivesNone) {
  ks::Harness h(stateParams());
  addPerson(h, {1.0, 0.0});
  h.run(5);
  h.core.setMode(Mode::TRACK);
  for (int k = 0; k < 20; ++k) {
    EXPECT_EQ(h.run(1).status, TargetStatus::NONE);
  }
}

TEST(TargetManager, LostInAcquireModeRestartsAcquisition) {
  ks::Harness h(stateParams());
  addPerson(h, {1.0, 0.0});
  ASSERT_GE(acquireScans(h, 15),
            0);  // ACQUIRE 에서 락, controller 가 아직 TRACK 을 안 보냄
  h.world.circles.clear();
  h.run(25);
  EXPECT_EQ(h.core.target().status, TargetStatus::NONE);
  EXPECT_FALSE(h.core.target().has_lost_position);
}

TEST(TargetManager, StaticObjectIsNotLockedWithoutCamera) {
  // 기본값: 움직인 적도 카메라 확인도 없는 다리 쌍 모양(기둥+의자 다리 등)은 락하지 않는다
  ks::Harness h(ks::lidarOnlyParams());
  addPerson(h, {1.0, 0.0});
  EXPECT_EQ(acquireScans(h, 30), -1);
  EXPECT_EQ(h.core.target().status, TargetStatus::NONE);
}

TEST(TargetManager, PersonWhoWalkedInIsLockedWithoutCamera) {
  ks::Harness h(ks::lidarOnlyParams());
  ks::WalkerSpec w;
  w.start = {2.0, 0.8};
  w.heading = -2.5;
  w.path = {{{1.0, 0.0}, 0.8, 100.0}};
  h.world.walkers.emplace_back(w);
  h.run(25);  // 걸어 들어와 선다
  h.core.setMode(Mode::ACQUIRE);
  int scans = -1;
  for (int k = 0; k < 15 && scans < 0; ++k) {
    if (h.run(1).status == TargetStatus::LOCKED) {
      scans = k;
    }
  }
  EXPECT_EQ(scans, 10);
}

TEST(TargetManager, StaticObjectIsLockableWhenCameraConfirms) {
  kt::TrackerParams p;  // require_camera = true
  ks::Harness h(p);
  h.world.walkers.emplace_back(ks::standingPerson({1.1, 0.0}));
  EXPECT_EQ(acquireScans(h, 25, true), 10);
}

TEST(TargetManager, StaticFurnitureNearZoneDoesNotBlockLock) {
  // 움직인 적 없는 다리 쌍 모양(가구)은 정지 트랙이 되면 경쟁자로 세지 않는다
  ks::Harness h(stateParams());
  addPerson(h, {1.0, 0.0});
  addPerson(h, {1.0, 0.45});
  h.run(35);  // 정지 트랙이 될 때까지
  h.core.setMode(Mode::ACQUIRE);
  int scans = -1;
  for (int k = 0; k < 15 && scans < 0; ++k) {
    if (h.run(1).status == TargetStatus::LOCKED) {
      scans = k;
    }
  }
  EXPECT_EQ(scans, 10);
}

TEST(TargetManager, TargetStopsBesideFurnitureLegAndStaysOnPerson) {
  // 실차 기록 회귀: 사람이 로봇 옆 가구 다리(0.5 m 거리) 곁에 멈추면
  // "사람 다리 + 가구 다리" 거짓 쌍으로 타깃이 가구 쪽으로 끌려가면 안 된다
  ks::Harness h(ks::lidarOnlyParams());
  h.world.circles.push_back({{0.04, 0.77}, 0.03});  // 가구 다리
  h.world.circles.push_back({{-0.20, 1.47}, 0.04});  // 또 하나 (0.75 m 떨어짐)
  h.run(40);  // 가구가 정지 트랙이 된다
  ks::WalkerSpec w;
  w.start = {2.2, -0.6};
  w.heading = 2.8;
  w.path = {{{1.3, 0.0}, 0.8, 0.0}, {{0.55, 0.8}, 0.6, 100.0}};
  h.world.walkers.emplace_back(w);
  h.run(15);
  h.core.setMode(Mode::ACQUIRE);
  int locked_at = -1;
  for (int k = 0; k < 20 && locked_at < 0; ++k) {
    if (h.run(1).status == TargetStatus::LOCKED) {
      locked_at = k;
    }
  }
  ASSERT_GE(locked_at, 0);
  h.core.setMode(Mode::TRACK);
  const int id = h.core.target().track_id;
  for (int k = 0; k < 80; ++k) {  // 걸어와 가구 옆에 서서 6 s 이상
    const auto& out = h.run(1);
    ASSERT_NE(out.status, TargetStatus::LOST) << "scan " << k;
    EXPECT_EQ(out.track_id, id) << "scan " << k;
    EXPECT_LT(kt::distance(out.position, h.world.walkers[0].center()), 0.3)
        << "scan " << k;
  }
}

TEST(TargetManager, CameraAliveButTargetUnseenGoesLost) {
  // 카메라가 살아 있는데(빈 프레임이 계속 옴) 확인 영역 안 타깃을 camera_lost_s 넘게 못 보면 LOST
  kt::TrackerParams p;  // require_camera = true
  ks::Harness h(p);
  h.world.walkers.emplace_back(ks::standingPerson({1.2, 0.0}));
  ASSERT_EQ(acquireScans(h, 25, true), 10);
  h.core.setMode(Mode::TRACK);
  int lost_at = -1;
  for (int k = 0; k < 60 && lost_at < 0; ++k) {
    h.run(1);
    h.camera({});  // 빈 프레임 (사람을 못 봄)
    if (h.core.target().status == TargetStatus::LOST) {
      lost_at = k;
    }
  }
  // 마지막 확인(1 s 유지) + camera_lost_s(3 s) 뒤
  EXPECT_GE(lost_at, 35);
  EXPECT_LE(lost_at, 45);
}

TEST(TargetManager, CameraDeadKeepsLidarTracking) {
  // 카메라 프레임이 끊기면 이 규칙을 쓰지 않는다 (SPEC S6: 카메라만 끊기면 감속해서 계속)
  kt::TrackerParams p;
  ks::Harness h(p);
  h.world.walkers.emplace_back(ks::standingPerson({1.2, 0.0}));
  ASSERT_EQ(acquireScans(h, 25, true), 10);
  h.core.setMode(Mode::TRACK);
  h.run(60);  // 카메라 프레임 없음
  EXPECT_EQ(h.core.target().status, TargetStatus::LOCKED);
}

TEST(TargetManager, OptionalAmbiguityTimeoutReacquiresAtPreAmbiguityPosition) {
  // 선택 기능(ambiguity.lost_s > 0): 모호가 계속되면 LOST, 재획득 기준은 모호 직전 타깃 위치
  kt::TrackerParams p = stateParams();
  p.ambiguity.lost_s = 2.0;
  ks::Harness h(p);
  h.world.walkers.emplace_back(ks::standingPerson({1.2, 0.0}));
  ASSERT_GE(acquireScans(h, 15), 0);
  h.core.setMode(Mode::TRACK);
  h.run(3);
  ks::WalkerSpec other;
  other.start = {1.3, 2.0};
  other.heading = -kt::kPi / 2.0;
  other.path = {{{1.3, 0.45}, 0.8, 100.0}};
  h.world.walkers.emplace_back(other);
  int lost_at = -1;
  for (int k = 0; k < 80 && lost_at < 0; ++k) {
    if (h.run(1).status == TargetStatus::LOST) {
      lost_at = k;
    }
  }
  ASSERT_GE(lost_at, 0);
  EXPECT_LT(kt::distance(h.core.target().lost_position, {1.2, 0.0}), 0.1);
  EXPECT_LT(kt::distance(h.core.target().lost_seen_position, {1.2, 0.0}), 0.1);
}
