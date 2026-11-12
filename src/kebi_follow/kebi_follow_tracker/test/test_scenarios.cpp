// 합성 시나리오 통합 시험 (고정 seed). 평가 도구(scenario_eval)는 여러 seed 로 같은 시나리오를 돈다.
#include <gtest/gtest.h>

#include "sim/scenario_library.hpp"

namespace ks = kebi_follow::tracker::sim;

namespace {
constexpr unsigned kSeed = 1;
constexpr double kMaxProcMs = 2.0;  // x86 기준 scan 당 처리 시간 상한

void expectFast(const ks::ScenarioMetrics& m) {
  EXPECT_LT(m.proc_max_ms, kMaxProcMs) << m.name;
}
}  // namespace

TEST(Scenarios, AcquireStaticPerson) {
  for (bool camera : {false, true}) {
    const auto m = ks::runScenario(ks::acquireStatic(camera, kSeed)).metrics;
    EXPECT_EQ(m.lock_walker, 0) << m.name;
    EXPECT_GE(m.lock_time, 0.95) << m.name;  // ACQUIRE 뒤 confirm_s(1.0) 유지
    EXPECT_LE(m.lock_time, 1.5) << m.name;
    expectFast(m);
  }
}

TEST(Scenarios, AcquireWaitsWhileTwoPeopleInZone) {
  for (bool camera : {false, true}) {
    const auto m = ks::runScenario(ks::acquireTwoPeople(camera, kSeed)).metrics;
    EXPECT_EQ(m.lock_walker, 0) << m.name;
    EXPECT_EQ(m.false_locks, 0) << m.name;
    EXPECT_GT(m.lock_time, 4.0)
        << m.name;  // 두 번째 사람이 6 s 에 떠나기 전에는 락 없음
  }
}

TEST(Scenarios, NoLockOnFurniture) {
  for (bool camera : {false, true}) {
    const auto m = ks::runScenario(ks::clutterOnly(camera, kSeed)).metrics;
    EXPECT_EQ(m.lock_walker, -2) << m.name;
    EXPECT_EQ(m.false_locks, 0) << m.name;
  }
}

TEST(Scenarios, FollowWalkingPerson) {
  const auto m = ks::runScenario(ks::followWalk(1.0, true, kSeed)).metrics;
  EXPECT_EQ(m.lock_walker, 0);
  EXPECT_EQ(m.id_switches, 0);
  EXPECT_EQ(m.wrong_scans, 0);
  EXPECT_GT(m.track_coverage, 0.95);
  EXPECT_LT(m.err_rms, 0.12);
  expectFast(m);
}

TEST(Scenarios, FollowThroughClutter) {
  const auto m = ks::runScenario(ks::followClutter(true, kSeed)).metrics;
  EXPECT_EQ(m.id_switches, 0);
  EXPECT_EQ(m.wrong_scans, 0);
  EXPECT_GT(m.track_coverage, 0.95);
  expectFast(m);
}

TEST(Scenarios, CrossingPersonDoesNotStealTarget) {
  // 1 m 앞을 가로지름: 타깃은 잠깐 가려진다(OCCLUDED). 타깃과 1 m 떨어져 있어 모호는 아니다
  const auto m = ks::runScenario(ks::crossing(1.0, 1.0, true, kSeed)).metrics;
  EXPECT_EQ(m.lock_walker, 0);
  EXPECT_EQ(m.wrong_scans, 0);
  EXPECT_EQ(m.id_switches, 0);
  EXPECT_EQ(m.false_locks, 0);
  EXPECT_GT(m.occluded_scans, 0);
}

TEST(Scenarios, CrossingCloseToTargetIsAmbiguous) {
  // 타깃 0.4 m 앞을 가로지름: 모호(S8 정지)가 떠야 하고 타깃을 뺏기지 않아야 한다
  const auto m = ks::runScenario(ks::crossing(1.6, 1.2, true, kSeed)).metrics;
  EXPECT_EQ(m.wrong_scans, 0);
  EXPECT_EQ(m.id_switches, 0);
  EXPECT_GT(m.ambiguous_scans, 0);
}

TEST(Scenarios, RobotRotationKeepsTrack) {
  const auto m = ks::runScenario(ks::rotateInPlace(true, kSeed)).metrics;
  EXPECT_EQ(m.id_switches, 0);
  EXPECT_EQ(m.wrong_scans, 0);
  EXPECT_LT(m.err_rms, 0.1);
}
