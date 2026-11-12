// ControllerCore: 상태기계 + 법칙 + 게이트 + 가감속을 한 주기 단위로 시험
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include "kebi_follow_controller/controller_core.hpp"

using namespace kebi_follow;

namespace {

constexpr double kDt = 0.05;  // 20 Hz

struct Harness {
  explicit Harness(const ControllerParams& p = ControllerParams{}) : core(p) {
    in.sensors.scan_ok = true;
    in.sensors.odom_ok = true;
    in.sensors.tf_ok = true;
    in.sensors.bottom_ir_ok = true;
    in.sensors.camera_ok = true;
    in.sensors.depth_points_ok = true;
    in.sensors.lifted_ok = true;
    in.sensors.fall_down_ok = true;
  }

  void target(double x, double y, TargetStatus st = TargetStatus::kLocked,
              double vx = 0.0) {
    in.target.received = true;
    in.target.status = st;
    in.target.position = {x, y};
    in.target.velocity = {vx, 0.0};
    in.target.msg_age = 0.05;
    in.target.occluded_time = 0.0;
    in.target.ambiguous = false;
  }
  void noTarget() { in.target = TargetInput{}; }

  CycleOutput step(double dt = kDt) {
    t += dt;
    in.now = t;
    in.dt = dt;
    in.scan = ObstacleSet{scan.data(), scan.size(), Pose2D{}};
    in.depth = ObstacleSet{depth.data(), depth.size(), Pose2D{}};
    last = core.step(in);
    return last;
  }
  CycleOutput run(double seconds) {
    const int n = static_cast<int>(std::lround(seconds / kDt));
    for (int i = 0; i < n; ++i)
      step();
    return last;
  }
  bool start() { return core.onStart(t); }
  bool stop() { return core.onStop(t); }

  // FOLLOWING 에서 정속까지 달린 상태로 만든다
  void cruise(double range = 3.0) {
    target(range, 0.0);
    start();
    run(3.0);
    ASSERT_EQ(last.state, FsmState::kFollowing);
    ASSERT_NEAR(last.v, 0.5, 1e-6);
  }

  ControllerCore core;
  CycleInput in;
  CycleOutput last;
  std::vector<Vec2> scan;
  std::vector<Vec2> depth;
  double t{0.0};
};

}  // namespace

TEST(Core, IdlePublishesNothing) {
  Harness h;
  h.target(2.0, 0.0);
  h.scan = {{0.5, 0.0}};
  for (int i = 0; i < 200; ++i) {
    const auto o = h.step();
    EXPECT_FALSE(o.publish);
    EXPECT_EQ(o.state, FsmState::kIdle);
    EXPECT_EQ(o.mode, ModeValue::kOff);
    EXPECT_EQ(o.detect_user, detect_user::kNone);
  }
  EXPECT_FALSE(h.stop());  // IDLE 중 STOP: 0 도 내지 않는다
  EXPECT_FALSE(h.step().publish);
  EXPECT_FALSE(h.core.onHalt(h.t, stop_reason::kCliff));
}

TEST(Core, AcquiringPublishesZeroAndReportsCandidate) {
  Harness h;
  EXPECT_TRUE(h.start());
  auto o = h.step();
  EXPECT_TRUE(o.publish);
  EXPECT_EQ(o.v, 0.0);
  EXPECT_EQ(o.w, 0.0);
  EXPECT_EQ(o.mode, ModeValue::kAcquire);
  EXPECT_EQ(o.detect_user, detect_user::kNoPerson);
  EXPECT_TRUE(o.stop_reasons & stop_reason::kNoTarget);
  h.target(1.0, 0.0, TargetStatus::kCandidate);
  h.run(0.5);
  EXPECT_EQ(h.last.state, FsmState::kAcquiring);
  EXPECT_EQ(h.last.detect_user, detect_user::kRecognized);
}

TEST(Core, AmbiguousLockDoesNotStartFollowing) {
  Harness h;
  h.start();
  h.target(1.0, 0.0);
  h.in.target.ambiguous = true;
  h.run(1.0);
  EXPECT_EQ(h.last.state, FsmState::kAcquiring);
}

TEST(Core, LockFollowAccelerateAndReportMoving) {
  Harness h;
  h.start();
  h.step();
  h.target(2.5, 0.0);
  auto o = h.step();
  EXPECT_EQ(o.state, FsmState::kFollowing);
  EXPECT_EQ(o.mode, ModeValue::kTrack);
  double v_prev = 0.0;
  for (int i = 0; i < 40; ++i) {
    o = h.step();
    EXPECT_GE(o.v, v_prev - 1e-12);
    EXPECT_LE(o.v - v_prev, 0.4 * kDt + 1e-9);  // 가속 0.4 m/s^2
    v_prev = o.v;
  }
  EXPECT_NEAR(o.v, 0.5, 0.05);
  EXPECT_EQ(o.detect_user, detect_user::kMoving);
}

TEST(Core, UpdateParamsKeepsStateAndAppliesNewCaps) {
  // 실행 중 조정: FSM·현재 속도는 유지하고, 새 상한까지 감속 제한으로 내려간다
  Harness h;
  h.start();
  h.step();
  h.target(2.5, 0.0);
  auto o = h.run(3.0);
  ASSERT_EQ(o.state, FsmState::kFollowing);
  ASSERT_NEAR(o.v, 0.5, 0.05);
  ControllerParams p = h.core.params();
  p.limiter.max_linear_speed = 0.2;
  p.law.max_linear_speed = 0.2;
  h.core.updateParams(p);
  o = h.step();
  EXPECT_EQ(o.state, FsmState::kFollowing);
  EXPECT_GT(o.v, 0.2);  // 즉시 0 이 아니라
  EXPECT_LT(o.v, 0.5);  // 감속 0.8 m/s^2 로 내려간다
  o = h.run(1.0);
  EXPECT_NEAR(o.v, 0.2, 1e-6);
  EXPECT_DOUBLE_EQ(h.core.params().limiter.max_linear_speed, 0.2);
}

TEST(Core, StopGivesThreeZerosThenSilence) {
  Harness h;
  h.cruise();
  EXPECT_TRUE(h.stop());  // 노드가 바로 0 을 1 번 낸다
  int zeros = 1;
  for (int i = 0; i < 40; ++i) {
    const auto o = h.step();
    EXPECT_EQ(o.state, FsmState::kIdle);
    EXPECT_EQ(o.detect_user, detect_user::kNone);
    if (o.publish) {
      EXPECT_EQ(o.v, 0.0);
      EXPECT_EQ(o.w, 0.0);
      ++zeros;
    }
  }
  EXPECT_EQ(zeros, 3);
}

TEST(Core, AcquireTimeoutEndsWithZerosAndDetectZero) {
  Harness h;
  h.start();
  int zeros_after_end = 0;
  for (int i = 0; i < 700; ++i) {
    const auto o = h.step();
    if (o.state == FsmState::kEnded) {
      EXPECT_EQ(o.detect_user, detect_user::kEnded);
      EXPECT_EQ(o.mode, ModeValue::kOff);
      zeros_after_end += o.publish ? 1 : 0;
    }
  }
  EXPECT_EQ(h.last.state, FsmState::kEnded);
  EXPECT_EQ(zeros_after_end, 3);
}

TEST(Core, OccludedHalfSpeedThenWaitingThenEnded) {
  Harness h;
  h.cruise();
  h.in.target.status = TargetStatus::kOccluded;
  h.in.target.occluded_time = 0.2;
  auto o = h.step();
  EXPECT_EQ(o.state, FsmState::kFollowing);
  EXPECT_NEAR(o.v_limit, 0.25, 1e-9);
  h.run(1.0);
  EXPECT_NEAR(h.last.v, 0.25, 1e-6);  // 감속 제한으로 절반까지
  h.in.target.occluded_time = 0.75;   // 측정 없는 시간 0.7 s 초과
  o = h.step();
  EXPECT_EQ(o.state, FsmState::kWaiting);
  EXPECT_TRUE(o.stop_reasons & stop_reason::kNoTarget);
  EXPECT_NEAR(o.v, 0.25 - 0.8 * kDt, 1e-6);  // 감속 제한 정지
  h.in.target.status = TargetStatus::kLost;
  h.run(1.0);
  EXPECT_EQ(h.last.v, 0.0);
  EXPECT_EQ(h.last.detect_user, detect_user::kNoPerson);
  h.run(29.0);
  EXPECT_EQ(h.last.state, FsmState::kEnded);
  EXPECT_EQ(h.last.detect_user, detect_user::kEnded);
}

TEST(Core, StaleTrackerMessagesCountAsLost) {
  Harness h;
  h.cruise();
  h.in.target.msg_age = 0.4;  // 메시지가 끊김 → 절반 속도
  EXPECT_NEAR(h.step().v_limit, 0.25, 1e-9);
  h.in.target.msg_age = 0.8;
  EXPECT_EQ(h.step().state, FsmState::kWaiting);
}

TEST(Core, AmbiguousWaitsAndReportsRecognized) {
  Harness h;
  h.cruise();
  h.in.target.ambiguous = true;
  auto o = h.step();
  EXPECT_EQ(o.state, FsmState::kWaiting);
  EXPECT_TRUE(o.stop_reasons & stop_reason::kAmbiguous);
  h.run(1.0);
  EXPECT_EQ(h.last.v, 0.0);
  EXPECT_EQ(h.last.detect_user, detect_user::kRecognized);
  h.in.target.ambiguous = false;
  h.run(0.4);
  EXPECT_EQ(h.last.state, FsmState::kWaiting);
  h.run(0.2);
  EXPECT_EQ(h.last.state, FsmState::kFollowing);
}

TEST(Core, SensorTimeoutsStopImmediately) {
  for (int which = 0; which < 4; ++which) {
    Harness h;
    h.cruise();
    if (which == 0)
      h.in.sensors.scan_ok = false;
    if (which == 1)
      h.in.sensors.odom_ok = false;
    if (which == 2)
      h.in.sensors.tf_ok = false;
    if (which == 3)
      h.in.sensors.bottom_ir_ok = false;
    const auto o = h.step();
    EXPECT_EQ(o.state, FsmState::kWaiting) << which;
    EXPECT_EQ(o.v, 0.0) << which;
    EXPECT_TRUE(o.stop_reasons & stop_reason::kSensorTimeout) << which;
  }
}

TEST(Core, CameraMissingCapsSpeed) {
  Harness h;
  h.cruise();
  h.in.sensors.camera_ok = false;
  auto o = h.step();
  EXPECT_EQ(o.state, FsmState::kFollowing);
  EXPECT_NEAR(o.v_limit, 0.3, 1e-9);
  h.run(1.0);
  EXPECT_NEAR(h.last.v, 0.3, 1e-6);
}

TEST(Core, HaltIsImmediateLatchedAndNeedsStart) {
  Harness h;
  h.cruise();
  // 노드가 바로 0 을 낸다
  EXPECT_TRUE(h.core.onHalt(h.t, stop_reason::kCliff));
  auto o = h.step();
  EXPECT_EQ(o.state, FsmState::kHalted);
  EXPECT_TRUE(o.publish);
  EXPECT_EQ(o.v, 0.0);
  EXPECT_EQ(o.w, 0.0);
  EXPECT_EQ(o.detect_user, detect_user::kEnded);
  EXPECT_EQ(o.mode, ModeValue::kTrack);
  EXPECT_TRUE(o.stop_reasons & stop_reason::kCliff);
  EXPECT_FALSE(h.core.onHalt(h.t, stop_reason::kCliff));  // 이미 HALTED
  h.run(0.5);
  EXPECT_FALSE(h.start());  // 홀드오프 1 s 안
  h.run(0.6);
  EXPECT_TRUE(h.start());
  // START 는 항상 재획득부터 (SPEC 5절). 이전 세션 타깃은 노드가 stamp 로 버린다
  h.noTarget();
  EXPECT_EQ(h.step().state, FsmState::kAcquiring);
  h.run(0.5);
  EXPECT_EQ(h.last.state, FsmState::kAcquiring);
  h.target(2.0, 0.0);  // 새로 락
  EXPECT_EQ(h.step().state, FsmState::kFollowing);
}

TEST(Core, PersistentHaltConditionBlocksRestart) {
  Harness h;
  h.cruise();
  h.in.halt_active = stop_reason::kRobotState;  // lifted 가 계속 true
  EXPECT_EQ(h.step().state, FsmState::kHalted);
  h.run(3.0);
  EXPECT_FALSE(h.start());
  h.in.halt_active = 0;
  h.run(1.1);
  EXPECT_TRUE(h.start());
}

TEST(Core, TooCloseStopsImmediatelyAndNeverReverses) {
  Harness h;
  h.cruise();
  h.target(0.75, 0.0, TargetStatus::kLocked, -0.5);
  auto o = h.step();
  EXPECT_EQ(o.state, FsmState::kFollowing);
  EXPECT_EQ(o.v, 0.0);
  EXPECT_TRUE(o.stop_reasons & stop_reason::kTooClose);
  // 사람이 계속 다가와도 물러서지 않는다
  for (double r = 0.75; r > 0.3; r -= 0.02) {
    h.target(r, 0.0, TargetStatus::kLocked, -0.5);
    EXPECT_EQ(h.step().v, 0.0);
  }
  h.run(0.5);
  EXPECT_EQ(h.last.detect_user, detect_user::kRecognized);
}

TEST(Core, ObstacleInsideBrakingDistanceStopsImmediately) {
  Harness h;
  h.cruise();
  h.scan = {{0.6, 0.0}};  // 앞면에서 0.38 m < 0.156 + 0.25
  const auto o = h.step();
  EXPECT_EQ(o.state, FsmState::kFollowing);
  EXPECT_EQ(o.v, 0.0);
  EXPECT_TRUE(o.stop_reasons & stop_reason::kObstacle);
  EXPECT_NEAR(o.scan_obstacle_distance, 0.38, 1e-9);
}

TEST(Core, ObstacleSlowdownZone) {
  Harness h;
  h.cruise();
  h.scan = {{0.22 + 0.625, 0.0}};  // d = 0.625 → 상한 0.25
  auto o = h.step();
  EXPECT_NEAR(o.v_limit, 0.25, 1e-9);
  EXPECT_NEAR(o.v, 0.5 - 0.8 * kDt, 1e-9);  // 즉시 정지가 아닌 감속
  h.run(0.5);
  EXPECT_LE(h.last.v, 0.25 + 1e-9);
  // 통로 밖 점은 무시
  h.scan = {{0.6, 0.4}};
  h.run(2.0);
  EXPECT_NEAR(h.last.v, 0.5, 1e-6);
}

TEST(Core, DepthObstacleUsesLargerMargin) {
  Harness h;
  h.cruise();
  // depth 만 본 점: 앞면 0.55 m (정지 여유 0.5)
  h.depth = {{0.22 + 0.55, 0.0}};
  auto o = h.step();
  EXPECT_NEAR(o.depth_obstacle_distance, 0.55, 1e-9);
  EXPECT_EQ(o.v, 0.0);  // 0.156 + 0.5 안
  EXPECT_TRUE(o.stop_reasons & stop_reason::kObstacle);
  // 같은 거리라도 camera 가 꺼져 있으면 depth 점은 쓰지 않는다 (대신 0.3 상한)
  Harness g;
  g.cruise();
  g.depth = {{0.22 + 0.55, 0.0}};
  g.in.sensors.camera_ok = false;
  g.in.sensors.depth_points_ok = false;
  EXPECT_GT(g.step().v, 0.0);
}

TEST(Core, TargetNeighbourhoodIsNotAnObstacle) {
  Harness h;
  h.cruise(1.5);
  h.scan = {{1.5, 0.1}, {1.5, -0.1}};  // 타깃 다리
  h.run(1.0);
  EXPECT_FALSE(h.last.stop_reasons & stop_reason::kObstacle);
  EXPECT_FALSE(std::isfinite(h.last.scan_obstacle_distance));
}

TEST(Core, InPlaceRotationAddsCreep) {
  Harness h;
  h.target(0.0, 1.5);  // 왼쪽 90°
  h.start();
  h.run(0.5);
  ASSERT_EQ(h.last.state, FsmState::kFollowing);
  EXPECT_GT(h.last.w, 0.0);
  EXPECT_DOUBLE_EQ(h.last.v, 0.002);  // S10: 베이스 워치독이 걸리도록
  EXPECT_EQ(h.last.detect_user, detect_user::kMoving);
}

TEST(Core, InPlaceRotationLimitedTo3s) {
  Harness h;
  // 오른쪽 90°, 시험에서는 로봇이 돌지 않으므로 계속 회전 요청
  h.target(0.0, -1.5);
  h.start();
  h.step();
  double rotating = 0.0;
  CycleOutput o;
  for (int i = 0; i < 100 && h.last.state == FsmState::kFollowing; ++i) {
    o = h.step();
    if (std::abs(o.w) > 0.05)
      rotating += kDt;
  }
  EXPECT_LE(rotating, 3.0 + kDt + 1e-9);
  EXPECT_GE(rotating, 2.9);
  EXPECT_EQ(h.last.state, FsmState::kWaiting);
  EXPECT_TRUE(h.last.stop_reasons & stop_reason::kRotationLimit);
  EXPECT_EQ(h.last.w, 0.0);
  // rearm 2 s + 확인 0.5 s 뒤 다시 회전
  h.run(2.4);
  EXPECT_EQ(h.last.state, FsmState::kWaiting);
  h.run(0.3);
  EXPECT_EQ(h.last.state, FsmState::kFollowing);
}

TEST(Core, RotationLimitClearsWhenTargetInFront) {
  Harness h;
  h.target(0.0, -1.5);
  h.start();
  h.run(3.5);
  ASSERT_EQ(h.last.state, FsmState::kWaiting);
  h.target(2.0, 0.0);  // 사람이 앞으로 왔다
  h.run(0.6);
  EXPECT_EQ(h.last.state, FsmState::kFollowing);
}

TEST(Core, RotationBlockedByNearObject) {
  Harness h;
  h.target(0.0, 1.5);
  h.scan = {{-0.1, -0.27}};  // 중심에서 0.29 m
  h.start();
  h.step();
  const auto o = h.step();
  EXPECT_EQ(o.state, FsmState::kWaiting);
  EXPECT_TRUE(o.stop_reasons & stop_reason::kRotationBlocked);
  EXPECT_EQ(o.w, 0.0);
  EXPECT_EQ(o.v, 0.0);
  h.scan.clear();
  h.run(0.6);
  EXPECT_EQ(h.last.state, FsmState::kFollowing);
}

TEST(Core, ArcMotionIsNotBlockedByRotationClearance) {
  Harness h;
  h.target(2.0, 0.3);
  h.scan = {{-0.1, -0.27}};  // 옆 뒤쪽 가까운 물체: 전진(원호)은 막지 않는다
  h.start();
  h.run(1.0);
  EXPECT_EQ(h.last.state, FsmState::kFollowing);
  EXPECT_GT(h.last.v, 0.1);
}

TEST(Core, WaitingResumeNeedsFreshLock) {
  Harness h;
  h.cruise();
  h.in.target.status = TargetStatus::kLost;
  h.run(1.0);
  ASSERT_EQ(h.last.state, FsmState::kWaiting);
  h.in.target.status = TargetStatus::kOccluded;
  h.in.target.occluded_time = 0.1;
  h.run(1.0);
  EXPECT_EQ(h.last.state, FsmState::kWaiting);
  h.in.target.status = TargetStatus::kLocked;
  h.run(0.4);
  EXPECT_EQ(h.last.state, FsmState::kWaiting);
  h.run(0.2);
  EXPECT_EQ(h.last.state, FsmState::kFollowing);
}

TEST(Core, OutputBoundsUnderRandomInputs) {
  Harness h;
  h.start();
  uint32_t rng = 7u;
  auto rnd = [&rng]() {
    rng = rng * 1664525u + 1013904223u;
    return (rng >> 8) / 16777216.0;
  };
  double v_prev = 0.0;
  for (int i = 0; i < 20000; ++i) {
    if (i % 7 == 0) {
      const auto st = static_cast<TargetStatus>(static_cast<int>(rnd() * 5));
      h.target(rnd() * 4.0 - 1.0, rnd() * 4.0 - 2.0, st, rnd() * 2.0 - 1.0);
      h.in.target.ambiguous = rnd() < 0.05;
      h.in.target.occluded_time = rnd();
      h.scan = {{rnd() * 3.0, rnd() * 2.0 - 1.0}};
      h.in.sensors.scan_ok = rnd() > 0.02;
      h.in.sensors.camera_ok = rnd() > 0.1;
    }
    if (rnd() < 0.002)
      h.core.onHalt(h.t, stop_reason::kCollision);
    if (rnd() < 0.003)
      h.start();
    if (rnd() < 0.001)
      h.stop();
    const auto o = h.step();
    EXPECT_GE(o.v, 0.0);
    EXPECT_LE(o.v, 0.5 + 1e-12);
    EXPECT_LE(std::abs(o.w), 0.8 + 1e-12);
    if (!isActive(o.state)) {
      EXPECT_TRUE(!o.publish || (o.v == 0.0 && o.w == 0.0));
    }
    // 가속은 항상 제한, 감속만 즉시 가능
    EXPECT_LE(o.v - v_prev, 0.4 * kDt + 0.002 + 1e-9);
    v_prev = o.publish ? o.v : 0.0;
  }
}

TEST(Core, ApproachingPersonNeverMakesRobotAdvanceInsideStopDistance) {
  Harness h;
  h.cruise(2.0);
  for (double r = 2.0; r > 0.4; r -= 0.05) {
    h.target(r, 0.0, TargetStatus::kLocked, -1.0);
    const auto o = h.step();
    EXPECT_GE(o.v, 0.0);
    if (r <= 0.8) {
      EXPECT_EQ(o.v, 0.0);
    }
  }
}

TEST(DetectUserFilter, DebounceAndImmediateZero) {
  DetectUserFilter f(0.3);
  EXPECT_EQ(f.update(detect_user::kNoPerson, 0.0), detect_user::kNoPerson);
  EXPECT_EQ(f.update(detect_user::kMoving, 0.1), detect_user::kNoPerson);
  EXPECT_EQ(f.update(detect_user::kMoving, 0.3), detect_user::kNoPerson);
  EXPECT_EQ(f.update(detect_user::kMoving, 0.4), detect_user::kMoving);
  EXPECT_EQ(f.update(detect_user::kRecognized, 0.45), detect_user::kMoving);
  // 깜빡임 무시
  EXPECT_EQ(f.update(detect_user::kMoving, 0.5), detect_user::kMoving);
  // 0 은 즉시
  EXPECT_EQ(f.update(detect_user::kEnded, 0.55), detect_user::kEnded);
  EXPECT_EQ(f.update(detect_user::kNoPerson, 0.6), detect_user::kNoPerson);
  EXPECT_EQ(f.update(detect_user::kNone, 0.7), detect_user::kNone);
}

TEST(ChangeOrPeriodic, ChangeAndOneHertz) {
  ChangeOrPeriodic p(1.0);
  EXPECT_TRUE(p.shouldPublish(1, 0.0));
  EXPECT_FALSE(p.shouldPublish(1, 0.5));
  EXPECT_TRUE(p.shouldPublish(2, 0.6));
  EXPECT_FALSE(p.shouldPublish(2, 1.5));
  EXPECT_TRUE(p.shouldPublish(2, 1.6));
}

TEST(Core, CycleTiming) {
  // 실제 크기: 라이다 401 점 + depth bin 120 점 + 타깃 경로 5 점
  Harness h;
  for (int i = 0; i < 401; ++i) {
    const double a = -M_PI + i * (2.0 * M_PI / 400.0);
    h.scan.push_back({2.0 * std::cos(a), 2.0 * std::sin(a)});
  }
  for (int i = 0; i < 120; ++i) {
    const double a = -0.8 + i * (1.6 / 120.0);
    h.depth.push_back({1.8 * std::cos(a), 1.8 * std::sin(a)});
  }
  const std::vector<Vec2> path{
      {1.3, 0.0}, {1.32, 0.0}, {1.34, 0.0}, {1.36, 0.0}, {1.4, 0.0}};
  h.in.depth_exclusion = Exclusion{path.data(), path.size()};
  h.target(1.4, 0.0);
  h.start();
  constexpr int kRuns = 20000;
  double worst = 0.0;
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < kRuns; ++i) {
    const auto a = std::chrono::steady_clock::now();
    h.step();
    worst = std::max(worst, std::chrono::duration<double, std::micro>(
                                std::chrono::steady_clock::now() - a)
                                .count());
  }
  const double mean = std::chrono::duration<double, std::micro>(
                          std::chrono::steady_clock::now() - t0)
                          .count() /
                      kRuns;
  std::printf("[timing] ControllerCore::step: mean %.1f us, max %.1f us\n",
              mean, worst);
  EXPECT_LT(mean, 1000.0);
}

TEST(Core, TooCloseStopsImmediatelyInWaitingToo) {
  // M3: 모호(WAITING) 중 사람이 0.75 m 로 다가오면 감속 제한 없이 즉시 0
  Harness h;
  h.cruise();
  h.in.target.ambiguous = true;
  auto o = h.step();
  ASSERT_EQ(o.state, FsmState::kWaiting);
  ASSERT_GT(o.v, 0.3);  // 아직 감속 중
  h.target(0.75, 0.0, TargetStatus::kLocked, -0.5);
  h.in.target.ambiguous = true;
  o = h.step();
  EXPECT_EQ(o.state, FsmState::kWaiting);
  EXPECT_EQ(o.v, 0.0);
  EXPECT_TRUE(o.stop_reasons & stop_reason::kTooClose);
  // 가림(측정 없는 시간 0.7 s 초과)으로 WAITING 이어도 예측 위치가 가까우면 즉시 0
  Harness g;
  g.cruise();
  g.in.target.status = TargetStatus::kOccluded;
  g.in.target.occluded_time = 0.8;
  o = g.step();
  ASSERT_EQ(o.state, FsmState::kWaiting);
  ASSERT_GT(o.v, 0.3);
  g.in.target.position = {0.7, 0.05};
  o = g.step();
  EXPECT_EQ(o.v, 0.0);
  EXPECT_TRUE(o.stop_reasons & stop_reason::kTooClose);
}

TEST(Core, CappedSpeedPathIsAlsoChecked) {
  // m2: 상한으로 v 가 줄면 같은 w 의 경로가 더 휜다 → 그 원호도 본다
  ControllerParams p;
  SafetyGate gate(p.corridor);
  const double bearing = degToRad(12.0);
  const Vec2 tgt{3.0 * std::cos(bearing), 3.0 * std::sin(bearing)};
  // 법칙: v 0.5, w = 1.2 (b - 0.05) → 곡률 0.38. 가림(절반) → v 0.25 → 곡률 0.76 (R 1.31)
  const double w = 1.2 * (bearing - 0.05);
  const double R1 = 0.25 / w;
  const Vec2 obst{R1 * std::sin(degToRad(80.0)),
                  R1 * (1.0 - std::cos(degToRad(80.0)))};
  ASSERT_FALSE(std::isfinite(gate.pathDistance(obst, 0.0)));
  ASSERT_FALSE(std::isfinite(gate.pathDistance(obst, w / 0.5)));
  ASSERT_TRUE(std::isfinite(gate.pathDistance(obst, w / 0.25)));
  for (int occluded = 0; occluded < 2; ++occluded) {
    Harness h;
    h.scan = {obst};
    h.target(tgt.x, tgt.y);
    h.start();
    h.run(2.0);
    if (occluded) {
      h.in.target.status = TargetStatus::kOccluded;
      h.in.target.occluded_time = 0.2;
    }
    const auto o = h.step();
    ASSERT_EQ(o.state, FsmState::kFollowing);
    if (occluded) {
      EXPECT_TRUE(std::isfinite(o.scan_obstacle_distance));
    } else {
      EXPECT_FALSE(std::isfinite(o.scan_obstacle_distance));
    }
  }
}

TEST(Core, LiftedOrFallDownStaleIsSensorTimeout) {
  for (int which = 0; which < 2; ++which) {
    Harness h;
    h.cruise();
    if (which == 0)
      h.in.sensors.lifted_ok = false;
    if (which == 1)
      h.in.sensors.fall_down_ok = false;
    const auto o = h.step();
    EXPECT_EQ(o.state, FsmState::kWaiting);
    EXPECT_EQ(o.v, 0.0);
    EXPECT_TRUE(o.stop_reasons & stop_reason::kSensorTimeout);
  }
}

TEST(Core, DegradedDepthCapsSpeedButKeepsPoints) {
  // M1: depth degraded → 0.3 상한, 직전 평면으로 낸 장애물 점은 계속 쓴다
  Harness h;
  h.cruise();
  h.in.sensors.camera_ok = false;
  h.in.sensors.depth_points_ok = true;
  h.depth = {{0.22 + 0.55, 0.0}};
  const auto o = h.step();
  EXPECT_EQ(o.v, 0.0);  // depth 점 정지 (0.156 + 0.5 안)
  EXPECT_NEAR(o.depth_obstacle_distance, 0.55, 1e-9);
  Harness g;
  g.cruise();
  g.in.sensors.camera_ok = false;
  g.in.sensors.depth_points_ok = false;
  g.depth = {{0.22 + 0.55, 0.0}};  // 점을 못 쓰면 무시 + 상한만
  const auto q = g.step();
  EXPECT_GT(q.v, 0.0);
  EXPECT_NEAR(q.v_limit, 0.3, 1e-9);
}
