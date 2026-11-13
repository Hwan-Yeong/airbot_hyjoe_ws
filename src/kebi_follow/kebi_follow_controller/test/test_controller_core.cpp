// ControllerCore: 상태기계 + 법칙 + 게이트 + 가감속을 한 주기 단위로 시험
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include "kebi_follow_controller/controller_core.hpp"
#include "kebi_follow_controller/tunables.hpp"

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

  // 단호 S 곡선으로 v 에서 0 까지 걸리는 시간 + 두 주기 (감속 a 가 jerk j 로 차오르고 끝에서 다시 줄어든다)
  double firmStopBound(double v) const {
    const auto& L = core.params().limiter;
    const double a = std::max(L.firm_linear_decel, L.max_linear_decel);
    const double j = std::max(L.firm_linear_jerk, L.max_linear_jerk);
    const double t = (j <= 0.0) ? v / a : (v >= a * a / j ? v / a + a / j : 2.0 * std::sqrt(v / j));
    return t + 2.0 * kDt;
  }
  double traveled{0.0};  // 마지막 단호 정지·감속 동안 간 거리 [m] (출력 v·dt 합)

  // 단호 정지 (S 곡선, 즉시 0 아님): 첫 출력 o 부터 v 가 늘지 않고, 후진 없고,
  // 감속은 단호 한계 안이며 max_s 안에 선다. 선 데 걸린 시간을 돌려준다
  double expectFirmStop(const CycleOutput& first, double v_before, double max_s,
                        const char* what) {
    const auto& L = core.params().limiter;
    const double dmax = std::max(L.firm_linear_decel, L.max_linear_decel) * kDt + 1e-9;
    double prev = v_before;
    CycleOutput o = first;
    double took = kDt;
    traveled = 0.0;
    for (int i = 0; i < 400; ++i) {
      traveled += o.v * kDt;
      EXPECT_LE(o.v, prev + 1e-9) << what;
      EXPECT_GE(o.v, 0.0) << what;
      EXPECT_LE(prev - o.v, dmax) << what;
      prev = o.v;
      if (o.v <= 1e-9) {
        break;
      }
      o = step();
      took += kDt;
    }
    EXPECT_LE(took, max_s + 1e-9) << what;
    return took;
  }

  static constexpr double kCapLandingTol = 0.005;  // [m/s]
  // 장애물 단호 감속: 첫 출력부터 v 가 늘지 않고, 감속은 단호 한계 안이며, max_s 안에 장애물 상한
  // (v_limit) 이하로 내려가 그 뒤로 넘지 않는다. 시험 하네스는 로봇이 실제로 다가가지 않아 상한이
  // 그대로이므로 0 까지가 아니라 상한까지만 본다 (실제로는 다가가며 상한이 줄어 정지 여유에서 선다)
  double expectFirmSlowdownToCap(const CycleOutput& first, double v_before,
                                 double max_s, const char* what) {
    const auto& L = core.params().limiter;
    const double dmax = std::max(L.firm_linear_decel, L.max_linear_decel) * kDt + 1e-9;
    double prev = v_before;
    CycleOutput o = first;
    double took = kDt;
    traveled = 0.0;
    for (int i = 0; i < 400; ++i) {
      traveled += o.v * kDt;
      EXPECT_LE(o.v, prev + 1e-9) << what;
      EXPECT_GE(o.v, 0.0) << what;
      EXPECT_LE(prev - o.v, dmax) << what;
      prev = o.v;
      if (o.v <= o.v_limit + 1e-6) {
        break;
      }
      o = step();
      took += kDt;
    }
    EXPECT_LE(took, max_s + 1e-9) << what;
    const double cap = o.v_limit;
    for (int i = 0; i < 20; ++i) {  // 상한 위로 다시 달리지 않는다 (S 곡선 착지 오차 5 mm/s 안)
      EXPECT_LE(step().v, cap + kCapLandingTol) << what;
    }
    return took;
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

TEST(Core, StopWhileMovingUsesFirmSCurveThenThreeZerosThenSilence) {
  // 2026-11-13 사용자 요구: STOP 명령도 S 곡선으로 선다 (즉시 0 은 바퀴가 미끄러졌다). HALT 는 즉시 0 그대로
  Harness h;
  h.cruise();
  EXPECT_FALSE(h.stop());  // 바로 0 을 내지 않는다
  EXPECT_TRUE(h.core.stopPending());
  auto first = h.step();
  EXPECT_EQ(first.state, FsmState::kFollowing);  // 서는 동안 활성 상태 (HALT·안전 검사 그대로)
  const double took = h.expectFirmStop(first, 0.5, h.firmStopBound(0.5), "stop command");
  EXPECT_GT(took, 0.3);  // 즉시 0 이 아니다
  EXPECT_EQ(h.last.state, FsmState::kIdle);  // 선 주기에 IDLE, 0 을 낸다
  EXPECT_TRUE(h.last.publish);
  EXPECT_FALSE(h.core.stopPending());
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

TEST(Core, StopWhileNotMovingGoesIdleAtOnce) {
  Harness h;
  h.target(3.0, 0.0);
  h.start();  // ACQUIRING: 움직이지 않는다
  EXPECT_TRUE(h.stop());  // 바로 IDLE, 노드가 0 을 한 번 낸다
  EXPECT_EQ(h.core.state(), FsmState::kIdle);
  EXPECT_FALSE(h.core.stopPending());
}

TEST(Core, HaltDuringStopCommandIsImmediateAndEndsIdle) {
  for (int level = 0; level < 2; ++level) {
    Harness h;
    h.cruise();
    h.stop();
    h.step();
    ASSERT_GT(h.last.v, 0.1);
    if (level == 0) {
      EXPECT_TRUE(h.core.onHalt(h.t, stop_reason::kCliff));  // 이벤트: 노드가 바로 0
    } else {
      h.in.halt_active = stop_reason::kCliff;  // 레벨 신호: 다음 주기
    }
    const auto o = h.step();
    EXPECT_EQ(o.v, 0.0);
    EXPECT_EQ(o.w, 0.0);
    EXPECT_EQ(o.state, FsmState::kIdle);  // STOP 요청대로 IDLE (HALTED 로 남아 0 을 계속 내지 않는다)
    EXPECT_FALSE(h.core.stopPending());
  }
}

TEST(Core, StartDuringStopCommandCancelsIt) {
  Harness h;
  h.cruise();
  h.stop();
  h.step();
  EXPECT_FALSE(h.start());  // FSM 은 아직 활성: 정지만 거둔다
  EXPECT_FALSE(h.core.stopPending());
  const auto o = h.run(3.0);
  EXPECT_EQ(o.state, FsmState::kFollowing);
  EXPECT_NEAR(o.v, 0.5, 1e-6);
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
  // 편안 정지: jerk 제한으로 감속이 차오른다 (즉시 0.8 m/s^2 아님)
  EXPECT_LT(o.v, 0.25);
  EXPECT_GT(o.v, 0.25 - 0.8 * kDt);
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

TEST(Core, SensorTimeoutsStopFirmly) {
  // 2026-11-13: 즉시 0 대신 단호 정지 (S 곡선). 0.5 m/s 에서 firmStopBound 안에 선다 (1.2·5 기본값 0.76 s)
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
    EXPECT_TRUE(o.stop_reasons & stop_reason::kSensorTimeout) << which;
    h.expectFirmStop(o, 0.5, h.firmStopBound(0.5), "sensor timeout");
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

TEST(Core, TooCloseStopsFirmlyAndNeverReverses) {
  Harness h;
  h.cruise();
  h.target(0.75, 0.0, TargetStatus::kLocked, -0.5);
  auto o = h.step();
  EXPECT_EQ(o.state, FsmState::kFollowing);
  EXPECT_TRUE(o.stop_reasons & stop_reason::kTooClose);
  h.expectFirmStop(o, 0.5, h.firmStopBound(0.5), "too close");
  // 사람이 계속 다가와도 물러서지 않고 다시 나아가지 않는다
  for (double r = 0.75; r > 0.3; r -= 0.02) {
    h.target(r, 0.0, TargetStatus::kLocked, -0.5);
    EXPECT_EQ(h.step().v, 0.0);
  }
  h.run(0.5);
  EXPECT_EQ(h.last.detect_user, detect_user::kRecognized);
}

TEST(Core, ObstacleInsideBrakingDistanceStopsFirmly) {
  Harness h;
  h.cruise();
  h.scan = {{0.6, 0.0}};  // 앞면에서 0.38 m < 0.25 + 0.5·0.25 + 0.156
  const auto o = h.step();
  EXPECT_EQ(o.state, FsmState::kFollowing);
  EXPECT_TRUE(o.stop_reasons & stop_reason::kObstacle);
  EXPECT_NEAR(o.scan_obstacle_distance, 0.38, 1e-9);
  // 단호 감속 동안 간 거리는 장애물(0.38 m)보다 0.1 m 이상 짧다
  h.expectFirmSlowdownToCap(o, 0.5, h.firmStopBound(0.5), "obstacle");
  EXPECT_LT(h.traveled, 0.38 - 0.1);
}

TEST(Core, ObstacleSlowdownZone) {
  Harness h;
  h.cruise();
  h.scan = {{0.22 + 0.625, 0.0}};  // d = 0.625 → 상한 0.25
  auto o = h.step();
  EXPECT_NEAR(o.v_limit, 0.25, 1e-9);
  // 즉시 정지가 아닌 감속. 안전 상한이 갑자기 내려가면 편안 감속 크기로 바로 건다 (kCap, 리뷰 2026-11-13)
  EXPECT_LT(o.v, 0.5);
  EXPECT_GE(o.v, 0.5 - 0.8 * kDt - 1e-9);
  h.run(1.0);
  EXPECT_LE(h.last.v, 0.25 + 1e-6);
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
  EXPECT_TRUE(o.stop_reasons & stop_reason::kObstacle);  // 0.156 + 0.5 안
  h.expectFirmSlowdownToCap(o, 0.5, h.firmStopBound(0.5), "depth obstacle");
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
  // 상한이 걸린 주기까지 포함 (그 주기엔 각속도를 끊지 않는다)
  EXPECT_LE(rotating, 3.0 + 2 * kDt + 1e-9);
  EXPECT_GE(rotating, 2.9);
  EXPECT_EQ(h.last.state, FsmState::kWaiting);
  EXPECT_TRUE(h.last.stop_reasons & stop_reason::kRotationLimit);
  // 각속도는 끊지 않고 편안 감속(S 곡선)으로 1 s 안에 0
  double w_prev = std::abs(h.last.w);
  double t_stop = 0.0;
  while (std::abs(h.last.w) > 1e-9 && t_stop < 1.0) {
    h.step();
    t_stop += kDt;
    EXPECT_LE(std::abs(h.last.w), w_prev + 1e-9);
    w_prev = std::abs(h.last.w);
  }
  EXPECT_EQ(h.last.w, 0.0);
  EXPECT_LE(t_stop, 1.0);
  // rearm 2 s + 확인 0.5 s 뒤 다시 회전 (상한 걸린 때부터)
  h.run(2.4 - t_stop);
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

namespace {
// config/follow_controller.yaml 의 실차 기본값 (2026-11-13)
ControllerParams yamlLikeParams() {
  ControllerParams p;
  p.limiter.max_linear_speed = 1.2;
  p.limiter.max_linear_accel = 0.8;
  p.limiter.max_linear_decel = 0.8;
  p.limiter.max_linear_jerk = 1.0;
  p.limiter.firm_linear_decel = 1.2;
  p.limiter.firm_linear_jerk = 5.0;
  p.law.follow_distance = 1.0;
  p.law.stop_distance = 0.5;
  p.law.approach_margin = 0.3;
  p.law.reaction_time = 0.2;
  p.law.k_v = 0.8;
  p.v_filter_s = 0.45;
  p.w_filter_s = 0.3;
  syncDerived(&p);
  return p;
}

// 1차원 걷기: 사람이 서 있다가 1.5 m/s^2 로 v_walk 까지 걷고, t_stop 에 decel 로 선다. 사람까지 최소 거리 [m]
double walkAndStop(double v_walk, double t_stop, double decel, double* max_robot_decel = nullptr,
                   double r0 = 1.3, double t_walk = 0.5) {
  Harness h(yamlLikeParams());
  double xp = r0;
  double xr = 0.0;
  double vp = 0.0;
  h.target(xp - xr, 0.0);
  h.start();
  double r_min = 10.0;
  double v_prev = 0.0;
  double dec_max = 0.0;
  for (int i = 0; i < 400; ++i) {
    const double T = i * kDt;
    if (T < t_walk) {
      vp = 0.0;
    } else if (T < t_stop) {
      vp = std::min(v_walk, vp + 1.5 * kDt);
    } else {
      vp = std::max(0.0, vp - decel * kDt);
    }
    xp += vp * kDt;
    h.target(xp - xr, 0.0, TargetStatus::kLocked, vp);
    const auto o = h.step();
    xr += o.v * kDt;
    r_min = std::min(r_min, xp - xr);
    dec_max = std::max(dec_max, (v_prev - o.v) / kDt);
    v_prev = o.v;
  }
  if (max_robot_decel != nullptr) {
    *max_robot_decel = dec_max;
  }
  return r_min;
}
}  // namespace

TEST(Core, PersonStopsWhileRobotCatchesUpKeepsMargin) {
  // 2026-11-13 실차 E2: 따라잡느라 가속하던 중 사람이 서면 편안 S 곡선만으로는 0.54 m 까지 다가갔다.
  // 가속을 미리 멈추고(1단), 그래도 모자라면 단호 S 곡선(2단)으로 stop_distance + margin/2 (0.65 m) 밖에서 선다
  const ControllerParams p = yamlLikeParams();
  const double keep = p.law.stop_distance + 0.5 * p.law.approach_margin;
  for (double t_stop : {1.5, 2.0, 2.5, 3.0, 4.0, 6.0}) {
    for (double v_walk : {0.8, 1.2}) {
      const double r_min = walkAndStop(v_walk, t_stop, 3.0);
      std::printf("walk %.1f m/s, stop at %.1f s -> min %.2f m\n", v_walk, t_stop, r_min);
      EXPECT_GT(r_min, keep - 0.03) << "walk " << v_walk << " stop " << t_stop;
    }
  }
}

TEST(Core, PersonStopsWhileRobotClosesLargeGapKeepsMargin) {
  // 2026-11-13 실차 E2 14 s 장면: 사람이 2 m 앞에서 천천히(0.3~0.5 m/s) 걷고 로봇은 간격을 좁히려 서 있다가
  // 가속하던 중 사람이 섰다 → 0.54 m. 가속 중인 것까지 넣은 정지 예측으로 0.65 m 밖에서 선다
  const ControllerParams p = yamlLikeParams();
  const double keep = p.law.stop_distance + 0.5 * p.law.approach_margin;
  for (double t_stop : {1.0, 1.5, 2.0, 2.5, 3.0}) {
    for (double v_walk : {0.3, 0.5}) {
      const double r_min = walkAndStop(v_walk, t_stop, 3.0, nullptr, 2.0, 0.0);
      std::printf("gap 2.0 m, walk %.1f m/s, stop at %.1f s -> min %.2f m\n", v_walk, t_stop, r_min);
      EXPECT_GT(r_min, keep - 0.03) << "walk " << v_walk << " stop " << t_stop;
    }
  }
}

TEST(Core, SteadyWalkUsesOnlyComfortDeceleration) {
  // 보통 걸음(1.0 m/s)을 이어서 따라가는 동안 단호 감속은 쓰지 않는다 (사람이 늦게 설 때만)
  const ControllerParams p = yamlLikeParams();
  double dec = 0.0;
  walkAndStop(1.0, 100.0, 3.0, &dec);  // 20 s 동안 서지 않는다
  EXPECT_LE(dec, p.limiter.max_linear_decel + 0.02);
}

namespace {
// 2차원 걷기 (odom): 사람이 꺾은선 path 를 speed 로 걷고, 로봇은 명령을 그대로 적분한다. 장애물은 원 기둥들.
// 로봇이 기둥 표면에 가장 가까이 간 거리, 마지막 로봇 위치, 발자취 조향 주기 수를 돌려준다
struct Walk2D {
  double min_clear{1e9};
  double x{0.0};
  double y{0.0};
  int trail_cycles{0};
  double min_range{1e9};
};
Walk2D walk2D(const std::vector<Vec2>& path, double speed, const std::vector<std::pair<Vec2, double>>& pillars,
              double seconds) {
  Harness h(yamlLikeParams());
  Walk2D r;
  double rx = 0.0, ry = 0.0, th = 0.0;
  Vec2 pp = path.front();
  std::size_t seg = 0;
  auto toBase = [&](const Vec2& q) {
    const double dx = q.x - rx, dy = q.y - ry;
    return Vec2{std::cos(th) * dx + std::sin(th) * dy, -std::sin(th) * dx + std::cos(th) * dy};
  };
  std::vector<Vec2> pts;
  for (const auto& pl : pillars) {
    for (int k = 0; k < 24; ++k) {
      const double a = 2.0 * M_PI * k / 24.0;
      pts.push_back({pl.first.x + pl.second * std::cos(a), pl.first.y + pl.second * std::sin(a)});
    }
  }
  h.in.odom_pose_ok = true;
  h.target(toBase(pp).x, toBase(pp).y);
  h.start();
  const int n = static_cast<int>(seconds / kDt);
  for (int i = 0; i < n; ++i) {
    const double T = i * kDt;
    Vec2 vel{0.0, 0.0};
    if (T > 0.5 && seg + 1 < path.size()) {
      double step = speed * kDt;
      while (step > 0.0 && seg + 1 < path.size()) {
        const Vec2 d{path[seg + 1].x - pp.x, path[seg + 1].y - pp.y};
        const double L = norm(d);
        if (L <= step) {
          pp = path[seg + 1];
          ++seg;
          step -= L;
        } else {
          pp = {pp.x + d.x / L * step, pp.y + d.y / L * step};
          vel = {d.x / L * speed, d.y / L * speed};
          step = 0.0;
        }
      }
    }
    h.scan.clear();
    for (const auto& q : pts) {
      h.scan.push_back(toBase(q));
    }
    const Vec2 pb = toBase(pp);
    h.target(pb.x, pb.y, TargetStatus::kLocked, std::cos(th) * vel.x + std::sin(th) * vel.y);
    h.in.target.velocity = {std::cos(th) * vel.x + std::sin(th) * vel.y, -std::sin(th) * vel.x + std::cos(th) * vel.y};
    h.in.odom_pose = Pose2D{rx, ry, th};
    const auto o = h.step();
    if (o.steer_mode == SteerMode::kTrail) {
      ++r.trail_cycles;
    }
    rx += o.v * std::cos(th) * kDt;
    ry += o.v * std::sin(th) * kDt;
    th += o.w * kDt;
    for (const auto& pl : pillars) {
      r.min_clear = std::min(r.min_clear, std::hypot(rx - pl.first.x, ry - pl.first.y) - pl.second - 0.22);
    }
    r.min_range = std::min(r.min_range, std::hypot(pp.x - rx, pp.y - ry));
  }
  r.x = rx;
  r.y = ry;
  return r;
}
}  // namespace

TEST(Core, FollowsPersonTrailAroundPillarInsteadOfWaiting) {
  // 2026-11-13 실차 E2: 사람이 기둥을 돌아가면 사람에게 곧장 가는 직선이 기둥에 막혀 로봇이 기다리기만 했다.
  // 발자취 따라가기: 직선이 막히면 사람이 지나간 길로 돌아가서 다시 따라간다
  const std::vector<Vec2> path = {{1.3, 0.0}, {1.7, 0.65}, {2.7, 0.65}, {3.3, 0.0}, {6.0, 0.0}};
  const auto r = walk2D(path, 0.8, {{{2.2, 0.0}, 0.15}}, 25.0);
  std::printf("pillar detour: robot end (%.2f, %.2f), min clearance %.2f m, trail cycles %d, min range %.2f\n",
              r.x, r.y, r.min_clear, r.trail_cycles, r.min_range);
  EXPECT_GT(r.trail_cycles, 0);
  EXPECT_GT(r.x, 3.5);            // 기둥(2.2 m)을 지나 사람 쪽으로 갔다
  EXPECT_GT(r.min_clear, 0.05);   // 기둥에 닿지 않는다
  EXPECT_GT(r.min_range, 0.5 - 0.05);
}

TEST(Core, OpenFloorStaysDirect) {
  // 막힌 것이 없으면 발자취를 쓰지 않는다 (예전과 같은 직선 추종)
  const std::vector<Vec2> path = {{1.3, 0.0}, {2.0, 0.8}, {5.0, 0.8}};
  const auto r = walk2D(path, 0.8, {}, 15.0);
  EXPECT_EQ(r.trail_cycles, 0);
  EXPECT_GT(r.x, 3.0);
}

TEST(Core, ApproachingPersonNeverMakesRobotAdvanceInsideStopDistance) {
  Harness h;
  h.cruise(2.0);
  // 사람이 1 m/s 로 다가오면: 제동 상한으로 미리 느려지고, 0.8 m 안에서는 단호 정지로 곧 선다
  double v_prev = 1.0;
  for (double r = 2.0; r > 0.4; r -= 0.05) {
    h.target(r, 0.0, TargetStatus::kLocked, -1.0);
    const auto o = h.step();
    EXPECT_GE(o.v, 0.0);
    if (r <= 0.8) {
      EXPECT_LE(o.v, v_prev + 1e-9);
      EXPECT_LE(o.v, 0.15);
    }
    if (r <= 0.6) {
      EXPECT_EQ(o.v, 0.0);
    }
    v_prev = o.v;
  }
}

namespace {
// 야간 기본값의 선속도 제한 (jerk 0.6: 리뷰가 지적한 지나침이 생기는 값)
ControllerParams nightLimiterParams() {
  ControllerParams p;
  p.limiter.max_linear_speed = 0.5;
  p.limiter.max_linear_accel = 0.5;
  p.limiter.max_linear_decel = 0.8;
  p.limiter.max_linear_jerk = 0.6;
  p.limiter.firm_linear_jerk = 15.0;
  p.law.max_linear_speed = 0.5;
  return p;
}
}  // namespace

TEST(Core, SuddenSafetyCapDoesNotOvershoot) {
  // 리뷰 2026-11-13: 가속 중 카메라 이상(0.3 m/s 상한)이 걸리면 상한을 지나치지 않고 바로 줄인다
  Harness h(nightLimiterParams());
  h.target(3.0, 0.0);
  h.start();
  for (int i = 0; i < 200 && !(h.last.v > 0.32 && h.core.limiter().linearAccel() > 0.3); ++i)
    h.step();
  ASSERT_GT(h.core.limiter().linearAccel(), 0.3);
  const double v0 = h.last.v;
  h.in.sensors.camera_ok = false;
  double v_max = v0;
  double t_reach = -1.0;
  for (int i = 0; i < 40; ++i) {
    h.step();
    v_max = std::max(v_max, h.last.v);
    if (t_reach < 0.0 && h.last.v <= 0.3 + 1e-6)
      t_reach = (i + 1) * kDt;
  }
  EXPECT_EQ(h.last.state, FsmState::kFollowing);
  EXPECT_LE(v_max, v0 + 0.03);
  ASSERT_GT(t_reach, 0.0);
  EXPECT_LE(t_reach, 0.6);
}

TEST(Core, TurnInPlaceRequestChecksRotationClearanceWhileSlowing) {
  // 리뷰 2026-11-13: 사람이 옆으로 비켜서면(방위 > 45°) 걸러진 선속도가 줄어드는 동안에도 회전 여유 검사를 건다
  Harness h(nightLimiterParams());
  h.cruise();
  h.scan = {{-0.1, -0.27}};  // 중심에서 0.29 m (뒤 오른쪽: 전진 통로는 막지 않는다)
  h.target(0.5, 1.4);        // 왼쪽 약 70°
  bool blocked = false;
  for (int i = 0; i < 6 && !blocked; ++i) {
    h.step();
    blocked = (h.last.state == FsmState::kWaiting) &&
              (h.last.stop_reasons & stop_reason::kRotationBlocked);
  }
  EXPECT_TRUE(blocked);
  // 장애물이 없으면: 선속도는 끊지 않고(부드럽게) 줄어 몇 초 안에 제자리 회전이 된다
  Harness g(nightLimiterParams());
  g.cruise();
  g.target(0.5, 1.4);
  double t = 0.0;
  double prev = g.last.v;
  while (g.last.v > 0.01 && t < 5.0) {
    g.step();
    t += kDt;
    EXPECT_LE(g.last.v, prev + 1e-9);
    prev = g.last.v;
  }
  EXPECT_LE(t, 3.5);
  EXPECT_EQ(g.last.state, FsmState::kFollowing);
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

TEST(Core, TooCloseStopsFirmlyInWaitingToo) {
  // M3: 모호(WAITING) 중 사람이 0.75 m 로 다가오면 편안 감속이 아닌 단호 정지 (2026-11-13: 즉시 0 → 단호 S 곡선)
  Harness h;
  h.cruise();
  h.in.target.ambiguous = true;
  auto o = h.step();
  ASSERT_EQ(o.state, FsmState::kWaiting);
  ASSERT_GT(o.v, 0.3);  // 아직 감속 중
  const double v_amb = o.v;
  h.target(0.75, 0.0, TargetStatus::kLocked, -0.5);
  h.in.target.ambiguous = true;
  o = h.step();
  EXPECT_EQ(o.state, FsmState::kWaiting);
  EXPECT_TRUE(o.stop_reasons & stop_reason::kTooClose);
  h.expectFirmStop(o, v_amb, h.firmStopBound(v_amb), "too close in waiting");
  // 가림(측정 없는 시간 0.7 s 초과)으로 WAITING 이어도 예측 위치가 가까우면 즉시 0
  Harness g;
  g.cruise();
  g.in.target.status = TargetStatus::kOccluded;
  g.in.target.occluded_time = 0.8;
  o = g.step();
  ASSERT_EQ(o.state, FsmState::kWaiting);
  ASSERT_GT(o.v, 0.3);
  const double v_occ = o.v;
  g.in.target.position = {0.7, 0.05};
  o = g.step();
  EXPECT_TRUE(o.stop_reasons & stop_reason::kTooClose);
  g.expectFirmStop(o, v_occ, g.firmStopBound(v_occ), "too close occluded");
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
    EXPECT_TRUE(o.stop_reasons & stop_reason::kSensorTimeout);
    h.expectFirmStop(o, 0.5, h.firmStopBound(0.5), "lifted/fall_down stale");
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
  EXPECT_NEAR(o.depth_obstacle_distance, 0.55, 1e-9);
  h.expectFirmSlowdownToCap(o, 0.5, h.firmStopBound(0.5), "degraded depth points");  // depth 점 (0.156 + 0.5 안)
  Harness g;
  g.cruise();
  g.in.sensors.camera_ok = false;
  g.in.sensors.depth_points_ok = false;
  g.depth = {{0.22 + 0.55, 0.0}};  // 점을 못 쓰면 무시 + 상한만
  const auto q = g.step();
  EXPECT_GT(q.v, 0.0);
  EXPECT_NEAR(q.v_limit, 0.3, 1e-9);
}
