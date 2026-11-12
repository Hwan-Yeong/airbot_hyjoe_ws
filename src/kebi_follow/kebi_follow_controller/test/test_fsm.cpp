// 상태기계: SPEC 5절 전이 전부
#include <gtest/gtest.h>

#include "kebi_follow_controller/follow_fsm.hpp"

using namespace kebi_follow;

namespace {

FsmInputs locked() {
  FsmInputs in;
  in.target_locked = true;
  return in;
}
FsmInputs waitFor(uint32_t reasons) {
  FsmInputs in;
  in.wait_reasons = reasons;
  return in;
}

// 시각 t 에 FOLLOWING 인 FSM
FollowFsm following(double t = 0.0) {
  FollowFsm f;
  f.start(t);
  f.update(t, locked());
  return f;
}

int drainBurst(FollowFsm* f) {
  int n = 0;
  while (f->consumeZeroBurst())
    ++n;
  return n;
}

}  // namespace

TEST(Fsm, StartsIdleSilent) {
  FollowFsm f;
  EXPECT_EQ(f.state(), FsmState::kIdle);
  EXPECT_EQ(f.zeroBurstRemaining(), 0);
  EXPECT_FALSE(f.update(1.0, locked()));  // IDLE 에서는 락이 와도 그대로
  EXPECT_EQ(f.state(), FsmState::kIdle);
}

TEST(Fsm, IdleStopIsNoop) {
  FollowFsm f;
  EXPECT_FALSE(f.stop(0.0));
  // 시작·IDLE 중 STOP 에는 0 을 내지 않는다
  EXPECT_EQ(f.zeroBurstRemaining(), 0);
}

TEST(Fsm, IdleStartToAcquiring) {
  FollowFsm f;
  EXPECT_TRUE(f.start(1.0));
  EXPECT_EQ(f.state(), FsmState::kAcquiring);
  EXPECT_EQ(f.zeroBurstRemaining(), 0);
}

TEST(Fsm, AcquiringLockToFollowing) {
  FollowFsm f;
  f.start(0.0);
  EXPECT_FALSE(f.update(0.5, FsmInputs{}));
  EXPECT_TRUE(f.update(1.0, locked()));
  EXPECT_EQ(f.state(), FsmState::kFollowing);
}

TEST(Fsm, AcquiringTimeoutToEndedWithBurst) {
  FollowFsm f;
  f.start(0.0);
  EXPECT_FALSE(f.update(29.9, FsmInputs{}));
  EXPECT_TRUE(f.update(30.0, FsmInputs{}));
  EXPECT_EQ(f.state(), FsmState::kEnded);
  EXPECT_EQ(drainBurst(&f), 3);
}

TEST(Fsm, RepeatedStartIgnoredWhileActive) {
  FollowFsm f;
  f.start(0.0);
  EXPECT_FALSE(f.start(20.0));  // ACQUIRING 타이머를 되돌리지 않는다
  EXPECT_TRUE(f.update(30.0, FsmInputs{}));
  EXPECT_EQ(f.state(), FsmState::kEnded);

  FollowFsm g = following();
  EXPECT_FALSE(g.start(1.0));
  EXPECT_EQ(g.state(), FsmState::kFollowing);
  g.update(2.0, waitFor(stop_reason::kNoTarget));
  EXPECT_FALSE(g.start(3.0));
  EXPECT_EQ(g.state(), FsmState::kWaiting);
}

TEST(Fsm, StopFromEveryActiveStateToIdleWithBurst) {
  for (int which = 0; which < 4; ++which) {
    FollowFsm f = following();
    if (which == 0) {
      f = FollowFsm();
      f.start(0.0);  // ACQUIRING
    } else if (which == 2) {
      f.update(1.0, waitFor(stop_reason::kAmbiguous));  // WAITING
    } else if (which == 3) {
      f.halt(1.0, stop_reason::kCliff);  // HALTED
    }
    ASSERT_TRUE(isActive(f.state()));
    EXPECT_TRUE(f.stop(2.0));
    EXPECT_EQ(f.state(), FsmState::kIdle);
    EXPECT_EQ(drainBurst(&f), 3) << "case " << which;
  }
}

TEST(Fsm, FollowingToWaitingOnAnyReason) {
  for (uint32_t r : {stop_reason::kNoTarget, stop_reason::kAmbiguous,
                     stop_reason::kSensorTimeout, stop_reason::kRotationBlocked,
                     stop_reason::kRotationLimit}) {
    FollowFsm f = following();
    EXPECT_FALSE(f.update(1.0, FsmInputs{}));
    EXPECT_TRUE(f.update(1.05, waitFor(r)));
    EXPECT_EQ(f.state(), FsmState::kWaiting);
  }
}

TEST(Fsm, WaitingResumesAfterConfirm) {
  FollowFsm f = following();
  f.update(1.0, waitFor(stop_reason::kNoTarget));
  EXPECT_FALSE(f.update(2.0, FsmInputs{}));  // 해소 시작
  EXPECT_FALSE(f.update(2.4, FsmInputs{}));
  // 다시 생기면 처음부터
  EXPECT_FALSE(f.update(2.45, waitFor(stop_reason::kAmbiguous)));
  EXPECT_FALSE(f.update(2.5, FsmInputs{}));
  EXPECT_FALSE(f.update(2.95, FsmInputs{}));
  EXPECT_TRUE(f.update(3.0, FsmInputs{}));
  EXPECT_EQ(f.state(), FsmState::kFollowing);
}

TEST(Fsm, WaitingTimeoutToEnded) {
  FollowFsm f = following();
  f.update(1.0, waitFor(stop_reason::kNoTarget));
  EXPECT_FALSE(f.update(30.9, waitFor(stop_reason::kNoTarget)));
  EXPECT_TRUE(f.update(31.0, waitFor(stop_reason::kNoTarget)));
  EXPECT_EQ(f.state(), FsmState::kEnded);
  EXPECT_EQ(drainBurst(&f), 3);
}

TEST(Fsm, HaltFromActiveStates) {
  FollowFsm a;
  a.start(0.0);
  EXPECT_TRUE(a.halt(0.1, stop_reason::kRobotState));
  EXPECT_EQ(a.state(), FsmState::kHalted);

  FollowFsm b = following();
  EXPECT_TRUE(b.halt(1.0, stop_reason::kCollision));
  EXPECT_EQ(b.haltReasons(), stop_reason::kCollision);
  EXPECT_FALSE(b.halt(1.1, stop_reason::kCliff));  // 이미 HALTED: 사유만 더함
  EXPECT_EQ(b.haltReasons(), stop_reason::kCollision | stop_reason::kCliff);

  FollowFsm c = following();
  c.update(1.0, waitFor(stop_reason::kNoTarget));
  EXPECT_TRUE(c.halt(1.5, stop_reason::kCliff));
  EXPECT_EQ(c.zeroBurstRemaining(), 0);  // HALTED 는 활성 상태: 주기 발행
}

TEST(Fsm, HaltIgnoredWhenInactive) {
  FollowFsm f;
  EXPECT_FALSE(f.halt(0.0, stop_reason::kCliff));
  EXPECT_EQ(f.state(), FsmState::kIdle);
  f.start(0.0);
  f.update(30.0, FsmInputs{});
  ASSERT_EQ(f.state(), FsmState::kEnded);
  EXPECT_FALSE(f.halt(31.0, stop_reason::kCliff));
  EXPECT_EQ(f.state(), FsmState::kEnded);
}

TEST(Fsm, HaltedIsLatched) {
  FollowFsm f = following();
  f.halt(1.0, stop_reason::kCliff);
  // 조건이 사라지고 락이 와도 그대로
  EXPECT_FALSE(f.update(10.0, locked()));
  EXPECT_FALSE(f.update(100.0, FsmInputs{}));
  EXPECT_EQ(f.state(), FsmState::kHalted);
}

TEST(Fsm, HaltedStartHoldoff) {
  FollowFsm f = following();
  f.halt(1.0, stop_reason::kCliff);
  // state_manager 의 START 반복으로 바로 풀리지 않는다
  EXPECT_FALSE(f.start(1.5));
  EXPECT_EQ(f.state(), FsmState::kHalted);
  f.halt(1.8, stop_reason::kCliff);  // 사유가 계속 오면 홀드오프 연장
  EXPECT_FALSE(f.start(2.5));
  EXPECT_TRUE(f.start(2.85));
  EXPECT_EQ(f.state(), FsmState::kAcquiring);  // 재획득부터
  EXPECT_EQ(f.haltReasons(), 0u);
}

TEST(Fsm, EndedStartAndStop) {
  FollowFsm f;
  f.start(0.0);
  f.update(30.0, FsmInputs{});
  ASSERT_EQ(f.state(), FsmState::kEnded);
  drainBurst(&f);
  EXPECT_TRUE(f.stop(31.0));
  EXPECT_EQ(f.state(), FsmState::kIdle);
  EXPECT_EQ(f.zeroBurstRemaining(), 0);  // ENDED → IDLE 은 0 을 더 내지 않는다

  FollowFsm g;
  g.start(0.0);
  g.update(30.0, FsmInputs{});
  drainBurst(&g);
  EXPECT_TRUE(g.start(40.0));
  EXPECT_EQ(g.state(), FsmState::kAcquiring);
}

TEST(Fsm, ModeMapping) {
  EXPECT_EQ(modeOf(FsmState::kIdle), ModeValue::kOff);
  EXPECT_EQ(modeOf(FsmState::kAcquiring), ModeValue::kAcquire);
  EXPECT_EQ(modeOf(FsmState::kFollowing), ModeValue::kTrack);
  EXPECT_EQ(modeOf(FsmState::kWaiting), ModeValue::kTrack);
  EXPECT_EQ(modeOf(FsmState::kHalted), ModeValue::kTrack);
  EXPECT_EQ(modeOf(FsmState::kEnded), ModeValue::kOff);
}
