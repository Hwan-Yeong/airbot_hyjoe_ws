#include "kebi_follow_controller/follow_fsm.hpp"

namespace kebi_follow {

void FollowFsm::enter(FsmState s, double now) {
  const FsmState prev = state_;
  state_ = s;
  since_ = now;
  clear_since_ = -1.0;
  if (s != FsmState::kHalted) {
    halt_reasons_ = 0;
  }
  // 활성 상태는 매 주기 발행한다
  if (isActive(s)) {
    zero_burst_ = 0;
    // 움직일 수 있던 상태에서 나갈 때만
  } else if (isActive(prev)) {
    zero_burst_ = p_.zero_burst_count;
  }
}

bool FollowFsm::start(double now) {
  switch (state_) {
    case FsmState::kIdle:
    case FsmState::kEnded:
      enter(FsmState::kAcquiring, now);
      return true;
    case FsmState::kHalted:
      // state_manager 가 START 를 반복해 보내도 HALT 직후 바로 풀리지 않게 한다
      if (now - last_halt_event_ < p_.halt_restart_holdoff_s) {
        return false;
      }
      enter(FsmState::kAcquiring, now);
      return true;
    default:
      return false;  // 이미 활성: 무시
  }
}

bool FollowFsm::stop(double now) {
  if (state_ == FsmState::kIdle) {
    return false;
  }
  enter(FsmState::kIdle, now);
  return true;
}

bool FollowFsm::halt(double now, uint32_t reasons) {
  if (!isActive(state_) || reasons == 0) {
    return false;
  }
  last_halt_event_ = now;
  if (state_ == FsmState::kHalted) {
    halt_reasons_ |= reasons;
    return false;
  }
  enter(FsmState::kHalted, now);
  halt_reasons_ = reasons;
  return true;
}

bool FollowFsm::update(double now, const FsmInputs& in) {
  switch (state_) {
    case FsmState::kAcquiring:
      if (in.target_locked) {
        enter(FsmState::kFollowing, now);
        return true;
      }
      if (now - since_ >= p_.acquire_timeout_s) {
        enter(FsmState::kEnded, now);
        return true;
      }
      return false;
    case FsmState::kFollowing:
      if (in.wait_reasons != 0) {
        enter(FsmState::kWaiting, now);
        return true;
      }
      return false;
    case FsmState::kWaiting:
      if (now - since_ >= p_.lost_end_timeout_s) {
        enter(FsmState::kEnded, now);
        return true;
      }
      if (in.wait_reasons != 0) {
        clear_since_ = -1.0;
        return false;
      }
      if (clear_since_ < 0.0) {
        clear_since_ = now;
      }
      if (now - clear_since_ >= p_.resume_confirm_s) {
        enter(FsmState::kFollowing, now);
        return true;
      }
      return false;
    default:
      return false;
  }
}

bool FollowFsm::consumeZeroBurst() {
  if (zero_burst_ <= 0) {
    return false;
  }
  --zero_burst_;
  return true;
}

}  // namespace kebi_follow
