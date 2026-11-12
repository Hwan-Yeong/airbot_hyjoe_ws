// 추종 상태기계 (SPEC 5절). ROS 의존 없음.
#pragma once

#include <cstdint>

#include "kebi_follow_controller/types.hpp"

namespace kebi_follow {

struct FsmParams {
  double acquire_timeout_s{30.0};  // ACQUIRING → ENDED
  double resume_confirm_s{0.5};    // WAITING 해소 유지 시간 → FOLLOWING
  double lost_end_timeout_s{30.0};  // WAITING 지속 → ENDED
  // 마지막 HALT 사유 후 이 시간 안의 START 무시
  double halt_restart_holdoff_s{1.0};
  int zero_burst_count{3};  // IDLE·ENDED 진입 시 0 발행 횟수
};

struct FsmInputs {
  // ACQUIRING → FOLLOWING: 신선한 LOCKED, 모호 아님
  bool target_locked{false};
  uint32_t wait_reasons{0};  // FOLLOWING → WAITING 사유 (0 이면 해소)
};

class FollowFsm {
 public:
  explicit FollowFsm(const FsmParams& p = FsmParams{}) : p_(p) {}

  // 이벤트 처리. 상태가 바뀌면 true
  bool start(double now);
  bool stop(double now);
  bool halt(double now, uint32_t reasons);
  // 주기 갱신. 상태가 바뀌면 true
  bool update(double now, const FsmInputs& in);

  FsmState state() const { return state_; }
  double stateSince() const { return since_; }
  uint32_t haltReasons() const { return halt_reasons_; }

  // IDLE·ENDED 진입 직후 보내야 할 0 명령. 남아 있으면 1 줄이고 true
  bool consumeZeroBurst();
  int zeroBurstRemaining() const { return zero_burst_; }

 private:
  void enter(FsmState s, double now);

  FsmParams p_;
  FsmState state_{FsmState::kIdle};
  double since_{0.0};
  double clear_since_{-1.0};  // WAITING 사유가 사라진 시각 (<0 이면 아직)
  double last_halt_event_{-kInf};
  uint32_t halt_reasons_{0};
  int zero_burst_{0};
};

}  // namespace kebi_follow
