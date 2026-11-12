// 타깃 획득·유지·상실·재획득·모호 판정 (SPEC 6.2). mode 는 follow_controller 가 정한다.
#pragma once

#include <cstdint>

#include "kebi_follow_tracker/geometry.hpp"
#include "kebi_follow_tracker/params.hpp"
#include "kebi_follow_tracker/people_tracker.hpp"

namespace kebi_follow {
namespace tracker {

// 값은 kebi_follow_msgs/FollowMode 와 같다
enum class Mode : uint8_t {
  OFF = 0,
  ACQUIRE = 1,
  TRACK = 2,
};

// 값은 kebi_follow_msgs/FollowTarget.STATUS_* 와 같다
enum class TargetStatus : uint8_t {
  NONE = 0,
  CANDIDATE = 1,
  LOCKED = 2,
  OCCLUDED = 3,
  LOST = 4,
};

struct TargetOutput {
  TargetStatus status{TargetStatus::NONE};
  int track_id{-1};
  Vec2 position;  // odom
  Vec2 velocity;  // odom
  double position_std{0.0};
  double occluded_time{0.0};
  bool camera_confirmed{false};
  bool ambiguous{false};
  // 디버그·시각화
  int zone_candidates{0};  // 획득·재획득 영역의 조건 만족 후보 수
  int competitors{0};  // 같은 영역(+여유)의 후보가 아닌 사람 트랙 수
  double hold_s{0.0};  // 획득·재획득 조건 유지 시간
  bool has_lost_position{false};
  Vec2 lost_position;  // odom, LOST 직전 예측 위치
  Vec2
      lost_seen_position;  // odom, 마지막 관측 위치 (재획득 영역은 이 둘을 잇는 선분 주변)
};

class TargetManager {
 public:
  explicit TargetManager(const TrackerParams& params);

  void setMode(Mode mode);
  Mode mode() const { return mode_; }
  // scan 처리 직후 호출. robot_pose: odom <- base_link (scan 시각)
  const TargetOutput& update(double t, const Pose2& robot_pose,
                             const PeopleTracker& tracker);
  const TargetOutput& output() const { return out_; }
  int lockedId() const { return phase_ == Phase::LOCKED ? target_id_ : -1; }

 private:
  enum class Phase : uint8_t {
    IDLE,
    ACQUIRING,
    LOCKED,
    LOST,
  };

  void clearTarget();
  void runAcquire(double t, const Pose2& robot_pose,
                  const PeopleTracker& tracker);
  void runLocked(double t, const Pose2& robot_pose,
                 const PeopleTracker& tracker);
  void runReacquire(double t, const Pose2& robot_pose,
                    const PeopleTracker& tracker);
  void enterLost(double t, const Pose2& robot_pose,
                 const PeopleTracker& tracker);
  void fillFromTrack(double t, const PersonTrack& tr);
  bool isPersonLike(double t, const PersonTrack& tr) const;
  bool isLockable(double t, const PersonTrack& tr) const;
  bool holdCandidate(double t, int id, double confirm_s);
  bool computeAmbiguity(double t, const PersonTrack& target,
                        const PeopleTracker& tracker) const;

  TrackerParams params_;
  Mode mode_{Mode::OFF};
  Phase phase_{Phase::IDLE};
  int target_id_{-1};
  int hold_id_{-1};
  double hold_start_{0.0};
  double reacquireDistance(const Vec2& p) const;
  bool cameraLostTarget(double t, const Pose2& robot_pose,
                        const PersonTrack& tr, const PeopleTracker& tracker);

  // 락 중 마지막으로 알던 타깃 상태 (트랙이 지워져도 LOST 위치로 쓴다)
  Vec2 known_position_;
  Vec2 known_seen_position_;  // 마지막 측정 갱신 직후 위치
  double known_std_{0.0};
  double known_update_{0.0};
  // 카메라 확인 영역 안에서 카메라 확인 없이 지낸 시작 시각 (-1: 아님)
  double unseen_since_{-1.0};
  double ambiguous_since_{-1.0};  // 모호가 시작된 시각 (-1: 모호 아님)
  Vec2
      clear_position_;  // 모호가 아닐 때 마지막 타깃 위치 (모호로 LOST 될 때 재획득 기준)
  bool lost_by_ambiguity_{false};
  // LOST 상태
  int lost_id_{-1};
  Vec2 lost_position_;
  Vec2 lost_seen_position_;
  double lost_std_{0.0};
  double lost_last_update_{0.0};
  TargetOutput out_;
};

}  // namespace tracker
}  // namespace kebi_follow
