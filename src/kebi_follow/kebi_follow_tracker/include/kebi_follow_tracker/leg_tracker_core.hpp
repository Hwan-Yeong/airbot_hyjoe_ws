// leg_tracker 처리 흐름 (ROS 비의존): scan → 다리 검출 → 다중 추적 → 카메라 융합 → 타깃 관리
#pragma once

#include <cstddef>
#include <vector>

#include "kebi_follow_tracker/geometry.hpp"
#include "kebi_follow_tracker/leg_detector.hpp"
#include "kebi_follow_tracker/params.hpp"
#include "kebi_follow_tracker/people_tracker.hpp"
#include "kebi_follow_tracker/target_manager.hpp"

namespace kebi_follow {
namespace tracker {

// 카메라 검출 1개 (CamLeg 에서 위치가 있는 것만)
struct CameraDetection {
  Vec2 position;  // base_link, 검출 stamp 시각
  double score{0.0};
  int group{-1};  // 같은 사람 상자에서 나온 검출 묶음 번호 (-1: 혼자). 묶음은 트랙 하나만 확인한다
  bool primary{false};  // 묶음의 대표 (사람 검출). 묶음 안에서 이 검출의 연관을 먼저 쓴다
};

class LegTrackerCore {
 public:
  explicit LegTrackerCore(const TrackerParams& params);

  void setMode(Mode mode) { target_.setMode(mode); }
  // 실행 중 조정 (tunables.hpp 의 값만 바뀐다고 본다: 다리 검출·칼만 값은 기동 때 것을 쓴다)
  void updateParams(const TrackerParams& params);
  Mode mode() const { return target_.mode(); }
  void reset();

  // stamp: scan 시각 [s], sensor_in_base: base_link <- scan 프레임, robot_pose: odom <- base_link
  const TargetOutput& processScan(double stamp, const ScanView& scan,
                                  const Pose2& sensor_in_base,
                                  const Pose2& robot_pose);
  // stamp: 검출 시각 (AMR 시간축), robot_pose: 그 시각의 odom <- base_link. 연관 수를 돌려준다.
  int processCamera(double stamp, const Pose2& robot_pose,
                    const std::vector<CameraDetection>& detections);

  const TrackerParams& params() const { return params_; }
  const LegDetector& detector() const { return detector_; }
  const PeopleTracker& tracker() const { return tracker_; }
  const TargetOutput& target() const { return target_.output(); }
  int lockedId() const { return target_.lockedId(); }
  double lastStamp() const { return last_stamp_; }
  const Pose2& lastRobotPose() const { return last_robot_pose_; }

 private:
  TrackerParams params_;
  LegDetector detector_;
  PeopleTracker tracker_;
  TargetManager target_;
  bool has_scan_{false};
  double last_stamp_{0.0};
  Pose2 last_robot_pose_;
  std::vector<Vec2> camera_odom_;
  std::vector<int> camera_group_;
  std::vector<char> camera_primary_;
};

}  // namespace tracker
}  // namespace kebi_follow
