#include "kebi_follow_tracker/leg_tracker_core.hpp"

namespace kebi_follow {
namespace tracker {

namespace {
constexpr std::size_t kReserveCameraDetections = 16;
}  // namespace

LegTrackerCore::LegTrackerCore(const TrackerParams& params)
    : params_(params),
      detector_(params.scan, params.leg),
      tracker_(params),
      target_(params) {
  camera_odom_.reserve(kReserveCameraDetections);
  camera_group_.reserve(kReserveCameraDetections);
  camera_primary_.reserve(kReserveCameraDetections);
}

void LegTrackerCore::reset() {
  tracker_.reset();
  const Mode mode = target_.mode();
  target_ = TargetManager(params_);
  target_.setMode(mode);
  has_scan_ = false;
}

const TargetOutput& LegTrackerCore::processScan(double stamp,
                                                const ScanView& scan,
                                                const Pose2& sensor_in_base,
                                                const Pose2& robot_pose) {
  detector_.detect(scan, sensor_in_base);
  tracker_.setProtectedTrack(
      target_
          .lockedId());  // 락 타깃은 오래 서 있어도 정지 물체 모델을 쓰지 않는다
  tracker_.update(stamp, robot_pose, detector_.segments());
  has_scan_ = true;
  last_stamp_ = stamp;
  last_robot_pose_ = robot_pose;
  return target_.update(stamp, robot_pose, tracker_);
}

int LegTrackerCore::processCamera(
    double stamp, const Pose2& robot_pose,
    const std::vector<CameraDetection>& detections) {
  if (!has_scan_ || stamp < last_stamp_ - params_.camera.max_age_s) {
    return 0;  // 추적 전이거나 너무 오래된 검출
  }
  tracker_.noteCameraFrame(stamp);  // 검출이 없는 프레임도 "카메라 살아 있음"
  camera_odom_.clear();
  camera_group_.clear();
  camera_primary_.clear();
  for (const auto& d : detections) {
    if (d.score >= params_.camera.min_score) {
      camera_odom_.push_back(transformPoint(robot_pose, d.position));
      camera_group_.push_back(d.group);
      camera_primary_.push_back(d.primary ? 1 : 0);
    }
  }
  return tracker_.associateCamera(stamp, camera_odom_, camera_group_, camera_primary_);
}

}  // namespace tracker
}  // namespace kebi_follow
