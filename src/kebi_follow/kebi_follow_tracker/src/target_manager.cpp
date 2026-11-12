#include "kebi_follow_tracker/target_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace kebi_follow {
namespace tracker {

namespace {
constexpr double kHoldEpsilonS = 1e-3;  // [s] 유지 시간 비교의 시각 오차 허용
// 미확정 트랙을 경쟁자로 세는 최소 관측 수 (한 번 스친 조각 제외)
constexpr int kMinTentativeHits = 2;
}  // namespace

TargetManager::TargetManager(const TrackerParams& params) : params_(params) {}

void TargetManager::clearTarget() {
  unseen_since_ = -1.0;
  ambiguous_since_ = -1.0;
  lost_by_ambiguity_ = false;
  target_id_ = -1;
  hold_id_ = -1;
  hold_start_ = 0.0;
  lost_id_ = -1;
  out_ = TargetOutput{};
}

void TargetManager::setMode(Mode mode) {
  if (mode == mode_) {
    return;
  }
  mode_ = mode;
  switch (mode) {
    case Mode::OFF:
      phase_ = Phase::IDLE;
      clearTarget();
      break;
    case Mode::ACQUIRE:
      // OFF 에서든 TRACK 에서든 ACQUIRE 로 오면 처음부터 다시 획득한다
      phase_ = Phase::ACQUIRING;
      clearTarget();
      break;
    case Mode::TRACK:
      // 락은 그대로 유지. 락 없이 TRACK 이 오면 따라갈 대상이 없다
      if (phase_ == Phase::ACQUIRING) {
        phase_ = Phase::IDLE;
        clearTarget();
      }
      break;
  }
}

bool TargetManager::isPersonLike(double t, const PersonTrack& tr) const {
  if (tr.stationary &&
      !tr.cameraConfirmed(t, params_.camera.confirm_window_s)) {
    return false;  // 움직인 적 없는 정지 트랙(가구 다리 쌍 등)은 카메라가 확인할 때만 사람으로 센다
  }
  if (!tr.confirmed && (tr.shapeEvidence() < params_.acquire.min_shape_score ||
                        tr.hits < kMinTentativeHits)) {
    return false;  // 막 생긴 트랙은 다리 쌍·덩어리로 두 번 이상 보였을 때만 사람으로 센다
  }
  return tr.personScore(t, params_.camera) >=
         params_.ambiguity.min_person_score;
}

bool TargetManager::isLockable(double t, const PersonTrack& tr) const {
  const AcquireParams& ap = params_.acquire;
  return tr.confirmed && tr.lockShape(t, params_) >= ap.min_shape_score &&
         (!ap.require_camera ||
          tr.cameraConfirmed(t, params_.camera.confirm_window_s)) &&
         t - tr.last_update_time <= ap.max_coast_s;
}

bool TargetManager::holdCandidate(double t, int id, double confirm_s) {
  if (hold_id_ != id) {
    hold_id_ = id;
    hold_start_ = t;
  }
  out_.hold_s = t - hold_start_;
  return out_.hold_s >= confirm_s - kHoldEpsilonS;
}

void TargetManager::fillFromTrack(double t, const PersonTrack& tr) {
  out_.track_id = tr.id;
  out_.position = tr.kf.position();
  out_.velocity = tr.kf.velocity();
  out_.position_std = tr.kf.positionStd();
  out_.occluded_time = t - tr.last_update_time;
  out_.camera_confirmed =
      tr.cameraConfirmed(t, params_.camera.confirm_window_s);
}

const TargetOutput& TargetManager::update(double t, const Pose2& robot_pose,
                                          const PeopleTracker& tracker) {
  switch (phase_) {
    case Phase::ACQUIRING:
      runAcquire(t, robot_pose, tracker);
      break;
    case Phase::LOCKED:
      runLocked(t, robot_pose, tracker);
      break;
    case Phase::LOST:
      runReacquire(t, robot_pose, tracker);
      break;
    case Phase::IDLE:
    default:
      out_ = TargetOutput{};
      break;
  }
  return out_;
}

// 획득: 영역(0.5~1.5 m, ±20°) 안의 사람 형상 + 카메라 확인 후보가 1명뿐인 상태가 confirm_s 유지
void TargetManager::runAcquire(double t, const Pose2& robot_pose,
                               const PeopleTracker& tracker) {
  const AcquireParams& ap = params_.acquire;
  const double max_bearing = ap.max_bearing_deg * kDegToRad;
  const double comp_bearing =
      (ap.max_bearing_deg + ap.competitor_margin_deg) * kDegToRad;
  const PersonTrack* best = nullptr;
  double best_range = std::numeric_limits<double>::infinity();
  int n_cand = 0;
  for (const auto& tr : tracker.tracks()) {
    const Vec2 pb = inverseTransformPoint(robot_pose, tr.kf.position());
    const double r = norm(pb);
    const double b = std::abs(bearing(pb));
    const bool in_zone =
        r >= ap.min_range && r <= ap.max_range && b <= max_bearing;
    if (in_zone && isLockable(t, tr)) {
      ++n_cand;
      if (r < best_range) {
        best_range = r;
        best = &tr;
      }
    }
  }
  // 후보 말고 경쟁 영역(획득 영역 + 여유)에 있는 다른 사람 트랙 수
  int n_others = 0;
  for (const auto& tr : tracker.tracks()) {
    if (&tr == best) {
      continue;
    }
    const Vec2 pb = inverseTransformPoint(robot_pose, tr.kf.position());
    const double r = norm(pb);
    const double b = std::abs(bearing(pb));
    const bool in_comp_zone = r >= ap.min_range - ap.competitor_margin &&
                              r <= ap.max_range + ap.competitor_margin &&
                              b <= comp_bearing;
    if (in_comp_zone && isPersonLike(t, tr)) {
      ++n_others;
    }
  }
  out_ = TargetOutput{};
  out_.zone_candidates = n_cand;
  out_.competitors = n_others;
  if (best == nullptr) {
    hold_id_ = -1;
    return;
  }
  fillFromTrack(t, *best);
  out_.status = TargetStatus::CANDIDATE;
  const bool unique = n_cand == 1 && n_others == 0;
  out_.ambiguous = !unique;
  if (!unique) {
    hold_id_ = -1;
    return;
  }
  if (holdCandidate(t, best->id, ap.confirm_s)) {
    target_id_ = best->id;
    known_seen_position_ = best->kf.position();
    clear_position_ = best->kf.position();
    ambiguous_since_ = -1.0;
    hold_id_ = -1;
    phase_ = Phase::LOCKED;
    runLocked(t, robot_pose, tracker);
  }
}

// 유지: 측정이 있으면 LOCKED, 없으면 OCCLUDED(예측). lost_s 초과나 불확실성 초과면 LOST
void TargetManager::runLocked(double t, const Pose2& robot_pose,
                              const PeopleTracker& tracker) {
  const PersonTrack* tr = tracker.findTrack(target_id_);
  if (tr != nullptr) {
    known_position_ = tr->kf.position();
    known_std_ = tr->kf.positionStd();
    known_update_ = tr->last_update_time;
    if (tr->updated) {
      known_seen_position_ = known_position_;
    }
  }
  if (tr == nullptr || t - tr->last_update_time > params_.target.lost_s ||
      known_std_ > params_.target.max_std ||
      cameraLostTarget(t, robot_pose, *tr, tracker)) {
    enterLost(t, robot_pose, tracker);
    return;
  }
  const bool ambiguous = computeAmbiguity(t, *tr, tracker);
  if (!ambiguous) {
    ambiguous_since_ = -1.0;
    clear_position_ = tr->kf.position();
  } else if (ambiguous_since_ < 0.0) {
    ambiguous_since_ = t;
  } else if (params_.ambiguity.lost_s > 0.0 &&
             t - ambiguous_since_ > params_.ambiguity.lost_s) {
    // 오래 모호하면 타깃이 다른 사람에게 끌려갔을 수 있다: 모호 직전 위치에서 다시 찾는다
    lost_by_ambiguity_ = true;
    enterLost(t, robot_pose, tracker);
    return;
  }
  out_ = TargetOutput{};
  fillFromTrack(t, *tr);
  out_.status = tr->updated ? TargetStatus::LOCKED : TargetStatus::OCCLUDED;
  out_.ambiguous = ambiguous;
}

// 카메라가 살아 있고 타깃이 카메라 확인 영역에 camera_lost_s 넘게 있었는데 카메라 확인이 없다
bool TargetManager::cameraLostTarget(double t, const Pose2& robot_pose,
                                     const PersonTrack& tr,
                                     const PeopleTracker& tracker) {
  const CameraParams& cp = params_.camera;
  const double limit = params_.target.camera_lost_s;
  const bool camera_alive = tracker.lastCameraFrame() >= 0.0 &&
                            t - tracker.lastCameraFrame() <= cp.alive_s;
  const Vec2 pb = inverseTransformPoint(robot_pose, tr.kf.position());
  const double r = norm(pb);
  const bool in_view =
      std::abs(bearing(pb)) <= cp.check_half_fov_deg * kDegToRad &&
      r >= cp.check_min_range && r <= cp.check_max_range;
  if (limit <= 0.0 || !camera_alive || !in_view ||
      tr.cameraConfirmed(t, cp.confirm_window_s)) {
    unseen_since_ = -1.0;
    return false;
  }
  if (unseen_since_ < 0.0) {
    unseen_since_ = t;
  }
  return t - unseen_since_ > limit;
}

void TargetManager::enterLost(double t, const Pose2& robot_pose,
                              const PeopleTracker& tracker) {
  hold_id_ = -1;
  unseen_since_ = -1.0;
  if (mode_ == Mode::ACQUIRE) {
    lost_by_ambiguity_ = false;
    ambiguous_since_ = -1.0;
    // 아직 획득 단계면 처음부터 다시 찾는다
    phase_ = Phase::ACQUIRING;
    target_id_ = -1;
    runAcquire(t, robot_pose, tracker);
    return;
  }
  phase_ = Phase::LOST;
  lost_id_ = target_id_;
  target_id_ = -1;
  lost_position_ = known_position_;
  lost_seen_position_ = params_.target.reacquire_from_last_seen
                            ? known_seen_position_
                            : known_position_;
  if (lost_by_ambiguity_) {
    lost_position_ = clear_position_;
    lost_seen_position_ = clear_position_;
  }
  lost_by_ambiguity_ = false;
  ambiguous_since_ = -1.0;
  lost_std_ = known_std_;
  lost_last_update_ = known_update_;
  runReacquire(t, robot_pose, tracker);
}

// 재획득: LOST 직전 예측 위치 reacquire_radius 안에서 같은 조건이 reacquire_confirm_s 유지
void TargetManager::runReacquire(double t, const Pose2& robot_pose,
                                 const PeopleTracker& tracker) {
  const double radius = params_.target.reacquire_radius;
  const double comp_radius = radius + params_.acquire.competitor_margin;
  const PersonTrack* best = nullptr;
  double best_dist = std::numeric_limits<double>::infinity();
  int n_cand = 0;
  for (const auto& tr : tracker.tracks()) {
    const double d = reacquireDistance(tr.kf.position());
    if (d <= radius && isLockable(t, tr)) {
      ++n_cand;
      if (d < best_dist) {
        best_dist = d;
        best = &tr;
      }
    }
  }
  int n_comp = 0;  // 후보 말고 영역(+여유) 안의 다른 사람 트랙
  for (const auto& tr : tracker.tracks()) {
    if (&tr != best && reacquireDistance(tr.kf.position()) <= comp_radius &&
        isPersonLike(t, tr)) {
      ++n_comp;
    }
  }
  out_ = TargetOutput{};
  out_.status = TargetStatus::LOST;
  out_.track_id = lost_id_;
  out_.position = lost_position_;
  out_.position_std = lost_std_;
  out_.occluded_time = t - lost_last_update_;
  out_.has_lost_position = true;
  out_.lost_position = lost_position_;
  out_.lost_seen_position = lost_seen_position_;
  out_.zone_candidates = n_cand;
  out_.competitors = n_comp;
  out_.ambiguous = n_cand > 1 || n_comp > 0;
  if (best == nullptr || out_.ambiguous) {
    hold_id_ = -1;
    return;
  }
  if (holdCandidate(t, best->id, params_.target.reacquire_confirm_s)) {
    target_id_ = best->id;
    known_seen_position_ = best->kf.position();
    clear_position_ = best->kf.position();
    ambiguous_since_ = -1.0;
    hold_id_ = -1;
    phase_ = Phase::LOCKED;
    runLocked(t, robot_pose, tracker);
  }
}

// 재획득 영역까지의 거리: 마지막 관측 위치 ~ LOST 직전 예측 위치 선분까지
double TargetManager::reacquireDistance(const Vec2& p) const {
  const Vec2 ab = lost_position_ - lost_seen_position_;
  const double len2 = squaredNorm(ab);
  double u = 0.0;
  if (len2 > 0.0) {
    u = std::clamp(dot(p - lost_seen_position_, ab) / len2, 0.0, 1.0);
  }
  return distance(p, lost_seen_position_ + u * ab);
}

// 모호: 타깃 radius 안의 다른 사람 트랙, 또는 타깃 게이트 안에 비슷한 비용의 사람 측정
bool TargetManager::computeAmbiguity(double t, const PersonTrack& target,
                                     const PeopleTracker& tracker) const {
  const AmbiguityParams& ap = params_.ambiguity;
  const Vec2 p = target.kf.position();
  for (const auto& tr : tracker.tracks()) {
    if (tr.id != target.id && isPersonLike(t, tr) &&
        distance(tr.kf.position(), p) < ap.radius) {
      return true;
    }
  }
  const auto& cands = tracker.candidates();
  const int own = target.updated ? target.candidate : -1;
  for (int c = 0; c < static_cast<int>(cands.size()); ++c) {
    if (c == own || (own >= 0 && tracker.candidatesOverlap(c, own))) {
      continue;
    }
    const double cost = tracker.priorCost(target.id, c);
    if (cost >= AssignmentSolver::kForbidden) {
      continue;  // 타깃 게이트 밖
    }
    const Candidate& cand = cands[c];
    bool person_like = false;
    if (cand.track_id >= 0) {
      const PersonTrack* other = tracker.findTrack(cand.track_id);
      person_like =
          other != nullptr && other->id != target.id && isPersonLike(t, *other);
    } else {
      person_like = cand.kind != CandidateKind::SINGLE;
    }
    if (!person_like) {
      continue;
    }
    // 타깃이 가려졌는데 게이트 안에 사람 측정이 있거나, 비용이 타깃 측정과 비슷하면 모호
    if (own < 0 || cost <= target.assoc_cost + ap.cost_margin) {
      return true;
    }
  }
  return false;
}

}  // namespace tracker
}  // namespace kebi_follow
