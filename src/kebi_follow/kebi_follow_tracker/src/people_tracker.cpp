#include "kebi_follow_tracker/people_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace kebi_follow {
namespace tracker {

namespace {
constexpr std::size_t kReserveSegments = 256;
constexpr std::size_t kReserveCandidates = 512;
constexpr std::size_t kReserveLegPairs = 4096;
constexpr std::size_t kReserveCostEntries = 64 * 512;
constexpr std::size_t kReserveCameraDetections = 16;
// 시간이 이만큼 거꾸로 가면 (bag 반복 등) 초기화
constexpr double kTimeJumpBackS = 1.0;
// 탐욕 충돌 해소(대체 경로)에서 다시 푸는 횟수 상한
constexpr int kMaxConflictRounds = 32;
constexpr int kMaxAssignmentSolves = 32;  // 단계당 분기 한정 헝가리안 풀이 상한
constexpr int kMaxSearchDepth = 8;  // 분기 깊이 상한 (풀어야 할 충돌 수)
constexpr std::size_t kReserveEdits = 1024;
// [m] scan.max_range 밖으로 이만큼 나가면 삭제
constexpr double kRobotRangeMargin = 1.0;
}  // namespace

bool PersonTrack::cameraConfirmed(double now, double window_s) const {
  return has_camera && now - last_camera_time <= window_s;
}

double PersonTrack::personScore(double now, const CameraParams& params) const {
  const double cam = cameraConfirmed(now, params.confirm_window_s)
                         ? params.person_weight
                         : 0.0;
  return 1.0 - (1.0 - shapeEvidence()) * (1.0 - cam);
}

double PersonTrack::lockShape(double now, const TrackerParams& params) const {
  const bool evidence =
      moved || cameraConfirmed(now, params.camera.confirm_window_s);
  return shapeEvidence() * (evidence ? 1.0 : params.track.static_shape_factor);
}

void PersonTrack::pushHistory(double t) {
  history[history_head] = {t, kf.position()};
  history_head = (history_head + 1) % kTrackHistorySize;
  history_size = std::min(history_size + 1, kTrackHistorySize);
}

Vec2 PersonTrack::positionAt(double t) const {
  if (history_size == 0) {
    return kf.position();
  }
  const int newest = (history_head + kTrackHistorySize - 1) % kTrackHistorySize;
  if (t >= history[newest].t) {
    return history[newest].p + (t - history[newest].t) * kf.velocity();
  }
  for (int k = 1; k < history_size; ++k) {
    const int older = (newest - k + kTrackHistorySize) % kTrackHistorySize;
    const int newer = (older + 1) % kTrackHistorySize;
    if (t >= history[older].t) {
      const double span = history[newer].t - history[older].t;
      const double w = span > 0.0 ? (t - history[older].t) / span : 1.0;
      return history[older].p + w * (history[newer].p - history[older].p);
    }
  }
  const int oldest =
      (newest - (history_size - 1) + kTrackHistorySize) % kTrackHistorySize;
  return history[oldest].p + (t - history[oldest].t) * kf.velocity();
}

PeopleTracker::PeopleTracker(const TrackerParams& params) : params_(params) {
  tracks_.reserve(params_.track.max_tracks);
  seg_odom_.reserve(kReserveSegments);
  seg_owner_.reserve(kReserveSegments);
  legs_.reserve(kReserveSegments);
  blobs_.reserve(kReserveSegments);
  leg_slot_.reserve(kReserveSegments);
  single_lookup_.reserve(kReserveSegments);
  merged_lookup_.reserve(kReserveSegments);
  pair_lookup_.reserve(kReserveLegPairs);
  near_legs_.reserve(kReserveSegments);
  candidates_.reserve(kReserveCandidates);
  cost_.reserve(kReserveCostEntries);
  miss_cost_.reserve(params_.track.max_tracks);
  cost_row_ids_.reserve(params_.track.max_tracks);
  stage_rows_.reserve(params_.track.max_tracks);
  stage_cost_.reserve(kReserveCostEntries);
  stage_assign_.reserve(params_.track.max_tracks);
  seg_claim_.reserve(kReserveSegments);
  birth_legs_.reserve(kReserveSegments);
  birth_used_.reserve(kReserveSegments);
  birth_pairs_.reserve(kReserveLegPairs);
  cam_cost_.reserve(kReserveCameraDetections *
                    (params_.track.max_tracks + kReserveCameraDetections));
  cam_assign_.reserve(kReserveCameraDetections);
  cam_track_pos_.reserve(params_.track.max_tracks);
  edits_.reserve(kReserveEdits);
  assign_stack_.resize(kMaxSearchDepth);
  for (auto& a : assign_stack_) {
    a.reserve(params_.track.max_tracks);
  }
  best_assign_.reserve(params_.track.max_tracks);
}

void PeopleTracker::reset() {
  tracks_.clear();
  candidates_.clear();
  cost_.clear();
  cost_row_ids_.clear();
  has_time_ = false;
  last_time_ = 0.0;
  last_camera_frame_ = -1.0;
}

const PersonTrack* PeopleTracker::findTrack(int id) const {
  for (const auto& tr : tracks_) {
    if (tr.id == id) {
      return &tr;
    }
  }
  return nullptr;
}

double PeopleTracker::measStd(CandidateKind kind) const {
  switch (kind) {
    case CandidateKind::PAIR:
      return params_.kf.meas_std_pair;
    case CandidateKind::MERGED:
      return params_.kf.meas_std_merged;
    case CandidateKind::SINGLE:
    default:
      return params_.kf.meas_std_single;
  }
}

double PeopleTracker::measStdFor(const PersonTrack& tr,
                                 const Candidate& c) const {
  if (useStaticModel(tr)) {
    return params_.kf.static_meas_std;
  }
  if (c.kind != CandidateKind::SINGLE) {
    return c.meas_std;
  }
  const KalmanParams& kp = params_.kf;
  const double w =
      kp.single_std_speed > 0.0
          ? std::min(1.0, norm(tr.kf.velocity()) / kp.single_std_speed)
          : 1.0;
  return kp.meas_std_single_standing +
         w * (kp.meas_std_single - kp.meas_std_single_standing);
}

bool PeopleTracker::useStaticModel(const PersonTrack& tr) const {
  return tr.stationary && tr.id != protected_id_;
}

double PeopleTracker::priorCost(int track_id, int candidate) const {
  const int cols = static_cast<int>(candidates_.size());
  if (candidate < 0 || candidate >= cols) {
    return AssignmentSolver::kForbidden;
  }
  for (std::size_t k = 0; k < cost_row_ids_.size(); ++k) {
    if (cost_row_ids_[k] == track_id) {
      return cost_[k * cols + candidate];
    }
  }
  return AssignmentSolver::kForbidden;
}

bool PeopleTracker::sharesSegment(int a, int b) const {
  const Candidate& ca = candidates_[a];
  const Candidate& cb = candidates_[b];
  return ca.seg_a == cb.seg_a || ca.seg_a == cb.seg_b ||
         (ca.seg_b >= 0 && (ca.seg_b == cb.seg_a || ca.seg_b == cb.seg_b));
}

bool PeopleTracker::candidatesOverlap(int a, int b) const {
  const int n = static_cast<int>(candidates_.size());
  if (a < 0 || b < 0 || a >= n || b >= n) {
    return false;
  }
  return sharesSegment(a, b);
}

bool PeopleTracker::usesOwnedSegment(int candidate) const {
  const Candidate& c = candidates_[candidate];
  return seg_owner_[c.seg_a] >= 0 || (c.seg_b >= 0 && seg_owner_[c.seg_b] >= 0);
}

int PeopleTracker::addCandidate(CandidateKind kind, int seg_a, int seg_b) {
  const std::vector<Segment>& segs = *segments_;
  const TrackParams& tp = params_.track;
  Candidate c;
  c.kind = kind;
  c.seg_a = seg_a;
  c.seg_b = seg_b;
  switch (kind) {
    case CandidateKind::PAIR:
      c.pos = 0.5 * (seg_odom_[seg_a] + seg_odom_[seg_b]);
      c.shape =
          tp.pair_shape * 0.5 * (segs[seg_a].leg_score + segs[seg_b].leg_score);
      break;
    case CandidateKind::MERGED:
      c.pos = seg_odom_[seg_a];
      c.shape = tp.merged_shape;
      break;
    case CandidateKind::SINGLE:
    default:
      c.pos = seg_odom_[seg_a];
      c.shape = tp.single_shape * segs[seg_a].leg_score;
      break;
  }
  c.meas_std = measStd(kind);
  candidates_.push_back(c);
  return static_cast<int>(candidates_.size()) - 1;
}

// 트랙 근처의 다리·덩어리로만 후보를 만든다 (다리 쌍은 보폭까지 허용)
void PeopleTracker::buildCandidates(const std::vector<Segment>& segments) {
  const int nseg = static_cast<int>(segments.size());
  candidates_.clear();
  legs_.clear();
  blobs_.clear();
  leg_slot_.assign(nseg, -1);
  single_lookup_.assign(nseg, -1);
  merged_lookup_.assign(nseg, -1);
  for (int s = 0; s < nseg; ++s) {
    if (segments[s].is_leg) {
      leg_slot_[s] = static_cast<int>(legs_.size());
      legs_.push_back(s);
    }
    if (segments[s].is_blob) {
      blobs_.push_back(s);
    }
  }
  const int nl = static_cast<int>(legs_.size());
  pair_lookup_.assign(static_cast<std::size_t>(nl) * nl, -1);
  const LegParams& lp = params_.leg;
  const AssocParams& ap = params_.assoc;

  for (const auto& tr : tracks_) {
    const Vec2 p = tr.kf.position();
    // 게이트 반경(다리 하나 기준) + 쌍이면 보폭 절반까지만 모은다
    const double r = useStaticModel(tr) ? params_.kf.static_meas_std
                                        : params_.kf.meas_std_single;
    const double var = tr.kf.positionStd() * tr.kf.positionStd() +
                       r * r;  // 걷는 사람 기준 (넉넉히)
    const double gate =
        std::min(ap.max_gate_dist, std::sqrt(ap.gate_chi2 * var));
    const double gather =
        std::min(ap.max_gate_dist, gate + 0.5 * lp.max_leg_spread);
    near_legs_.clear();
    for (int s : legs_) {
      if (distance(seg_odom_[s], p) <= gather) {
        near_legs_.push_back(s);
        if (single_lookup_[s] < 0) {
          single_lookup_[s] = addCandidate(CandidateKind::SINGLE, s, -1);
        }
      }
    }
    for (int s : blobs_) {
      if (merged_lookup_[s] < 0 && distance(seg_odom_[s], p) <= gather) {
        merged_lookup_[s] = addCandidate(CandidateKind::MERGED, s, -1);
      }
    }
    const int nn = static_cast<int>(near_legs_.size());
    for (int i = 0; i < nn; ++i) {
      for (int j = i + 1; j < nn; ++j) {
        int a = near_legs_[i];
        int b = near_legs_[j];
        if (leg_slot_[a] > leg_slot_[b]) {
          std::swap(a, b);
        }
        const double sep = distance(seg_odom_[a], seg_odom_[b]);
        if (sep < lp.pair_min_sep || sep > lp.max_leg_spread) {
          continue;
        }
        int& slot = pair_lookup_[static_cast<std::size_t>(leg_slot_[a]) * nl +
                                 leg_slot_[b]];
        if (slot < 0) {
          slot = addCandidate(CandidateKind::PAIR, a, b);
        }
      }
    }
  }
}

// 예측 상태 기준 비용 = Mahalanobis^2 + ln|S| (측정 종류마다 R 이 달라 정규화 항을 넣는다)
void PeopleTracker::computeCosts() {
  const int rows = static_cast<int>(tracks_.size());
  const int cols = static_cast<int>(candidates_.size());
  cost_.assign(static_cast<std::size_t>(rows) * cols,
               AssignmentSolver::kForbidden);
  miss_cost_.resize(rows);
  cost_row_ids_.resize(rows);
  const AssocParams& ap = params_.assoc;
  for (int k = 0; k < rows; ++k) {
    const PersonTrack& tr = tracks_[k];
    cost_row_ids_[k] = tr.id;
    const Vec2 p = tr.kf.position();
    double d2 = 0.0;
    double log_det = 0.0;
    Candidate single;
    single.kind = CandidateKind::SINGLE;
    tr.kf.innovation(p, measStdFor(tr, single), d2, log_det);
    miss_cost_[k] =
        ap.gate_chi2 + log_det;  // 게이트 경계의 다리 하나 비용 (보너스 없음)
    for (int c = 0; c < cols; ++c) {
      const Candidate& cand = candidates_[c];
      if (distance(cand.pos, p) > ap.max_gate_dist) {
        continue;
      }
      tr.kf.innovation(cand.pos, measStdFor(tr, cand), d2, log_det);
      if (d2 > ap.gate_chi2) {
        continue;
      }
      const int explained = cand.seg_b >= 0 ? 2 : 1;
      cost_[static_cast<std::size_t>(k) * cols + c] =
          d2 + log_det - ap.segment_bonus * explained;
    }
  }
}

// 확정 트랙 먼저(1단계), 남은 분할로 미확정 트랙(2단계). 각 단계는 헝가리안 + 분할 공유 충돌 해소
void PeopleTracker::assignStage(bool confirmed_stage) {
  stage_rows_.clear();
  for (int k = 0; k < static_cast<int>(tracks_.size()); ++k) {
    if (tracks_[k].confirmed == confirmed_stage) {
      stage_rows_.push_back(k);
    }
  }
  const int nr = static_cast<int>(stage_rows_.size());
  const int nc = static_cast<int>(candidates_.size());
  if (nr == 0 || nc == 0) {
    return;
  }
  const int width = nc + nr;  // 후보 + 행마다 "측정 없음" 열
  stage_cost_.assign(static_cast<std::size_t>(nr) * width,
                     AssignmentSolver::kForbidden);
  for (int i = 0; i < nr; ++i) {
    const int k = stage_rows_[i];
    double* row = stage_cost_.data() + static_cast<std::size_t>(i) * width;
    for (int c = 0; c < nc; ++c) {
      if (!usesOwnedSegment(c)) {
        row[c] = cost_[static_cast<std::size_t>(k) * nc + c];
      }
    }
    row[nc + i] = miss_cost_[k];
  }

  seg_claim_.assign(seg_owner_.size(), -1);
  edits_.clear();
  best_cost_ = std::numeric_limits<double>::infinity();
  solves_left_ = kMaxAssignmentSolves;
  searchStage(nr, nc, width, 0);
  if (std::isfinite(best_cost_)) {
    stage_assign_ = best_assign_;
  } else {
    greedyResolve(nr, nc,
                  width);  // 풀이 상한에 걸림: 비용이 큰 쪽이 양보하는 근사
  }

  for (int i = 0; i < nr; ++i) {
    const int c = stage_assign_[i];
    if (c < 0 || c >= nc || usesOwnedSegment(c)) {
      continue;  // 측정 없음 (충돌 해소 상한에 걸린 경우도 먼저 차지한 쪽만 쓴다)
    }
    const int k = stage_rows_[i];
    PersonTrack& tr = tracks_[k];
    tr.candidate = c;
    tr.assoc_cost = stage_cost_[static_cast<std::size_t>(i) * width + c];
    candidates_[c].track_id = tr.id;
    seg_owner_[candidates_[c].seg_a] = k;
    if (candidates_[c].seg_b >= 0) {
      seg_owner_[candidates_[c].seg_b] = k;
    }
  }
}

bool PeopleTracker::findConflict(const std::vector<int>& assign, int nr, int nc,
                                 int& row_a, int& row_b, int& seg) {
  std::fill(seg_claim_.begin(), seg_claim_.end(), -1);
  for (int i = 0; i < nr; ++i) {
    const int c = assign[i];
    if (c < 0 || c >= nc) {
      continue;
    }
    const int segs[2] = {candidates_[c].seg_a, candidates_[c].seg_b};
    for (int s : segs) {
      if (s < 0) {
        continue;
      }
      if (seg_claim_[s] >= 0) {
        row_a = seg_claim_[s];
        row_b = i;
        seg = s;
        return true;
      }
      seg_claim_[s] = i;
    }
  }
  return false;
}

void PeopleTracker::forbidSegment(int row, int seg, int nc, int width) {
  for (int c = 0; c < nc; ++c) {
    if (candidates_[c].seg_a != seg && candidates_[c].seg_b != seg) {
      continue;
    }
    const std::size_t idx = static_cast<std::size_t>(row) * width + c;
    if (stage_cost_[idx] < AssignmentSolver::kForbidden) {
      edits_.push_back({idx, stage_cost_[idx]});
      stage_cost_[idx] = AssignmentSolver::kForbidden;
    }
  }
}

void PeopleTracker::undoEdits(std::size_t mark) {
  while (edits_.size() > mark) {
    stage_cost_[edits_.back().index] = edits_.back().value;
    edits_.pop_back();
  }
}

// 충돌(두 트랙이 같은 분할)을 만나면 "한쪽이 그 분할을 못 쓴다" 두 갈래로 나눠 푼다.
// 금지가 늘수록 헝가리안 최적값은 커지므로 지금까지의 최선보다 나쁘면 더 내려가지 않는다.
void PeopleTracker::searchStage(int nr, int nc, int width, int depth) {
  if (solves_left_ <= 0) {
    return;
  }
  --solves_left_;
  std::vector<int>& assign = assign_stack_[depth];
  solver_.solve(stage_cost_, nr, width, assign);
  double total = 0.0;
  for (int i = 0; i < nr; ++i) {
    total += assign[i] >= 0
                 ? stage_cost_[static_cast<std::size_t>(i) * width + assign[i]]
                 : AssignmentSolver::kForbidden;
  }
  if (total >= best_cost_) {
    return;
  }
  int row_a = -1;
  int row_b = -1;
  int seg = -1;
  if (!findConflict(assign, nr, nc, row_a, row_b, seg)) {
    best_cost_ = total;
    best_assign_ = assign;
    return;
  }
  if (depth + 1 >= kMaxSearchDepth) {
    return;
  }
  for (int row : {row_a, row_b}) {
    const std::size_t mark = edits_.size();
    forbidSegment(row, seg, nc, width);
    searchStage(nr, nc, width, depth + 1);
    undoEdits(mark);
  }
}

void PeopleTracker::greedyResolve(int nr, int nc, int width) {
  for (int round = 0; round < kMaxConflictRounds; ++round) {
    solver_.solve(stage_cost_, nr, width, stage_assign_);
    int i = -1;
    int j = -1;
    int seg = -1;
    if (!findConflict(stage_assign_, nr, nc, i, j, seg)) {
      return;
    }
    const double cost_i =
        stage_cost_[static_cast<std::size_t>(i) * width + stage_assign_[i]];
    const double cost_j =
        stage_cost_[static_cast<std::size_t>(j) * width + stage_assign_[j]];
    forbidSegment(cost_i > cost_j ? i : j, seg, nc, width);
  }
}

void PeopleTracker::applyAssignments(double t) {
  const TrackParams& tp = params_.track;
  for (auto& tr : tracks_) {
    if (tr.candidate < 0) {
      ++tr.misses;
      continue;
    }
    const Candidate& c = candidates_[tr.candidate];
    tr.kf.update(c.pos, measStdFor(tr, c));
    tr.kf.clampSpeed(params_.kf.max_speed);
    ++tr.hits;
    tr.misses = 0;
    tr.last_update_time = t;
    tr.kind = c.kind;
    tr.updated = true;
    tr.shape_score += tp.shape_alpha * (c.shape - tr.shape_score);
    tr.shape_peak = std::max(tr.shape_peak, tr.shape_score);
    if (c.kind == CandidateKind::PAIR) {
      ++tr.pair_hits;
    }
    tr.far_count = distance(tr.kf.position(), tr.birth_pos) > tp.moved_dist
                       ? tr.far_count + 1
                       : 0;
    tr.moved = tr.moved || tr.far_count >= tp.moved_scans;
    // 정지 트랙: 움직인 적 없이 오래 보인 트랙. 서 있던 사람이 걷기 시작해 움직임이 확인되면 해제된다
    tr.stationary = !tr.moved && tr.hits >= tp.static_min_hits &&
                    t - tr.birth_time >= tp.static_min_s;
    if (!tr.confirmed && tr.hits >= tp.confirm_hits) {
      tr.confirmed = true;
    }
  }
}

void PeopleTracker::addTrack(double t, CandidateKind kind, const Vec2& pos,
                             double meas_std, double shape) {
  if (static_cast<int>(tracks_.size()) >= params_.track.max_tracks) {
    return;
  }
  for (const auto& tr : tracks_) {
    if (distance(tr.kf.position(), pos) < params_.track.duplicate_radius) {
      return;
    }
  }
  PersonTrack tr;
  tr.id = next_id_;
  next_id_ = next_id_ == std::numeric_limits<int>::max() ? 1 : next_id_ + 1;
  tr.kf.init(pos, meas_std, params_.kf.init_vel_std);
  tr.confirmed = params_.track.confirm_hits <= 1;
  tr.hits = 1;
  tr.birth_time = t;
  tr.last_update_time = t;
  tr.last_predict_time = t;
  tr.kind = kind;
  tr.shape_score = shape;
  tr.shape_peak = shape;
  tr.pair_hits = kind == CandidateKind::PAIR ? 1 : 0;
  tr.birth_pos = pos;
  tr.updated = true;
  tracks_.push_back(tr);
}

void PeopleTracker::spawnTracks(double t,
                                const std::vector<Segment>& segments) {
  const int nseg = static_cast<int>(segments.size());
  const TrackParams& tp = params_.track;
  const LegParams& lp = params_.leg;
  const double person_score_min = params_.ambiguity.min_person_score;
  birth_used_.assign(nseg, 1);
  birth_legs_.clear();
  for (int s = 0; s < nseg; ++s) {
    const Segment& seg = segments[s];
    if (seg_owner_[s] >= 0 || !(seg.is_leg || seg.is_blob)) {
      continue;
    }
    bool suppressed = false;
    for (const auto& tr : tracks_) {
      const double d = distance(seg_odom_[s], tr.kf.position());
      if (d < tp.duplicate_radius) {
        suppressed = true;
        break;
      }
      if (!tr.confirmed ||
          tr.personScore(t, params_.camera) < person_score_min ||
          (tr.stationary &&
           !tr.cameraConfirmed(t, params_.camera.confirm_window_s))) {
        continue;  // 정지 트랙(가구일 수 있음) 옆에서는 새 사람이 생길 수 있다
      }
      // 사람 트랙 바로 옆 조각은 새 사람으로 만들지 않는다. 이번에 다리 하나만 받은 트랙이면
      // 다른 다리일 수 있으므로 보폭 거리까지 막는다 (그 다리 기준)
      bool near_own_leg = false;
      if (tr.candidate >= 0 &&
          candidates_[tr.candidate].kind == CandidateKind::SINGLE &&
          seg.is_leg) {
        const int own = candidates_[tr.candidate].seg_a;
        near_own_leg =
            distance(seg_odom_[s], seg_odom_[own]) <= lp.max_leg_spread;
      }
      if (d < tp.birth_suppress_radius || near_own_leg) {
        suppressed = true;
        break;
      }
    }
    if (suppressed) {
      continue;
    }
    birth_used_[s] = 0;
    if (seg.is_leg) {
      birth_legs_.push_back(s);
    }
  }

  // 다리 쌍 (간격이 작은 쌍부터)
  birth_pairs_.clear();
  const int nl = static_cast<int>(birth_legs_.size());
  for (int i = 0; i < nl; ++i) {
    for (int j = i + 1; j < nl; ++j) {
      const int a = birth_legs_[i];
      const int b = birth_legs_[j];
      const double sep = distance(seg_odom_[a], seg_odom_[b]);
      if (sep >= lp.pair_min_sep && sep <= lp.pair_max_sep) {
        birth_pairs_.push_back({sep, a, b});
      }
    }
  }
  std::sort(
      birth_pairs_.begin(), birth_pairs_.end(),
      [](const PairOption& x, const PairOption& y) { return x.sep < y.sep; });
  for (const auto& po : birth_pairs_) {
    if (birth_used_[po.a] || birth_used_[po.b]) {
      continue;
    }
    birth_used_[po.a] = birth_used_[po.b] = 1;
    const double shape = tp.pair_shape * 0.5 *
                         (segments[po.a].leg_score + segments[po.b].leg_score);
    addTrack(t, CandidateKind::PAIR, 0.5 * (seg_odom_[po.a] + seg_odom_[po.b]),
             params_.kf.meas_std_pair, shape);
  }
  // 붙은 덩어리
  for (int s = 0; s < nseg; ++s) {
    if (!birth_used_[s] && segments[s].is_blob) {
      birth_used_[s] = 1;
      addTrack(t, CandidateKind::MERGED, seg_odom_[s],
               params_.kf.meas_std_merged, tp.merged_shape);
    }
  }
  // 다리 하나
  if (tp.allow_single_birth) {
    for (int s : birth_legs_) {
      if (!birth_used_[s]) {
        birth_used_[s] = 1;
        addTrack(t, CandidateKind::SINGLE, seg_odom_[s],
                 params_.kf.meas_std_single,
                 tp.single_shape * segments[s].leg_score);
      }
    }
  }
}

bool PeopleTracker::outranks(double t, const PersonTrack& a,
                             const PersonTrack& b) const {
  if ((a.id == protected_id_) != (b.id == protected_id_)) {
    return a.id == protected_id_;
  }
  const double w = params_.camera.confirm_window_s;
  if (a.cameraConfirmed(t, w) != b.cameraConfirmed(t, w)) {
    return a.cameraConfirmed(t, w);
  }
  return a.hits != b.hits ? a.hits > b.hits : a.id < b.id;
}

// 한 사람에 트랙 둘이 붙으면 다리를 하나씩 나눠 가져 둘 다 유지된다. 붙어 있는 시간을 세어 지운다.
// 정지 트랙(가구)은 사람 옆에 있어도 별개 물체이므로 대상이 아니다.
void PeopleTracker::markDuplicates(double t) {
  const TrackParams& tp = params_.track;
  for (auto& a : tracks_) {
    if (!a.confirmed || a.stationary) {
      a.close_count = 0;
      continue;
    }
    bool close = false;
    for (const auto& b : tracks_) {
      if (&a == &b || !b.confirmed || b.stationary) {
        continue;
      }
      if (distance(a.kf.position(), b.kf.position()) < tp.merge_radius &&
          outranks(t, b, a)) {
        close = true;
        break;
      }
    }
    a.close_count = close ? a.close_count + 1 : 0;
  }
}

void PeopleTracker::deleteTracks(double t, const Pose2& robot_pose) {
  const TrackParams& tp = params_.track;
  const Vec2 robot{robot_pose.x, robot_pose.y};
  const double max_range = params_.scan.max_range + kRobotRangeMargin;
  tracks_.erase(
      std::remove_if(tracks_.begin(), tracks_.end(),
                     [&](const PersonTrack& tr) {
                       if (!tr.confirmed) {
                         if (tr.misses > tp.tentative_max_misses) {
                           return true;
                         }
                       } else if (t - tr.last_update_time > tp.delete_s ||
                                  tr.kf.positionStd() > tp.delete_std ||
                                  tr.close_count >= tp.merge_scans) {
                         return true;
                       }
                       return distance(tr.kf.position(), robot) > max_range;
                     }),
      tracks_.end());
}

void PeopleTracker::update(double t, const Pose2& robot_pose,
                           const std::vector<Segment>& segments) {
  if (has_time_ && t < last_time_ - kTimeJumpBackS) {
    reset();
  }
  has_time_ = true;
  last_time_ = std::max(last_time_, t);
  segments_ = &segments;

  const int nseg = static_cast<int>(segments.size());
  seg_odom_.resize(nseg);
  for (int s = 0; s < nseg; ++s) {
    seg_odom_[s] = transformPoint(robot_pose, segments[s].center);
  }
  seg_owner_.assign(nseg, -1);

  for (auto& tr : tracks_) {
    // 지난 scan 에 측정이 없었으면(가림) 속도를 줄이며 예측한다
    tr.kf.predict(
        t - tr.last_predict_time,
        useStaticModel(tr) ? params_.kf.static_accel_std : params_.kf.accel_std,
        tr.misses > 0 ? params_.kf.coast_tau_s : 0.0);
    tr.last_predict_time = std::max(tr.last_predict_time, t);
    tr.updated = false;
    tr.candidate = -1;
    tr.assoc_cost = 0.0;
  }

  buildCandidates(segments);
  computeCosts();
  assignStage(true);
  assignStage(false);
  applyAssignments(t);
  spawnTracks(t, segments);
  markDuplicates(t);
  deleteTracks(t, robot_pose);
  for (auto& tr : tracks_) {
    tr.pushHistory(t);
  }
  segments_ = nullptr;
}

int PeopleTracker::associateCamera(double t,
                                   const std::vector<Vec2>& positions) {
  const int nd = static_cast<int>(positions.size());
  const int nt = static_cast<int>(tracks_.size());
  if (nd == 0 || nt == 0) {
    return 0;
  }
  const double gate = params_.camera.gate;
  const int width = nt + nd;
  cam_track_pos_.resize(nt);
  for (int k = 0; k < nt; ++k) {
    cam_track_pos_[k] = tracks_[k].positionAt(t);
  }
  cam_cost_.assign(static_cast<std::size_t>(nd) * width,
                   AssignmentSolver::kForbidden);
  for (int i = 0; i < nd; ++i) {
    double* row = cam_cost_.data() + static_cast<std::size_t>(i) * width;
    for (int k = 0; k < nt; ++k) {
      const double d = distance(positions[i], cam_track_pos_[k]);
      if (d <= gate) {
        row[k] = d;
      }
    }
    row[nt + i] = gate;
  }
  solver_.solve(cam_cost_, nd, width, cam_assign_);
  int matched = 0;
  for (int i = 0; i < nd; ++i) {
    const int k = cam_assign_[i];
    if (k < 0 || k >= nt) {
      continue;
    }
    PersonTrack& tr = tracks_[k];
    if (!tr.has_camera || t > tr.last_camera_time) {
      tr.last_camera_time = t;
    }
    tr.has_camera = true;
    ++tr.camera_hits;
    ++matched;
  }
  return matched;
}

}  // namespace tracker
}  // namespace kebi_follow
