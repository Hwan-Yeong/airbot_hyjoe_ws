// 사람 다중 추적 (odom 좌표). 다리·덩어리 관측 → 사람 후보 → GNN 연관 → CV 칼만 필터
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "kebi_follow_tracker/geometry.hpp"
#include "kebi_follow_tracker/hungarian.hpp"
#include "kebi_follow_tracker/kalman_cv.hpp"
#include "kebi_follow_tracker/leg_detector.hpp"
#include "kebi_follow_tracker/params.hpp"

namespace kebi_follow {
namespace tracker {

// 값은 kebi_follow_msgs/LegTrack.KIND_* 와 같다
enum class CandidateKind : uint8_t {
  PAIR = 0,
  SINGLE = 1,
  MERGED = 2,
};

// 사람 후보 (측정 가설). 한 분할이 여러 후보에 들어갈 수 있고, 연관에서 겹치지 않게 고른다.
struct Candidate {
  CandidateKind kind{CandidateKind::SINGLE};
  int seg_a{-1};  // detector 분할 번호
  int seg_b{-1};  // PAIR 일 때 두 번째 다리
  Vec2 pos;       // odom
  double meas_std{0.0};
  double shape{0.0};  // 형상 점수에 넣을 값
  int track_id{-1};   // 이번 scan 에 연관된 트랙 (-1: 없음)
};

struct TrackHistoryEntry {
  double t{0.0};
  Vec2 p;
};

// scan 10 Hz 기준 2 s (카메라 지연 보정용)
constexpr int kTrackHistorySize = 20;

struct PersonTrack {
  int id{-1};
  KalmanCV kf;
  bool confirmed{false};
  int hits{0};    // 측정 갱신 횟수
  int misses{0};  // 연속 미측정 횟수
  double birth_time{0.0};
  double last_update_time{0.0};
  double last_predict_time{0.0};
  double last_camera_time{0.0};
  bool has_camera{false};
  int camera_hits{0};
  CandidateKind kind{CandidateKind::SINGLE};
  double shape_score{0.0};  // 다리 쌍·덩어리로 보인 정도 (EMA, 0..1)
  double shape_peak{0.0};   // shape_score 최고값
  int pair_hits{0};         // 다리 쌍 측정 수
  Vec2 birth_pos;
  // 생성 위치에서 moved_dist 밖에 moved_scans 연속으로 있었던 적이 있다
  bool moved{false};
  int far_count{0};  // 생성 위치에서 moved_dist 밖에 있던 연속 갱신 수
  // 정지 트랙: 움직인 적 없이 static_min_s·static_min_hits 이상 (정지 물체 모델)
  bool stationary{false};
  // 더 높은 우선순위 트랙과 merge_radius 안에 붙어 있던 연속 scan 수
  int close_count{0};
  // 이번 scan 의 연관 결과
  bool updated{false};
  int candidate{-1};
  double assoc_cost{0.0};
  // 지난 위치 (카메라 stamp 시각의 위치를 구할 때)
  std::array<TrackHistoryEntry, kTrackHistorySize> history{};
  int history_head{0};
  int history_size{0};

  bool cameraConfirmed(double now, double window_s) const;
  // 형상 증거: 걸어온(움직인) 트랙은 최고 형상 점수를 유지한다 (옆모습으로 서면 다리 하나만 보이므로)
  double shapeEvidence() const {
    return moved ? std::max(shape_score, shape_peak) : shape_score;
  }
  // 0..1, 형상 증거와 카메라 확인의 noisy-OR
  double personScore(double now, const CameraParams& params) const;
  // 락 후보 판정용 형상 점수: 움직인 적도 카메라 확인도 없으면 static_shape_factor 배
  double lockShape(double now, const TrackerParams& params) const;
  // 시각 t 의 위치: 기록 보간, 범위 밖이면 등속 외삽
  Vec2 positionAt(double t) const;
  void pushHistory(double t);
};

class PeopleTracker {
 public:
  explicit PeopleTracker(const TrackerParams& params);

  void reset();
  // 정지 물체 모델을 쓰지 않을 트랙 (락 타깃). -1 이면 없음
  void setProtectedTrack(int id) { protected_id_ = id; }
  // segments: base_link 기준 detector 출력, robot_pose: odom <- base_link (scan 시각)
  void update(double t, const Pose2& robot_pose,
              const std::vector<Segment>& segments);
  // 카메라 검출(odom 위치, 시각 t)과 트랙을 연관해 camera 확인 시각을 갱신. 연관 수를 돌려준다.
  int associateCamera(double t, const std::vector<Vec2>& positions);

  const std::vector<PersonTrack>& tracks() const { return tracks_; }
  const PersonTrack* findTrack(int id) const;
  const std::vector<Candidate>& candidates() const { return candidates_; }
  // 이번 scan 의 예측 상태 기준 연관 비용. 게이트 밖이거나 모르는 트랙이면 kForbidden
  double priorCost(int track_id, int candidate) const;
  bool candidatesOverlap(int a, int b) const;
  bool hasTime() const { return has_time_; }
  // 카메라 프레임을 받은 마지막 시각 (검출이 없어도). 없으면 음수
  double lastCameraFrame() const { return last_camera_frame_; }
  void noteCameraFrame(double t) {
    last_camera_frame_ = std::max(last_camera_frame_, t);
  }
  double lastTime() const { return last_time_; }

 private:
  int addCandidate(CandidateKind kind, int seg_a, int seg_b);
  void buildCandidates(const std::vector<Segment>& segments);
  void computeCosts();
  void assignStage(bool confirmed_stage);
  // 분할 공유 충돌이 없는 최소 비용 배정: 헝가리안(완화 문제, 하한) + 충돌 분기 한정
  void searchStage(int nr, int nc, int width, int depth);
  bool findConflict(const std::vector<int>& assign, int nr, int nc, int& row_a,
                    int& row_b, int& seg);
  void forbidSegment(int row, int seg, int nc, int width);
  void undoEdits(std::size_t mark);
  void greedyResolve(int nr, int nc, int width);
  void applyAssignments(double t);
  void spawnTracks(double t, const std::vector<Segment>& segments);
  void addTrack(double t, CandidateKind kind, const Vec2& pos, double meas_std,
                double shape);
  void deleteTracks(double t, const Pose2& robot_pose);
  void markDuplicates(double t);
  bool outranks(double t, const PersonTrack& a, const PersonTrack& b) const;
  double measStd(CandidateKind kind) const;
  // 트랙 tr 에 후보 c 를 붙일 때의 측정 잡음 (정지 물체 모델, 다리 하나는 속력에 따라)
  double measStdFor(const PersonTrack& tr, const Candidate& c) const;
  bool useStaticModel(const PersonTrack& tr) const;
  bool usesOwnedSegment(int candidate) const;
  bool sharesSegment(int a, int b) const;

  TrackerParams params_;
  std::vector<PersonTrack> tracks_;
  int next_id_{1};
  int protected_id_{-1};
  double last_camera_frame_{-1.0};
  bool has_time_{false};
  double last_time_{0.0};

  // scan 마다 다시 채우는 버퍼 (용량 재사용)
  const std::vector<Segment>* segments_{nullptr};
  std::vector<Vec2> seg_odom_;
  std::vector<int> seg_owner_;  // 연관된 트랙 번호 (tracks_ 인덱스), -1 없음
  std::vector<int> legs_;           // 다리 분할 번호
  std::vector<int> blobs_;          // 덩어리 분할 번호
  std::vector<int> leg_slot_;       // 분할 → legs_ 위치
  std::vector<int> single_lookup_;  // 분할 → SINGLE 후보
  std::vector<int> merged_lookup_;  // 분할 → MERGED 후보
  std::vector<int> pair_lookup_;    // (다리 i, 다리 j) → PAIR 후보
  std::vector<int> near_legs_;
  std::vector<Candidate> candidates_;
  std::vector<double> cost_;  // tracks x candidates (예측 상태 기준)
  std::vector<double> miss_cost_;
  std::vector<int> cost_row_ids_;  // cost_ 행의 트랙 id
  std::vector<int> stage_rows_;
  std::vector<double> stage_cost_;
  std::vector<int> stage_assign_;
  std::vector<int> seg_claim_;
  struct CostEdit {
    std::size_t index;
    double value;
  };
  std::vector<CostEdit> edits_;  // 분기에서 금지한 항목 (되돌리기용)
  std::vector<std::vector<int>> assign_stack_;  // 분기 깊이별 배정
  std::vector<int> best_assign_;
  double best_cost_{0.0};
  int solves_left_{0};
  std::vector<int> birth_legs_;
  std::vector<char> birth_used_;
  struct PairOption {
    double sep;
    int a;
    int b;
  };
  std::vector<PairOption> birth_pairs_;
  std::vector<double> cam_cost_;
  std::vector<int> cam_assign_;
  std::vector<Vec2> cam_track_pos_;
  AssignmentSolver solver_;
};

}  // namespace tracker
}  // namespace kebi_follow
