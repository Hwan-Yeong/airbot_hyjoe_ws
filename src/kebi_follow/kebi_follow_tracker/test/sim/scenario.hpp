// 합성 시나리오 실행기: 세계·로봇·카메라를 시간 순서로 돌리며 LegTrackerCore 를 직접 호출하고
// 정답(사람 위치)과 비교한 지표를 만든다. 로봇 비의존 시험 도구.
#pragma once

#include <functional>
#include <string>
#include <vector>

#include "kebi_follow_tracker/leg_tracker_core.hpp"
#include "sim/scan_sim.hpp"

namespace kebi_follow {
namespace tracker {
namespace sim {

struct CameraModel {
  bool enabled{true};
  double period{0.1};    // [s]
  double jitter{0.01};   // [s]
  double latency{0.15};  // [s] 도착 - stamp
  double latency_std{0.03};
  double stamp_error_std{0.02};  // [s] AMR 시간축 변환 오차
  double half_fov{0.71};         // [rad] 640 px, fx 369.8
  double min_range{0.4};
  double max_range{5.0};
  double detect_prob{0.9};
  double pos_std{0.07};  // [m]
  // [m] 다리 앞면 depth 를 쓰는 데서 오는 치우침
  double bias_toward_robot{0.05};
  double false_rate{0.0};  // 프레임당 오검출 기대 수 (시야 안 임의 위치)
  double score{0.8};
};

struct ModeEvent {
  double t{0.0};  // 시나리오 시작 기준 [s]
  Mode mode{Mode::OFF};
};

struct RobotMotion {
  Pose2 start;
  // >= 0 이면 이 시각부터 정답 타깃을 따라간다 (추적 결과와 무관)
  double follow_start{-1.0};
  double follow_distance{1.0};
  double v_max{0.5};
  double w_max{0.8};
  double lin_accel{0.4};
  double lin_decel{0.8};
  double ang_accel{1.5};
  double k_v{1.0};
  double k_w{1.5};
  // >= 0 이면 [rotate_start, rotate_end) 동안 제자리 회전
  double rotate_start{-1.0};
  double rotate_end{-1.0};
  double rotate_rate{0.0};
};

struct ScenarioSpec {
  std::string name;
  double duration{10.0};
  double scan_period{0.1};
  double scan_jitter{0.003};
  LidarModel lidar;
  CameraModel camera;
  std::vector<WalkerSpec> walkers;
  std::vector<Circle> circles;
  std::vector<Wall> walls;
  int target_walker{0};  // 따라갈 사람 (평가 기준). -1 이면 없음 (오락 시험)
  std::vector<ModeEvent> modes;
  RobotMotion robot;
  TrackerParams params;
  unsigned seed{1};
  // 디버그: scan 처리 직후 호출 (t: 시나리오 시각)
  std::function<void(double, const LegTrackerCore&, const World&, const Pose2&)>
      on_scan;
};

struct ScanRecord {
  double t{0.0};  // 시나리오 시작 기준
  Mode mode{Mode::OFF};
  TargetStatus status{TargetStatus::NONE};
  int track_id{-1};
  bool ambiguous{false};
  bool camera_confirmed{false};
  Vec2 est;        // 타깃 추정 (base_link)
  Vec2 truth;      // 정답 타깃 중심 (base_link)
  Vec2 truth_vel;  // 정답 타깃 속도 (odom)
  Vec2 est_vel;    // 추정 속도 (odom)
  // 추정 위치에서 0.5 m 안의 가장 가까운 사람 (-1: 없음)
  int matched_walker{-1};
  double proc_ms{0.0};
  int n_tracks{0};
  int n_confirmed{0};
  // 사람 근처가 아닌데 사람 같은(person_score>=기준) 확정 트랙 수
  int n_false_person{0};
  int n_target_legs{0};  // 이번 scan 에서 타깃 사람 근처 다리·덩어리 분할 수
};

struct ScenarioMetrics {
  std::string name;
  int scans{0};
  double lock_time{-1.0};  // ACQUIRE 시작 → 첫 LOCKED [s]
  int lock_walker{-2};  // 처음 락한 대상 (-1: 사람 아님, -2: 락 없음)
  int false_locks{0};  // 타깃이 아닌 대상으로 락된 횟수 (획득·재획득)
  int id_switches{0};  // LOCKED/OCCLUDED 중 track_id 가 바뀐 횟수
  int reacquires{0};   // LOST → LOCKED 횟수
  int wrong_scans{0};  // 락 중 추정이 정답 타깃이 아닌 scan 수
  int locked_scans{0};
  int occluded_scans{0};
  int lost_scans{0};
  int ambiguous_scans{0};  // 락 중 모호 scan 수
  double err_rms{0.0};     // LOCKED 위치 오차 [m]
  double err_p95{0.0};
  double err_max{0.0};
  double vel_err_rms{0.0};  // LOCKED 속도 오차 [m/s]
  double lag_ms{0.0};       // 걷는 중 진행 방향 오차 / 속도 [ms]
  double false_person_mean{0.0};  // scan 당 사람 아닌 사람 같은 트랙 수
  int false_person_ids{0};
  double proc_mean_ms{0.0};
  double proc_p99_ms{0.0};
  double proc_max_ms{0.0};
  // 락 후 LOCKED·OCCLUDED 이면서 정답 타깃인 scan 비율
  double track_coverage{0.0};
};

struct ScenarioResult {
  std::vector<ScanRecord> records;
  ScenarioMetrics metrics;
};

ScenarioResult runScenario(const ScenarioSpec& spec);

}  // namespace sim
}  // namespace tracker
}  // namespace kebi_follow
