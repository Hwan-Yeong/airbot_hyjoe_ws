// 실행 중 조정 가능한 leg_tracker 파라미터 표 (노드·시험 공용). ROS 의존 없음.
// 조정 UI 등에서 ros2 param set 으로 바꿀 수 있다. 범위 밖이거나 서로 모순되면 거부하고,
// 받은 값은 다음 scan 처리 시작에 한꺼번에 적용한다. 나머지는 기동 때만 읽는다 (read_only).
#pragma once

#include <string>

#include "kebi_follow_tracker/params.hpp"

namespace kebi_follow {
namespace tracker {

struct TrackerTunable {
  const char* name;
  double lo;
  double hi;
  const char* desc;
  double (*get)(const TrackerParams&);
  void (*set)(TrackerParams&, double);
};

constexpr double kMinAcquireSpan = 0.2;  // acquire.min_range + 이 값 <= acquire.max_range [m]

inline const TrackerTunable kTrackerTunables[] = {
    // 모호 (S8): 타깃 가까이 다른 사람이 있으면 멈춘다
    {"ambiguity.radius", 0.2, 1.5, "모호: 타깃 주변 이 반경 안 다른 사람이면 정지 [m] (작을수록 덜 멈춤)",
     [](const TrackerParams& p) { return p.ambiguity.radius; },
     [](TrackerParams& p, double v) { p.ambiguity.radius = v; }},
    {"ambiguity.min_person_score", 0.0, 1.0, "모호: '사람'으로 셀 점수 하한 (클수록 덜 멈춤)",
     [](const TrackerParams& p) { return p.ambiguity.min_person_score; },
     [](TrackerParams& p, double v) { p.ambiguity.min_person_score = v; }},
    {"ambiguity.lost_s", 0.0, 10.0, "모호가 이만큼 이어지면 놓침 처리 후 재획득 [s] (0: 끔)",
     [](const TrackerParams& p) { return p.ambiguity.lost_s; },
     [](TrackerParams& p, double v) { p.ambiguity.lost_s = v; }},
    // 카메라 확인
    {"camera.gate", 0.1, 1.0, "카메라 검출과 트랙을 잇는 거리 [m]",
     [](const TrackerParams& p) { return p.camera.gate; },
     [](TrackerParams& p, double v) { p.camera.gate = v; }},
    {"camera.person_weight", 0.0, 1.0, "사람 점수에서 카메라 확인 비중",
     [](const TrackerParams& p) { return p.camera.person_weight; },
     [](TrackerParams& p, double v) { p.camera.person_weight = v; }},
    // 처음 고정
    {"acquire.min_range", 0.3, 1.5, "처음 고정: 최소 거리 [m]",
     [](const TrackerParams& p) { return p.acquire.min_range; },
     [](TrackerParams& p, double v) { p.acquire.min_range = v; }},
    {"acquire.max_range", 0.8, 3.0, "처음 고정: 최대 거리 [m]",
     [](const TrackerParams& p) { return p.acquire.max_range; },
     [](TrackerParams& p, double v) { p.acquire.max_range = v; }},
    {"acquire.max_bearing_deg", 5.0, 60.0, "처음 고정: 좌우 각도 [deg]",
     [](const TrackerParams& p) { return p.acquire.max_bearing_deg; },
     [](TrackerParams& p, double v) { p.acquire.max_bearing_deg = v; }},
    {"acquire.confirm_s", 0.2, 3.0, "처음 고정: 조건 유지 시간 [s]",
     [](const TrackerParams& p) { return p.acquire.confirm_s; },
     [](TrackerParams& p, double v) { p.acquire.confirm_s = v; }},
    // 놓침·재획득
    {"target.lost_s", 0.5, 5.0, "측정 없이 이만큼 지나면 놓침 [s] (track.delete_s 이하)",
     [](const TrackerParams& p) { return p.target.lost_s; },
     [](TrackerParams& p, double v) { p.target.lost_s = v; }},
    {"target.reacquire_radius", 0.3, 2.0, "놓친 뒤 다시 잡는 반경 [m]",
     [](const TrackerParams& p) { return p.target.reacquire_radius; },
     [](TrackerParams& p, double v) { p.target.reacquire_radius = v; }},
    {"target.reacquire_confirm_s", 0.1, 2.0, "다시 잡기 조건 유지 시간 [s]",
     [](const TrackerParams& p) { return p.target.reacquire_confirm_s; },
     [](TrackerParams& p, double v) { p.target.reacquire_confirm_s = v; }},
    {"target.camera_lost_s", 0.0, 10.0, "카메라가 시야 안 타깃을 이만큼 못 보면 놓침 [s] (0: 끔)",
     [](const TrackerParams& p) { return p.target.camera_lost_s; },
     [](TrackerParams& p, double v) { p.target.camera_lost_s = v; }},
};

inline const TrackerTunable* findTrackerTunable(const std::string& name) {
  for (const auto& t : kTrackerTunables) {
    if (name == t.name) {
      return &t;
    }
  }
  return nullptr;
}

// 서로 모순되는 값이면 그 이유, 아니면 빈 문자열
inline std::string trackerInconsistency(const TrackerParams& p) {
  if (p.acquire.min_range + kMinAcquireSpan > p.acquire.max_range) {
    return "acquire.min_range + 0.2 m 가 acquire.max_range 보다 크다";
  }
  if (p.target.lost_s > p.track.delete_s) {
    return "target.lost_s 가 track.delete_s 보다 크다 (트랙이 먼저 지워진다)";
  }
  return {};
}

}  // namespace tracker
}  // namespace kebi_follow
