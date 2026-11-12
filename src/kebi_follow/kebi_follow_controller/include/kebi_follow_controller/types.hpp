// 공용 타입·상수. ROS 의존 없음 (단위 시험에서 그대로 쓴다).
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace kebi_follow {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kPi = 3.14159265358979323846;

inline double degToRad(double deg) {
  return deg * kPi / 180.0;
}
inline double radToDeg(double rad) {
  return rad * 180.0 / kPi;
}
inline double wrapAngle(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}

struct Vec2 {
  double x{0.0};
  double y{0.0};
};

inline double norm(const Vec2& v) {
  return std::hypot(v.x, v.y);
}
inline double distance(const Vec2& a, const Vec2& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}
inline Vec2 rotate(const Vec2& v, double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c * v.x - s * v.y, s * v.x + c * v.y};
}

// 2D 강체 변환. "부모 좌표계에서 본 자식 좌표계의 자세"
struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// 자식 좌표의 점 p 를 부모 좌표로
inline Vec2 transformPoint(const Pose2D& T, const Vec2& p) {
  const Vec2 r = rotate(p, T.yaw);
  return {T.x + r.x, T.y + r.y};
}
inline Pose2D compose(const Pose2D& a, const Pose2D& b) {
  const Vec2 t = transformPoint(a, {b.x, b.y});
  return {t.x, t.y, wrapAngle(a.yaw + b.yaw)};
}
inline Pose2D inverse(const Pose2D& a) {
  const Vec2 t = rotate({-a.x, -a.y}, -a.yaw);
  return {t.x, t.y, -a.yaw};
}
// odom 자세 두 개 → "then 시점 base 좌표"를 "now 시점 base 좌표"로 바꾸는 변환
inline Pose2D relativePose(const Pose2D& odom_now, const Pose2D& odom_then) {
  return compose(inverse(odom_now), odom_then);
}

// kebi_follow_msgs/FollowState.FSM_* 와 같은 값
enum class FsmState : uint8_t {
  kIdle = 0,
  kAcquiring = 1,
  kFollowing = 2,
  kWaiting = 3,
  kHalted = 4,
  kEnded = 5,
};

// kebi_follow_msgs/FollowMode 와 같은 값
enum class ModeValue : uint8_t { kOff = 0, kAcquire = 1, kTrack = 2 };

// kebi_follow_msgs/FollowTarget.STATUS_* 와 같은 값
enum class TargetStatus : uint8_t {
  kNone = 0,
  kCandidate = 1,
  kLocked = 2,
  kOccluded = 3,
  kLost = 4,
};

// kebi_follow_msgs/FollowState.STOP_* 와 같은 값 (노드에서 static_assert)
namespace stop_reason {
constexpr uint32_t kNoTarget = 1u;
constexpr uint32_t kTooClose = 2u;
constexpr uint32_t kObstacle = 4u;
constexpr uint32_t kRotationBlocked = 8u;
constexpr uint32_t kCliff = 16u;
constexpr uint32_t kCollision = 32u;
constexpr uint32_t kSensorTimeout = 64u;
constexpr uint32_t kAmbiguous = 128u;
constexpr uint32_t kRobotState = 256u;
constexpr uint32_t kRotationLimit = 512u;
// HALTED 로 보내는 사유
constexpr uint32_t kHaltMask = kCliff | kCollision | kRobotState;
}  // namespace stop_reason

// state_manager 계약 (SPEC 5절)
namespace ui_cmd {
constexpr uint16_t kStart = 1;
constexpr uint16_t kStop = 3;
}  // namespace ui_cmd

namespace detect_user {
constexpr int16_t kEnded = 0;       // 종료 (ENDED, HALTED)
constexpr int16_t kRecognized = 1;  // 인식, 정지 중
constexpr int16_t kNoPerson = 2;    // 영역 안에 사람 없음
constexpr int16_t kMoving = 3;      // 이동 중
constexpr int16_t kNone = -1;       // 발행 안 함 (IDLE)
}  // namespace detect_user

inline ModeValue modeOf(FsmState s) {
  switch (s) {
    case FsmState::kAcquiring:
      return ModeValue::kAcquire;
    case FsmState::kFollowing:
    case FsmState::kWaiting:
    case FsmState::kHalted:
      return ModeValue::kTrack;
    default:
      return ModeValue::kOff;
  }
}

// /cmd_vel 을 주기 발행하는 상태 (IDLE·ENDED 는 진입 시 0 몇 번만)
inline bool isActive(FsmState s) {
  return s == FsmState::kAcquiring || s == FsmState::kFollowing ||
         s == FsmState::kWaiting || s == FsmState::kHalted;
}

inline const char* toString(FsmState s) {
  switch (s) {
    case FsmState::kIdle:
      return "IDLE";
    case FsmState::kAcquiring:
      return "ACQUIRING";
    case FsmState::kFollowing:
      return "FOLLOWING";
    case FsmState::kWaiting:
      return "WAITING";
    case FsmState::kHalted:
      return "HALTED";
    case FsmState::kEnded:
      return "ENDED";
  }
  return "?";
}

}  // namespace kebi_follow
