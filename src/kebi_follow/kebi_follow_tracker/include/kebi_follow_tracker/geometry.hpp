// 평면 기하 도구 (ROS 비의존)
#pragma once

#include <cmath>

namespace kebi_follow {
namespace tracker {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kDegToRad = kPi / 180.0;

struct Vec2 {
  double x{0.0};
  double y{0.0};
};

inline Vec2 operator+(const Vec2& a, const Vec2& b) {
  return {a.x + b.x, a.y + b.y};
}
inline Vec2 operator-(const Vec2& a, const Vec2& b) {
  return {a.x - b.x, a.y - b.y};
}
inline Vec2 operator*(double s, const Vec2& a) {
  return {s * a.x, s * a.y};
}
inline double dot(const Vec2& a, const Vec2& b) {
  return a.x * b.x + a.y * b.y;
}
inline double squaredNorm(const Vec2& a) {
  return a.x * a.x + a.y * a.y;
}
inline double norm(const Vec2& a) {
  return std::sqrt(squaredNorm(a));
}
inline double distance(const Vec2& a, const Vec2& b) {
  return norm(a - b);
}
inline double bearing(const Vec2& a) {
  return std::atan2(a.y, a.x);
}

inline double wrapAngle(double a) {
  a = std::fmod(a + kPi, kTwoPi);
  if (a < 0.0) {
    a += kTwoPi;
  }
  return a - kPi;
}

// 평면 자세: parent <- child (child 원점의 parent 좌표와 yaw)
struct Pose2 {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline Vec2 rotate(const Vec2& v, double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c * v.x - s * v.y, s * v.x + c * v.y};
}

// child 좌표의 점 → parent 좌표
inline Vec2 transformPoint(const Pose2& pose, const Vec2& p) {
  const Vec2 r = rotate(p, pose.yaw);
  return {pose.x + r.x, pose.y + r.y};
}

// parent 좌표의 점 → child 좌표
inline Vec2 inverseTransformPoint(const Pose2& pose, const Vec2& p) {
  return rotate({p.x - pose.x, p.y - pose.y}, -pose.yaw);
}

// a * b (a: parent <- mid, b: mid <- child)
inline Pose2 compose(const Pose2& a, const Pose2& b) {
  const Vec2 t = transformPoint(a, {b.x, b.y});
  return {t.x, t.y, wrapAngle(a.yaw + b.yaw)};
}

}  // namespace tracker
}  // namespace kebi_follow
