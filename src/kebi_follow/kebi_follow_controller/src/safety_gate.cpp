#include "kebi_follow_controller/safety_gate.hpp"

#include <algorithm>
#include <cmath>

namespace kebi_follow {

namespace {
// 이보다 작으면 직진으로 계산 [1/m]
constexpr double kStraightCurvature = 1e-3;
}  // namespace

double SafetyGate::pathDistance(const Vec2& p, double curvature) const {
  double s = 0.0;
  double lat = 0.0;
  if (std::abs(curvature) < kStraightCurvature) {
    if (p.x <= 0.0) {
      return kInf;
    }
    s = p.x;
    lat = std::abs(p.y);
  } else {
    // 회전 중심 (0, R). 왼쪽 회전 R>0
    const double R = 1.0 / curvature;
    const double aR = std::abs(R);
    const double sgn = (curvature > 0.0) ? 1.0 : -1.0;
    const double phi = std::atan2(p.x, aR - sgn * p.y);  // 중심에서 본 진행 각
    if (phi <= 0.0) {
      return kInf;
    }
    s = phi * aR;
    lat = std::abs(std::hypot(p.x, p.y - R) - aR);
  }
  if (lat > p_.half_width || s > p_.lookahead) {
    return kInf;
  }
  const double r = p_.robot_radius;
  const double front = (lat < r) ? std::sqrt(r * r - lat * lat) : 0.0;
  return std::max(0.0, s - front);
}

bool SafetyGate::excluded(const Vec2& p, const Exclusion& ex) const {
  if (ex.points == nullptr || ex.count == 0) {
    return false;
  }
  const double r2 = p_.target_exclusion_radius * p_.target_exclusion_radius;
  if (ex.count == 1) {
    const double dx = p.x - ex.points[0].x;
    const double dy = p.y - ex.points[0].y;
    return dx * dx + dy * dy < r2;
  }
  for (std::size_t i = 0; i + 1 < ex.count; ++i) {
    // 선분 a-b 까지 거리
    const Vec2& a = ex.points[i];
    const Vec2& b = ex.points[i + 1];
    const double ux = b.x - a.x;
    const double uy = b.y - a.y;
    const double len2 = ux * ux + uy * uy;
    double s = 0.0;
    if (len2 > 0.0) {
      s = std::clamp(((p.x - a.x) * ux + (p.y - a.y) * uy) / len2, 0.0, 1.0);
    }
    const double dx = p.x - (a.x + s * ux);
    const double dy = p.y - (a.y + s * uy);
    if (dx * dx + dy * dy < r2) {
      return true;
    }
  }
  return false;
}

double SafetyGate::corridorDistance(const ObstacleSet& set, double curvature,
                                    bool include_straight,
                                    const Exclusion& exclusion) const {
  double best = kInf;
  if (set.points == nullptr) {
    return best;
  }
  const bool check_arc = std::abs(curvature) >= kStraightCurvature;
  for (std::size_t i = 0; i < set.count; ++i) {
    const Vec2 q = transformPoint(set.to_now, set.points[i]);
    if (excluded(q, exclusion)) {
      continue;
    }
    if (check_arc) {
      best = std::min(best, pathDistance(q, curvature));
    }
    if (include_straight || !check_arc) {
      best = std::min(best, pathDistance(q, 0.0));
    }
  }
  return best;
}

double SafetyGate::minRadius(const ObstacleSet& set) const {
  double best = kInf;
  if (set.points == nullptr) {
    return best;
  }
  for (std::size_t i = 0; i < set.count; ++i) {
    best = std::min(best, norm(transformPoint(set.to_now, set.points[i])));
  }
  return best;
}

double pathCurvature(double v, double w, double min_speed,
                     double max_curvature) {
  if (v < min_speed) {
    return 0.0;
  }
  return std::clamp(w / v, -max_curvature, max_curvature);
}

double obstacleSpeedCap(double d, double stop_margin, double slow_length,
                        double decel, double v_max) {
  if (!std::isfinite(d)) {
    return v_max;
  }
  const double free = d - stop_margin;
  if (free <= 0.0) {
    return 0.0;
  }
  const double ramp =
      (slow_length > 0.0) ? v_max * std::min(1.0, free / slow_length) : v_max;
  const double brake = std::sqrt(2.0 * decel * free);
  return std::min({ramp, brake, v_max});
}

bool obstacleStopRequired(double d, double stop_margin, double v_now,
                          double decel) {
  if (!std::isfinite(d)) {
    return false;
  }
  const double v = std::max(0.0, v_now);
  const double braking = (decel > 0.0) ? v * v / (2.0 * decel) : kInf;
  return d <= stop_margin + braking;
}

}  // namespace kebi_follow
