#include "sim/scan_sim.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace kebi_follow {
namespace tracker {
namespace sim {

namespace {
constexpr double kArriveTol = 0.05;        // [m] 웨이포인트 도착 판정
constexpr double kStanceFraction = 0.6;    // 보행 주기 중 디딤 비율
constexpr double kStepLengthBase = 0.40;   // [m] 보폭 = base + slope * v
constexpr double kStepLengthSlope = 0.25;  // [s]
// 다리 앞뒤 진폭 = ratio * 보폭 (정강이 높이)
constexpr double kAmplitudeRatio = 0.5;
// [m/s] 이 속도 아래에서는 진폭을 줄인다
constexpr double kFullGaitSpeed = 0.3;
constexpr double kMinGaitSpeed = 0.02;  // [m/s]
constexpr double kEdgeJump = 0.10;  // [m] 빔 안에서 거리 차가 이보다 크면 경계
// 경계 판정용 보조 광선 각도 (빔 간격 비율)
constexpr double kSubRayRatio = 0.35;
// 방향 오차가 크면 걸음을 늦춘다 (cos 가중)
constexpr double kHeadingSlowdown = 1.0;

double smoothStep(double u) {
  u = std::clamp(u, 0.0, 1.0);
  return u * u * (3.0 - 2.0 * u);
}

// 보행 위상 φ(0..1) 의 다리 앞뒤 위치 (진폭 1)
double legOffset(double phase) {
  phase -= std::floor(phase);
  if (phase < kStanceFraction) {
    return 1.0 - 2.0 * phase / kStanceFraction;
  }
  return -1.0 +
         2.0 * smoothStep((phase - kStanceFraction) / (1.0 - kStanceFraction));
}

bool rayCircle(const Vec2& o, const Vec2& d, const Circle& c, double& t) {
  const Vec2 f = o - c.c;
  const double b = dot(f, d);
  const double cc = dot(f, f) - c.r * c.r;
  const double disc = b * b - cc;
  if (disc < 0.0) {
    return false;
  }
  const double s = std::sqrt(disc);
  double hit = -b - s;
  if (hit <= 0.0) {
    hit = -b + s;
  }
  if (hit <= 0.0) {
    return false;
  }
  t = hit;
  return true;
}

bool raySegment(const Vec2& o, const Vec2& d, const Wall& w, double& t) {
  const Vec2 e = w.b - w.a;
  const double den = d.x * e.y - d.y * e.x;
  if (std::abs(den) < 1e-12) {
    return false;
  }
  const Vec2 ao = w.a - o;
  const double s = (ao.x * e.y - ao.y * e.x) / den;  // 광선 거리
  const double u = (ao.x * d.y - ao.y * d.x) / den;  // 선분 위치
  if (s <= 0.0 || u < 0.0 || u > 1.0) {
    return false;
  }
  t = s;
  return true;
}
}  // namespace

Walker::Walker(const WalkerSpec& spec)
    : spec_(spec), pos_(spec.start), heading_(spec.heading) {}

Vec2 Walker::velocity() const {
  return {speed_ * std::cos(heading_), speed_ * std::sin(heading_)};
}

void Walker::step(double t, double dt) {
  double v_des = 0.0;
  if (t >= spec_.start_time && wp_ < spec_.path.size()) {
    if (pause_left_ > 0.0) {
      pause_left_ -= dt;
    } else {
      const Waypoint& wp = spec_.path[wp_];
      const Vec2 to = wp.p - pos_;
      const double dist = norm(to);
      if (dist < kArriveTol) {
        pause_left_ = wp.pause;
        ++wp_;
      } else {
        const double err = wrapAngle(bearing(to) - heading_);
        const double turn =
            std::clamp(err, -spec_.turn_rate * dt, spec_.turn_rate * dt);
        heading_ = wrapAngle(heading_ + turn);
        v_des =
            wp.speed *
            std::max(0.0,
                     std::cos(std::min(std::abs(err) * kHeadingSlowdown, kPi)));
        const bool stop_here = wp.pause > 0.0 || wp_ + 1 == spec_.path.size();
        if (stop_here) {
          v_des = std::min(v_des, std::sqrt(2.0 * spec_.accel * dist));
        }
      }
    }
  }
  speed_ += std::clamp(v_des - speed_, -spec_.accel * dt, spec_.accel * dt);
  pos_ = pos_ + (speed_ * dt) * Vec2{std::cos(heading_), std::sin(heading_)};

  // 보행 위상: 걸음 수 = v / 보폭, 한 주기 = 두 걸음
  const double step_len = kStepLengthBase + kStepLengthSlope * speed_;
  if (speed_ > kMinGaitSpeed) {
    phase_ += 0.5 * speed_ / step_len * dt;
  }
  amplitude_ =
      kAmplitudeRatio * step_len * std::min(1.0, speed_ / kFullGaitSpeed);
}

int Walker::legs(Circle out[2]) const {
  if (spec_.skirt_radius > 0.0) {
    out[0] = {pos_, spec_.skirt_radius};
    return 1;
  }
  const Vec2 fwd{std::cos(heading_), std::sin(heading_)};
  const Vec2 left{-fwd.y, fwd.x};
  const double h = spec_.hip_half_width;
  out[0] = {pos_ + h * left + (amplitude_ * legOffset(phase_)) * fwd,
            spec_.leg_radius};
  out[1] = {pos_ - h * left + (amplitude_ * legOffset(phase_ + 0.5)) * fwd,
            spec_.leg_radius};
  return 2;
}

void World::step(double t, double dt) {
  for (auto& w : walkers) {
    w.step(t, dt);
  }
}

double World::raycast(const Vec2& origin, double angle, int* hit_walker) const {
  const Vec2 d{std::cos(angle), std::sin(angle)};
  double best = std::numeric_limits<double>::infinity();
  int who = -1;
  double t = 0.0;
  for (const auto& c : circles) {
    if (rayCircle(origin, d, c, t) && t < best) {
      best = t;
      who = -1;
    }
  }
  for (const auto& w : walls) {
    if (raySegment(origin, d, w, t) && t < best) {
      best = t;
      who = -1;
    }
  }
  Circle legs[2];
  for (std::size_t k = 0; k < walkers.size(); ++k) {
    const int n = walkers[k].legs(legs);
    for (int i = 0; i < n; ++i) {
      if (rayCircle(origin, d, legs[i], t) && t < best) {
        best = t;
        who = static_cast<int>(k);
      }
    }
  }
  if (hit_walker != nullptr) {
    *hit_walker = who;
  }
  return best;
}

void simulateScan(const World& world, const Pose2& robot_pose,
                  const LidarModel& lidar, std::mt19937& rng,
                  std::vector<float>& ranges) {
  const Pose2 sensor = compose(robot_pose, lidar.mount);
  const Vec2 origin{sensor.x, sensor.y};
  std::normal_distribution<double> gauss(0.0, 1.0);
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  const double inf = std::numeric_limits<double>::infinity();
  ranges.assign(lidar.count, std::numeric_limits<float>::infinity());
  const double sub = kSubRayRatio * lidar.angle_increment;
  for (int i = 0; i < lidar.count; ++i) {
    const double a = sensor.yaw + lidar.angle_min + i * lidar.angle_increment;
    double r = world.raycast(origin, a);
    if (lidar.edge_mixed_prob > 0.0) {
      // 빔 폭 안에 경계가 있으면 앞·뒤 사이 거리가 나올 수 있다
      const double r1 = world.raycast(origin, a - sub);
      const double r2 = world.raycast(origin, a + sub);
      const double lo = std::min({r, r1, r2});
      const double hi = std::max({r, r1, r2});
      if (lo < inf && hi - lo > kEdgeJump && uni(rng) < lidar.edge_mixed_prob) {
        r = hi < inf ? lo + uni(rng) * (hi - lo) : lo;
      }
    }
    if (!(r < inf) || r > lidar.range_max) {
      continue;
    }
    if (uni(rng) < lidar.dropout_prob) {
      continue;
    }
    r += (lidar.noise_std + lidar.noise_per_m * r) * gauss(rng);
    if (r < lidar.range_min) {
      continue;
    }
    ranges[i] = static_cast<float>(r);
  }
}

void addChair(World& world, const Vec2& center, double yaw, double size,
              double leg_radius) {
  const double h = 0.5 * size;
  const Vec2 corners[4] = {{h, h}, {h, -h}, {-h, h}, {-h, -h}};
  for (const auto& c : corners) {
    world.circles.push_back({center + rotate(c, yaw), leg_radius});
  }
}

void addTable(World& world, const Vec2& center, double yaw, double length,
              double width, double leg_radius) {
  const double hl = 0.5 * length;
  const double hw = 0.5 * width;
  const Vec2 corners[4] = {{hl, hw}, {hl, -hw}, {-hl, hw}, {-hl, -hw}};
  for (const auto& c : corners) {
    world.circles.push_back({center + rotate(c, yaw), leg_radius});
  }
}

void addBox(World& world, const Vec2& center, double yaw, double length,
            double width) {
  const double hl = 0.5 * length;
  const double hw = 0.5 * width;
  const Vec2 p[4] = {
      center + rotate({hl, hw}, yaw), center + rotate({hl, -hw}, yaw),
      center + rotate({-hl, -hw}, yaw), center + rotate({-hl, hw}, yaw)};
  for (int k = 0; k < 4; ++k) {
    world.walls.push_back({p[k], p[(k + 1) % 4]});
  }
}

}  // namespace sim
}  // namespace tracker
}  // namespace kebi_follow
