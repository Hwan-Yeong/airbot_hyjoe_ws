// 합성 2D 라이다 세계: 정적 원·선분 + 걷는 사람(다리 두 개 또는 치마 원) + 광선 추적.
// 로봇 비의존 시험 도구 (단위 시험·시나리오 평가에서 쓴다).
#pragma once

#include <random>
#include <vector>

#include "kebi_follow_tracker/geometry.hpp"

namespace kebi_follow {
namespace tracker {
namespace sim {

struct Circle {
  Vec2 c;
  double r{0.0};
};

struct Wall {
  Vec2 a;
  Vec2 b;
};

struct LidarModel {
  int count{401};
  double angle_min{-3.14};
  double angle_increment{0.0157};
  double range_min{0.03};
  double range_max{8.0};
  double noise_std{0.006};    // [m] 거리 잡음 (가까운 곳)
  double noise_per_m{0.003};  // [m/m] 거리에 비례해 늘어나는 잡음
  double dropout_prob{0.01};  // 무반사 확률
  // 경계 빔이 앞·뒤 물체 사이 거리로 나오는 확률 (mixed pixel)
  double edge_mixed_prob{0.3};
  Pose2 mount;  // base_link <- 센서
};

struct Waypoint {
  Vec2 p;
  double speed{1.0};  // [m/s] 이 지점까지 걷는 속도
  double pause{0.0};  // [s] 도착 후 서 있는 시간
};

struct WalkerSpec {
  Vec2 start;
  double heading{0.0};
  double start_time{0.0};  // 이 시각부터 경로를 걷는다
  std::vector<Waypoint> path;
  double leg_radius{0.06};
  double hip_half_width{0.09};  // 두 다리 중심 좌우 간격의 절반
  double skirt_radius{0.0};     // > 0 이면 다리 대신 원 하나
  double accel{0.8};            // [m/s^2]
  double turn_rate{3.0};        // [rad/s]
};

// 보행 모델: 디딤 다리는 바닥에 머물고 흔드는 다리가 앞으로 나간다 (디딤 60%, 흔듦 40%)
class Walker {
 public:
  explicit Walker(const WalkerSpec& spec);
  void step(double t, double dt);
  Vec2 center() const { return pos_; }
  Vec2 velocity() const;
  double heading() const { return heading_; }
  double speed() const { return speed_; }
  int legs(Circle out[2]) const;  // 보이는 원 수 (1: 치마, 2: 다리)
  const WalkerSpec& spec() const { return spec_; }

 private:
  WalkerSpec spec_;
  Vec2 pos_;
  double heading_{0.0};
  double speed_{0.0};
  double phase_{0.0};
  double amplitude_{0.0};
  std::size_t wp_{0};
  double pause_left_{0.0};
};

class World {
 public:
  std::vector<Circle> circles;
  std::vector<Wall> walls;
  std::vector<Walker> walkers;

  void step(double t, double dt);
  // origin 에서 angle 방향으로 처음 닿는 거리 (없으면 +inf). hit_walker: 맞은 사람 번호 또는 -1
  double raycast(const Vec2& origin, double angle,
                 int* hit_walker = nullptr) const;
};

// 로봇 자세(odom <- base_link)에서 본 LaserScan ranges 를 만든다 (무반사 = +inf)
void simulateScan(const World& world, const Pose2& robot_pose,
                  const LidarModel& lidar, std::mt19937& rng,
                  std::vector<float>& ranges);

// 정적 물체 도우미
void addChair(World& world, const Vec2& center, double yaw, double size = 0.42,
              double leg_radius = 0.018);
void addTable(World& world, const Vec2& center, double yaw, double length = 1.2,
              double width = 0.7, double leg_radius = 0.025);
void addBox(World& world, const Vec2& center, double yaw, double length,
            double width);

}  // namespace sim
}  // namespace tracker
}  // namespace kebi_follow
