// 단위 시험용: 합성 세계 + LegTrackerCore 를 scan 단위로 돌린다
#pragma once

#include <random>
#include <vector>

#include "kebi_follow_tracker/leg_tracker_core.hpp"
#include "sim/scan_sim.hpp"

namespace kebi_follow {
namespace tracker {
namespace sim {

constexpr double kHarnessT0 = 1000.0;     // [s]
constexpr double kHarnessSubStep = 0.01;  // [s]
constexpr double kScanPeriod = 0.1;       // [s]

inline TrackerParams lidarOnlyParams() {
  TrackerParams p;
  p.acquire.require_camera = false;
  return p;
}

// 서 있는 사람 (경로 없음): 두 다리가 좌우로 나란히
inline WalkerSpec standingPerson(const Vec2& at, double heading = kPi) {
  WalkerSpec w;
  w.start = at;
  w.heading = heading;
  return w;
}

struct Harness {
  explicit Harness(const TrackerParams& params, unsigned seed = 1)
      : core(params), rng(seed) {
    lidar.edge_mixed_prob = 0.0;
    lidar.dropout_prob = 0.0;
  }

  // 세계를 dt 만큼 진행하고 scan 한 장을 처리한다
  const TargetOutput& scan(double dt = kScanPeriod) {
    const int n = static_cast<int>(dt / kHarnessSubStep + 0.5);
    for (int k = 0; k < n; ++k) {
      t += kHarnessSubStep;
      robot.yaw = wrapAngle(robot.yaw + yaw_rate * kHarnessSubStep);
      world.step(t - kHarnessT0, kHarnessSubStep);
    }
    simulateScan(world, robot, lidar, rng, ranges);
    ScanView view;
    view.ranges = ranges.data();
    view.count = ranges.size();
    view.angle_min = lidar.angle_min;
    view.angle_increment = lidar.angle_increment;
    view.range_min = lidar.range_min;
    view.range_max = lidar.range_max;
    return core.processScan(t, view, lidar.mount, robot);
  }

  // 지금 시각의 사람 위치로 카메라 검출을 넣는다 (indices 의 사람만)
  int camera(const std::vector<int>& indices, double age = 0.0) {
    std::vector<CameraDetection> dets;
    for (int i : indices) {
      CameraDetection d;
      d.position = inverseTransformPoint(robot, world.walkers[i].center());
      d.score = 0.9;
      dets.push_back(d);
    }
    return core.processCamera(t - age, robot, dets);
  }

  const TargetOutput& run(int scans, bool with_camera = false) {
    for (int k = 0; k < scans; ++k) {
      scan();
      if (with_camera) {
        std::vector<int> all;
        for (int i = 0; i < static_cast<int>(world.walkers.size()); ++i) {
          all.push_back(i);
        }
        camera(all);
      }
    }
    return core.target();
  }

  LegTrackerCore core;
  World world;
  LidarModel lidar;
  Pose2 robot;
  double yaw_rate{0.0};
  double t{kHarnessT0};
  std::mt19937 rng;
  std::vector<float> ranges;
};

}  // namespace sim
}  // namespace tracker
}  // namespace kebi_follow
