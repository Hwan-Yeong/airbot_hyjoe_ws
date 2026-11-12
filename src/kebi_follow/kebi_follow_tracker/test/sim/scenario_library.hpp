// 표준 합성 시나리오 모음 (시험과 평가 도구가 같이 쓴다). 좌표: odom, 로봇은 원점에서 +x 를 본다.
#pragma once

#include <algorithm>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "sim/scenario.hpp"

namespace kebi_follow {
namespace tracker {
namespace sim {

inline ScenarioSpec baseSpec(const std::string& name, bool camera,
                             unsigned seed) {
  ScenarioSpec s;
  s.name = name + (camera ? "_cam" : "_lidar");
  s.seed = seed;
  s.camera.enabled = camera;
  s.params.acquire.require_camera = camera;
  return s;
}

inline WalkerSpec person(const Vec2& at, double heading,
                         std::vector<Waypoint> path = {},
                         double start_time = 0.0) {
  WalkerSpec w;
  w.start = at;
  w.heading = heading;
  w.path = std::move(path);
  w.start_time = start_time;
  return w;
}

// 가구를 시나리오 정적 물체로 옮긴다
template <typename Fn>
void addStatic(ScenarioSpec& s, Fn&& build) {
  World w;
  build(w);
  s.circles.insert(s.circles.end(), w.circles.begin(), w.circles.end());
  s.walls.insert(s.walls.end(), w.walls.begin(), w.walls.end());
}

// 시나리오 공통 시각: 사람은 0 s 부터 제자리로 걸어 들어와 선다(움직인 이력). 2 s 에 ACQUIRE,
// 획득 뒤 3.6 s 에 TRACK, 4.5 s 부터 본 동작.
constexpr double kAcquireAt = 2.0;
constexpr double kTrackAt = 3.6;
constexpr double kActionAt = 4.5;
constexpr double kWalkInSpeed = 0.8;

// from 에서 at 으로 걸어 들어와 선 뒤 (kActionAt 까지) path 를 이어 걷는다
inline WalkerSpec walkIn(const Vec2& from, const Vec2& at,
                         std::vector<Waypoint> path = {},
                         double hold_until = kActionAt) {
  WalkerSpec w;
  w.start = from;
  w.heading = bearing(at - from);
  const double arrive =
      distance(from, at) / kWalkInSpeed + kWalkInSpeed / w.accel;
  w.path.push_back({at, kWalkInSpeed, std::max(0.0, hold_until - arrive)});
  w.path.insert(w.path.end(), path.begin(), path.end());
  return w;
}

// 앞 1.0 m 에 선 사람 1명 획득
inline ScenarioSpec acquireStatic(bool camera, unsigned seed) {
  ScenarioSpec s = baseSpec("acquire_static", camera, seed);
  s.duration = 6.0;
  s.walkers = {walkIn({1.9, 0.9}, {1.0, 0.05}, {}, 100.0)};
  s.modes = {{kAcquireAt, Mode::ACQUIRE}};
  return s;
}

// 영역 안에 두 명 → 획득하지 않고, 한 명이 떠나면 남은 사람을 획득
inline ScenarioSpec acquireTwoPeople(bool camera, unsigned seed) {
  ScenarioSpec s = baseSpec("acquire_two", camera, seed);
  s.duration = 11.0;
  s.walkers = {
      walkIn({1.9, 1.0}, {1.0, 0.2}, {}, 100.0),
      walkIn({2.0, -1.0}, {1.2, -0.3}, {{{2.5, -2.5}, 1.0, 0.0}}, 6.0)};
  s.modes = {{kAcquireAt, Mode::ACQUIRE}};
  return s;
}

// 획득 후 걷는 사람을 따라간다 (속도 v): 직진 · 왼쪽 90° · 멈춤 2 s · 왼쪽 · 오른쪽
inline ScenarioSpec followWalk(double v, bool camera, unsigned seed) {
  char name[32];
  std::snprintf(name, sizeof(name), "follow_walk_%.1f", v);
  ScenarioSpec s = baseSpec(name, camera, seed);
  s.duration = 36.0;
  s.walkers = {walkIn({1.8, 0.7}, {1.0, 0.0},
                      {{{5.0, 0.0}, v, 0.0},
                       {{5.0, 3.5}, v, 2.0},
                       {{2.0, 3.5}, v, 0.0},
                       {{2.0, 6.5}, v, 0.0}})};
  s.modes = {{kAcquireAt, Mode::ACQUIRE}, {kTrackAt, Mode::TRACK}};
  s.robot.follow_start = kTrackAt;
  // 추적기 시험이므로 빠른 보행에서는 로봇도 따라가게 한다 (SPEC S1 상한 0.5 m/s 는 controller 몫)
  s.robot.v_max = std::max(s.robot.v_max, v - 0.1);
  return s;
}

// followWalk 경로 옆의 의자·탁자·벽·통 (로봇-사람 시선을 막지 않게 둔다)
inline void addFurniture(ScenarioSpec& s) {
  addStatic(s, [](World& w) {
    addChair(w, {3.0, -0.8}, 0.3);
    addChair(w, {3.0, 0.9}, -0.2);
    addTable(w, {6.3, 1.5}, kPi / 2.0);
    addChair(w, {3.5, 4.4}, 0.0);
    addBox(w, {1.0, 2.2}, 0.0, 0.4, 0.4);            // 상자
    w.circles.push_back({{6.2, 0.9}, 0.15});         // 휴지통
    w.walls.push_back({{-1.0, -1.6}, {7.0, -1.6}});  // 복도 벽
    w.walls.push_back({{7.0, -1.6}, {7.0, 6.0}});
  });
}

inline ScenarioSpec followClutter(bool camera, unsigned seed) {
  ScenarioSpec s = followWalk(1.0, camera, seed);
  s.name = camera ? "follow_clutter_cam" : "follow_clutter_lidar";
  addFurniture(s);
  return s;
}

// 사람 없이 가구만 → 락이 생기면 안 된다 (카메라는 가끔 오검출)
inline ScenarioSpec clutterOnly(bool camera, unsigned seed) {
  ScenarioSpec s = baseSpec("clutter_only", camera, seed);
  s.duration = 15.0;
  s.target_walker = -1;
  s.camera.false_rate = 0.05;
  addStatic(s, [](World& w) {
    addChair(w, {1.1, 0.0}, 0.2);  // 획득 영역 한가운데 의자
    addTable(w, {2.2, -0.6}, 0.1);
    w.circles.push_back({{0.9, 0.5}, 0.15});  // 휴지통 (덩어리처럼 보임)
    w.circles.push_back({{1.4, -0.35}, 0.04});  // 스탠드 기둥
    addBox(w, {1.8, 0.8}, 0.6, 0.35, 0.35);
    w.walls.push_back({{3.0, -2.0}, {3.0, 2.0}});
  });
  s.modes = {{0.5, Mode::ACQUIRE}};
  return s;
}

// 다른 사람이 로봇과 타깃 사이를 가로지른다 (타깃은 2 m 에 서 있음, 로봇 정지)
inline ScenarioSpec crossing(double cross_x, double speed, bool camera,
                             unsigned seed) {
  char name[40];
  std::snprintf(name, sizeof(name), "crossing_x%.1f_v%.1f", cross_x, speed);
  ScenarioSpec s = baseSpec(name, camera, seed);
  s.duration = 20.0;
  s.walkers = {
      walkIn({1.8, 0.8}, {1.0, 0.0}, {{{2.0, 0.0}, 0.6, 0.0}}),
      person({cross_x, 2.5}, -kPi / 2.0, {{{cross_x, -2.5}, speed, 0.0}}, 8.0)};
  s.modes = {{kAcquireAt, Mode::ACQUIRE}, {kTrackAt, Mode::TRACK}};
  return s;
}

// 걷는 타깃을 따라가는 중 다른 사람이 가로지른다
inline ScenarioSpec crossingWhileFollowing(bool camera, unsigned seed) {
  ScenarioSpec s = baseSpec("crossing_follow", camera, seed);
  s.duration = 18.0;
  s.walkers = {walkIn({1.8, 0.7}, {1.0, 0.0}, {{{6.0, 0.0}, 0.7, 0.0}}),
               person({3.6, 2.5}, -kPi / 2.0, {{{3.6, -2.5}, 1.2, 0.0}}, 7.4)};
  s.modes = {{kAcquireAt, Mode::ACQUIRE}, {kTrackAt, Mode::TRACK}};
  s.robot.follow_start = kTrackAt;
  return s;
}

// 칸막이 뒤로 숨었다가 다시 나온다 → OCCLUDED → LOST → 재획득
inline ScenarioSpec occlusionReacquire(bool camera, unsigned seed) {
  ScenarioSpec s = baseSpec("occlusion_reacquire", camera, seed);
  s.duration = 18.0;
  // 마지막에 로봇 쪽으로 반 걸음 (옆모습이면 다리 하나만 보여 사람 형상이 안 된다)
  s.walkers = {walkIn({1.9, -0.8}, {1.2, 0.0},
                      {{{1.6, 1.3}, 0.6, 3.5},
                       {{1.3, 0.3}, 0.6, 0.0},
                       {{1.15, 0.25}, 0.4, 0.0}})};
  s.walls.push_back(
      {{1.3, 0.75}, {1.3, 1.9}});  // 칸막이 (로봇에서 보면 사람을 가린다)
  s.modes = {{kAcquireAt, Mode::ACQUIRE}, {kTrackAt, Mode::TRACK}};
  return s;
}

// 타깃은 서 있고 로봇이 제자리 회전 (0.8 rad/s, 6 s) → odom 일관성
inline ScenarioSpec rotateInPlace(bool camera, unsigned seed) {
  ScenarioSpec s = baseSpec("rotate_in_place", camera, seed);
  s.duration = 12.0;
  s.walkers = {walkIn({1.9, 1.0}, {1.2, 0.2}, {}, 100.0)};
  s.modes = {{kAcquireAt, Mode::ACQUIRE}, {kTrackAt, Mode::TRACK}};
  s.robot.rotate_start = kActionAt;
  s.robot.rotate_end = kActionAt + 6.0;
  s.robot.rotate_rate = 0.8;
  return s;
}

// 치마·넓은 바지 (다리 대신 원 하나) 로 걷기
inline ScenarioSpec skirtWalk(bool camera, unsigned seed) {
  ScenarioSpec s = followWalk(1.0, camera, seed);
  s.name = camera ? "skirt_walk_cam" : "skirt_walk_lidar";
  s.walkers[0].skirt_radius = 0.22;
  return s;
}

// 로봇 정지, 사람이 1 m ↔ 3.5 m 를 오간다 (먼 거리 다리 점 수 부족)
inline ScenarioSpec farWalk(bool camera, unsigned seed) {
  ScenarioSpec s = baseSpec("far_walk", camera, seed);
  s.duration = 24.0;
  s.walkers = {walkIn({1.8, 0.7}, {1.0, 0.0},
                      {{{3.5, 0.3}, 1.0, 1.0},
                       {{1.0, 0.0}, 1.0, 1.0},
                       {{3.5, -0.3}, 1.4, 1.0},
                       {{1.2, 0.0}, 1.4, 0.0}})};
  s.modes = {{kAcquireAt, Mode::ACQUIRE}, {kTrackAt, Mode::TRACK}};
  return s;
}

// 다른 사람과 0.7 m 간격으로 나란히 걷는다 (두 번째 사람은 획득 영역 밖 35° 에서 기다린다)
inline ScenarioSpec sideBySide(bool camera, unsigned seed) {
  ScenarioSpec s = baseSpec("side_by_side", camera, seed);
  s.duration = 18.0;
  s.walkers = {walkIn({1.8, -0.7}, {1.0, 0.0}, {{{7.0, 0.0}, 0.8, 0.0}}),
               walkIn({1.7, 1.4}, {1.0, 0.7}, {{{7.0, 0.7}, 0.8, 0.0}})};
  s.modes = {{kAcquireAt, Mode::ACQUIRE}, {kTrackAt, Mode::TRACK}};
  s.robot.follow_start = kTrackAt;
  return s;
}

inline std::vector<ScenarioSpec> allScenarios(bool camera, unsigned seed) {
  return {
      acquireStatic(camera, seed),
      acquireTwoPeople(camera, seed),
      clutterOnly(camera, seed),
      followWalk(0.5, camera, seed),
      followWalk(1.0, camera, seed),
      followWalk(1.4, camera, seed),
      followClutter(camera, seed),
      skirtWalk(camera, seed),
      farWalk(camera, seed),
      crossing(1.0, 1.0, camera, seed),
      crossing(1.6, 1.2, camera, seed),
      crossingWhileFollowing(camera, seed),
      sideBySide(camera, seed),
      occlusionReacquire(camera, seed),
      rotateInPlace(camera, seed),
  };
}

}  // namespace sim
}  // namespace tracker
}  // namespace kebi_follow
