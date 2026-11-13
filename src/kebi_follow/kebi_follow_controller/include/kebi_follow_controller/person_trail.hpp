#pragma once

// 사람이 지나간 길 (odom 좌표). 2026-11-13 실차 E2: 사람이 기둥·벽 모서리를 돌아가면 사람에게 곧장 가는
// 직선이 막혀 로봇이 기다리기만 했다. 직선이 막혔을 때 이 길 위 앞쪽 점을 향한다 (사람이 이미 지나간 길).
#include <cstddef>
#include <vector>

#include "kebi_follow_controller/types.hpp"

namespace kebi_follow {

class PersonTrail {
 public:
  void reset() { pts_.clear(); }

  // 사람 위치 p (odom) 를 마지막 점에서 spacing 넘게 떨어졌을 때만 덧붙인다. max_points 를 넘으면 오래된 것부터 버린다
  void add(const Vec2& p, double spacing, std::size_t max_points);

  // 로봇(odom) 에서 가장 가까운 점보다 앞(이미 지나간 부분)을 버린다
  void pruneBehind(const Vec2& robot);

  const std::vector<Vec2>& points() const { return pts_; }
  bool empty() const { return pts_.empty(); }

 private:
  std::vector<Vec2> pts_;
};

}  // namespace kebi_follow
