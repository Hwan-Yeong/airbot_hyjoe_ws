#include "kebi_follow_controller/person_trail.hpp"

#include <algorithm>

namespace kebi_follow {

void PersonTrail::add(const Vec2& p, double spacing, std::size_t max_points) {
  if (!pts_.empty() && norm({p.x - pts_.back().x, p.y - pts_.back().y}) < spacing) {
    return;
  }
  pts_.push_back(p);
  if (pts_.size() > max_points) {
    pts_.erase(pts_.begin(), pts_.begin() + static_cast<std::ptrdiff_t>(pts_.size() - max_points));
  }
}

void PersonTrail::pruneBehind(const Vec2& robot) {
  if (pts_.size() < 2) {
    return;
  }
  std::size_t best = 0;
  double best_d = norm({pts_[0].x - robot.x, pts_[0].y - robot.y});
  for (std::size_t i = 1; i < pts_.size(); ++i) {
    const double d = norm({pts_[i].x - robot.x, pts_[i].y - robot.y});
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  pts_.erase(pts_.begin(), pts_.begin() + static_cast<std::ptrdiff_t>(best));
}

}  // namespace kebi_follow
