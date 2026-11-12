// odom 자세 이력 (고정 크기 링 버퍼). stamp 시각의 자세를 선형 보간으로 찾는다. ROS 의존 없음.
#pragma once

#include <cstddef>
#include <vector>

#include "kebi_follow_controller/types.hpp"

namespace kebi_follow {

class PoseHistory {
 public:
  explicit PoseHistory(std::size_t capacity = 512)
      : t_(capacity), p_(capacity) {}

  void clear() {
    start_ = 0;
    size_ = 0;
  }

  // 시간 순서로 넣는다. 시간이 거꾸로 가면(bag 재시작 등) 비우고 다시 쌓는다
  void add(double t, const Pose2D& p) {
    const std::size_t cap = t_.size();
    if (cap == 0) {
      return;
    }
    if (size_ > 0 && t < t_[index(size_ - 1)]) {
      clear();
    }
    if (size_ == cap) {
      start_ = (start_ + 1) % cap;
      --size_;
    }
    const std::size_t i = index(size_);
    t_[i] = t;
    p_[i] = p;
    ++size_;
  }

  bool latest(double* t, Pose2D* p) const {
    if (size_ == 0) {
      return false;
    }
    const std::size_t i = index(size_ - 1);
    *t = t_[i];
    *p = p_[i];
    return true;
  }

  // t 시각 자세. 최신보다 max_ahead 이내로 늦은 t 는 최신 자세를 쓴다. 범위 밖이면 false
  bool at(double t, double max_ahead, Pose2D* out) const {
    if (size_ == 0) {
      return false;
    }
    const std::size_t newest = index(size_ - 1);
    if (t >= t_[newest]) {
      if (t - t_[newest] > max_ahead) {
        return false;
      }
      *out = p_[newest];
      return true;
    }
    if (t < t_[index(0)]) {
      return false;
    }
    // t_[lo] <= t < t_[hi] 인 이웃 찾기
    std::size_t lo = 0;
    std::size_t hi = size_ - 1;
    while (hi - lo > 1) {
      const std::size_t mid = (lo + hi) / 2;
      if (t_[index(mid)] <= t) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    const Pose2D& a = p_[index(lo)];
    const Pose2D& b = p_[index(hi)];
    const double span = t_[index(hi)] - t_[index(lo)];
    const double s = (span > 0.0) ? (t - t_[index(lo)]) / span : 0.0;
    out->x = a.x + s * (b.x - a.x);
    out->y = a.y + s * (b.y - a.y);
    out->yaw = wrapAngle(a.yaw + s * wrapAngle(b.yaw - a.yaw));
    return true;
  }

  std::size_t size() const { return size_; }

 private:
  std::size_t index(std::size_t i) const { return (start_ + i) % t_.size(); }

  std::vector<double> t_;
  std::vector<Pose2D> p_;
  std::size_t start_{0};
  std::size_t size_{0};
};

}  // namespace kebi_follow
