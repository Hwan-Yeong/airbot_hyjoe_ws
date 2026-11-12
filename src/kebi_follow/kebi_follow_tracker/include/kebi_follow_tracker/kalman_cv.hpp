// 2D 등속(CV) 칼만 필터. 상태 [x, y, vx, vy] (odom)
#pragma once

#include <Eigen/Core>

#include "kebi_follow_tracker/geometry.hpp"

namespace kebi_follow {
namespace tracker {

class KalmanCV {
 public:
  void init(const Vec2& z, double pos_std, double vel_std);
  // 이산 백색 가속 잡음 모델. damping_tau_s > 0 이면 속도를 exp(-dt/tau) 로 줄인다 (가림 중 예측)
  void predict(double dt, double accel_std, double damping_tau_s = 0.0);
  // 측정 z 의 혁신 공분산 S = P_pos + R. d2 = Mahalanobis 거리 제곱, log_det = ln|S|
  void innovation(const Vec2& z, double meas_std, double& d2,
                  double& log_det) const;
  void update(const Vec2& z, double meas_std);
  void clampSpeed(double max_speed);

  Vec2 position() const { return {x_(0), x_(1)}; }
  Vec2 velocity() const { return {x_(2), x_(3)}; }
  Eigen::Matrix2d positionCov() const { return P_.topLeftCorner<2, 2>(); }
  // 위치 1-sigma (공분산 최대 고유값의 제곱근)
  double positionStd() const;
  const Eigen::Vector4d& state() const { return x_; }
  const Eigen::Matrix4d& covariance() const { return P_; }

 private:
  Eigen::Vector4d x_{Eigen::Vector4d::Zero()};
  Eigen::Matrix4d P_{Eigen::Matrix4d::Identity()};
};

}  // namespace tracker
}  // namespace kebi_follow
