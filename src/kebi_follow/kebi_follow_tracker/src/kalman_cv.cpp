#include "kebi_follow_tracker/kalman_cv.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/LU>

namespace kebi_follow {
namespace tracker {

void KalmanCV::init(const Vec2& z, double pos_std, double vel_std) {
  x_ << z.x, z.y, 0.0, 0.0;
  P_.setZero();
  P_(0, 0) = P_(1, 1) = pos_std * pos_std;
  P_(2, 2) = P_(3, 3) = vel_std * vel_std;
}

void KalmanCV::predict(double dt, double accel_std, double damping_tau_s) {
  if (dt <= 0.0) {
    return;
  }
  // 감쇠가 있으면 v' = a v, x' = x + tau (1 - a) v
  double a = 1.0;
  double b = dt;
  if (damping_tau_s > 0.0) {
    a = std::exp(-dt / damping_tau_s);
    b = damping_tau_s * (1.0 - a);
  }
  Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
  F(0, 2) = F(1, 3) = b;
  F(2, 2) = F(3, 3) = a;
  const double q = accel_std * accel_std;
  const double dt2 = dt * dt;
  const double q_pp = q * dt2 * dt2 / 4.0;
  const double q_pv = q * dt2 * dt / 2.0;
  const double q_vv = q * dt2;
  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  Q(0, 0) = Q(1, 1) = q_pp;
  Q(0, 2) = Q(2, 0) = Q(1, 3) = Q(3, 1) = q_pv;
  Q(2, 2) = Q(3, 3) = q_vv;
  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q;
}

void KalmanCV::innovation(const Vec2& z, double meas_std, double& d2,
                          double& log_det) const {
  Eigen::Matrix2d S = P_.topLeftCorner<2, 2>();
  S(0, 0) += meas_std * meas_std;
  S(1, 1) += meas_std * meas_std;
  const Eigen::Vector2d y(z.x - x_(0), z.y - x_(1));
  const double det = S.determinant();
  d2 = y.dot(S.inverse() * y);
  log_det = std::log(det);
}

void KalmanCV::update(const Vec2& z, double meas_std) {
  Eigen::Matrix2d S = P_.topLeftCorner<2, 2>();
  S(0, 0) += meas_std * meas_std;
  S(1, 1) += meas_std * meas_std;
  const Eigen::Vector2d y(z.x - x_(0), z.y - x_(1));
  // H = [I 0] 이므로 P H^T = P 의 앞 두 열
  const Eigen::Matrix<double, 4, 2> PHt = P_.leftCols<2>();
  const Eigen::Matrix<double, 4, 2> K = PHt * S.inverse();
  x_ += K * y;
  // Joseph 형태: (I-KH) P (I-KH)^T + K R K^T
  Eigen::Matrix4d IKH = Eigen::Matrix4d::Identity();
  IKH.leftCols<2>() -= K;
  const double r = meas_std * meas_std;
  P_ = IKH * P_ * IKH.transpose() + r * K * K.transpose();
}

void KalmanCV::clampSpeed(double max_speed) {
  const double speed = std::hypot(x_(2), x_(3));
  if (speed > max_speed && speed > 0.0) {
    const double s = max_speed / speed;
    x_(2) *= s;
    x_(3) *= s;
  }
}

double KalmanCV::positionStd() const {
  // 2x2 대칭 행렬 최대 고유값
  const double a = P_(0, 0);
  const double b = P_(0, 1);
  const double d = P_(1, 1);
  const double half_tr = 0.5 * (a + d);
  const double disc = std::sqrt(0.25 * (a - d) * (a - d) + b * b);
  return std::sqrt(std::max(0.0, half_tr + disc));
}

}  // namespace tracker
}  // namespace kebi_follow
