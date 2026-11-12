// 시험용 합성 depth 영상: 바닥(z=0) + 축 정렬 상자 + 세로 원기둥(다리)
// 카메라 자세는 로봇 URDF 와 같은 구조: base_link → camera_link(x 0.177, z 0.26, yaw -0.05, pitch)
// → 광학 회전(rpy -90°, 0, -90°) → inusensor_depth (광학 x 로 +0.03)
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "kebi_follow_controller/depth_obstacle_checker.hpp"

namespace kebi_follow {
namespace synth {

struct Box {
  double x0, x1, y0, y1, z0, z1;
};

struct Cylinder {
  double cx, cy, r, z0, z1;
};

struct Scene {
  std::vector<Box> boxes;
  std::vector<Cylinder> cylinders;
  bool floor{true};
  double floor_max_dist{
      1e9};  // 이보다 먼 바닥은 depth 없음 (광택·검은 바닥) [m, base 수평]
};

// 행 우선 3x3
inline void mul(const double A[9], const double B[9], double out[9]) {
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out[r * 3 + c] =
          A[r * 3] * B[c] + A[r * 3 + 1] * B[3 + c] + A[r * 3 + 2] * B[6 + c];
    }
  }
}

// ROS rpy → R = Rz(yaw) Ry(pitch) Rx(roll)
inline void rpy(double roll, double pitch, double yaw, double R[9]) {
  const double cr = std::cos(roll), sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  const double Rz[9] = {cy, -sy, 0, sy, cy, 0, 0, 0, 1};
  const double Ry[9] = {cp, 0, sp, 0, 1, 0, -sp, 0, cp};
  const double Rx[9] = {1, 0, 0, 0, cr, -sr, 0, sr, cr};
  double t[9];
  mul(Rz, Ry, t);
  mul(t, Rx, R);
}

constexpr double kCamX = 0.177;
constexpr double kCamZ = 0.26;
constexpr double kCamYaw = -0.05;
constexpr double kDepthOffset = 0.03;  // 광학 x 로

// base ← inusensor_depth (광학). pitch_deg: camera_link pitch (+ 아래)
inline CameraExtrinsics urdfExtrinsics(double pitch_deg) {
  double Rcl[9], Ropt[9];
  rpy(0.0, pitch_deg * M_PI / 180.0, kCamYaw, Rcl);
  rpy(-M_PI / 2.0, 0.0, -M_PI / 2.0, Ropt);
  CameraExtrinsics T;
  mul(Rcl, Ropt, T.R);
  // inusensor_depth 원점 = camera_link + R·(0.03, 0, 0)_광학
  T.t[0] = kCamX + T.R[0] * kDepthOffset;
  T.t[1] = 0.0 + T.R[3] * kDepthOffset;
  T.t[2] = kCamZ + T.R[6] * kDepthOffset;
  return T;
}

// 광선 p(z) = t + z d (z = 광학 깊이)와 상자의 교차: 가장 가까운 양의 z
inline bool hitBox(const double t[3], const double d[3], const Box& b,
                   double* z) {
  double lo = 1e-6, hi = 1e9;
  const double mn[3] = {b.x0, b.y0, b.z0};
  const double mx[3] = {b.x1, b.y1, b.z1};
  for (int k = 0; k < 3; ++k) {
    if (std::abs(d[k]) < 1e-12) {
      if (t[k] < mn[k] || t[k] > mx[k])
        return false;
      continue;
    }
    double a = (mn[k] - t[k]) / d[k];
    double c = (mx[k] - t[k]) / d[k];
    if (a > c)
      std::swap(a, c);
    lo = std::max(lo, a);
    hi = std::min(hi, c);
    if (lo > hi)
      return false;
  }
  *z = lo;
  return true;
}

inline bool hitCylinder(const double t[3], const double d[3], const Cylinder& c,
                        double* z) {
  const double ox = t[0] - c.cx, oy = t[1] - c.cy;
  const double A = d[0] * d[0] + d[1] * d[1];
  const double B = 2.0 * (ox * d[0] + oy * d[1]);
  const double C = ox * ox + oy * oy - c.r * c.r;
  const double disc = B * B - 4 * A * C;
  if (A < 1e-12 || disc < 0)
    return false;
  const double s = (-B - std::sqrt(disc)) / (2 * A);
  if (s <= 0)
    return false;
  const double h = t[2] + s * d[2];
  if (h < c.z0 || h > c.z1)
    return false;
  *z = s;
  return true;
}

struct Image {
  int width{848};
  int height{480};
  std::vector<uint8_t> data;  // 16UC1 little-endian
  std::size_t step() const { return static_cast<std::size_t>(width) * 2; }
  void set(int u, int v, uint16_t mm) {
    std::memcpy(&data[static_cast<std::size_t>(v) * step() + u * 2], &mm, 2);
  }
  uint16_t get(int u, int v) const {
    uint16_t mm;
    std::memcpy(&mm, &data[static_cast<std::size_t>(v) * step() + u * 2], 2);
    return mm;
  }
};

// true_pitch_deg 자세의 카메라로 장면을 찍는다 (mm 양자화, 범위 밖은 0)
inline Image render(const CameraIntrinsics& K, double true_pitch_deg,
                    const Scene& scene, double max_depth_m = 6.0) {
  Image img;
  img.width = K.width;
  img.height = K.height;
  img.data.assign(img.step() * img.height, 0);
  const CameraExtrinsics T = urdfExtrinsics(true_pitch_deg);
  for (int v = 0; v < K.height; ++v) {
    for (int u = 0; u < K.width; ++u) {
      const double o[3] = {(u - K.cx) / K.fx, (v - K.cy) / K.fy, 1.0};
      double d[3];
      for (int r = 0; r < 3; ++r) {
        d[r] =
            T.R[r * 3] * o[0] + T.R[r * 3 + 1] * o[1] + T.R[r * 3 + 2] * o[2];
      }
      double best = 1e9;
      if (scene.floor && d[2] < 0) {
        const double zf = -T.t[2] / d[2];
        const double fx = T.t[0] + zf * d[0];
        const double fy = T.t[1] + zf * d[1];
        if (std::hypot(fx, fy) <= scene.floor_max_dist) {
          best = zf;
        }
      }
      double z;
      for (const auto& b : scene.boxes) {
        if (hitBox(T.t, d, b, &z) && z < best)
          best = z;
      }
      for (const auto& c : scene.cylinders) {
        if (hitCylinder(T.t, d, c, &z) && z < best)
          best = z;
      }
      if (best < max_depth_m) {
        img.set(u, v, static_cast<uint16_t>(std::lround(best * 1000.0)));
      }
    }
  }
  return img;
}

}  // namespace synth
}  // namespace kebi_follow
