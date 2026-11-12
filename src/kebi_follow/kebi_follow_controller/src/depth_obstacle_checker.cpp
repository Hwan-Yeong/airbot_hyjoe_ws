#include "kebi_follow_controller/depth_obstacle_checker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>

namespace kebi_follow {

namespace {

// 프레임마다 같은 씨앗: 같은 입력 → 같은 결과
constexpr uint32_t kRngSeed = 0x9E3779B9u;
constexpr uint32_t kRngMul = 1664525u;  // LCG (Numerical Recipes)
constexpr uint32_t kRngAdd = 1013904223u;
constexpr int kRngShift = 8;  // LCG 하위 비트는 주기가 짧아 버린다
constexpr double kDegenerateDet = 1e-9;  // 세 점이 거의 한 직선
constexpr double kMinBinDeg = 0.1;
constexpr int kMinSamplesForPlane = 3;
constexpr double kMsPerSecond = 1000.0;

// 행 우선 3x3 곱
void matMul(const double A[9], const double B[9], double out[9]) {
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out[r * 3 + c] = A[r * 3 + 0] * B[0 * 3 + c] +
                       A[r * 3 + 1] * B[1 * 3 + c] +
                       A[r * 3 + 2] * B[2 * 3 + c];
    }
  }
}

// x 축 회전
void rotX(double angle, double out[9]) {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  const double m[9] = {1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c};
  std::memcpy(out, m, sizeof(m));
}

double det3(double a, double b, double c, double d, double e, double f,
            double g, double h, double i) {
  return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
}

// [m00 m01 m02; m10 m11 m12; m20 m21 m22] x = r (Cramer)
bool solve3(const double m[9], const double r[3], double x[3]) {
  const double d = det3(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
  if (std::abs(d) < kDegenerateDet) {
    return false;
  }
  x[0] = det3(r[0], m[1], m[2], r[1], m[4], m[5], r[2], m[7], m[8]) / d;
  x[1] = det3(m[0], r[0], m[2], m[3], r[1], m[5], m[6], r[2], m[8]) / d;
  x[2] = det3(m[0], m[1], r[0], m[3], m[4], r[1], m[6], m[7], r[2]) / d;
  return true;
}

bool hostIsBigEndian() {
  const uint16_t probe = 1;
  uint8_t first = 0;
  std::memcpy(&first, &probe, 1);
  return first == 0;
}

}  // namespace

CameraExtrinsics toOpticalExtrinsics(const double R[9], const double t[3],
                                     FrameConvention conv, bool* optical) {
  // R·e_z 와 R·e_x 중 전방(base x) 성분이 큰 쪽이 카메라 정면
  bool is_optical = true;
  if (conv == FrameConvention::kBody) {
    is_optical = false;
  } else if (conv == FrameConvention::kAuto) {
    is_optical = R[2] >= R[0];
  }
  CameraExtrinsics T;
  std::copy(t, t + 3, T.t);
  if (is_optical) {
    std::copy(R, R + 9, T.R);
  } else {
    // 본체 좌표 = (z, -x, -y)_광학
    const double B[9] = {0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0};
    matMul(R, B, T.R);
  }
  if (optical != nullptr) {
    *optical = is_optical;
  }
  return T;
}

DepthObstacleChecker::DepthObstacleChecker(const DepthCheckerParams& p)
    : p_(p) {
  p_.stride = std::max(1, p_.stride);
  k_ = std::clamp(p_.min_points_per_bin, 1, Bin::kMaxK);
  bin_rad_ = degToRad(std::max(kMinBinDeg, p_.bin_deg));
  bins_.resize(static_cast<std::size_t>(std::ceil(2.0 * kPi / bin_rad_)));
  subset_.reserve(static_cast<std::size_t>(std::max(1, p_.floor_max_samples)));
}

void DepthObstacleChecker::configure(const CameraIntrinsics& K,
                                     const CameraExtrinsics& T,
                                     double pitch_correction_rad) {
  K_ = K;
  // 숙임 보정은 카메라 자체 가로축(광학 x) 기준 회전: 아래로 숙이면 광학 x 축으로 -pitch
  double rx[9];
  rotX(-pitch_correction_rad, rx);
  double M[9];
  matMul(T.R, rx, M);
  for (int i = 0; i < 3; ++i) {
    t_[i] = static_cast<float>(T.t[i]);
  }
  // 광축(광학 z)의 base 방향으로 기준 숙임을 구한다 (+ 가 아래)
  nominal_pitch_deg_ = radToDeg(std::atan2(-M[8], std::hypot(M[2], M[5])));

  const int v_end =
      (p_.max_row >= 0) ? std::min(p_.max_row, K.height) : K.height;
  cols_ = (K.width + p_.stride - 1) / p_.stride;
  rows_ = (std::max(0, v_end) + p_.stride - 1) / p_.stride;
  const std::size_t n = static_cast<std::size_t>(cols_) * rows_;
  ray_x_.assign(n, 0.0f);
  ray_y_.assign(n, 0.0f);
  ray_z_.assign(n, 0.0f);
  for (int j = 0; j < rows_; ++j) {
    const double oy = (j * p_.stride - K.cy) / K.fy;
    for (int i = 0; i < cols_; ++i) {
      const double ox = (i * p_.stride - K.cx) / K.fx;
      const std::size_t idx = static_cast<std::size_t>(j) * cols_ + i;
      ray_x_[idx] = static_cast<float>(M[0] * ox + M[1] * oy + M[2]);
      ray_y_[idx] = static_cast<float>(M[3] * ox + M[4] * oy + M[5]);
      ray_z_[idx] = static_cast<float>(M[6] * ox + M[7] * oy + M[8]);
    }
  }
  px_.assign(n, 0.0f);
  py_.assign(n, 0.0f);
  pz_.assign(n, 0.0f);
  cand_.clear();
  cand_.reserve(n);
  near_cand_.clear();
  near_cand_.reserve(n);
  // 기준 자세에서 광선이 로봇 앞 가까운 바닥에 닿는 표본 (유효 depth 비율 판단용)
  near_mask_.assign(n, 0);
  near_total_ = 0;
  for (std::size_t idx = 0; idx < n; ++idx) {
    // 바닥을 향하지 않음
    if (ray_z_[idx] >= 0.0f) {
      continue;
    }
    const double s = -t_[2] / ray_z_[idx];
    const double x = t_[0] + s * ray_x_[idx];
    const double y = t_[1] + s * ray_y_[idx];
    if (x >= p_.near_region_min && x <= p_.near_region_max &&
        std::abs(y) <= p_.near_region_half_width) {
      near_mask_[idx] = 1;
      ++near_total_;
    }
  }
  have_last_ = false;
  boot_count_ = 0;
  boot_sum_ = FloorPlane{};
  configured_ = (K.fx > 0.0 && K.fy > 0.0 && n > 0);
}

uint32_t DepthObstacleChecker::nextRandom() {
  rng_ = rng_ * kRngMul + kRngAdd;
  return rng_ >> kRngShift;
}

bool DepthObstacleChecker::fitFloor(const FloorPlane& ref, double max_tilt_rad,
                                    double max_offset, bool near_only,
                                    FloorPlane* plane, int* inliers,
                                    int* near_inliers) {
  const int nn = static_cast<int>(near_cand_.size());
  if (nn < std::max(p_.floor_near_min_inliers, kMinSamplesForPlane)) {
    return false;
  }
  // 가설은 가까운 띠 점에서만
  subset_.clear();
  const int max_samples = std::max(kMinSamplesForPlane, p_.floor_max_samples);
  const int step = (nn + max_samples - 1) / max_samples;
  for (int i = 0; i < nn; i += step) {
    subset_.push_back(near_cand_[i]);
  }
  const int ns = static_cast<int>(subset_.size());
  const double tan_tilt = std::tan(max_tilt_rad);
  const double thr = p_.floor_inlier_threshold;
  // 기준 평면에서 벗어난 정도
  auto within = [&](const double x[3]) {
    return std::hypot(x[0] - ref.a, x[1] - ref.b) <= tan_tilt &&
           std::abs(x[2] - ref.c) <= max_offset;
  };

  rng_ = kRngSeed;
  int best_count = 0;
  double best[3] = {0.0, 0.0, 0.0};
  for (int it = 0; it < p_.floor_iterations; ++it) {
    const int i1 = static_cast<int>(nextRandom() % ns);
    const int i2 = static_cast<int>(nextRandom() % ns);
    const int i3 = static_cast<int>(nextRandom() % ns);
    if (i1 == i2 || i2 == i3 || i1 == i3) {
      continue;
    }
    const int a = subset_[i1];
    const int b = subset_[i2];
    const int c = subset_[i3];
    const double m[9] = {px_[a], py_[a], 1.0,    px_[b], py_[b],
                         1.0,    px_[c], py_[c], 1.0};
    const double r[3] = {pz_[a], pz_[b], pz_[c]};
    double x[3];
    if (!solve3(m, r, x) || !within(x)) {
      continue;
    }
    int count = 0;
    for (int s = 0; s < ns; ++s) {
      const int k = subset_[s];
      if (std::abs(pz_[k] - (x[0] * px_[k] + x[1] * py_[k] + x[2])) < thr) {
        ++count;
      }
    }
    if (count > best_count) {
      best_count = count;
      std::copy(x, x + 3, best);
    }
  }
  if (best_count < kMinSamplesForPlane) {
    return false;
  }

  // 가설 평면 근처 점으로 최소제곱 (가까운 띠만 또는 전체 후보)
  const std::vector<int>& pool = near_only ? near_cand_ : cand_;
  double sxx = 0, sxy = 0, sx = 0, syy = 0, sy = 0, sn = 0;
  double sxz = 0, syz = 0, sz = 0;
  for (int k : pool) {
    const double x = px_[k];
    const double y = py_[k];
    const double z = pz_[k];
    if (std::abs(z - (best[0] * x + best[1] * y + best[2])) >= thr) {
      continue;
    }
    sxx += x * x;
    sxy += x * y;
    sx += x;
    syy += y * y;
    sy += y;
    sn += 1.0;
    sxz += x * z;
    syz += y * z;
    sz += z;
  }
  if (sn < p_.floor_min_inliers) {
    return false;
  }
  const double m[9] = {sxx, sxy, sx, sxy, syy, sy, sx, sy, sn};
  const double r[3] = {sxz, syz, sz};
  double x[3];
  if (!solve3(m, r, x) || !within(x)) {
    return false;
  }
  int near = 0;
  for (int k : near_cand_) {
    if (std::abs(pz_[k] - (x[0] * px_[k] + x[1] * py_[k] + x[2])) < thr) {
      ++near;
    }
  }
  plane->a = x[0];
  plane->b = x[1];
  plane->c = x[2];
  *inliers = static_cast<int>(sn);
  *near_inliers = near;
  return near >= p_.floor_near_min_inliers;
}

bool DepthObstacleChecker::process(const uint8_t* data, std::size_t step,
                                   int width, int height, bool big_endian,
                                   double now, DepthResult* out) {
  if (!configured_ || data == nullptr || out == nullptr || width != K_.width ||
      height != K_.height ||
      step < static_cast<std::size_t>(width) * sizeof(uint16_t)) {
    return false;
  }
  const auto t_start = std::chrono::steady_clock::now();
  const bool swap = (big_endian != hostIsBigEndian());

  const float band = static_cast<float>(p_.floor_band);
  const float fmin2 =
      static_cast<float>(p_.floor_min_distance * p_.floor_min_distance);
  const float fmax2 =
      static_cast<float>(p_.floor_max_distance * p_.floor_max_distance);
  const float scale = static_cast<float>(p_.depth_scale);
  const float zmin = static_cast<float>(p_.min_depth);
  const float zmax = static_cast<float>(p_.max_depth);

  // 1) 표본 픽셀 → base 좌표 점
  int n = 0;
  int near_valid = 0;
  cand_.clear();
  near_cand_.clear();
  const float near_min = static_cast<float>(p_.floor_near_min);
  const float near_max = static_cast<float>(p_.floor_near_max);
  for (int j = 0; j < rows_; ++j) {
    const uint8_t* row = data + static_cast<std::size_t>(j * p_.stride) * step;
    const std::size_t base = static_cast<std::size_t>(j) * cols_;
    for (int i = 0; i < cols_; ++i) {
      uint16_t raw;
      std::memcpy(&raw, row + static_cast<std::size_t>(i * p_.stride) * 2, 2);
      if (swap) {
        raw = static_cast<uint16_t>((raw >> 8) | (raw << 8));
      }
      // 무효
      if (raw == 0) {
        continue;
      }
      const float z = raw * scale;
      if (z < zmin || z > zmax) {
        continue;
      }
      const std::size_t idx = base + i;
      near_valid += near_mask_[idx];
      const float x = t_[0] + z * ray_x_[idx];
      const float y = t_[1] + z * ray_y_[idx];
      const float h = t_[2] + z * ray_z_[idx];
      px_[n] = x;
      py_[n] = y;
      pz_[n] = h;
      if (std::abs(h) < band) {
        const float r2 = x * x + y * y;
        if (r2 >= fmin2 && r2 <= fmax2) {
          cand_.push_back(n);
          if (x >= near_min && x <= near_max) {
            near_cand_.push_back(n);
          }
        }
      }
      ++n;
    }
  }
  out->valid_samples = n;
  out->near_valid_ratio =
      (near_total_ > 0) ? static_cast<double>(near_valid) / near_total_ : 0.0;

  // 2) 바닥 평면 (기준 평면으로 조용히 대신하지 않는다)
  const bool tracking = have_last_ && now - last_plane_time_ <= p_.floor_hold_s;
  int inliers = 0;
  int near_inliers = 0;
  if (!p_.floor_fit) {
    // 명시 설정: 추정 없이 기준 자세 평면 고정 (시험·비교용)
    last_plane_ = FloorPlane{};
    last_plane_time_ = now;
    have_last_ = true;
    out->floor_source = FloorSource::kFitted;
  } else {
    // 추적: 고정 anchor 대비 좁은 범위, 시작·복구: 기준 자세 대비 넓은 범위 + 가까운 띠만
    const FloorPlane ref = tracking ? anchor_ : FloorPlane{};
    const double tilt_gate = degToRad(tracking ? p_.floor_max_tilt_change_deg
                                               : p_.floor_max_tilt_deg);
    const double offset_gate =
        tracking ? p_.floor_max_offset_change : p_.floor_max_offset;
    FloorPlane fit;
    const bool fitted = fitFloor(ref, tilt_gate, offset_gate, !tracking, &fit,
                                 &inliers, &near_inliers);
    if (fitted && tracking) {
      // 출력 평면의 프레임당 변화 제한 (anchor 는 그대로)
      const double rate = std::tan(degToRad(p_.floor_rate_tilt_deg));
      last_plane_.a += std::clamp(fit.a - last_plane_.a, -rate, rate);
      last_plane_.b += std::clamp(fit.b - last_plane_.b, -rate, rate);
      last_plane_.c += std::clamp(fit.c - last_plane_.c, -p_.floor_rate_offset,
                                  p_.floor_rate_offset);
      last_plane_time_ = now;
      out->floor_source = FloorSource::kFitted;
    } else if (fitted) {
      // 시작·복구: 서로 맞는 추정이 연속 bootstrap_frames 번 → 평균을 anchor 로
      const double tight = std::tan(degToRad(p_.floor_max_tilt_change_deg));
      const bool consistent =
          boot_count_ > 0 &&
          std::hypot(fit.a - boot_plane_.a, fit.b - boot_plane_.b) <= tight &&
          std::abs(fit.c - boot_plane_.c) <= p_.floor_max_offset_change;
      if (!consistent) {
        boot_count_ = 0;
        boot_sum_ = FloorPlane{};
      }
      ++boot_count_;
      boot_sum_.a += fit.a;
      boot_sum_.b += fit.b;
      boot_sum_.c += fit.c;
      boot_plane_ = fit;
      if (boot_count_ >= p_.floor_bootstrap_frames) {
        anchor_ =
            FloorPlane{boot_sum_.a / boot_count_, boot_sum_.b / boot_count_,
                       boot_sum_.c / boot_count_};
        last_plane_ = anchor_;
        last_plane_time_ = now;
        have_last_ = true;
        boot_count_ = 0;
        boot_sum_ = FloorPlane{};
        out->floor_source = FloorSource::kFitted;
      } else {
        out->floor_source =
            have_last_ ? FloorSource::kStale : FloorSource::kNone;
      }
    } else if (tracking) {
      out->floor_source = FloorSource::kHeld;
    } else {
      boot_count_ = 0;
      boot_sum_ = FloorPlane{};
      out->floor_source = have_last_ ? FloorSource::kStale : FloorSource::kNone;
    }
  }
  out->floor_inliers = inliers;
  out->floor_near_inliers = near_inliers;
  out->healthy = (out->floor_source == FloorSource::kFitted ||
                  out->floor_source == FloorSource::kHeld) &&
                 out->near_valid_ratio >= p_.min_near_valid_ratio;
  const FloorPlane plane = last_plane_;
  out->floor = plane;
  out->pitch_deg = nominal_pitch_deg_ + radToDeg(std::atan(plane.a));
  out->points.clear();
  out->points.reserve(bins_.size());  // 첫 프레임 뒤로는 할당 없음
  // 믿을 평면이 없으면 장애물 점도 내지 않는다
  if (out->floor_source == FloorSource::kNone) {
    out->obstacle_samples = 0;
    out->proc_ms = std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - t_start)
                       .count() *
                   kMsPerSecond;
    return true;
  }

  // 3) 장애물 점 → 방위 bin
  for (auto& b : bins_) {
    b.count = 0;
  }
  const double inv_norm =
      1.0 / std::sqrt(1.0 + plane.a * plane.a + plane.b * plane.b);
  const double h_min = p_.min_obstacle_height - p_.height_tolerance;
  const double h_max = p_.max_obstacle_height;
  const double body2 = p_.body_exclusion_radius * p_.body_exclusion_radius;
  const double maxd2 = p_.max_obstacle_distance * p_.max_obstacle_distance;
  const int nbins = static_cast<int>(bins_.size());
  int n_obs = 0;
  for (int k = 0; k < n; ++k) {
    const double x = px_[k];
    const double y = py_[k];
    const double h =
        (pz_[k] - (plane.a * x + plane.b * y + plane.c)) * inv_norm;
    if (h < h_min || h > h_max) {
      continue;
    }
    const double r2 = x * x + y * y;
    if (r2 < body2 || r2 > maxd2) {
      continue;
    }
    ++n_obs;
    const float r = static_cast<float>(std::sqrt(r2));
    int bi = static_cast<int>((std::atan2(y, x) + kPi) / bin_rad_);
    bi = std::clamp(bi, 0, nbins - 1);
    Bin& b = bins_[bi];
    if (b.count >= k_ && r >= b.r[k_ - 1]) {
      continue;
    }
    // r 오름차순 삽입, k_ 개 유지
    int pos = std::min(b.count, k_ - 1);
    while (pos > 0 && b.r[pos - 1] > r) {
      b.r[pos] = b.r[pos - 1];
      b.x[pos] = b.x[pos - 1];
      b.y[pos] = b.y[pos - 1];
      --pos;
    }
    b.r[pos] = r;
    b.x[pos] = static_cast<float>(x);
    b.y[pos] = static_cast<float>(y);
    b.count = std::min(b.count + 1, k_);
  }
  out->obstacle_samples = n_obs;
  for (const auto& b : bins_) {
    if (b.count >= k_) {
      out->points.push_back({b.x[k_ - 1], b.y[k_ - 1]});
    }
  }
  out->proc_ms =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start)
          .count() *
      kMsPerSecond;
  return true;
}

}  // namespace kebi_follow
