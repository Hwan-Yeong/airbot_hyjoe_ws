#include "kebi_follow_tracker/leg_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace kebi_follow {
namespace tracker {

namespace {
constexpr std::size_t kReservePoints = 1024;  // 401 빔 + 360° 이음부 복사본
constexpr std::size_t kReserveSegments = 256;
constexpr std::size_t kReserveRays = 1024;
// 첫/끝 빔 중복·한 바퀴 판정 허용 (빔 간격 비율)
constexpr double kRayToleranceRatio = 0.5;
constexpr double kMinChord = 1e-6;  // [m]
constexpr double kMinFitDeterminant = 1e-12;
}  // namespace

LegDetector::LegDetector(const ScanParams& scan_params,
                         const LegParams& leg_params)
    : scan_params_(scan_params), leg_params_(leg_params) {
  points_.reserve(kReservePoints);
  segments_.reserve(kReserveSegments);
  ray_cos_.reserve(kReserveRays);
  ray_sin_.reserve(kReserveRays);
  ray_range_.reserve(kReserveRays);
}

double LegDetector::jumpThreshold(double range, int steps) const {
  return std::max(scan_params_.jump_min,
                  scan_params_.jump_ratio * range * angle_increment_ * steps);
}

void LegDetector::updateRayTable(const ScanView& scan, int ray_count) {
  if (static_cast<int>(ray_cos_.size()) == ray_count &&
      table_angle_min_ == scan.angle_min &&
      table_angle_increment_ == scan.angle_increment) {
    return;
  }
  ray_cos_.resize(ray_count);
  ray_sin_.resize(ray_count);
  for (int i = 0; i < ray_count; ++i) {
    const double a = scan.angle_min + i * scan.angle_increment;
    ray_cos_[i] = std::cos(a);
    ray_sin_[i] = std::sin(a);
  }
  table_angle_min_ = scan.angle_min;
  table_angle_increment_ = scan.angle_increment;
}

void LegDetector::detect(const ScanView& scan, const Pose2& sensor_in_base) {
  points_.clear();
  segments_.clear();
  sensor_in_base_ = sensor_in_base;
  angle_increment_ = std::abs(scan.angle_increment);
  if (scan.ranges == nullptr || scan.count == 0 || angle_increment_ <= 0.0) {
    return;
  }

  // 360° 스캔이면 처음과 끝이 이어진다. 마지막 빔이 첫 빔과 같은 방향이면 뺀다.
  int ray_count = static_cast<int>(scan.count);
  const double span = angle_increment_ * (ray_count - 1);
  if (ray_count > 1 &&
      std::abs(span - kTwoPi) < kRayToleranceRatio * angle_increment_) {
    --ray_count;
  }
  const bool circular = angle_increment_ * ray_count >=
                        kTwoPi - kRayToleranceRatio * angle_increment_;
  updateRayTable(scan, ray_count);
  ray_count_ = ray_count;
  circular_ = circular;
  ray_range_.assign(ray_count, std::numeric_limits<double>::infinity());

  const double min_range = std::max(0.0, scan.range_min);
  const double max_range = scan.range_max > 0.0
                               ? scan.range_max
                               : std::numeric_limits<double>::infinity();
  const double body_radius =
      scan_params_.robot_radius + scan_params_.body_margin;
  const double body_r2 = body_radius * body_radius;
  const double max_r2 = scan_params_.max_range * scan_params_.max_range;
  const double cs = std::cos(sensor_in_base.yaw);
  const double sn = std::sin(sensor_in_base.yaw);

  int seg_first = 0;
  int last_ray = -1;
  for (int i = 0; i < ray_count; ++i) {
    const double r = scan.ranges[i];
    if (!std::isfinite(r) || r < min_range || r > max_range) {
      continue;
    }
    const double sx = r * ray_cos_[i];
    const double sy = r * ray_sin_[i];
    const Vec2 p{sensor_in_base.x + cs * sx - sn * sy,
                 sensor_in_base.y + sn * sx + cs * sy};
    const double d2 = squaredNorm(p);
    if (d2 < body_r2 || d2 > max_r2) {
      continue;  // 로봇 몸체 또는 추적 거리 밖
    }
    ray_range_[i] = r;
    const int idx = static_cast<int>(points_.size());
    if (last_ray >= 0) {
      const ScanPoint& prev = points_.back();
      const int steps = i - last_ray;
      const bool gap_break = steps - 1 > scan_params_.max_invalid_gap;
      if (gap_break ||
          distance(prev.p, p) > jumpThreshold(std::min(prev.range, r), steps)) {
        segments_.push_back(Segment{});
        segments_.back().first = seg_first;
        segments_.back().count = idx - seg_first;
        seg_first = idx;
      }
    }
    points_.push_back({p, r, i});
    last_ray = i;
  }
  if (!points_.empty()) {
    segments_.push_back(Segment{});
    segments_.back().first = seg_first;
    segments_.back().count = static_cast<int>(points_.size()) - seg_first;
  }
  mergeWrapAround(circular, ray_count);

  for (auto& seg : segments_) {
    computeFeatures(seg);
    classify(seg);
  }
}

void LegDetector::mergeWrapAround(bool circular, int ray_count) {
  if (!circular || segments_.size() < 2) {
    return;
  }
  const Segment& head = segments_.front();
  const Segment& tail = segments_.back();
  // 마지막 빔 쪽 끝점
  const ScanPoint a = points_[tail.first + tail.count - 1];
  const ScanPoint b = points_[head.first];  // 첫 빔 쪽 끝점
  const int steps = ray_count - a.ray + b.ray;
  if (steps - 1 > scan_params_.max_invalid_gap ||
      distance(a.p, b.p) > jumpThreshold(std::min(a.range, b.range), steps)) {
    return;
  }
  // 끝 분할 뒤에 첫 분할 점을 복사해 붙여 한 분할로 만든다 (끝 분할이 points_ 의 마지막)
  const int head_first = head.first;
  const int head_count = head.count;
  for (int k = 0; k < head_count; ++k) {
    const ScanPoint copy = points_[head_first + k];
    points_.push_back(copy);
  }
  segments_.back().count += head_count;
  segments_.erase(segments_.begin());
}

// ray 에서 direction(-1/+1) 쪽으로 무효 빔을 건너뛴 첫 이웃이 range 보다 확실히 가까운지
double LegDetector::occluderGap(int ray, int direction, double range) const {
  for (int step = 1; step <= scan_params_.max_invalid_gap + 1; ++step) {
    int j = ray + direction * step;
    if (j < 0 || j >= ray_count_) {
      if (!circular_) {
        return 0.0;
      }
      j = (j + ray_count_) % ray_count_;
    }
    const double r = ray_range_[j];
    if (std::isfinite(r)) {
      return r < range - scan_params_.jump_min ? range - r : 0.0;
    }
  }
  return 0.0;
}

void LegDetector::computeFeatures(Segment& seg) const {
  const int n = seg.count;
  const ScanPoint* pts = points_.data() + seg.first;
  Vec2 sum;
  double range_sum = 0.0;
  for (int k = 0; k < n; ++k) {
    sum = sum + pts[k].p;
    range_sum += pts[k].range;
  }
  seg.centroid = (1.0 / n) * sum;
  seg.range = range_sum / n;
  seg.chord = distance(pts[0].p, pts[n - 1].p);
  seg.width = seg.chord + seg.range * angle_increment_;  // 양끝 바깥 반 빔씩
  seg.occluder_gap_first = occluderGap(pts[0].ray, -1, pts[0].range);
  seg.occluder_gap_last = occluderGap(pts[n - 1].ray, +1, pts[n - 1].range);
  seg.occluded_first = seg.occluder_gap_first > 0.0;
  seg.occluded_last = seg.occluder_gap_last > 0.0;

  // 보이는 면의 중심은 실제 중심보다 센서 쪽에 있다 → 빔 방향으로 밀어 준다
  const Vec2 origin{sensor_in_base_.x, sensor_in_base_.y};
  Vec2 ray_dir = seg.centroid - origin;
  const double ray_len = norm(ray_dir);
  if (ray_len > kMinChord) {
    ray_dir = (1.0 / ray_len) * ray_dir;
  }
  const double offset =
      leg_params_.center_offset_ratio *
      std::min(0.5 * seg.width, leg_params_.center_offset_max_radius);
  seg.center = seg.centroid + offset * ray_dir;

  // 볼록량: 안쪽 점이 양끝 직선에서 센서 쪽으로 나온 평균 거리
  seg.bulge = 0.0;
  if (n >= 3 && seg.chord > kMinChord) {
    const Vec2 u = (1.0 / seg.chord) * (pts[n - 1].p - pts[0].p);
    Vec2 normal{-u.y, u.x};
    if (dot(normal, origin - pts[0].p) < 0.0) {
      normal = -1.0 * normal;
    }
    double s = 0.0;
    for (int k = 1; k < n - 1; ++k) {
      s += dot(pts[k].p - pts[0].p, normal);
    }
    seg.bulge = s / (n - 2);
  }

  // 원 적합 (대수적 최소제곱, 무게중심 기준 좌표)
  seg.arc_valid = false;
  if (n >= leg_params_.arc_min_points) {
    double suu = 0.0, svv = 0.0, suv = 0.0, suuu = 0.0, svvv = 0.0, suvv = 0.0,
           svuu = 0.0;
    for (int k = 0; k < n; ++k) {
      const double u = pts[k].p.x - seg.centroid.x;
      const double v = pts[k].p.y - seg.centroid.y;
      suu += u * u;
      svv += v * v;
      suv += u * v;
      suuu += u * u * u;
      svvv += v * v * v;
      suvv += u * v * v;
      svuu += v * u * u;
    }
    const double det = suu * svv - suv * suv;
    if (std::abs(det) > kMinFitDeterminant) {
      const double bu = 0.5 * (suuu + suvv);
      const double bv = 0.5 * (svvv + svuu);
      const double uc = (bu * svv - bv * suv) / det;
      const double vc = (bv * suu - bu * suv) / det;
      seg.arc_center = {seg.centroid.x + uc, seg.centroid.y + vc};
      seg.arc_radius = std::sqrt(uc * uc + vc * vc + (suu + svv) / n);
      double res = 0.0;
      for (int k = 0; k < n; ++k) {
        const double e = distance(pts[k].p, seg.arc_center) - seg.arc_radius;
        res += e * e;
      }
      seg.arc_residual = std::sqrt(res / n);
      seg.arc_valid = std::isfinite(seg.arc_radius);
    }
  }
}

void LegDetector::classify(Segment& seg) const {
  const LegParams& lp = leg_params_;
  seg.is_leg = false;
  seg.is_blob = false;
  seg.leg_score = 0.0;
  if (seg.bulge < -lp.concave_tol) {
    return;  // 오목 (벽 안쪽 모서리 등)
  }
  if (std::max(seg.occluder_gap_first, seg.occluder_gap_last) >
      lp.max_occluder_gap) {
    return;  // 훨씬 앞 물체(사람 등)의 그림자 가장자리에서 잘린 배경 조각 (뒤 벽 등)
  }
  if (seg.occluded_first && seg.occluded_last) {
    return;  // 양쪽이 앞 물체에 가려 잘린 조각 (다리 사이로 보이는 벽 등). 폭을 알 수 없다
  }
  const int min_points = std::max(
      lp.min_points, static_cast<int>(std::lround(
                         lp.min_point_width / (seg.range * angle_increment_))));
  if (seg.count >= min_points && seg.width >= lp.min_width &&
      seg.width <= lp.max_width) {
    seg.is_leg = true;
    seg.leg_score = 1.0;
    if (seg.count >= lp.arc_min_points) {
      const Vec2 origin{sensor_in_base_.x, sensor_in_base_.y};
      // 원 중심이 보이는 면보다 센서에서 멀어야 볼록한 원호
      const bool convex = seg.arc_valid && dot(seg.arc_center - seg.centroid,
                                               seg.centroid - origin) > 0.0;
      if (!convex || seg.arc_radius > lp.arc_max_radius ||
          seg.arc_residual > lp.arc_max_residual) {
        seg.leg_score = lp.arc_fail_score;
      }
    }
  }
  // 붙은 다리와 평평한 면은 이 라이다 잡음(~1 cm)에서 모양으로 구별되지 않는다.
  // 상자·통 같은 정적 덩어리는 추적 단계(움직임·다리 쌍 이력)에서 낮춘다.
  if (seg.count >= lp.blob_min_points && seg.width >= lp.blob_min_width &&
      seg.width <= lp.blob_max_width) {
    seg.is_blob = true;
  }
}

}  // namespace tracker
}  // namespace kebi_follow
