#include "kebi_follow_detector/depth_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace kebi_follow
{

namespace
{

constexpr double kMadToSigma = 1.4826;
constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
constexpr double kMaxLegSeparationM = 0.8;  // two legs farther apart are not one person
constexpr double kDiscRadiusBoxRatio = 0.1;  // single visible knee/ankle: disc radius / box height
constexpr double kParallaxMinRangeM = 0.4;  // parallax pre-check is exact beyond this range
constexpr float kParallaxSlackPx = 10.0f;
constexpr float kGraySaturation = 0.2f;     // below: hue is unreliable, use brightness bins
constexpr float kDarkValue = 0.15f;
constexpr double kRangeBinM = 0.01;         // range histogram resolution of the cluster search
constexpr double kProjectionMargin = 0.5;   // pinhole projection beyond this (x image size) is rejected

struct Point
{
  float x;
  float y;
  float z;
  float r;  // horizontal range from base_link
  float u;  // RGB pixel
  float v;
};

struct Rect
{
  float u0{0.0f};
  float v0{0.0f};
  float u1{0.0f};
  float v1{0.0f};
  bool contains(float u, float v) const {return u >= u0 && u <= u1 && v >= v0 && v <= v1;}
  bool empty() const {return u1 <= u0 || v1 <= v0;}
};

struct Capsule
{
  bool active{false};
  float ax{0.0f};
  float ay{0.0f};
  float bx{0.0f};
  float by{0.0f};
  float half_width{0.0f};
  float score{0.0f};
  Rect bounds;

  float distance(float u, float v) const
  {
    const float dx = bx - ax;
    const float dy = by - ay;
    const float len2 = dx * dx + dy * dy;
    float t = len2 > 0.0f ? ((u - ax) * dx + (v - ay) * dy) / len2 : 0.0f;
    t = std::clamp(t, 0.0f, 1.0f);
    return std::hypot(u - (ax + t * dx), v - (ay + t * dy));
  }
};

struct Region
{
  Rect person;
  std::array<Capsule, 2> legs;
};

struct Accumulator
{
  int person_total{0};
  int person_valid{0};
  std::vector<Point> person_points;
  std::array<int, 2> leg_total{0, 0};
  std::array<int, 2> leg_valid{0, 0};
  std::array<std::vector<Point>, 2> leg_points;
};

struct Cluster
{
  bool ok{false};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double spread{0.0};
  float r_lo{0.0f};  // members: r_lo <= r <= r_hi
  float r_hi{0.0f};
  int n{0};
};

Rect clamp_rect(Rect r, int width, int height)
{
  r.u0 = std::max(r.u0, 0.0f);
  r.v0 = std::max(r.v0, 0.0f);
  r.u1 = std::min(r.u1, static_cast<float>(width - 1));
  r.v1 = std::min(r.v1, static_cast<float>(height - 1));
  return r;
}

Region build_region(const Detection & det, const FusionConfig & c, int width, int height)
{
  Region reg;
  const Rect box = clamp_rect({det.x1, det.y1, det.x2, det.y2}, width, height);
  const float bw = det.x2 - det.x1;
  const float bh = det.y2 - det.y1;
  const auto visible = [&](int k) {
      return det.has_keypoints && det.keypoints[k].visibility >= c.keypoint_vis_threshold &&
             box.contains(det.keypoints[k].x, det.keypoints[k].y);
    };
  const int knees[2] = {kLeftKnee, kRightKnee};
  const int ankles[2] = {kLeftAnkle, kRightAnkle};
  bool any = false;
  float hu0 = 1e9f;
  float hv0 = 1e9f;
  float hu1 = -1e9f;
  for (int side = 0; side < 2; ++side) {
    const bool kv = visible(knees[side]);
    const bool av = visible(ankles[side]);
    if (!kv && !av) {
      continue;
    }
    any = true;
    Capsule & cap = reg.legs[side];
    cap.active = true;
    const Keypoint & a = det.keypoints[kv ? knees[side] : ankles[side]];
    const Keypoint & b = det.keypoints[av ? ankles[side] : knees[side]];
    cap.ax = a.x;
    cap.ay = a.y;
    cap.bx = b.x;
    cap.by = b.y;
    const float len = std::hypot(b.x - a.x, b.y - a.y);
    if (kv && av && len > 0.0f) {
      const float ext = static_cast<float>(c.leg_foot_extension_ratio);
      cap.bx = b.x + (b.x - a.x) * ext;  // include the foot below the ankle
      cap.by = b.y + (b.y - a.y) * ext;
      cap.half_width = std::max(
        static_cast<float>(c.leg_min_half_width_px),
        static_cast<float>(c.leg_half_width_ratio) * len);
      cap.score = det.score * std::min(a.visibility, b.visibility);
    } else {
      cap.half_width = std::max(
        static_cast<float>(c.leg_min_half_width_px),
        static_cast<float>(kDiscRadiusBoxRatio) * bh);
      cap.score = det.score * a.visibility;
    }
    cap.bounds = clamp_rect(
      {std::min(cap.ax, cap.bx) - cap.half_width, std::min(cap.ay, cap.by) - cap.half_width,
        std::max(cap.ax, cap.bx) + cap.half_width, std::max(cap.ay, cap.by) + cap.half_width},
      width, height);
    for (const Keypoint * p : {&a, &b}) {
      hu0 = std::min(hu0, p->x);
      hu1 = std::max(hu1, p->x);
      hv0 = std::min(hv0, p->y);
    }
  }
  if (any) {
    const float m = static_cast<float>(c.keypoint_margin_ratio) * bw;
    reg.person = clamp_rect({hu0 - m, hv0 - m, hu1 + m, box.v1}, width, height);
    reg.person.u0 = std::max(reg.person.u0, box.u0);
    reg.person.u1 = std::min(reg.person.u1, box.u1);
    reg.person.v0 = std::max(reg.person.v0, box.v0);
  } else {
    const float cu = 0.5f * (det.x1 + det.x2);
    const float half = 0.5f * static_cast<float>(c.leg_region_width_fraction) * bw;
    reg.person = clamp_rect(
      {cu - half, det.y2 - static_cast<float>(c.leg_region_fraction) * bh, cu + half, det.y2},
      width, height);
  }
  return reg;
}

float median_of(std::vector<float> & v)
{
  const size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + mid, v.end());
  return v[mid];
}

// Nearest dense range cluster, O(n) with a 1 cm range histogram: the first window of cluster_window_m
// holding enough points starts the cluster, which then extends `extent` metres further.
Cluster near_cluster(const std::vector<Point> & pts, const FusionConfig & c, double extent)
{
  Cluster out;
  const size_t n = pts.size();
  const size_t need = std::max<size_t>(
    static_cast<size_t>(std::max(c.cluster_min_points, 1)),
    static_cast<size_t>(std::ceil(c.cluster_min_fraction * static_cast<double>(n))));
  if (n < need) {
    return out;
  }
  float rmax = 0.0f;
  for (const Point & p : pts) {
    rmax = std::max(rmax, p.r);
  }
  const int bins = static_cast<int>(rmax / kRangeBinM) + 2;
  std::vector<int> hist(static_cast<size_t>(bins), 0);
  for (const Point & p : pts) {
    ++hist[static_cast<size_t>(p.r / kRangeBinM)];
  }
  const int win = std::max(1, static_cast<int>(std::lround(c.cluster_window_m / kRangeBinM)));
  int start = -1;
  size_t sum = 0;
  for (int b = 0; b < bins; ++b) {
    sum += hist[b];
    if (b >= win) {
      sum -= hist[b - win];
    }
    if (sum >= need) {
      start = std::max(0, b - win + 1);
      break;
    }
  }
  if (start < 0) {
    return out;
  }
  while (hist[start] == 0) {
    ++start;
  }
  out.r_lo = static_cast<float>(start * kRangeBinM);
  out.r_hi = out.r_lo + static_cast<float>(extent);
  std::vector<float> xs;
  std::vector<float> ys;
  std::vector<float> zs;
  for (const Point & p : pts) {
    if (p.r >= out.r_lo && p.r <= out.r_hi) {
      xs.push_back(p.x);
      ys.push_back(p.y);
      zs.push_back(p.z);
    }
  }
  out.n = static_cast<int>(xs.size());
  std::vector<float> dev(xs.size());
  out.x = median_of(xs);
  out.y = median_of(ys);
  out.z = median_of(zs);
  size_t k = 0;
  for (const Point & p : pts) {
    if (p.r >= out.r_lo && p.r <= out.r_hi) {
      dev[k++] = static_cast<float>(std::hypot(p.x - out.x, p.y - out.y));
    }
  }
  out.spread = kMadToSigma * median_of(dev);
  out.ok = true;
  return out;
}

double position_std(const FusionConfig & c, double x, double y, double spread)
{
  const double r = std::hypot(x, y);
  const double s = std::hypot(c.depth_noise_k * r * r, c.spread_gain * spread);
  return std::clamp(s, c.position_std_min_m, c.position_std_max_m);
}

void rgb_to_hsv(float r, float g, float b, float & h, float & s, float & v)
{
  const float mx = std::max({r, g, b});
  const float mn = std::min({r, g, b});
  const float d = mx - mn;
  v = mx;
  s = mx > 0.0f ? d / mx : 0.0f;
  if (d <= 0.0f) {
    h = 0.0f;
  } else if (mx == r) {
    h = 60.0f * std::fmod((g - b) / d, 6.0f);
  } else if (mx == g) {
    h = 60.0f * ((b - r) / d + 2.0f);
  } else {
    h = 60.0f * ((r - g) / d + 4.0f);
  }
  if (h < 0.0f) {
    h += 360.0f;
  }
}

std::vector<float> appearance_histogram(
  const std::vector<Point> & pts, float r_lo, float r_hi, const cv::Mat & color, bool bgr,
  const FusionConfig & c)
{
  const int chroma = c.hue_bins * c.sat_bins;
  std::vector<float> hist(static_cast<size_t>(chroma + c.gray_bins), 0.0f);
  float total = 0.0f;
  for (const Point & p : pts) {
    if (p.r < r_lo || p.r > r_hi) {
      continue;
    }
    const int u = static_cast<int>(std::lround(p.u));
    const int v = static_cast<int>(std::lround(p.v));
    if (u < 0 || v < 0 || u >= color.cols || v >= color.rows) {
      continue;
    }
    const cv::Vec3b px = color.at<cv::Vec3b>(v, u);
    const float r = (bgr ? px[2] : px[0]) / 255.0f;
    const float g = px[1] / 255.0f;
    const float b = (bgr ? px[0] : px[2]) / 255.0f;
    float h;
    float s;
    float val;
    rgb_to_hsv(r, g, b, h, s, val);
    int bin;
    if (s < kGraySaturation || val < kDarkValue) {
      bin = chroma + std::min(c.gray_bins - 1, static_cast<int>(val * c.gray_bins));
    } else {
      const int hb = std::min(c.hue_bins - 1, static_cast<int>(h / 360.0f * c.hue_bins));
      const int sb = std::min(
        c.sat_bins - 1,
        static_cast<int>((s - kGraySaturation) / (1.0f - kGraySaturation) * c.sat_bins));
      bin = hb * c.sat_bins + sb;
    }
    hist[bin] += 1.0f;
    total += 1.0f;
  }
  if (total <= 0.0f) {
    return {};
  }
  for (float & x : hist) {
    x /= total;
  }
  return hist;
}

// pixel -> normalized ray of a plumb_bob camera (fixed-point iteration, as cv::undistortPoints)
void undistort(const CameraModel & cam, double u, double v, double & xn, double & yn)
{
  const double x0 = (u - cam.cx) / cam.fx;
  const double y0 = (v - cam.cy) / cam.fy;
  xn = x0;
  yn = y0;
  if (!cam.has_distortion()) {
    return;
  }
  const auto & d = cam.dist;
  constexpr int kIterations = 10;
  for (int it = 0; it < kIterations; ++it) {
    const double r2 = xn * xn + yn * yn;
    const double radial = 1.0 + d[0] * r2 + d[1] * r2 * r2 + d[4] * r2 * r2 * r2;
    const double dx = 2.0 * d[2] * xn * yn + d[3] * (r2 + 2.0 * xn * xn);
    const double dy = d[2] * (r2 + 2.0 * yn * yn) + 2.0 * d[3] * xn * yn;
    xn = (x0 - dx) / radial;
    yn = (y0 - dy) / radial;
  }
}

}  // namespace

// ---------------------------------------------------------------------------------------------------
bool CameraModel::has_distortion() const
{
  for (const double d : dist) {
    if (d != 0.0) {
      return true;
    }
  }
  return false;
}

bool CameraModel::project(double x, double y, double z, double & u, double & v) const
{
  if (z <= 1e-6) {
    return false;
  }
  double xn = x / z;
  double yn = y / z;
  // far outside the view the distortion polynomial folds back into the image: reject those points
  const double u0 = fx * xn + cx;
  const double v0 = fy * yn + cy;
  if (u0 < -kProjectionMargin * width || u0 > (1.0 + kProjectionMargin) * width ||
    v0 < -kProjectionMargin * height || v0 > (1.0 + kProjectionMargin) * height)
  {
    return false;
  }
  if (has_distortion()) {
    const double r2 = xn * xn + yn * yn;
    const double radial = 1.0 + dist[0] * r2 + dist[1] * r2 * r2 + dist[4] * r2 * r2 * r2;
    const double xd = xn * radial + 2.0 * dist[2] * xn * yn + dist[3] * (r2 + 2.0 * xn * xn);
    const double yd = yn * radial + dist[2] * (r2 + 2.0 * yn * yn) + 2.0 * dist[3] * xn * yn;
    xn = xd;
    yn = yd;
  }
  u = fx * xn + cx;
  v = fy * yn + cy;
  return true;
}

void Rigid3::apply(const double in[3], double out[3]) const
{
  out[0] = r[0] * in[0] + r[1] * in[1] + r[2] * in[2] + t[0];
  out[1] = r[3] * in[0] + r[4] * in[1] + r[5] * in[2] + t[1];
  out[2] = r[6] * in[0] + r[7] * in[1] + r[8] * in[2] + t[2];
}

Rigid3 Rigid3::operator*(const Rigid3 & o) const
{
  Rigid3 c;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      c.r[i * 3 + j] = r[i * 3] * o.r[j] + r[i * 3 + 1] * o.r[3 + j] + r[i * 3 + 2] * o.r[6 + j];
    }
    c.t[i] = r[i * 3] * o.t[0] + r[i * 3 + 1] * o.t[1] + r[i * 3 + 2] * o.t[2] + t[i];
  }
  return c;
}

Rigid3 Rigid3::from_quaternion(
  double qx, double qy, double qz, double qw, double tx, double ty,
  double tz)
{
  const double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (n > 0.0) {
    qx /= n;
    qy /= n;
    qz /= n;
    qw /= n;
  }
  Rigid3 o;
  o.r = {1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw),
    2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw),
    2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)};
  o.t = {tx, ty, tz};
  return o;
}

Rigid3 Rigid3::from_xyz_rpy(double x, double y, double z, double roll, double pitch, double yaw)
{
  // fixed-axis roll-pitch-yaw as in URDF: R = Rz(yaw) Ry(pitch) Rx(roll)
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cyw = std::cos(yaw);
  const double syw = std::sin(yaw);
  Rigid3 o;
  o.r = {cyw * cp, cyw * sp * sr - syw * cr, cyw * sp * cr + syw * sr,
    syw * cp, syw * sp * sr + cyw * cr, syw * sp * cr - cyw * sr,
    -sp, cp * sr, cp * cr};
  o.t = {x, y, z};
  return o;
}

Rigid3 Rigid3::rotation_y(double angle)
{
  Rigid3 o;
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  o.r = {c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c};
  return o;
}

// ---------------------------------------------------------------------------------------------------
DepthFusion::DepthFusion(const FusionConfig & config)
: config_(config)
{
  config_.depth_stride = std::max(1, config_.depth_stride);
}

void DepthFusion::set_geometry(
  const CameraModel & rgb, const CameraModel & depth, const Rigid3 & rgb_from_depth,
  const Rigid3 & base_from_rgb)
{
  rgb_ = rgb;
  depth_ = depth;
  rgb_from_depth_ = rgb_from_depth;
  base_from_rgb_ = base_from_rgb;
  ready_ = rgb.valid() && depth.valid();
  if (!ready_) {
    return;
  }
  const int s = config_.depth_stride;
  grid_w_ = (depth.width + s - 1) / s;
  grid_h_ = (depth.height + s - 1) / s;
  const size_t n = static_cast<size_t>(grid_w_) * grid_h_;
  ray_x_.assign(n, 0.0f);
  ray_y_.assign(n, 0.0f);
  nominal_u_.assign(n, kNan);
  nominal_v_.assign(n, kNan);

  for (int gy = 0; gy < grid_h_; ++gy) {
    for (int gx = 0; gx < grid_w_; ++gx) {
      const size_t i = static_cast<size_t>(gy) * grid_w_ + gx;
      double xn;
      double yn;
      undistort(depth, gx * s, gy * s, xn, yn);
      ray_x_[i] = static_cast<float>(xn);
      ray_y_[i] = static_cast<float>(yn);
    }
  }
  const double zn = config_.nominal_depth_m;
  for (size_t i = 0; i < n; ++i) {
    const double pd[3] = {ray_x_[i] * zn, ray_y_[i] * zn, zn};
    double pr[3];
    rgb_from_depth_.apply(pd, pr);
    double u;
    double v;
    if (rgb_.project(pr[0], pr[1], pr[2], u, v)) {
      nominal_u_[i] = static_cast<float>(u);
      nominal_v_[i] = static_cast<float>(v);
    }
  }
  // how far a sample can move from its nominal RGB position (parallax), for the cheap pre-check
  const double baseline = std::hypot(
    rgb_from_depth_.t[0], std::hypot(rgb_from_depth_.t[1], rgb_from_depth_.t[2]));
  const double near = std::max(config_.depth_min_m, kParallaxMinRangeM);
  parallax_margin_px_ = static_cast<float>(
    std::max(rgb_.fx, rgb_.fy) * baseline * std::max(0.0, 1.0 / near - 1.0 / zn)) +
    kParallaxSlackPx;
}

std::vector<PersonEstimate> DepthFusion::fuse(
  const cv::Mat & depth, const cv::Mat & color, bool color_is_bgr,
  const std::vector<Detection> & detections) const
{
  std::vector<PersonEstimate> out(detections.size());
  if (!ready_ || detections.empty() || depth.empty() || depth.cols != depth_.width ||
    depth.rows != depth_.height || (depth.type() != CV_16UC1 && depth.type() != CV_32FC1))
  {
    return out;
  }
  const FusionConfig & c = config_;
  std::vector<Region> regions;
  regions.reserve(detections.size());
  Rect any{1e9f, 1e9f, -1e9f, -1e9f};
  for (const Detection & d : detections) {
    regions.push_back(build_region(d, c, rgb_.width, rgb_.height));
    const Region & r = regions.back();
    std::array<Rect, 3> parts{r.person, r.legs[0].bounds, r.legs[1].bounds};
    for (size_t k = 0; k < parts.size(); ++k) {
      if ((k == 0 || r.legs[k - 1].active) && !parts[k].empty()) {
        any.u0 = std::min(any.u0, parts[k].u0);
        any.v0 = std::min(any.v0, parts[k].v0);
        any.u1 = std::max(any.u1, parts[k].u1);
        any.v1 = std::max(any.v1, parts[k].v1);
      }
    }
  }
  if (any.empty()) {
    return out;
  }
  const Rect coarse{any.u0 - parallax_margin_px_, any.v0 - parallax_margin_px_,
    any.u1 + parallax_margin_px_, any.v1 + parallax_margin_px_};

  std::vector<Accumulator> acc(detections.size());
  const int s = c.depth_stride;
  const bool is16 = depth.type() == CV_16UC1;
  const double r_excl2 = c.robot_exclusion_radius_m * c.robot_exclusion_radius_m;
  for (int gy = 0; gy < grid_h_; ++gy) {
    const int v = gy * s;
    const uint16_t * row16 = is16 ? depth.ptr<uint16_t>(v) : nullptr;
    const float * row32 = is16 ? nullptr : depth.ptr<float>(v);
    for (int gx = 0; gx < grid_w_; ++gx) {
      const size_t i = static_cast<size_t>(gy) * grid_w_ + gx;
      const float nu = nominal_u_[i];
      const float nv = nominal_v_[i];
      if (std::isnan(nu) || !coarse.contains(nu, nv)) {
        continue;
      }
      const int u = gx * s;
      const double z = is16 ? row16[u] * c.depth_unit_m : static_cast<double>(row32[u]);
      const bool valid = std::isfinite(z) && z >= c.depth_min_m && z <= c.depth_max_m;
      double pr[3] = {0.0, 0.0, 0.0};
      double ur = nu;
      double vr = nv;
      if (valid) {
        const double pd[3] = {ray_x_[i] * z, ray_y_[i] * z, z};
        rgb_from_depth_.apply(pd, pr);
        if (!rgb_.project(pr[0], pr[1], pr[2], ur, vr)) {
          continue;
        }
      }
      const float fu = static_cast<float>(ur);
      const float fv = static_cast<float>(vr);
      if (!any.contains(fu, fv)) {
        continue;
      }
      bool have_base = false;
      bool usable = false;
      Point p{};
      for (size_t d = 0; d < detections.size(); ++d) {
        const Region & reg = regions[d];
        const bool in_person = reg.person.contains(fu, fv);
        int leg_side = -1;
        float best = std::numeric_limits<float>::max();
        for (int side = 0; side < 2; ++side) {
          const Capsule & cap = reg.legs[side];
          if (!cap.active || !cap.bounds.contains(fu, fv)) {
            continue;
          }
          const float dist = cap.distance(fu, fv);
          if (dist <= cap.half_width && dist < best) {
            best = dist;
            leg_side = side;
          }
        }
        if (!in_person && leg_side < 0) {
          continue;
        }
        if (valid && !have_base) {
          have_base = true;
          double pb[3];
          base_from_rgb_.apply(pr, pb);
          const double r2 = pb[0] * pb[0] + pb[1] * pb[1];
          usable = pb[2] >= c.floor_height_m && pb[2] <= c.max_height_m && r2 >= r_excl2;
          p = {static_cast<float>(pb[0]), static_cast<float>(pb[1]), static_cast<float>(pb[2]),
            static_cast<float>(std::sqrt(r2)), fu, fv};
        }
        Accumulator & a = acc[d];
        if (in_person) {
          ++a.person_total;
          if (valid) {
            ++a.person_valid;
            if (usable) {
              a.person_points.push_back(p);
            }
          }
        }
        if (leg_side >= 0) {
          ++a.leg_total[leg_side];
          if (valid) {
            ++a.leg_valid[leg_side];
            if (usable) {
              a.leg_points[leg_side].push_back(p);
            }
          }
        }
      }
    }
  }

  for (size_t d = 0; d < detections.size(); ++d) {
    Accumulator & a = acc[d];
    PersonEstimate & est = out[d];
    est.valid_ratio = a.person_total > 0 ?
      static_cast<double>(a.person_valid) / a.person_total : 0.0;
    for (int side = 0; side < 2; ++side) {
      const Capsule & cap = regions[d].legs[side];
      LegEstimate & leg = est.legs[side];
      leg.side = side;
      if (!cap.active) {
        continue;
      }
      leg.score = cap.score;
      leg.u_min = cap.bounds.u0;
      leg.v_min = cap.bounds.v0;
      leg.u_max = cap.bounds.u1;
      leg.v_max = cap.bounds.v1;
      leg.valid_ratio = a.leg_total[side] > 0 ?
        static_cast<double>(a.leg_valid[side]) / a.leg_total[side] : 0.0;
      const Cluster cl = near_cluster(a.leg_points[side], c, c.leg_cluster_extent_m);
      if (cl.ok) {
        leg.valid = true;
        leg.x = cl.x;
        leg.y = cl.y;
        leg.z = cl.z;
        leg.points = cl.n;
        leg.position_std = position_std(c, cl.x, cl.y, cl.spread);
      }
    }
    const Cluster cl = near_cluster(a.person_points, c, c.person_cluster_extent_m);
    const LegEstimate & l0 = est.legs[0];
    const LegEstimate & l1 = est.legs[1];
    if (l0.valid && l1.valid && std::hypot(l0.x - l1.x, l0.y - l1.y) <= kMaxLegSeparationM) {
      est.has_position = true;
      est.from_legs = true;
      est.x = 0.5 * (l0.x + l1.x);
      est.y = 0.5 * (l0.y + l1.y);
      est.z = cl.ok ? cl.z : 0.5 * (l0.z + l1.z);
      est.points = l0.points + l1.points;
      est.position_std = std::clamp(
        0.5 * std::hypot(l0.position_std, l1.position_std), c.position_std_min_m,
        c.position_std_max_m);
    } else if (cl.ok) {
      est.has_position = true;
      est.x = cl.x;
      est.y = cl.y;
      est.z = cl.z;
      est.points = cl.n;
      est.position_std = position_std(c, cl.x, cl.y, cl.spread);
    }
    if (c.appearance && cl.ok && !color.empty() && color.type() == CV_8UC3) {
      est.appearance = appearance_histogram(
        a.person_points, cl.r_lo, cl.r_hi, color, color_is_bgr, c);
    }
  }
  return out;
}

}  // namespace kebi_follow
