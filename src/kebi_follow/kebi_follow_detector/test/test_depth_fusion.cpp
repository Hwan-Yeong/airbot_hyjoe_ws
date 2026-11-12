#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/core.hpp>

#include "kebi_follow_detector/depth_fusion.hpp"

using kebi_follow::CameraModel;
using kebi_follow::DepthFusion;
using kebi_follow::Detection;
using kebi_follow::FusionConfig;
using kebi_follow::Rigid3;

namespace
{

constexpr double kDeg = M_PI / 180.0;
constexpr double kPitch = 9.0 * kDeg;   // camera looks 9 deg down
constexpr double kLegX = 1.2;           // leg axes in base_link [m]
constexpr double kLegY = 0.1;
constexpr double kLegRadius = 0.06;
constexpr double kLegHeight = 0.85;
constexpr double kWallX = 3.0;

Rigid3 inverse(const Rigid3 & a)
{
  Rigid3 o;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      o.r[i * 3 + j] = a.r[j * 3 + i];
    }
  }
  for (int i = 0; i < 3; ++i) {
    o.t[i] = -(o.r[i * 3] * a.t[0] + o.r[i * 3 + 1] * a.t[1] + o.r[i * 3 + 2] * a.t[2]);
  }
  return o;
}

struct Scene
{
  CameraModel rgb;
  CameraModel depth;
  Rigid3 base_from_rgb;
  Rigid3 rgb_from_depth;

  Scene()
  {
    rgb.width = 640;
    rgb.height = 480;
    rgb.fx = rgb.fy = 370.0;  // generic values of the same order as the robot camera
    rgb.cx = 320.0;
    rgb.cy = 268.0;
    depth.width = 848;
    depth.height = 480;
    depth.fx = 424.0;
    depth.fy = 340.0;
    depth.cx = 424.0;
    depth.cy = 240.0;
    Rigid3 optical;  // optical (x right, y down, z forward) -> base axes (x forward, y left, z up)
    optical.r = {0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0};
    base_from_rgb = Rigid3::rotation_y(kPitch) * optical;
    base_from_rgb.t = {0.18, 0.0, 0.26};
    rgb_from_depth.t = {0.06, 0.0, 0.0};
  }

  // ray-cast depth image [mm]: floor z = 0, wall x = 3, two vertical leg cylinders
  cv::Mat render_depth(bool with_legs = true) const
  {
    const Rigid3 base_from_depth = base_from_rgb * rgb_from_depth;
    cv::Mat img(depth.height, depth.width, CV_16UC1, cv::Scalar(0));
    const double * o = base_from_depth.t.data();
    for (int v = 0; v < depth.height; ++v) {
      for (int u = 0; u < depth.width; ++u) {
        const double ray[3] = {(u - depth.cx) / depth.fx, (v - depth.cy) / depth.fy, 1.0};
        double d[3];
        Rigid3 rot = base_from_depth;
        rot.t = {0.0, 0.0, 0.0};
        rot.apply(ray, d);  // t along d is the depth Z
        double best = 1e9;
        if (d[2] < 0.0) {
          best = std::min(best, -o[2] / d[2]);
        }
        if (d[0] > 0.0) {
          best = std::min(best, (kWallX - o[0]) / d[0]);
        }
        if (with_legs) {
          for (const double cy : {kLegY, -kLegY}) {
            const double a = d[0] * d[0] + d[1] * d[1];
            const double b = 2.0 * (d[0] * (o[0] - kLegX) + d[1] * (o[1] - cy));
            const double c = (o[0] - kLegX) * (o[0] - kLegX) + (o[1] - cy) * (o[1] - cy) -
              kLegRadius * kLegRadius;
            const double disc = b * b - 4.0 * a * c;
            if (a > 0.0 && disc >= 0.0) {
              const double t = (-b - std::sqrt(disc)) / (2.0 * a);
              const double z = o[2] + t * d[2];
              if (t > 0.0 && z >= 0.0 && z <= kLegHeight) {
                best = std::min(best, t);
              }
            }
          }
        }
        if (best < 1e8) {
          img.at<uint16_t>(v, u) = static_cast<uint16_t>(std::lround(best * 1000.0));
        }
      }
    }
    return img;
  }

  bool to_pixel(double x, double y, double z, float & u, float & v) const
  {
    const Rigid3 rgb_from_base = inverse(base_from_rgb);
    const double pb[3] = {x, y, z};
    double pr[3];
    rgb_from_base.apply(pb, pr);
    double uu;
    double vv;
    if (!rgb.project(pr[0], pr[1], pr[2], uu, vv)) {
      return false;
    }
    u = static_cast<float>(uu);
    v = static_cast<float>(vv);
    return true;
  }

  // detection covering both legs; keypoints on the leg axes (left = +y leg)
  Detection detection(bool keypoints) const
  {
    Detection d;
    d.score = 0.9f;
    d.x1 = d.y1 = 1e9f;
    d.x2 = d.y2 = -1e9f;
    for (const double cy : {kLegY, -kLegY}) {
      for (double z = 0.0; z <= kLegHeight; z += 0.05) {
        for (double a = 0.0; a < 2.0 * M_PI; a += M_PI / 8.0) {
          float u;
          float v;
          if (to_pixel(kLegX + kLegRadius * std::cos(a), cy + kLegRadius * std::sin(a), z, u, v)) {
            d.x1 = std::min(d.x1, u);
            d.x2 = std::max(d.x2, u);
            d.y1 = std::min(d.y1, v);
            d.y2 = std::max(d.y2, v);
          }
        }
      }
    }
    d.x1 = std::max(d.x1, 0.0f);
    d.y1 = std::max(d.y1, 0.0f);
    d.x2 = std::min(d.x2, 639.0f);
    d.y2 = std::min(d.y2, 479.0f);
    d.has_keypoints = keypoints;
    if (keypoints) {
      const int idx[4] = {kebi_follow::kLeftKnee, kebi_follow::kLeftAnkle, kebi_follow::kRightKnee,
        kebi_follow::kRightAnkle};
      const double ys[4] = {kLegY, kLegY, -kLegY, -kLegY};
      const double zs[4] = {0.45, 0.08, 0.45, 0.08};
      for (int k = 0; k < 4; ++k) {
        auto & p = d.keypoints[idx[k]];
        to_pixel(kLegX - kLegRadius, ys[k], zs[k], p.x, p.y);
        p.visibility = 0.9f;
      }
    }
    return d;
  }
};

}  // namespace

TEST(Rigid3, RpyMatchesQuaternionAndInverse)
{
  const double roll = 0.1;
  const double pitch = -0.3;
  const double yaw = 0.7;
  const Rigid3 a = Rigid3::from_xyz_rpy(1.0, 2.0, 3.0, roll, pitch, yaw);
  // quaternion of Rz(yaw) Ry(pitch) Rx(roll)
  const double cr = std::cos(roll / 2);
  const double sr = std::sin(roll / 2);
  const double cp = std::cos(pitch / 2);
  const double sp = std::sin(pitch / 2);
  const double cy = std::cos(yaw / 2);
  const double sy = std::sin(yaw / 2);
  const Rigid3 b = Rigid3::from_quaternion(
    sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy, 1.0, 2.0, 3.0);
  for (int i = 0; i < 9; ++i) {
    EXPECT_NEAR(a.r[i], b.r[i], 1e-12);
  }
  const Rigid3 id = a * inverse(a);
  for (int i = 0; i < 9; ++i) {
    EXPECT_NEAR(id.r[i], (i % 4 == 0) ? 1.0 : 0.0, 1e-12);
  }
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(id.t[i], 0.0, 1e-12);
  }
}

TEST(Rigid3, PositivePitchLooksDown)
{
  const Rigid3 r = Rigid3::rotation_y(kPitch);
  const double fwd[3] = {1.0, 0.0, 0.0};
  double out[3];
  r.apply(fwd, out);
  EXPECT_NEAR(out[2], -std::sin(kPitch), 1e-12);
}

TEST(CameraModel, ProjectionWithDistortion)
{
  Scene s;
  double u;
  double v;
  ASSERT_TRUE(s.rgb.project(0.0, 0.0, 2.0, u, v));
  EXPECT_NEAR(u, s.rgb.cx, 1e-9);
  EXPECT_NEAR(v, s.rgb.cy, 1e-9);
  EXPECT_FALSE(s.rgb.project(0.0, 0.0, -1.0, u, v));
  CameraModel d = s.rgb;
  d.dist = {-0.05, 0.05, 0.001, -0.001, -0.03};  // mild barrel distortion
  double ud;
  double vd;
  ASSERT_TRUE(d.project(0.5, 0.3, 1.0, ud, vd));
  ASSERT_TRUE(s.rgb.project(0.5, 0.3, 1.0, u, v));
  EXPECT_GT(std::hypot(ud - u, vd - v), 0.1);  // distortion is applied
  EXPECT_LT(std::hypot(ud - u, vd - v), 10.0);
  // 70 deg off axis: the polynomial would fold the point back into the image; it must be rejected
  EXPECT_FALSE(d.project(std::tan(70.0 * kDeg), 0.0, 1.0, ud, vd));
}

TEST(DepthFusion, LegsWithKeypoints)
{
  Scene s;
  DepthFusion f;
  f.set_geometry(s.rgb, s.depth, s.rgb_from_depth, s.base_from_rgb);
  ASSERT_TRUE(f.ready());
  const cv::Mat depth = s.render_depth();
  const Detection det = s.detection(true);
  ASSERT_GT(det.y2 - det.y1, 100.0f);
  const auto est = f.fuse(depth, cv::Mat(), true, {det});
  ASSERT_EQ(est.size(), 1u);
  const auto & e = est[0];
  ASSERT_TRUE(e.has_position);
  EXPECT_TRUE(e.from_legs);
  // visible front surfaces are up to one radius nearer than the axes
  EXPECT_NEAR(e.x, kLegX - 0.5 * kLegRadius, 0.04);
  EXPECT_NEAR(e.y, 0.0, 0.02);
  EXPECT_GT(e.z, 0.05);
  EXPECT_NEAR(e.valid_ratio, 1.0, 1e-9);
  EXPECT_GE(e.position_std, f.config().position_std_min_m);
  EXPECT_LT(e.position_std, 0.1);
  ASSERT_TRUE(e.legs[0].valid);
  ASSERT_TRUE(e.legs[1].valid);
  EXPECT_NEAR(e.legs[0].y, kLegY, 0.03);   // left keypoints were put on the +y leg
  EXPECT_NEAR(e.legs[1].y, -kLegY, 0.03);
  EXPECT_NEAR(e.legs[0].x, kLegX - 0.5 * kLegRadius, 0.04);
  EXPECT_GT(e.legs[0].points, 20);
  EXPECT_GT(e.legs[0].score, 0.0f);
}

TEST(DepthFusion, LowerBoxWithoutKeypoints)
{
  Scene s;
  DepthFusion f;
  f.set_geometry(s.rgb, s.depth, s.rgb_from_depth, s.base_from_rgb);
  const auto est = f.fuse(s.render_depth(), cv::Mat(), true, {s.detection(false)});
  ASSERT_EQ(est.size(), 1u);
  ASSERT_TRUE(est[0].has_position);
  EXPECT_FALSE(est[0].from_legs);
  EXPECT_FALSE(est[0].legs[0].valid);
  EXPECT_NEAR(est[0].x, kLegX - 0.5 * kLegRadius, 0.05);
  EXPECT_NEAR(est[0].y, 0.0, 0.05);
}

TEST(DepthFusion, FloorAndWallOnlyGiveNoPosition)
{
  Scene s;
  DepthFusion f;
  f.set_geometry(s.rgb, s.depth, s.rgb_from_depth, s.base_from_rgb);
  // the box sees only floor (removed by height) and the far wall: the wall is the nearest cluster
  Detection det = s.detection(false);
  const auto est = f.fuse(s.render_depth(false), cv::Mat(), true, {det});
  ASSERT_EQ(est.size(), 1u);
  if (est[0].has_position) {
    EXPECT_GT(est[0].x, kWallX - 0.1);  // never a phantom position at the leg distance
  }
}

TEST(DepthFusion, NoDepthNoPosition)
{
  Scene s;
  DepthFusion f;
  f.set_geometry(s.rgb, s.depth, s.rgb_from_depth, s.base_from_rgb);
  const cv::Mat empty(s.depth.height, s.depth.width, CV_16UC1, cv::Scalar(0));
  const auto est = f.fuse(empty, cv::Mat(), true, {s.detection(true)});
  ASSERT_EQ(est.size(), 1u);
  EXPECT_FALSE(est[0].has_position);
  EXPECT_DOUBLE_EQ(est[0].valid_ratio, 0.0);
  // wrong depth size is rejected, not misread
  const cv::Mat wrong(240, 424, CV_16UC1, cv::Scalar(1000));
  EXPECT_FALSE(f.fuse(wrong, cv::Mat(), true, {s.detection(true)})[0].has_position);
}

TEST(DepthFusion, AppearanceHistogram)
{
  Scene s;
  DepthFusion f;
  f.set_geometry(s.rgb, s.depth, s.rgb_from_depth, s.base_from_rgb);
  const cv::Mat color(480, 640, CV_8UC3, cv::Scalar(200, 60, 30));  // BGR blue trousers
  const auto est = f.fuse(s.render_depth(), color, true, {s.detection(true)});
  ASSERT_EQ(est.size(), 1u);
  const auto & h = est[0].appearance;
  ASSERT_EQ(h.size(), static_cast<size_t>(8 * 4 + 4));
  // RGB (30, 60, 200): hue 229.4 deg -> bin 5 of 8, saturation 0.85 -> bin 3 of 4
  EXPECT_NEAR(h[5 * 4 + 3], 1.0f, 1e-6f);
  float sum = 0.0f;
  for (const float x : h) {
    sum += x;
  }
  EXPECT_NEAR(sum, 1.0f, 1e-5f);
}
