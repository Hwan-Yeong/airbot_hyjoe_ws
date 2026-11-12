// depth 장애물 검사: 합성 영상 (TF pitch 0 + 보정 9°, 실제 숙임 8.5/9/9.5/10.5°)
#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>

#include "kebi_follow_controller/depth_obstacle_checker.hpp"
#include "kebi_follow_controller/safety_gate.hpp"
#include "synthetic_depth.hpp"

using namespace kebi_follow;

namespace {

constexpr double kNominalPitchDeg = 9.0;  // camera_pitch_correction_deg 기본값
constexpr double kFramePeriod = 0.1;      // depth 약 10 Hz
constexpr int kWarmup = 4;  // 시작 시 평면을 받으려면 연속 3 프레임 필요

DepthObstacleChecker makeChecker(const DepthCheckerParams& p = {}) {
  DepthObstacleChecker checker(p);
  checker.configure(CameraIntrinsics{}, synth::urdfExtrinsics(0.0),
                    degToRad(kNominalPitchDeg));
  return checker;
}

DepthResult feed(DepthObstacleChecker* checker, const synth::Image& img,
                 double now) {
  DepthResult r;
  EXPECT_TRUE(checker->process(img.data.data(), img.step(), img.width,
                               img.height, false, now, &r));
  return r;
}

// 같은 장면을 kWarmup 프레임 보여 준 뒤 마지막 결과
DepthResult run(const synth::Scene& scene, double true_pitch_deg,
                DepthCheckerParams p = DepthCheckerParams{},
                synth::Image* img_out = nullptr) {
  DepthObstacleChecker checker = makeChecker(p);
  synth::Image img = synth::render(CameraIntrinsics{}, true_pitch_deg, scene);
  DepthResult r;
  for (int i = 0; i < kWarmup; ++i) {
    r = feed(&checker, img, i * kFramePeriod);
  }
  if (img_out)
    *img_out = img;
  return r;
}

// 진행 경로(직진) 안 최근접 거리
double corridor(const DepthResult& r, const Vec2* exclusion = nullptr) {
  SafetyGate gate;
  ObstacleSet set{r.points.data(), r.points.size(), Pose2D{}};
  Exclusion ex;
  if (exclusion != nullptr) {
    ex = Exclusion{exclusion, 1};
  }
  return gate.corridorDistance(set, 0.0, true, ex);
}

}  // namespace

class FlatFloor : public ::testing::TestWithParam<double> {};

TEST_P(FlatFloor, NoObstacleAndPitchEstimated) {
  const double pitch = GetParam();
  const DepthResult r = run(synth::Scene{}, pitch);
  EXPECT_EQ(r.floor_source, FloorSource::kFitted);
  EXPECT_TRUE(r.healthy);
  EXPECT_TRUE(r.points.empty()) << "false obstacles: " << r.points.size();
  EXPECT_NEAR(r.pitch_deg, pitch, 0.2);
}

INSTANTIATE_TEST_SUITE_P(Pitch, FlatFloor,
                         ::testing::Values(8.5, 9.0, 9.5, 10.5));

class FixedPlane : public ::testing::TestWithParam<double> {};

TEST_P(FixedPlane, NoObstacleWithinHalfDegree) {
  // 바닥 추정을 끄면(명시 설정) 기준 9° 평면 고정: ±0.5° 에서 2 m 안은 오검출 없음
  DepthCheckerParams p;
  p.floor_fit = false;
  const DepthResult r = run(synth::Scene{}, GetParam(), p);
  EXPECT_EQ(r.floor_source, FloorSource::kFitted);
  EXPECT_TRUE(r.points.empty());
}

INSTANTIATE_TEST_SUITE_P(Pitch, FixedPlane, ::testing::Values(8.5, 9.0, 9.5));

TEST(DepthChecker, BootstrapNeedsConsistentFrames) {
  // 시작 직후에는 평면이 없다 → 장애물 점 없음 + degraded
  DepthObstacleChecker checker = makeChecker();
  const synth::Image img =
      synth::render(CameraIntrinsics{}, 9.0, synth::Scene{});
  DepthResult r = feed(&checker, img, 0.0);
  EXPECT_EQ(r.floor_source, FloorSource::kNone);
  EXPECT_FALSE(r.healthy);
  r = feed(&checker, img, 0.1);
  EXPECT_EQ(r.floor_source, FloorSource::kNone);
  r = feed(&checker, img, 0.2);
  EXPECT_EQ(r.floor_source, FloorSource::kFitted);
  EXPECT_TRUE(r.healthy);
}

class Box5cm : public ::testing::TestWithParam<double> {};

TEST_P(Box5cm, DetectedAt1m) {
  synth::Scene scene;
  scene.boxes.push_back({1.0, 1.1, -0.1, 0.1, 0.0, 0.05});  // 앞면 x=1.0
  const DepthResult r = run(scene, GetParam());
  ASSERT_FALSE(r.points.empty());
  // 로봇 앞면(0.22) 기준 약 0.78 m
  EXPECT_NEAR(corridor(r), 0.78, 0.06);
  for (const auto& p : r.points) {
    EXPECT_GT(p.x, 0.95);
    EXPECT_LT(p.x, 1.15);
    EXPECT_LT(std::abs(p.y), 0.15);
  }
}

INSTANTIATE_TEST_SUITE_P(Pitch, Box5cm, ::testing::Values(8.5, 9.0, 9.5, 10.5));

TEST(DepthChecker, LowBoxBelowThresholdIgnored) {
  synth::Scene scene;
  scene.boxes.push_back({1.0, 1.1, -0.1, 0.1, 0.0, 0.02});  // 2 cm (문턱 높이)
  const DepthResult r = run(scene, 9.0);
  EXPECT_TRUE(r.points.empty());
}

TEST(DepthChecker, FloorHolesNoFalsePositive) {
  // 광택 바닥: 0.7~1.1 m 밖 바닥은 depth 없음 + 무작위 구멍 30%
  DepthObstacleChecker checker = makeChecker();
  synth::Scene scene;
  scene.boxes.push_back({1.0, 1.1, -0.1, 0.1, 0.0, 0.05});
  synth::Image img = synth::render(CameraIntrinsics{}, 9.3, scene);
  uint32_t rng = 12345u;
  for (int v = 0; v < img.height; ++v) {
    for (int u = 0; u < img.width; ++u) {
      rng = rng * 1664525u + 1013904223u;
      const uint16_t mm = img.get(u, v);
      const bool glossy_gap = (v > 300 && (mm < 700 || mm > 1100));
      if ((rng >> 24) < 77 || glossy_gap) {
        img.set(u, v, 0);
      }
    }
  }
  DepthResult r;
  for (int i = 0; i < kWarmup; ++i)
    r = feed(&checker, img, i * kFramePeriod);
  // 상자는 여전히 보이고, 그 밖의 오검출은 없다
  ASSERT_FALSE(r.points.empty());
  EXPECT_TRUE(r.healthy);
  for (const auto& p : r.points) {
    EXPECT_GT(p.x, 0.95);
    EXPECT_LT(p.x, 1.15);
  }
}

TEST(DepthChecker, EmptyFrameIsDegradedNotHealthy) {
  // M1: 무효 프레임(전부 0)은 평면이 hold 안이어도 healthy 가 아니다
  DepthObstacleChecker checker = makeChecker();
  const synth::Image floor_img =
      synth::render(CameraIntrinsics{}, 9.4, synth::Scene{});
  synth::Image empty = floor_img;
  std::fill(empty.data.begin(), empty.data.end(), 0);
  DepthResult r;
  for (int i = 0; i < kWarmup; ++i)
    r = feed(&checker, floor_img, 10.0 + i * 0.1);
  ASSERT_TRUE(r.healthy);
  r = feed(&checker, empty, 11.0);
  EXPECT_EQ(r.floor_source, FloorSource::kHeld);
  EXPECT_DOUBLE_EQ(r.near_valid_ratio, 0.0);
  EXPECT_FALSE(r.healthy);
  r = feed(&checker, empty, 12.5);  // hold 2 s 지남
  EXPECT_EQ(r.floor_source, FloorSource::kStale);
  EXPECT_FALSE(r.healthy);
}

TEST(DepthChecker, NearRegionBlindIsDegraded) {
  // M1: 가까운 바닥(0.6~1.2 m)이 전혀 안 보이고 먼 곳만 보이면 degraded
  DepthObstacleChecker checker = makeChecker();
  synth::Scene scene;
  const synth::Image good = synth::render(CameraIntrinsics{}, 9.0, scene);
  DepthResult r;
  for (int i = 0; i < kWarmup; ++i)
    r = feed(&checker, good, i * kFramePeriod);
  ASSERT_TRUE(r.healthy);
  synth::Image blind = good;
  for (int v = 280; v < blind.height; ++v) {
    for (int u = 0; u < blind.width; ++u)
      blind.set(u, v, 0);
  }
  r = feed(&checker, blind, 1.0);
  EXPECT_LT(r.near_valid_ratio, DepthCheckerParams{}.min_near_valid_ratio);
  EXPECT_FALSE(r.healthy);
}

TEST(DepthChecker, NoSilentNominalFallback) {
  // M2: 바닥이 안 보이는 채로 시작하면 기준 평면을 쓰지 않는다 (점 없음, degraded)
  DepthObstacleChecker checker = makeChecker();
  synth::Scene scene;
  scene.floor = false;
  // 떠 있는 물체만 보임
  scene.boxes.push_back({1.0, 1.1, -0.1, 0.1, 0.12, 0.3});
  const synth::Image img = synth::render(CameraIntrinsics{}, 9.0, scene);
  for (int i = 0; i < 10; ++i) {
    const DepthResult r = feed(&checker, img, i * kFramePeriod);
    EXPECT_EQ(r.floor_source, FloorSource::kNone);
    EXPECT_FALSE(r.healthy);
    EXPECT_TRUE(r.points.empty());
  }
}

// M2: 먼 바닥 depth 가 없고(0.8 m 까지만) 넓고 낮은 상자가 0.6~1.2 m 에 있을 때
//     상자를 바닥으로 받아 놓치면 안 된다: 검출하거나 degraded 여야 한다
struct WideBoxCase {
  double pitch;
  double height;
  double width;
  double front;
};

class FarFloorMissingWideBox : public ::testing::TestWithParam<WideBoxCase> {};

TEST_P(FarFloorMissingWideBox, DetectedOrDegraded) {
  const WideBoxCase c = GetParam();
  synth::Scene clear;
  clear.floor_max_dist = 0.8;
  synth::Scene with_box = clear;
  with_box.boxes.push_back(
      {c.front, c.front + 0.4, -c.width / 2.0, c.width / 2.0, 0.0, c.height});
  const double expect_d = c.front - 0.22;
  const CameraIntrinsics K;
  const synth::Image img_clear = synth::render(K, c.pitch, clear);
  const synth::Image img_box = synth::render(K, c.pitch, with_box);

  // (a) 바닥으로 먼저 평면을 잡은 뒤 상자가 나타남
  {
    DepthObstacleChecker checker = makeChecker();
    double t = 0.0;
    for (int i = 0; i < 5; ++i, t += kFramePeriod)
      feed(&checker, img_clear, t);
    for (int i = 0; i < 10; ++i, t += kFramePeriod) {
      const DepthResult r = feed(&checker, img_box, t);
      const double d = corridor(r);
      const bool detected = std::isfinite(d) && std::abs(d - expect_d) < 0.08;
      EXPECT_TRUE(detected || !r.healthy)
          << "warm: missed at frame " << i << " d=" << d << " src "
          << static_cast<int>(r.floor_source);
      EXPECT_TRUE(detected) << "warm: frame " << i << " d=" << d;
    }
  }
  // (b) 처음부터 상자가 있음 (시작 직후)
  {
    DepthObstacleChecker checker = makeChecker();
    for (int i = 0; i < 10; ++i) {
      const DepthResult r = feed(&checker, img_box, i * kFramePeriod);
      const double d = corridor(r);
      const bool detected = std::isfinite(d) && std::abs(d - expect_d) < 0.08;
      EXPECT_TRUE(detected || !r.healthy)
          << "cold: healthy but missed at frame " << i << " d=" << d;
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
    Sweep, FarFloorMissingWideBox,
    ::testing::Values(
        WideBoxCase{9.0, 0.05, 0.3, 0.6}, WideBoxCase{9.0, 0.05, 0.6, 0.9},
        WideBoxCase{9.0, 0.05, 1.2, 1.0}, WideBoxCase{9.0, 0.05, 1.2, 1.2},
        WideBoxCase{9.0, 0.07, 1.2, 0.6}, WideBoxCase{9.0, 0.07, 0.6, 1.2},
        WideBoxCase{10.5, 0.05, 0.3, 1.2}, WideBoxCase{10.5, 0.05, 1.2, 0.9},
        WideBoxCase{10.5, 0.05, 1.2, 1.2}, WideBoxCase{10.5, 0.07, 1.2, 0.6},
        WideBoxCase{10.5, 0.07, 0.6, 1.0}, WideBoxCase{10.5, 0.07, 1.2, 1.2}));

TEST(DepthChecker, PlaneChangeIsRateLimited) {
  // 추적 중 숙임이 한 번에 0.8° 바뀌면 프레임당 0.5° 까지만 따라간다
  DepthObstacleChecker checker = makeChecker();
  const CameraIntrinsics K;
  const synth::Image a = synth::render(K, 9.0, synth::Scene{});
  const synth::Image b = synth::render(K, 9.8, synth::Scene{});
  DepthResult r;
  for (int i = 0; i < kWarmup; ++i)
    r = feed(&checker, a, i * kFramePeriod);
  ASSERT_NEAR(r.pitch_deg, 9.0, 0.2);
  r = feed(&checker, b, 1.0);
  EXPECT_EQ(r.floor_source, FloorSource::kFitted);
  EXPECT_NEAR(r.pitch_deg, 9.5, 0.15);
  r = feed(&checker, b, 1.1);
  EXPECT_NEAR(r.pitch_deg, 9.8, 0.2);
}

TEST(DepthChecker, FloorFitFailureHoldsThenStale) {
  DepthCheckerParams p;
  p.floor_hold_s = 2.0;
  DepthObstacleChecker checker = makeChecker(p);
  const synth::Image floor_img =
      synth::render(CameraIntrinsics{}, 9.4, synth::Scene{});
  synth::Scene no_floor;
  no_floor.floor = false;
  // 앞이 막힘, 바닥 안 보임
  no_floor.boxes.push_back({0.9, 1.5, -1.0, 1.0, 0.12, 0.8});
  const synth::Image blocked = synth::render(CameraIntrinsics{}, 9.4, no_floor);
  DepthResult r;
  for (int i = 0; i < kWarmup; ++i)
    r = feed(&checker, floor_img, 10.0 + i * 0.1);
  ASSERT_EQ(r.floor_source, FloorSource::kFitted);
  r = feed(&checker, blocked, 11.0);
  EXPECT_EQ(r.floor_source, FloorSource::kHeld);
  EXPECT_FALSE(r.points.empty());  // 막힌 것은 장애물
  r = feed(&checker, blocked, 12.5);
  EXPECT_EQ(r.floor_source, FloorSource::kStale);
  EXPECT_FALSE(r.healthy);
  EXPECT_FALSE(r.points.empty());  // degraded 여도 직전 평면으로 장애물은 낸다
}

TEST(DepthChecker, RobotBodyRowsIgnored) {
  // 아래쪽 행에 로봇 몸체 윗면(z 0.24)이 찍힌다: 너무 가까워 min_depth 밖이거나 몸체 반경 안
  synth::Scene scene;
  scene.boxes.push_back({-0.22, 0.22, -0.22, 0.22, 0.0, 0.24});
  synth::Image img;
  const DepthResult r = run(scene, 9.0, DepthCheckerParams{}, &img);
  EXPECT_TRUE(r.points.empty());
  // 몸체 행에 잡음 depth(0.25~0.28 m)가 섞여도 body_exclusion_radius 안이면 무시돼야 한다
  int noisy = 0;
  for (int v = 400; v < img.height; ++v) {
    for (int u = 0; u < img.width; u += 3) {
      img.set(u, v, static_cast<uint16_t>(250 + (u + v) % 30));
      ++noisy;
    }
  }
  DepthObstacleChecker checker = makeChecker();
  DepthResult r2;
  for (int i = 0; i < kWarmup; ++i)
    r2 = feed(&checker, img, i * kFramePeriod);
  ASSERT_GT(noisy, 0);
  for (const auto& p : r2.points) {
    EXPECT_GT(norm(p), DepthCheckerParams{}.body_exclusion_radius);
  }
}

TEST(DepthChecker, TargetLegsExcludedByGate) {
  // 1.05 m 앞에 선 사람 다리 두 개: 장애물로 나오지만 타깃 제외 반경 안
  synth::Scene scene;
  scene.cylinders.push_back({1.05, 0.1, 0.06, 0.0, 0.9});
  scene.cylinders.push_back({1.05, -0.1, 0.06, 0.0, 0.9});
  const DepthResult r = run(scene, 9.0);
  ASSERT_FALSE(r.points.empty());
  EXPECT_TRUE(std::isfinite(corridor(r)));
  const Vec2 target{1.05, 0.0};
  EXPECT_FALSE(std::isfinite(corridor(r, &target)));
}

TEST(DepthChecker, WallBeyondRangeIgnored) {
  synth::Scene scene;
  scene.boxes.push_back({2.4, 2.5, -2.0, 2.0, 0.0, 2.0});  // 2.4 m 벽
  const DepthResult r = run(scene, 9.0);
  EXPECT_TRUE(r.points.empty());
}

TEST(DepthChecker, FrameConventionAutoDetect) {
  // 광학 프레임 TF 는 그대로, x 정면(본체) 프레임 TF 는 광학으로 바꿔 같은 결과
  const CameraExtrinsics opt = synth::urdfExtrinsics(0.0);
  bool optical = false;
  const CameraExtrinsics a =
      toOpticalExtrinsics(opt.R, opt.t, FrameConvention::kAuto, &optical);
  EXPECT_TRUE(optical);
  double body[9];
  synth::rpy(0.0, 0.0, synth::kCamYaw, body);  // camera_link (x 정면)
  const CameraExtrinsics b =
      toOpticalExtrinsics(body, opt.t, FrameConvention::kAuto, &optical);
  EXPECT_FALSE(optical);
  for (int i = 0; i < 9; ++i) {
    EXPECT_NEAR(a.R[i], opt.R[i], 1e-12);
    EXPECT_NEAR(b.R[i], opt.R[i], 1e-12);
  }
  const CameraExtrinsics forced =
      toOpticalExtrinsics(opt.R, opt.t, FrameConvention::kBody, &optical);
  EXPECT_FALSE(optical);
  // 광학 프레임을 본체로 잘못 보면 광축(R·e_z)의 전방 성분이 틀어진다
  EXPECT_GT(std::abs(forced.R[2] - opt.R[2]), 0.5);
}

TEST(DepthChecker, SideBoxPartlyInCorridor) {
  synth::Scene scene;
  scene.boxes.push_back({1.2, 1.3, -0.2, 0.0, 0.0, 0.1});
  const DepthResult a = run(scene, 9.0);
  ASSERT_FALSE(a.points.empty());
  EXPECT_NEAR(corridor(a), 0.98, 0.06);
}

TEST(DepthChecker, RejectsWrongSize) {
  DepthObstacleChecker checker = makeChecker();
  std::vector<uint8_t> data(640 * 480 * 2, 0);
  DepthResult r;
  EXPECT_FALSE(checker.process(data.data(), 640 * 2, 640, 480, false, 0.0, &r));
}

TEST(DepthChecker, Timing) {
  synth::Scene scene;
  scene.boxes.push_back({1.0, 1.1, -0.1, 0.1, 0.0, 0.05});
  scene.cylinders.push_back({1.5, 0.1, 0.06, 0.0, 0.9});
  DepthObstacleChecker checker = makeChecker();
  const synth::Image img = synth::render(CameraIntrinsics{}, 9.2, scene);
  DepthResult r;
  constexpr int kRuns = 50;
  double total = 0.0, worst = 0.0;
  for (int i = 0; i < kRuns; ++i) {
    r = feed(&checker, img, i * kFramePeriod);
    total += r.proc_ms;
    worst = std::max(worst, r.proc_ms);
  }
  std::printf(
      "[timing] depth process: mean %.2f ms, max %.2f ms (%d samples/frame)\n",
      total / kRuns, worst, r.valid_samples);
  EXPECT_LT(total / kRuns, 10.0);
}
