#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "kebi_follow_detector/yolo_decode.hpp"

using kebi_follow::Detection;
using kebi_follow::DecodeConfig;
using kebi_follow::TensorType;
using kebi_follow::TensorView;

namespace
{

constexpr int kW = 640;
constexpr int kH = 480;
constexpr int kRegMax = 16;
constexpr int kStrides[3] = {8, 16, 32};
constexpr float kNegLogit = -50.0f;

// Float tensors of one model output set, filled with "nothing detected".
struct Outputs
{
  int per_level;
  int reg_max;
  std::vector<std::vector<float>> data;
  std::vector<TensorView> views;

  Outputs(int per_level_, int reg_max_)
  : per_level(per_level_), reg_max(reg_max_)
  {
    for (const int s : kStrides) {
      const int h = kH / s;
      const int w = kW / s;
      add(4 * reg_max, h, w, kNegLogit);
      add(1, h, w, 0.0f);
      if (per_level == 3) {
        add(51, h, w, 0.0f);
      }
    }
  }

  void add(int c, int h, int w, float fill)
  {
    data.emplace_back(static_cast<size_t>(c) * h * w, fill);
    TensorView v;
    v.data = data.back().data();
    v.type = TensorType::kFloat32;
    v.channels = c;
    v.height = h;
    v.width = w;
    views.push_back(v);
    for (size_t k = 0; k < views.size(); ++k) {  // vector growth moved the buffers
      views[k].data = data[k].data();
    }
  }

  float & at(int out, int c, int y, int x)
  {
    const TensorView & v = views[out];
    return data[out][(static_cast<size_t>(c) * v.height + y) * v.width + x];
  }

  // a box with l/t/r/b distances (grid units) and score at cell (i, j) of level l
  void set_cell(int l, int i, int j, float score, const float dist[4])
  {
    const int box = l * per_level;
    at(box + 1, 0, i, j) = score;
    for (int side = 0; side < 4; ++side) {
      if (reg_max == 1) {
        at(box, side, i, j) = dist[side];
        continue;
      }
      const int k0 = static_cast<int>(std::floor(dist[side]));
      const float f = dist[side] - k0;
      at(box, side * reg_max + k0, i, j) = std::log(std::max(1.0f - f, 1e-12f));
      if (k0 + 1 < reg_max) {
        at(box, side * reg_max + k0 + 1, i, j) = std::log(std::max(f, 1e-12f));
      }
    }
  }
};

void expect_box(const Detection & d, float x1, float y1, float x2, float y2, float tol)
{
  EXPECT_NEAR(d.x1, x1, tol);
  EXPECT_NEAR(d.y1, y1, tol);
  EXPECT_NEAR(d.x2, x2, tol);
  EXPECT_NEAR(d.y2, y2, tol);
}

}  // namespace

TEST(YoloDecode, LayoutCheck)
{
  Outputs det(2, kRegMax);
  EXPECT_EQ(kebi_follow::check_output_layout(det.views, 2, kW, kH), "");
  EXPECT_NE(kebi_follow::check_output_layout(det.views, 3, kW, kH), "");
  EXPECT_NE(kebi_follow::check_output_layout(det.views, 2, 640, 640), "");
  Outputs pose(3, kRegMax);
  EXPECT_EQ(kebi_follow::check_output_layout(pose.views, 3, kW, kH), "");
}

TEST(YoloDecode, DflBoxAtEveryStride)
{
  Outputs o(2, kRegMax);
  const float d0[4] = {2.25f, 3.5f, 1.75f, 6.0f};
  o.set_cell(0, 30, 40, 0.9f, d0);   // stride 8
  const float d1[4] = {1.0f, 2.0f, 3.0f, 4.5f};
  o.set_cell(1, 5, 7, 0.8f, d1);     // stride 16
  const float d2[4] = {0.5f, 0.5f, 0.5f, 0.5f};
  o.set_cell(2, 14, 19, 0.7f, d2);   // stride 32
  const auto dets = kebi_follow::decode_yolo(o.views, 2, kW, DecodeConfig{0.5f, 0.45f, 10});
  ASSERT_EQ(dets.size(), 3u);
  expect_box(dets[0], (40.5f - 2.25f) * 8, (30.5f - 3.5f) * 8, (40.5f + 1.75f) * 8, (30.5f + 6.0f) * 8, 1e-3f);
  EXPECT_FLOAT_EQ(dets[0].score, 0.9f);
  expect_box(dets[1], (7.5f - 1.0f) * 16, (5.5f - 2.0f) * 16, (7.5f + 3.0f) * 16, (5.5f + 4.5f) * 16, 1e-3f);
  expect_box(dets[2], 19.0f * 32, 14.0f * 32, 20.0f * 32, 15.0f * 32, 1e-3f);
  EXPECT_FALSE(dets[0].has_keypoints);
}

TEST(YoloDecode, ThresholdNmsAndStableTies)
{
  Outputs o(2, kRegMax);
  const float d[4] = {3.0f, 3.0f, 3.0f, 3.0f};  // 48 px boxes
  o.set_cell(0, 20, 20, 0.6f, d);
  o.set_cell(0, 20, 21, 0.6f, d);   // same (quantized) score, IoU 0.71: the first visited survives
  o.set_cell(0, 20, 40, 0.9f, d);   // best, no overlap
  o.set_cell(0, 50, 70, 0.2f, d);   // below threshold
  const auto dets = kebi_follow::decode_yolo(o.views, 2, kW, DecodeConfig{0.35f, 0.45f, 10});
  ASSERT_EQ(dets.size(), 2u);
  EXPECT_FLOAT_EQ(dets[0].score, 0.9f);
  EXPECT_NEAR(dets[0].x1, (40.5f - 3.0f) * 8, 1e-3f);
  EXPECT_NEAR(dets[1].x1, (20.5f - 3.0f) * 8, 1e-3f);
}

TEST(YoloDecode, NmsSuppressesOverlap)
{
  Outputs o(2, kRegMax);
  const float d[4] = {3.0f, 3.0f, 3.0f, 3.0f};
  o.set_cell(0, 20, 20, 0.6f, d);   // IoU 0.5 with the best -> suppressed (> 0.45)
  o.set_cell(0, 20, 22, 0.9f, d);
  o.set_cell(0, 20, 26, 0.7f, d);   // shift 32 px: IoU 0.2 -> kept
  const auto dets = kebi_follow::decode_yolo(o.views, 2, kW, DecodeConfig{0.35f, 0.45f, 10});
  ASSERT_EQ(dets.size(), 2u);
  EXPECT_FLOAT_EQ(dets[0].score, 0.9f);
  EXPECT_FLOAT_EQ(dets[1].score, 0.7f);
}

TEST(YoloDecode, MaxDetections)
{
  Outputs o(2, kRegMax);
  const float d[4] = {0.5f, 0.5f, 0.5f, 0.5f};
  for (int j = 0; j < 20; j += 2) {
    o.set_cell(0, 10, j, 0.5f + 0.01f * j, d);
  }
  const auto dets = kebi_follow::decode_yolo(o.views, 2, kW, DecodeConfig{0.35f, 0.45f, 3});
  ASSERT_EQ(dets.size(), 3u);
  EXPECT_GT(dets[0].score, dets[1].score);
}

TEST(YoloDecode, PoseKeypoints)
{
  Outputs o(3, kRegMax);
  const float d[4] = {2.0f, 2.0f, 2.0f, 2.0f};
  o.set_cell(1, 10, 12, 0.8f, d);  // stride 16
  const int kpt = 1 * 3 + 2;
  o.at(kpt, 3 * kebi_follow::kLeftKnee, 10, 12) = 0.25f;       // x raw
  o.at(kpt, 3 * kebi_follow::kLeftKnee + 1, 10, 12) = -0.5f;   // y raw
  o.at(kpt, 3 * kebi_follow::kLeftKnee + 2, 10, 12) = 2.0f;    // vis logit
  const auto dets = kebi_follow::decode_yolo(o.views, 3, kW, DecodeConfig{0.5f, 0.45f, 10});
  ASSERT_EQ(dets.size(), 1u);
  ASSERT_TRUE(dets[0].has_keypoints);
  const auto & k = dets[0].keypoints[kebi_follow::kLeftKnee];
  EXPECT_NEAR(k.x, (2 * 0.25f + 12) * 16, 1e-4f);
  EXPECT_NEAR(k.y, (2 * -0.5f + 10) * 16, 1e-4f);
  EXPECT_NEAR(k.visibility, 1.0f / (1.0f + std::exp(-2.0f)), 1e-5f);
  EXPECT_NEAR(dets[0].keypoints[0].visibility, 0.5f, 1e-6f);  // logit 0
}

TEST(YoloDecode, PlainDistancesRegMax1)
{
  Outputs o(2, 1);
  const float d[4] = {1.5f, 2.5f, 0.75f, 3.0f};
  o.set_cell(2, 7, 9, 0.66f, d);
  const auto dets = kebi_follow::decode_yolo(o.views, 2, kW, DecodeConfig{0.5f, 0.45f, 10});
  ASSERT_EQ(dets.size(), 1u);
  expect_box(dets[0], (9.5f - 1.5f) * 32, (7.5f - 2.5f) * 32, (9.5f + 0.75f) * 32, (7.5f + 3.0f) * 32, 1e-4f);
}

TEST(YoloDecode, Int8MatchesFloatWithinQuantization)
{
  Outputs o(2, kRegMax);
  const float d[4] = {2.3f, 4.1f, 1.6f, 5.2f};
  o.set_cell(0, 33, 44, 0.87f, d);
  const auto ref = kebi_follow::decode_yolo(o.views, 2, kW, DecodeConfig{0.5f, 0.45f, 10});
  // quantize every tensor per tensor (asymmetric int8) like the RKNN outputs
  std::vector<std::vector<int8_t>> q(o.views.size());
  std::vector<TensorView> qv = o.views;
  for (size_t t = 0; t < o.views.size(); ++t) {
    const auto & src = o.data[t];
    const float mn = *std::min_element(src.begin(), src.end());
    const float mx = std::max(*std::max_element(src.begin(), src.end()), mn + 1e-3f);
    const float scale = (mx - mn) / 255.0f;
    const int zp = static_cast<int>(std::lround(-128 - mn / scale));
    q[t].resize(src.size());
    for (size_t i = 0; i < src.size(); ++i) {
      q[t][i] = static_cast<int8_t>(std::clamp<long>(std::lround(src[i] / scale) + zp, -128, 127));
    }
    qv[t].data = q[t].data();
    qv[t].type = TensorType::kInt8;
    qv[t].zero_point = zp;
    qv[t].scale = scale;
  }
  const auto dets = kebi_follow::decode_yolo(qv, 2, kW, DecodeConfig{0.5f, 0.45f, 10});
  ASSERT_EQ(dets.size(), 1u);
  ASSERT_EQ(ref.size(), 1u);
  expect_box(dets[0], ref[0].x1, ref[0].y1, ref[0].x2, ref[0].y2, 0.5f);
  EXPECT_NEAR(dets[0].score, ref[0].score, 0.01f);
}

// Stored outputs of the AMR NPU (fm_rknn_bench --dump: float32, NCHW, outputs concatenated in order)
// against the Python reference decode (fmdet_common.decode, stable sort). Data is kept outside the
// repository: KEBI_DET_TEST_DATA=<dir with manifest.txt>.
// manifest line: <dump.bin> <outputs_per_level> <box_channels> <reference.txt>
// reference line: x1 y1 x2 y2 score [x y vis for the 17 keypoints]
TEST(YoloDecode, StoredNpuDumpsMatchPythonReference)
{
  const char * dir = std::getenv("KEBI_DET_TEST_DATA");
  if (dir == nullptr) {
    GTEST_SKIP() << "KEBI_DET_TEST_DATA not set";
  }
  std::ifstream manifest(std::string(dir) + "/manifest.txt");
  ASSERT_TRUE(manifest.good()) << "no manifest.txt in " << dir;
  std::string line;
  int cases = 0;
  while (std::getline(manifest, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream ls(line);
    std::string bin;
    std::string ref;
    int per_level = 0;
    int box_c = 0;
    ls >> bin >> per_level >> box_c >> ref;
    SCOPED_TRACE(bin);
    std::ifstream f(std::string(dir) + "/" + bin, std::ios::binary | std::ios::ate);
    ASSERT_TRUE(f.good());
    const size_t bytes = static_cast<size_t>(f.tellg());
    f.seekg(0);
    std::vector<float> flat(bytes / sizeof(float), 0.0f);
    f.read(reinterpret_cast<char *>(flat.data()), static_cast<std::streamsize>(bytes));
    std::vector<TensorView> views;
    size_t off = 0;
    for (const int s : kStrides) {
      const int h = kH / s;
      const int w = kW / s;
      const int chans[3] = {box_c, 1, 51};
      for (int k = 0; k < per_level; ++k) {
        TensorView v;
        v.data = flat.data() + off;
        v.channels = chans[k];
        v.height = h;
        v.width = w;
        off += static_cast<size_t>(v.channels) * h * w;
        views.push_back(v);
      }
    }
    ASSERT_EQ(off, flat.size());
    const auto dets = kebi_follow::decode_yolo(views, per_level, kW, DecodeConfig{0.25f, 0.45f, 100});
    std::ifstream rf(std::string(dir) + "/" + ref);
    ASSERT_TRUE(rf.good());
    std::vector<std::vector<float>> rows;
    while (std::getline(rf, line)) {
      std::istringstream rs(line);
      std::vector<float> row;
      float x;
      while (rs >> x) {
        row.push_back(x);
      }
      if (!row.empty()) {
        rows.push_back(row);
      }
    }
    ASSERT_EQ(dets.size(), rows.size());
    for (size_t i = 0; i < rows.size(); ++i) {
      expect_box(dets[i], rows[i][0], rows[i][1], rows[i][2], rows[i][3], 0.02f);
      EXPECT_NEAR(dets[i].score, rows[i][4], 1e-4f);
      if (per_level == 3) {
        ASSERT_EQ(rows[i].size(), 5u + 51u);
        for (int k = 0; k < kebi_follow::kNumKeypoints; ++k) {
          EXPECT_NEAR(dets[i].keypoints[k].x, rows[i][5 + 3 * k], 0.02f);
          EXPECT_NEAR(dets[i].keypoints[k].y, rows[i][6 + 3 * k], 0.02f);
          EXPECT_NEAR(dets[i].keypoints[k].visibility, rows[i][7 + 3 * k], 1e-4f);
        }
      }
    }
    ++cases;
  }
  EXPECT_GT(cases, 0);
}
