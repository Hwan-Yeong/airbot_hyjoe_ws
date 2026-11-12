// YOLO person detector output decoding (CPU side of the NPU-friendly head).
//
// Model output layout, per stride s in (8, 16, 32), in this order:
//   box_s{s}   [1, 4*reg_max, H/s, W/s]  DFL logits, side order (l, t, r, b), reg_max = 16
//                                         (reg_max = 1: plain l/t/r/b distances in grid units)
//   score_s{s} [1, 1, H/s, W/s]          person score, sigmoid already applied
//   kpt_s{s}   [1, 51, H/s, W/s]         pose models only: 17 x (x, y, visibility logit), raw
// Decoding of cell (i, j):
//   d_side = sum_k softmax(logits_side)[k] * k
//   box  = ((j + 0.5 - d_l) s, (i + 0.5 - d_t) s, (j + 0.5 + d_r) s, (i + 0.5 + d_b) s)
//   kpt  = ((2 x + j) s, (2 y + i) s), visibility = sigmoid(v)
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace kebi_follow
{

enum class TensorType : uint8_t { kFloat32, kInt8, kUint8 };

// One output tensor [1, C, H, W], NCHW, contiguous. Quantized values are (q - zero_point) * scale.
struct TensorView
{
  const void * data{nullptr};
  TensorType type{TensorType::kFloat32};
  int32_t zero_point{0};
  float scale{1.0f};
  int channels{0};
  int height{0};
  int width{0};

  float at(int c, int y, int x) const
  {
    const size_t i = (static_cast<size_t>(c) * height + y) * width + x;
    switch (type) {
      case TensorType::kInt8:
        return (static_cast<const int8_t *>(data)[i] - zero_point) * scale;
      case TensorType::kUint8:
        return (static_cast<const uint8_t *>(data)[i] - zero_point) * scale;
      case TensorType::kFloat32:
      default:
        return static_cast<const float *>(data)[i];
    }
  }
};

constexpr int kNumKeypoints = 17;
// COCO keypoint indices used by the leg logic
constexpr int kLeftHip = 11;
constexpr int kRightHip = 12;
constexpr int kLeftKnee = 13;
constexpr int kRightKnee = 14;
constexpr int kLeftAnkle = 15;
constexpr int kRightAnkle = 16;

struct Keypoint
{
  float x{0.0f};
  float y{0.0f};
  float visibility{0.0f};
};

struct Detection
{
  float x1{0.0f};
  float y1{0.0f};
  float x2{0.0f};
  float y2{0.0f};
  float score{0.0f};
  bool has_keypoints{false};
  std::array<Keypoint, kNumKeypoints> keypoints{};
};

struct DecodeConfig
{
  float score_threshold{0.35f};
  float nms_iou{0.45f};
  int max_detections{10};
};

// Empty string if `outputs` match the layout above for an input of input_width x input_height.
std::string check_output_layout(
  const std::vector<TensorView> & outputs, int outputs_per_level, int input_width,
  int input_height);

// outputs_per_level: 2 (detection) or 3 (pose). Returns boxes in input pixels, best first, after NMS.
// Candidates are visited level by level in row-major order and sorted stably, so equal (quantized)
// scores keep a deterministic order.
std::vector<Detection> decode_yolo(
  const std::vector<TensorView> & outputs, int outputs_per_level, int input_width,
  const DecodeConfig & config);

float box_iou(const Detection & a, const Detection & b);

}  // namespace kebi_follow
