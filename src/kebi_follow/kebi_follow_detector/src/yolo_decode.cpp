#include "kebi_follow_detector/yolo_decode.hpp"

#include <algorithm>
#include <cmath>

namespace kebi_follow
{

namespace
{

constexpr int kStrideCount = 3;
constexpr int kMaxRegMax = 32;
constexpr int kBoxSides = 4;
constexpr int kKeypointValues = 3;

float sigmoid(float x) {return 1.0f / (1.0f + std::exp(-x));}

}  // namespace

float box_iou(const Detection & a, const Detection & b)
{
  const float iw = std::max(0.0f, std::min(a.x2, b.x2) - std::max(a.x1, b.x1));
  const float ih = std::max(0.0f, std::min(a.y2, b.y2) - std::max(a.y1, b.y1));
  const float inter = iw * ih;
  const float uni = (a.x2 - a.x1) * (a.y2 - a.y1) + (b.x2 - b.x1) * (b.y2 - b.y1) - inter;
  return uni > 0.0f ? inter / uni : 0.0f;
}

std::string check_output_layout(
  const std::vector<TensorView> & outputs, int outputs_per_level, int input_width,
  int input_height)
{
  if (outputs_per_level != 2 && outputs_per_level != 3) {
    return "outputs_per_level must be 2 (det) or 3 (pose)";
  }
  if (static_cast<int>(outputs.size()) != kStrideCount * outputs_per_level) {
    return "expected " + std::to_string(kStrideCount * outputs_per_level) + " outputs, got " +
           std::to_string(outputs.size());
  }
  for (int l = 0; l < kStrideCount; ++l) {
    const TensorView & box = outputs[l * outputs_per_level];
    const TensorView & score = outputs[l * outputs_per_level + 1];
    const std::string lvl = "level " + std::to_string(l) + ": ";
    if (score.channels != 1 || score.width <= 0 || score.height <= 0) {
      return lvl + "score tensor must be [1,1,H,W]";
    }
    if (input_width % score.width != 0 || input_height % score.height != 0 ||
      input_width / score.width != input_height / score.height)
    {
      return lvl + "grid " + std::to_string(score.width) + "x" + std::to_string(score.height) +
             " does not divide the input " + std::to_string(input_width) + "x" +
             std::to_string(input_height);
    }
    if (box.height != score.height || box.width != score.width || box.channels % kBoxSides != 0 ||
      box.channels / kBoxSides > kMaxRegMax)
    {
      return lvl + "box tensor must be [1,4*reg_max,H,W] with reg_max <= 32";
    }
    if (outputs_per_level == 3) {
      const TensorView & kpt = outputs[l * outputs_per_level + 2];
      if (kpt.channels != kNumKeypoints * kKeypointValues || kpt.height != score.height ||
        kpt.width != score.width)
      {
        return lvl + "keypoint tensor must be [1,51,H,W]";
      }
    }
  }
  return {};
}

std::vector<Detection> decode_yolo(
  const std::vector<TensorView> & outputs, int outputs_per_level, int input_width,
  const DecodeConfig & config)
{
  std::vector<Detection> candidates;
  const int levels = static_cast<int>(outputs.size()) / outputs_per_level;
  for (int l = 0; l < levels; ++l) {
    const TensorView & box = outputs[l * outputs_per_level];
    const TensorView & score = outputs[l * outputs_per_level + 1];
    const TensorView * kpt = outputs_per_level == 3 ? &outputs[l * outputs_per_level + 2] : nullptr;
    const float stride = static_cast<float>(input_width) / score.width;
    const int reg_max = box.channels / kBoxSides;
    for (int i = 0; i < score.height; ++i) {
      for (int j = 0; j < score.width; ++j) {
        const float s = score.at(0, i, j);
        if (s < config.score_threshold) {
          continue;
        }
        float dist[kBoxSides];
        for (int side = 0; side < kBoxSides; ++side) {
          if (reg_max == 1) {
            dist[side] = box.at(side, i, j);
            continue;
          }
          float logit[kMaxRegMax];
          float mx = -1e30f;
          for (int k = 0; k < reg_max; ++k) {
            logit[k] = box.at(side * reg_max + k, i, j);
            mx = std::max(mx, logit[k]);
          }
          float sum = 0.0f;
          float acc = 0.0f;
          for (int k = 0; k < reg_max; ++k) {
            const float e = std::exp(logit[k] - mx);
            sum += e;
            acc += e * static_cast<float>(k);
          }
          dist[side] = acc / sum;
        }
        Detection det;
        det.x1 = (j + 0.5f - dist[0]) * stride;
        det.y1 = (i + 0.5f - dist[1]) * stride;
        det.x2 = (j + 0.5f + dist[2]) * stride;
        det.y2 = (i + 0.5f + dist[3]) * stride;
        det.score = s;
        if (kpt != nullptr) {
          det.has_keypoints = true;
          for (int k = 0; k < kNumKeypoints; ++k) {
            Keypoint & p = det.keypoints[k];
            p.x = (2.0f * kpt->at(kKeypointValues * k, i, j) + j) * stride;
            p.y = (2.0f * kpt->at(kKeypointValues * k + 1, i, j) + i) * stride;
            p.visibility = sigmoid(kpt->at(kKeypointValues * k + 2, i, j));
          }
        }
        candidates.push_back(det);
      }
    }
  }
  std::stable_sort(
    candidates.begin(), candidates.end(),
    [](const Detection & a, const Detection & b) {return a.score > b.score;});
  std::vector<Detection> kept;
  std::vector<bool> suppressed(candidates.size(), false);
  for (size_t a = 0; a < candidates.size(); ++a) {
    if (suppressed[a]) {
      continue;
    }
    kept.push_back(candidates[a]);
    if (static_cast<int>(kept.size()) >= config.max_detections) {
      break;
    }
    for (size_t b = a + 1; b < candidates.size(); ++b) {
      if (!suppressed[b] && box_iou(candidates[a], candidates[b]) > config.nms_iou) {
        suppressed[b] = true;
      }
    }
  }
  return kept;
}

}  // namespace kebi_follow
