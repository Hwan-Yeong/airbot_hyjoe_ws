#pragma once

#include <atomic>
#include <cmath>
#include <mutex>
#include <vector>
#include "sensor_msgs/msg/point_cloud2.hpp"

// already defined in "pcl-1.12/pcl/point_cloud.h"
// #define DEG2RAD(x) ((x) * M_PI / 180.0)
// #define RAD2DEG(x) ((x) * 180.0 / M_PI)

struct Point {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  Point() = default;
  Point(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
  bool operator==(const Point& other) const {
    return (std::fabs(x - other.x) < 1e-6) && (std::fabs(y - other.y) < 1e-6) &&
           (std::fabs(z - other.z) < 1e-6);
  }
  bool operator!=(const Point& other) const { return !(*this == other); }
};

struct Orientation {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  Orientation() = default;
  Orientation(double roll_, double pitch_, double yaw_)
      : roll(roll_), pitch(pitch_), yaw(yaw_) {}
  bool operator==(const Orientation& other) const {
    return (std::fabs(roll - other.roll) < 1e-6) &&
           (std::fabs(pitch - other.pitch) < 1e-6) &&
           (std::fabs(yaw - other.yaw) < 1e-6);
  }
  bool operator!=(const Orientation& other) const { return !(*this == other); }
};

struct Pose {
  Point position;
  Orientation orientation;
  Pose() = default;
  Pose(const Point& pos, const Orientation& ori)
      : position(pos), orientation(ori) {}
  bool operator==(const Pose& other) const {
    return (position == other.position) && (orientation == other.orientation);
  }
  bool operator!=(const Pose& other) const { return !(*this == other); }
};

template <typename MsgT>
struct SensorBuffer {
  std::mutex mtx;
  std::atomic<bool> updated{false};
  typename MsgT::SharedPtr latest_msg;
  rclcpp::Time receive_time;

  void Reset() {
    updated.store(false);
    latest_msg.reset();
    receive_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
};

// Multizone ToF Calibration
constexpr int kValueTruncate = 3;
constexpr double kSubtractTf = 0.17;

struct MToFCalibSession {
  // Target data indices to collect (13, 14, 15)
  static constexpr int kTargetIndices[3] = {13, 14, 15};

  bool is_finish_sampling = true;
  int sample_count = 0;
  int attempt_count = 0;  // Attempt count

  // Data after TF conversion (samples[0] is 13, samples[1] is 14...)
  std::vector<float> samples[3];
  // Raw data (for update check)
  std::vector<float> origins[3];
  std::array<int, 3> non_renewal_counts{0, 0, 0};

  std::array<float, 3> stats{-1.0f, -1.0f, -1.0f};  // Final statistics

  void Reset() {  // Initialization function processed at once
    is_finish_sampling = false;
    sample_count = 0;
    attempt_count = 0;
    for (int i = 0; i < 3; ++i) {
      samples[i].clear();
      origins[i].clear();
      non_renewal_counts[i] = 0;
      stats[i] = -1.0f;
    }
  }
};

enum class TofSide { kLeft, kRight, kBoth };

inline std::string EnumToString(TofSide state) {
  switch (state) {
    case TofSide::kLeft:
      return "LEFT";
    case TofSide::kRight:
      return "RIGHT";
    case TofSide::kBoth:
      return "BOTH";
    default:
      return "UNKNOWN";
  }
}

enum class MToFCalibState {
  kInactive = 0,
  kActiveLeft = 1,
  kActiveRight = 2,
};

inline std::string EnumToString(MToFCalibState state) {
  switch (state) {
    case MToFCalibState::kInactive:
      return "INACTIVE";
    case MToFCalibState::kActiveLeft:
      return "ACTIVE_LEFT";
    case MToFCalibState::kActiveRight:
      return "ACTIVE_RIGHT";
    default:
      return "UNKNOWN";
  }
}

enum class MToFCalibResult {
  kInactive,
  kRunning,
  kPass = 2,
  kFailOutOfRange = 3,
  kFailUnstableRange = 4,
  kFailTimeOut,
  kFailDataNonRenewal = 8,
  kFailUnknown,
};

inline std::string EnumToString(MToFCalibResult state) {
  switch (state) {
    case MToFCalibResult::kRunning:
      return "RUNNING";
    case MToFCalibResult::kPass:
      return "PASS";
    case MToFCalibResult::kFailOutOfRange:
      return "FAIL_OUT_OF_RANGE";
    case MToFCalibResult::kFailUnstableRange:
      return "FAIL_UNSTABLE_RANGE";
    case MToFCalibResult::kFailTimeOut:
      return "FAIL_TIME_OUT";
    case MToFCalibResult::kFailDataNonRenewal:
      return "FAIL_DATA_NON_RENEWAL";
    case MToFCalibResult::kFailUnknown:
      return "FAIL_UNKNOWN";
    default:
      return "UNKNOWN";
  }
}

struct TofCalibrationParam {
  std::string method;
  int sampling_count;
  double pass_min_value;
  double pass_max_value;
  double pass_diff_th;
  double time_out_sec;
  int data_non_renewal_count;
};

struct MToFPubData {
  MToFPubData() {
    idx_13 = idx_14 = idx_15 = min = min_ref = max = max_ref = median = result =
        0.0;
    pub_data.clear();
  }
  float idx_13;
  float idx_14;
  float idx_15;
  float min;
  float min_ref;
  float max;
  float max_ref;
  float median;
  float result;
  std::vector<float> pub_data;
};

struct MToFCalibData {
  MToFCalibData() {
    left = MToFPubData();
    right = MToFPubData();
  }
  double TruncateToN(double value, int n) {
    double scale = std::pow(10.0, n);
    return std::round(value * scale) / scale;
  }
  void SetMinValue(TofSide side, float value, float value_ref) {
    if (side == TofSide::kLeft) {
      left.min = (TruncateToN(value, kValueTruncate) - kSubtractTf);
      left.min_ref = (TruncateToN(value_ref, kValueTruncate) - kSubtractTf);
    } else if (side == TofSide::kRight) {
      right.min = (TruncateToN(value, kValueTruncate) - kSubtractTf);
      right.min_ref = (TruncateToN(value_ref, kValueTruncate) - kSubtractTf);
    }
  }
  void SetMaxValue(TofSide side, float value, float value_ref) {
    if (side == TofSide::kLeft) {
      left.max = (TruncateToN(value, kValueTruncate)) - kSubtractTf;
      left.max_ref = (TruncateToN(value_ref, kValueTruncate) - kSubtractTf);
    } else if (side == TofSide::kRight) {
      right.max = (TruncateToN(value, kValueTruncate) - kSubtractTf);
      right.max_ref = (TruncateToN(value_ref, kValueTruncate) - kSubtractTf);
    }
  }
  void SetMedianValue(TofSide side, float value) {
    if (side == TofSide::kLeft) {
      left.median = (TruncateToN(value, kValueTruncate) - kSubtractTf);
    } else if (side == TofSide::kRight) {
      right.median = (TruncateToN(value, kValueTruncate) - kSubtractTf);
    }
  }
  void SetResult(TofSide side, float result) {
    if (side == TofSide::kLeft) {
      left.result = result;
    } else if (side == TofSide::kRight) {
      right.result = result;
    }
  }
  void SetCalibValue(TofSide side, float value_13, float value_14,
                     float value_15) {
    if (side == TofSide::kLeft) {
      left.idx_13 = (TruncateToN(value_13, kValueTruncate) - kSubtractTf);
      left.idx_14 = (TruncateToN(value_14, kValueTruncate) - kSubtractTf);
      left.idx_15 = (TruncateToN(value_15, kValueTruncate) - kSubtractTf);
    } else if (side == TofSide::kRight) {
      right.idx_13 = (TruncateToN(value_13, kValueTruncate) - kSubtractTf);
      right.idx_14 = (TruncateToN(value_14, kValueTruncate) - kSubtractTf);
      right.idx_15 = (TruncateToN(value_15, kValueTruncate) - kSubtractTf);
    }
  }
  void SetPublishValue(TofSide side) {
    if (side == TofSide::kLeft) {
      left.pub_data = {static_cast<float>(side),
                       left.idx_13,
                       left.idx_14,
                       left.idx_15,
                       left.min,
                       left.min_ref,
                       left.max,
                       left.max_ref,
                       left.median,
                       left.result};
    } else if (side == TofSide::kRight) {
      right.pub_data = {static_cast<float>(side),
                        right.idx_13,
                        right.idx_14,
                        right.idx_15,
                        right.min,
                        right.min_ref,
                        right.max,
                        right.max_ref,
                        right.median,
                        right.result};
    }
  }
  MToFPubData left;
  MToFPubData right;
};

enum class SensorType {
  kTofMono,
  kTofMultiLeft,
  kTofMultiRight,
  kCamera,
  kBottomIr,
  kCollisionFront,
  kCollisionRear,
  kDepthCamera
};
