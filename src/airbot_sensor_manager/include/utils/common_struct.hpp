#ifndef __COMMON_STRUCT__
#define __COMMON_STRUCT__

#include <cmath>
#include <vector>
#include <mutex>
#include <atomic>
#include "sensor_msgs/msg/point_cloud2.hpp"

#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

struct tPoint
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    tPoint() = default;
    tPoint(double x_, double y_, double z_)
    : x(x_), y(y_), z(z_) {}
    bool operator == (const tPoint& other) const {
        return (std::fabs(x - other.x) < 1e-6) &&
               (std::fabs(y - other.y) < 1e-6) &&
               (std::fabs(z - other.z) < 1e-6);
    }
    bool operator != (const tPoint& other) const {
        return !(*this == other);
    }
};

struct tOrientation
{
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tOrientation() = default;
    tOrientation(double roll_, double pitch_, double yaw_)
    : roll(roll_), pitch(pitch_), yaw(yaw_) {}
    bool operator == (const tOrientation& other) const {
        return (std::fabs(roll - other.roll) < 1e-6) &&
               (std::fabs(pitch - other.pitch) < 1e-6) &&
               (std::fabs(yaw - other.yaw) < 1e-6);
    }
    bool operator != (const tOrientation& other) const {
        return !(*this == other);
    }
};

struct tPose
{
    tPoint position;
    tOrientation orientation;
    tPose() = default;
    tPose(const tPoint& pos, const tOrientation& ori)
    : position(pos), orientation(ori) {}
    bool operator == (const tPose& other) const {
        return (position == other.position) && (orientation == other.orientation);
    }
    bool operator != (const tPose& other) const {
        return !(*this == other);
    }
};

template<typename MsgT>
struct tSensorBuffer {
    std::mutex mtx;
    std::atomic<bool> updated{false};
    typename MsgT::SharedPtr latest_msg;
    unsigned int publishing_cnt = 0;
    std::unordered_map<std::string, unsigned int> publishing_cnt_map;

    void reset() {
        updated.store(false);
        latest_msg.reset();
        publishing_cnt = 0;
        publishing_cnt_map.clear();
    }
};

// Multizone ToF Calibration
#define VALUE_TRUNCATE (3)
#define SUBTRAC_TF (0.17)

struct tMToFCalibSession {
    static constexpr int TARGET_INDICES[3] = {13, 14, 15}; // 수집할 데이터 인덱스 (13, 14, 15)

    bool is_finish_sampling = true;
    int sample_count = 0;
    int attempt_count = 0; // 시도 횟수 count

    std::vector<float> samples[3];      // TF 변환 후 데이터 (samples[0]은 13번, samples[1]은 14번...)
    std::vector<float> origins[3];      // Raw 데이터 (갱신 체크용)
    std::array<int, 3> non_renewal_counts{0, 0, 0};

    std::array<float, 3> stats{-1.0f, -1.0f, -1.0f}; // 최종 통계값

    void reset() { // 초기화 함수 한 번에 처리
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

enum class TOF_SIDE
{
    LEFT,
    RIGHT,
    BOTH
};

inline std::string enumToString(TOF_SIDE state) {
    switch (state) {
        case TOF_SIDE::LEFT:        return "LEFT";
        case TOF_SIDE::RIGHT:       return "RIGHT";
        case TOF_SIDE::BOTH:        return "BOTH";
        default:                    return "UNKNOWN";
    }
}

enum class MTOF_CALIB_STATE {
    INACTIVE        = 0,
    ACTIVE_LEFT     = 1,
    ACTIVE_RIGHT    = 2,
};

inline std::string enumToString(MTOF_CALIB_STATE state) {
    switch (state) {
        case MTOF_CALIB_STATE::INACTIVE:     return "INACTIVE";
        case MTOF_CALIB_STATE::ACTIVE_LEFT:  return "ACTIVE_LEFT";
        case MTOF_CALIB_STATE::ACTIVE_RIGHT: return "ACTIVE_RIGHT";
        default:                             return "UNKNOWN";
    }
}

enum class MTOF_CALIB_RESULT {
    INACTIVE,
    RUNNING,
    PASS = 2,
    FAIL_OUT_OF_RANGE = 3,
    FAIL_UNSTABLE_RANGE = 4,
    FAIL_TIME_OUT,
    FAIL_DATA_NON_RENEWAL = 8,
    FAIL_UNKNOWN,
};

inline std::string enumToString(MTOF_CALIB_RESULT state) {
    switch (state) {
        case MTOF_CALIB_RESULT::RUNNING:                return "RUNNING";
        case MTOF_CALIB_RESULT::PASS:                   return "PASS";
        case MTOF_CALIB_RESULT::FAIL_OUT_OF_RANGE:      return "FAIL_OUT_OF_RANGE";
        case MTOF_CALIB_RESULT::FAIL_UNSTABLE_RANGE:    return "FAIL_UNSTABLE_RANGE";
        case MTOF_CALIB_RESULT::FAIL_TIME_OUT:          return "FAIL_TIME_OUT";
        case MTOF_CALIB_RESULT::FAIL_DATA_NON_RENEWAL:  return "FAIL_DATA_NON_RENEWAL";
        case MTOF_CALIB_RESULT::FAIL_UNKNOWN:           return "FAIL_UNKNOWN";
        default:                                        return "UNKNOWN";
    }
}

struct tTofCalibrationParam {
    std::string method;
    int sampling_count;
    double pass_min_value;
    double pass_max_value;
    double pass_diff_th;
    double time_out_sec;
    int data_non_renewal_count;
};

struct MTOF_PUB_DATA {
    MTOF_PUB_DATA() {
        idx_13 = idx_14 = idx_15 = min = min_ref = max = max_ref = median = result = 0.0;
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

struct MTOF_CALIB_DATA {
    MTOF_CALIB_DATA() {
        left = MTOF_PUB_DATA();
        right = MTOF_PUB_DATA();
    }
    double truncate_to_n(double value, int n) {
        double scale = std::pow(10.0, n);
        return std::round(value * scale) / scale;
    }
    void setMinValue(TOF_SIDE _side, float _value, float _value_ref) {
        if (_side == TOF_SIDE::LEFT){
            left.min = (truncate_to_n(_value, VALUE_TRUNCATE) - SUBTRAC_TF);
            left.min_ref = (truncate_to_n(_value_ref, VALUE_TRUNCATE) - SUBTRAC_TF);
        } else if (_side == TOF_SIDE::RIGHT){
            right.min = (truncate_to_n(_value, VALUE_TRUNCATE) - SUBTRAC_TF);
            right.min_ref = (truncate_to_n(_value_ref, VALUE_TRUNCATE) - SUBTRAC_TF);
        }
    }
    void setMaxValue(TOF_SIDE _side, float _value, float _value_ref) {
        if (_side == TOF_SIDE::LEFT){
            left.max = (truncate_to_n(_value, VALUE_TRUNCATE)) - SUBTRAC_TF;
            left.max_ref = (truncate_to_n(_value_ref, VALUE_TRUNCATE) - SUBTRAC_TF);
        } else if (_side == TOF_SIDE::RIGHT){
            right.max = (truncate_to_n(_value, VALUE_TRUNCATE) - SUBTRAC_TF);
            right.max_ref = (truncate_to_n(_value_ref, VALUE_TRUNCATE) - SUBTRAC_TF);
        }
    }
    void setMedianValue(TOF_SIDE _side, float _value) {
        if (_side == TOF_SIDE::LEFT){
            left.median = (truncate_to_n(_value, VALUE_TRUNCATE) - SUBTRAC_TF);
        } else if (_side == TOF_SIDE::RIGHT){
            right.median = (truncate_to_n(_value, VALUE_TRUNCATE) - SUBTRAC_TF);
        }
    }
    void setResult(TOF_SIDE _side, float _result) {
        if (_side == TOF_SIDE::LEFT) {
            left.result = _result;
        } else if (_side == TOF_SIDE::RIGHT) {
            right.result = _result;
        }
    }
    void setCalibValue(TOF_SIDE _side, float _value_13, float _value_14, float _value_15) {
        if (_side == TOF_SIDE::LEFT){
            left.idx_13 = (truncate_to_n(_value_13, VALUE_TRUNCATE) - SUBTRAC_TF);
            left.idx_14 = (truncate_to_n(_value_14, VALUE_TRUNCATE) - SUBTRAC_TF);
            left.idx_15 = (truncate_to_n(_value_15, VALUE_TRUNCATE) - SUBTRAC_TF);
        } else if (_side == TOF_SIDE::RIGHT){
            right.idx_13 = (truncate_to_n(_value_13, VALUE_TRUNCATE) - SUBTRAC_TF);
            right.idx_14 = (truncate_to_n(_value_14, VALUE_TRUNCATE) - SUBTRAC_TF);
            right.idx_15 = (truncate_to_n(_value_15, VALUE_TRUNCATE) - SUBTRAC_TF);
        }
    }
    void setPublishValue(TOF_SIDE _side) {
        if (_side == TOF_SIDE::LEFT) {
            left.pub_data = {static_cast<float>(_side), left.idx_13, left.idx_14, left.idx_15, 
                left.min, left.min_ref, left.max, left.max_ref, left.median, left.result};
        } else if (_side == TOF_SIDE::RIGHT) {
            right.pub_data = {static_cast<float>(_side), right.idx_13, right.idx_14, right.idx_15, 
                right.min, right.min_ref, right.max, right.max_ref, right.median, right.result};
        }
    }
    MTOF_PUB_DATA left;
    MTOF_PUB_DATA right;
};

#endif