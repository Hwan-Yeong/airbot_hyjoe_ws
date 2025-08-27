#ifndef __COMMON_STRUCT__
#define __COMMON_STRUCT__

#pragma once
#include <cmath>
#include <vector>
#include <string>
#include "sensor_msgs/msg/point_cloud2.hpp"

#define VALUE_TRUNCATE (3)
#define SUBTRAC_TF (0.17)


enum class MTOF_CALIB_STATE {
    INACTIVE        = 0,
    ACTIVE_LEFT     = 1,
    ACTIVE_RIGHT    = 2,
};

enum class TOF_SIDE
{
    LEFT,
    RIGHT,
    BOTH
};

inline std::string enumToString(MTOF_CALIB_STATE state) {
    switch (state) {
        case MTOF_CALIB_STATE::INACTIVE:     return "INACTIVE";
        case MTOF_CALIB_STATE::ACTIVE_LEFT:  return "ACTIVE_LEFT";
        case MTOF_CALIB_STATE::ACTIVE_RIGHT: return "ACTIVE_RIGHT";
        default:                             return "UNKNOWN";
    }
}

namespace MTOF_CALIB_SET {
    inline uint8_t INACTIVE    =   0;
    inline uint8_t RUNNING     =   1;
    inline uint8_t PASS        =   2;  
    inline uint8_t FAIL        =   3;
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

struct tTofPitchAngle {
    double bot_left;
    double bot_right;
};

struct tFilteredPointCloud {
    sensor_msgs::msg::PointCloud2 pointcloud;
    std::vector<bool> zero_dist_mask;
};

struct tCalibration {
    std::string method;
    int sampling_count;
    double pass_min_value;
    double pass_max_value;
    double pass_diff_th;
    double timeout;
    int data_non_renewal_count;
};

struct tSensor {
    // general
    bool use = false;
    std::string topic;
    std::string topic_idx;
    std::string topic_row;
    int publish_rate = 0;
    // tof
    double pitch_angle_deg = 0.0;
    std::vector<int> sub_cell_idx_array;
    tCalibration calibration;
    // camera
    float pc_resolution = 0.0;
    bool direction = true; // 정방향(CCW+):True, 역방향(CW+):False
    double object_max_dist = 0.0;
    double camera_object_ignore_pitch_th = 0.0;
    std::vector<std::string> class_id;
};

struct tSensorConfig {
    tSensor one_d_tof;
    tSensor multi_tof;
    tSensor multi_tof_left;
    tSensor multi_tof_right;
    tSensor camera;
    tSensor cliff;
    tSensor collision;
};

struct tFilter {
    bool use = false;
    std::vector<int> enabled_4x4_idx;
    double alpha = 0.0;
    int window_size = 0;
    double max_distance_th = 0.0;
};

struct tFilterConfig {
    tFilter moving_average;
    tFilter low_pass;
    tFilter complementary;
};

#endif