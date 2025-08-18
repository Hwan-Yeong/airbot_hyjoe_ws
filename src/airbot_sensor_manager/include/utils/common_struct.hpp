#ifndef __COMMON_STRUCT__
#define __COMMON_STRUCT__

#include <cmath>
#include <vector>
#include <string>
#include "sensor_msgs/msg/point_cloud2.hpp"


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
    RUNNING,
    PASS,
    FAIL_OUT_OF_RANGE,
    FAIL_UNSTABLE_RANGE,
    FAIL_TIME_OUT,
    FAIL_UNKNOWN,
};

inline std::string enumToString(MTOF_CALIB_RESULT state) {
    switch (state) {
        case MTOF_CALIB_RESULT::RUNNING:                return "RUNNING";
        case MTOF_CALIB_RESULT::PASS:                   return "PASS";
        case MTOF_CALIB_RESULT::FAIL_OUT_OF_RANGE:      return "FAIL_OUT_OF_RANGE";
        case MTOF_CALIB_RESULT::FAIL_UNSTABLE_RANGE:    return "FAIL_UNSTABLE_RANGE";
        case MTOF_CALIB_RESULT::FAIL_TIME_OUT:          return "FAIL_TIME_OUT";
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