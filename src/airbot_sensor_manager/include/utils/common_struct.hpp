#ifndef __COMMON_STRUCT__
#define __COMMON_STRUCT__

#include <cmath>
#include <vector>
#include "sensor_msgs/msg/point_cloud2.hpp"


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

struct tSensor {
    bool use = false;
    std::string topic;
    int publish_rate = 0;
    double pitch_angle_deg = 0.0;
    std::vector<int> sub_cell_idx_array;
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