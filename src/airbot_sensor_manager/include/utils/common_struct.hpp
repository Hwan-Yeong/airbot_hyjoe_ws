#ifndef __COMMON_STRUCT__
#define __COMMON_STRUCT__

#include <cmath>
#include <vector>


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

#endif