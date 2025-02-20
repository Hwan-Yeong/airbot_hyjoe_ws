#ifndef __POSITION_HPP__
#define __POSITION_HPP__

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

namespace A1::perception
{
class Position
{
   public:
    Position() = default;
    ~Position() = default;

    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    float roll = 0.0;
    float pitch = 0.0;
    float yaw = 0.0;

    void setFromQuaternion(const tf2::Quaternion& q)
    {
        tf2::Matrix3x3 m(q);
        double r, p, y;
        m.getRPY(r, p, y);
        roll = static_cast<float>(r);
        pitch = static_cast<float>(p);
        yaw = static_cast<float>(y);
    }

    tf2::Transform getTransform() const
    {
        tf2::Transform transform;
        transform.setOrigin(tf2::Vector3(x, y, z));
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        transform.setRotation(q);
        return transform;
    }
};
}  // namespace A1::perception

#endif  // __POSITION_HPP__