#ifndef __MOTOR_STATUS_HPP__
#define __MOTOR_STATUS_HPP__
#include <cmath>

namespace A1::maneuver
{
class MotorStatus
{
   public:
    MotorStatus() = default;
    ~MotorStatus() = default;

    /**
     * @brief 현재 전진 상태인지 확인
     */
    bool isMoveToForward()
    {
        return left_rpm > 0 && right_rpm < 0;
    }

    /**
     * @brief 현재 후진 상태인지 확인
     */
    bool isMoveToBack()
    {
        return left_rpm < 0 && right_rpm > 0;
    }

    /**
     * @brief RPM이 느린 상태인지 확인
     */
    bool isLowRpm()
    {
        return std::fabs(left_rpm) < 5 && std::fabs(right_rpm) < 5;
    }

    /**
     * @brief 회전중인지 확인
     */
    bool isRotate()
    {
        return left_rpm * right_rpm > 0;
    }

    /**
     * @brief 완전 정지 상태인지 확인
     */
    bool isStop()
    {
        return left_rpm == 0 && right_rpm == 0;
    }

    void setLeftRpm(float left_motor_rpm)
    {
        left_rpm = left_motor_rpm;
    }
    void setRightRpm(float right_motor_rpm)
    {
        right_rpm = right_motor_rpm;
    }

    /**
     * @brief RPM으로 속도 계샨
     */
    double getVelocity()
    {
        double r_rpm = right_rpm * (-1);
        return ((left_rpm + r_rpm) / 2.0) * (0.102 * M_PI) / 60;
    }

   private:
    float left_rpm = 0.0;
    float right_rpm = 0.0;
};
}  // namespace A1::maneuver

#endif  // __MOTOR_STATUS_HPP__