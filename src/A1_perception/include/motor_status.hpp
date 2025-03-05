#ifndef __MOTOR_STATUS_HPP__
#define __MOTOR_STATUS_HPP__

namespace A1::perception
{
class MotorStatus
{
   public:
    MotorStatus() = default;
    ~MotorStatus() = default;

    bool isMoveToFoward()
    {
        return left_rpm > 0 && right_rpm < 0;
    }

    bool isMoveToBack()
    {
        return left_rpm < 0 && right_rpm > 0;
    }

    bool isRotate()
    {
        return left_rpm * right_rpm > 0;
    }

    void setLeftRpm(float left_motor_rpm)
    {
        left_rpm = left_motor_rpm;
    }
    void setRightRpm(float right_motor_rpm)
    {
        right_rpm = right_motor_rpm;
    }

   private:
    float left_rpm = 0.0;
    float right_rpm = 0.0;
};
}  // namespace A1::perception

#endif  // __MOTOR_STATUS_HPP__