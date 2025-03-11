#include "error_monitor/fall_down_monitor/fall_down_monitor.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <memory>

FallDownMonitor::FallDownMonitor()
{
}

FallDownMonitor::~FallDownMonitor()
{
}

bool FallDownMonitor::errorMonitor(robot_custom_msgs::msg::BottomIrData bottom, sensor_msgs::msg::Imu imu)
{
    // 전도 에러 판단

    bool bottomdata_range = false;
    bool imu_range = false;
    int count = 0;

    // 밑 센서값 조정
    // 값이 방향에 따라서 일정하게 변하지 않기 때문에
    // 방향을 나누어서 값 변화에 대해서 전도 현상값의 범위를 조정해야 함
    // front - front_L - back_L - back - back_R - front_R
    if(bottom.ff == 1)
    {
        count++;
    }

    if(bottom.fl == 1)
    {
        count++;
    }

    if(bottom.fr == 1)
    {
        count++;
    }

    if(bottom.bb == 1)
    {
        count++;
    }

    if(bottom.bl == 1)
    {
        count++;
    }

    if(bottom.br == 1)
    {
        count++;
    }

    if(count >= 3)
    {
        bottomdata_range = true;
    }
    else
    {
        bottomdata_range = false;
    }

    // imu 센서값 조정
    // 선형 가속도 값을 roll, pitch 각도값으로 변환

    double deg_pitch, deg_roll;  // 각도 값
    double roll, pitch, yaw;
    get_rpy_from_quaternion(imu.orientation, roll, pitch, yaw);


    deg_pitch = pitch * 180.0 / M_PI;
    deg_roll = roll * 180.0 / M_PI;


    if(abs(deg_pitch) >= 60 || abs(deg_roll) >= 60)
    {
        imu_range = true;
    }

    if(imu_range && bottomdata_range)  // 데이터 값에 따른 결정
    {
        return true;        // 전도가 일어남
        RCLCPP_WARN(rclcpp::get_logger("fall_down_monitor"), "Detected (ff : %d)  (fl : %d) (fr :%d) (bb : %d) (bl : %d) (br : %d)  (pich : %.3f) (roll : %.3f)", bottom.ff, bottom.fr, bottom.fr, bottom.bb, bottom.bl, bottom.br, deg_pitch, deg_roll);
    }
    else
    {
        return false;       // 전도가 일어나지 않음
        RCLCPP_WARN(rclcpp::get_logger("fall_down_monitor"), "Not Detected (ff : %d)  (fl : %d) (fr :%d) (bf : %d) (bl : %d) (br : %d)  (pich : %.3f) (roll : %.3f)", bottom.ff, bottom.fr, bottom.fr, bottom.bb, bottom.bl, bottom.br, deg_pitch, deg_roll);
    }
}

void FallDownMonitor::get_rpy_from_quaternion(const geometry_msgs::msg::Quaternion& quaternion, double& roll, double& pitch, double& yaw) 
{
        tf2::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
}
