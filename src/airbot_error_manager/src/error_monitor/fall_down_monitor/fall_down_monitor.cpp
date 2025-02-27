#include "error_monitor/fall_down_monitor/fall_down_monitor.hpp"

FallDownMonitor::FallDownMonitor()
{
}

FallDownMonitor::~FallDownMonitor()
{
}

bool FallDownMonitor::errorMonitor(std_msgs::msg::UInt8 bottom, sensor_msgs::msg::Imu imu)
{
    // 전도 에러 판단
    return false;
}
