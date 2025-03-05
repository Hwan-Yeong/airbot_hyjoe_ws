#ifndef __FALL_DOWN_MONITOR_HPP__
#define __FALL_DOWN_MONITOR_HPP__

#include "std_msgs/msg/u_int8.hpp"
#include "sensor_msgs/msg/imu.hpp"


class FallDownMonitor
{
public:
    FallDownMonitor();
    ~FallDownMonitor();

    bool errorMonitor(std_msgs::msg::UInt8 bottom, sensor_msgs::msg::Imu imu);

private:
};

#endif //__FALL_DOWN_MONITOR_HPP__