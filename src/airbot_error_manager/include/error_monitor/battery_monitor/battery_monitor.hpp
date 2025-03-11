#ifndef __BATTERY_MONITOR_HPP__
#define __BATTERY_MONITOR_HPP__

#include "rclcpp/rclcpp.hpp"        // 추가
#include "rclcpp/qos.hpp"           // 추가
#include "std_msgs/msg/bool.hpp"    // 추가
#include "std_msgs/msg/u_int8.hpp"
#include "robot_custom_msgs/msg/battery_status.hpp"

class BatteryMonitor
{
public:
    BatteryMonitor();
    ~BatteryMonitor();

    bool batteryMonitor(robot_custom_msgs::msg::BatteryStatus batterystatus);

private:

// 추가
};

#endif //__FALL_DOWN_MONITOR_HPP__



