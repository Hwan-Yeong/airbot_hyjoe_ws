#ifndef __FALL_DOWN_MONITOR_HPP__
#define __FALL_DOWN_MONITOR_HPP__

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "robot_custom_msgs/msg/bottom_ir_data.hpp"

class FallDownMonitor
{
public:
    FallDownMonitor();
    ~FallDownMonitor();

    bool errorMonitor(robot_custom_msgs::msg::BottomIrData bottom, sensor_msgs::msg::Imu imu);

private:
    void get_rpy_from_quaternion(const geometry_msgs::msg::Quaternion& quaternion, double& roll, double& pitch, double& yaw);
};

#endif //__FALL_DOWN_MONITOR_HPP__