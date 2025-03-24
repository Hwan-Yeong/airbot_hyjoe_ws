#ifndef ODOM_MONITOR_HPP
#define ODOM_MONITOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>   
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class OdomMonitor : public rclcpp::Node {
public:
    OdomMonitor();

private:
    // Subscriber callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    // Timer callback for checking data timeout
    void data_timer_callback();
    double angle_diff(double angle1, double angle2);
    double round_to_decimal_places (double value, int decimal_places);

    // ROS2 Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // Timer for checking both odometry & IMU data timeouts
    rclcpp::TimerBase::SharedPtr data_timer_;

    // Last received messages
    nav_msgs::msg::Odometry last_odom_;
    geometry_msgs::msg::Twist last_cmd_vel_;
    sensor_msgs::msg::Imu last_imu_;

    // Time tracking for message reception
    rclcpp::Time last_odom_time_;
    rclcpp::Time last_imu_time_;

    // Flags for first message reception
    bool first_odom_received_ = false;
    bool first_imu_received_ = false;
};

#endif  // AIRBOT_MOTION_ANOMALY_DETECTOR_ODOM_MONITOR_HPP_
