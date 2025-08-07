#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
class ScanMonitorNode : public rclcpp::Node {
public:
    ScanMonitorNode();

private:
    void cmdCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void scanFrontStateCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void scanBackStateCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void checkScanHealth();
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::TimerBase::SharedPtr monitor_timer_;
    std::chrono::steady_clock::time_point last_scan_time_;
    std::chrono::steady_clock::time_point prev_scan_time_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr scan_front_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr scan_back_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr scan_monitor_cmd_sub_;
    
    bool monitor_enabled_ = false;
    bool scan_received_ = false;
    bool scan_front_state_ = false;
    bool scan_back_state_ = false;
    bool scanOk = false;
    double lower_bound_ms_;
    double upper_bound_ms_;
    double timeout_ms_;  // 스캔 데이터 수신 타임아웃 임계값
};