#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>

#define USE_LIDAR_STATE_CHECK 1

class ScanMonitorNode : public rclcpp::Node {
public:
    ScanMonitorNode();

private:
    
    #if USE_LIDAR_STATE_CHECK > 0
    void lidar_cmd_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void checkScanState();
    void checkScanHz();
    void publishLidarState(uint8_t state);
    void publishScanHzState(bool state);
    #else
    void cmdCallback(const std_msgs::msg::Bool::SharedPtr msg);
    #endif
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

    #if USE_LIDAR_STATE_CHECK > 0
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cmd_lidar_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr lidar_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr scanHz_state_pub_;
    #else
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr scan_monitor_cmd_sub_;
    #endif

     #if USE_LIDAR_STATE_CHECK > 0
    bool bLidarCmd = false;
    /* lidar_state
    0: OFF
    1: ON
    2: STARTING
    3: STOPING
    */
    uint8_t lidar_state = 0;
    uint8_t hz_check_count_ = 0;
    uint8_t lidar_off_count_ = 0;
    bool isScanHzOk = false;
    #else
    bool monitor_enabled_ = false;
    bool scanOk = false;
    #endif
    bool scan_front_state_ = false;
    bool scan_back_state_ = false;
    double lower_bound_ms_;
    double upper_bound_ms_;
    double timeout_ms_;  // 스캔 데이터 수신 타임아웃 임계값
};