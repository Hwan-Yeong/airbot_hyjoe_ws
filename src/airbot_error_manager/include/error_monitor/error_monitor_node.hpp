#pragma once

#include <memory>
#include <vector>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "error_monitor/robot_state_blackboard.hpp"
#include "error_monitor/monitors/low_battery.hpp"
#include "error_monitor/monitors/battery_discharging.hpp"
#include "error_monitor/monitors/fall_down.hpp"
#include "error_monitor/monitors/board_overheat.hpp"
#include "error_monitor/monitors/charging.hpp"
#include "error_monitor/monitors/lift.hpp"
#include "error_monitor/monitors/cliff_detection.hpp"
#include "error_monitor/monitors/tof.hpp"
#include "error_monitor/monitors/ai_communication.hpp"

using namespace std::chrono_literals;

/**
 * @brief 에러 판단에 필요한 모든 외부 데이터를 구독하고,
 * 하나의 중앙 구조체(RobotStateBlackboard)에 최신화하는 노드입니다.
 * 각 에러 판단은 독립적인 모니터 객체가 수행합니다.
 */
class ErrorMonitorNode : public rclcpp::Node
{
public:
    ErrorMonitorNode();
    ~ErrorMonitorNode();

    void init();

    template<typename MonitorType>
    void addMonitor(std::shared_ptr<MonitorType> monitor) {
        monitor->setNode(this);
        monitor->loadParams(MonitorType::paramNamespace());
        monitor->printParams();
        monitor->startMonitor(blackboard_);
        monitors_.push_back(monitor);
    }

private:
    void checkMemoryUsage();

    std::shared_ptr<RobotStateBlackboard> blackboard_;

    // Subscribers
    rclcpp::Subscription<robot_custom_msgs::msg::BottomIrData>::SharedPtr bottom_ir_data_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::BatteryStatus>::SharedPtr battery_status_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::StationData>::SharedPtr station_data_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::RobotState>::SharedPtr robot_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::TofData>::SharedPtr tof_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ai_version_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::AiTemperature>::SharedPtr ai_temperature_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::ApTemperature>::SharedPtr ap_temperature_sub_;

    std::vector<std::shared_ptr<ErrorMonitorBase>> monitors_;

    rclcpp::TimerBase::SharedPtr memory_monitor_timer_;
};
