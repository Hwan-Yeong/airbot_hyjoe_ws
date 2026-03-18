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
    void initVariables();
    void setParams();
    
    // Callbacks to update Blackboard
    void bottomIrDataCallback(const robot_custom_msgs::msg::BottomIrData::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void batteryCallback(const robot_custom_msgs::msg::BatteryStatus::SharedPtr msg);
    void stationDataCallback(const robot_custom_msgs::msg::StationData::SharedPtr msg);
    void robotStateCallback(const robot_custom_msgs::msg::RobotState::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void tofCallback(const robot_custom_msgs::msg::TofData::SharedPtr msg);
    void aiVerCallback(const std_msgs::msg::String::SharedPtr msg);
    void aiTemperatureCallback(const robot_custom_msgs::msg::AiTemperature::SharedPtr msg);
    void apTemperatureCallback(const robot_custom_msgs::msg::ApTemperature::SharedPtr msg);
    void checkMemoryUsage();
    void checkSensorDelays();

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
    rclcpp::TimerBase::SharedPtr sensor_delay_check_timer_;
};
