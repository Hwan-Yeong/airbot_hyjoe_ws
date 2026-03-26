#pragma once

#include <memory>
#include <vector>
#include <fstream>
#include <yaml-cpp/yaml.h>
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
#include "error_monitor/monitors/cliff.hpp"
#include "error_monitor/monitors/tof.hpp"
#include "error_monitor/monitors/ai_communication.hpp"

using namespace std::chrono_literals;

/**
 * @class ErrorMonitorNode
 * @brief The central ROS 2 Node that manages all error monitoring activities.
 * 
 * This node serves as the main entry point and centralized data broker for the error 
 * monitoring system. Its primary purpose is to subscribe to all necessary sensor 
 * and state topics from the ROS 2 network and continuously update a shared data 
 * structure (`RobotStateBlackboard`).
 * 
 * Usage:
 * Instead of embedding individual error evaluation logic directly inside this node,
 * it acts as a manager. It instantiates individual rule-specific monitors (derived 
 * from `ErrorMonitorBase`), hands them a reference to the `RobotStateBlackboard`, 
 * and lets them operate independently. This decouples the network I/O from the 
 * actual error filtering mathematics, promoting better modularity.
 */
class ErrorMonitorNode : public rclcpp::Node
{
public:
    ErrorMonitorNode();
    ~ErrorMonitorNode();

    /**
     * @brief Initializes the node by registering and kicking off all default error monitors.
     */
    void init();

    /**
     * @brief Registers an individual error monitor into the node's execution pipeline.
     * 
     * This function utilizes dynamic polymorphism to seamlessly hook up the monitor 
     * to the ROS 2 ecosystem. It automatically feeds the node pointer to the monitor, 
     * commands it to load its specific configuration parameters from the given YAML section, 
     * prints those parameters to the console, and formally starts the monitor's internal callback loop.
     * 
     * @param monitor A shared pointer containing any monitor class that securely inherits 
     *                from `ErrorMonitorBase`.
     * @param config The entire parsed YAML Node for the monitors to use.
     */
    void addMonitor(std::shared_ptr<ErrorMonitorBase> monitor, const YAML::Node& config) {
        monitor->setNode(this);
        std::string ns = monitor->paramNamespace();
        if (config && config[ns]) {
            monitor->loadParams(config[ns]);
        } else {
            RCLCPP_WARN(this->get_logger(), "No YAML config found for monitor: %s, using defaults", ns.c_str());
            YAML::Node empty_node; 
            monitor->loadParams(empty_node);
        }
        monitor->printParams();
        monitor->startMonitor(blackboard_);
        monitors_.push_back(monitor);
    }

private:
    /**
     * @brief Periodically logs the internal resource and memory usage of the process.
     */
    void checkMemoryUsage();

    /** Shared memory container populated by this node and read by individual monitors. */
    std::shared_ptr<RobotStateBlackboard> blackboard_;

    // ROS 2 Subscribers for pulling in ambient sensor and system data
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

    /** Registry of all active monitor instances currently managed by this node. */
    std::vector<std::shared_ptr<ErrorMonitorBase>> monitors_;

    /** Periodic timer assigned to trigger memory usage checks. */
    rclcpp::TimerBase::SharedPtr memory_monitor_timer_;
};
