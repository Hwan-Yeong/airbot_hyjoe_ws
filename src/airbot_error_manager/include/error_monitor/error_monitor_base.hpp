#pragma once

#include <string>
#include <memory>
#include <initializer_list>
#include <algorithm>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "error_monitor/robot_state_blackboard.hpp"

/**
 * @enum SensorState
 * @brief Represents the real-time communication freshness of a sensor.
 */
enum class SensorState { NORMAL, DELAYED, TIMEOUT };

/**
 * @class ErrorMonitorBase
 * @brief Abstract base class defining the foundational blueprint for all individual error monitors.
 * 
 * Purpose:
 * This class establishes a strict, unified interface ensuring that every specialized 
 * error monitoring component (such as LowBattery, FallDown, BoardOverheat) adheres 
 * to a standardized execution lifecycle. 
 * 
 * Usage:
 * Any new monitor must inherit from this class and explicitly implement its pure 
 * virtual functions (`paramNamespace`, `loadParams`, `printParams`, `startMonitor`).
 * By doing so, the central `ErrorMonitorNode` can polymorphically manage an arbitrary 
 * number of monitors without knowing their specific internal logic. 
 * 
 * It also ships with built-in utility functions, notably `checkSensorState()`, 
 * to relieve child classes from having to write redundant sensor timeout detection logic.
 */
class ErrorMonitorBase
{
public:
    virtual ~ErrorMonitorBase() = default;

    /**
     * @brief Declares the specific ROS parameter namespace for the monitor. (Mandatory override)
     * 
     * This pure virtual function forces the developer to define the parameter block 
     * name associated with their monitor.
     * 
     * @note MUST exactly match the YAML key defined in `config/error_manager_params.yaml`.
     * @return const std::string The parameter namespace string (e.g., "fall_down_error").
     */
    virtual const std::string paramNamespace() const = 0;

    /**
     * @brief Fetches necessary algorithm parameters from the ROS parameter server.
     * @param ns The namespace provided by `paramNamespace()`.
     */
    virtual void loadParams(const std::string& ns) = 0;

    /**
     * @brief Prints the loaded parameters to the console for initialization verification.
     */
    virtual void printParams() const = 0;

    /**
     * @brief Initiates the monitor's internal evaluation loop and binds it to the blackboard.
     * @param blackboard A shared pointer to the central repository of real-time sensor data.
     */
    virtual void startMonitor(std::shared_ptr<RobotStateBlackboard> blackboard) = 0;

    /**
     * @brief Injects the parent ROS node raw pointer into the monitor.
     * 
     * We use a raw pointer here (`rclcpp::Node*`) rather than a `shared_ptr` to strictly 
     * prevent cyclic reference memory leaks between the Monitor and the Node.
     * 
     * @param node Raw pointer of the `ErrorMonitorNode`.
     */
    void setNode(rclcpp::Node* node) {
        node_ptr_ = node;
    }
protected:
    rclcpp::Node* node_ptr_ = nullptr;
    std::shared_ptr<RobotStateBlackboard> blackboard_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    SensorState current_sensor_state_ = SensorState::NORMAL;

    /**
     * @brief A centralized helper to evaluate if sensor data streams are healthy, delayed, or dead.
     * 
     * It scans an arbitrary number of sensor update timestamps, calculates the maximum 
     * elapsed delay against the current system time, and sorts the condition into a 
     * 3-tier state. It also handles one-off console logging automatically when 
     * state transitions occur.
     * 
     * Thresholds:
     * - DELAYED: Triggers when the maximum delay exceeds `sensor_period_ms * 3`.
     * - TIMEOUT: Triggers when the maximum delay exceeds `sensor_period_ms * 30`.
     * 
     * Usage inside child monitor:
     * ```cpp
     * if (checkSensorState(paramNamespace(), 10, {ir.last_update_time, imu.last_update_time}) != SensorState::NORMAL) return;
     * ```
     * 
     * @param monitor_name Name of the parent monitor passing this check (for logging context).
     * @param sensor_period_ms The ideal nominal update frequency of the sensors in milliseconds.
     * @param stamps A braced initializer list of `steady_clock::time_point` objects from the sensors being evaluated.
     * @return SensorState Returns NORMAL, DELAYED, or TIMEOUT.
     */
    SensorState checkSensorState(const std::string& monitor_name, int sensor_period_ms, std::initializer_list<std::chrono::steady_clock::time_point> stamps) {
        if (stamps.size() == 0) return SensorState::NORMAL;

        auto now = std::chrono::steady_clock::now();
        long max_delay_ms = 0;

        for (const auto& stamp : stamps) {
            if (stamp.time_since_epoch().count() == 0) {
                max_delay_ms = std::max(max_delay_ms, 999999L);
                continue;
            }
            auto delay_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - stamp).count();
            max_delay_ms = std::max(max_delay_ms, static_cast<long>(delay_ms));
        }

        int delay_limit = sensor_period_ms * 3;
        int timeout_limit = sensor_period_ms * 30;

        SensorState new_state = SensorState::NORMAL;
        if (max_delay_ms > timeout_limit) {
            new_state = SensorState::TIMEOUT;
        } else if (max_delay_ms > delay_limit) {
            new_state = SensorState::DELAYED;
        }

        if (current_sensor_state_ != new_state) {
            if (new_state == SensorState::TIMEOUT) {
                RCLCPP_ERROR(node_ptr_->get_logger(),
                    "[%s] Sensor Communication Timeout Error (Critical)! Delay: %ld ms. Suspending monitor...",
                    monitor_name.c_str(),
                    max_delay_ms
                );
            } else if (new_state == SensorState::DELAYED) {
                RCLCPP_WARN(node_ptr_->get_logger(),
                    "[%s] Sensor Delayed (Holding state). Delay: %ld ms. Suspending monitor...",
                    monitor_name.c_str(),
                    max_delay_ms
                );
            } else if (new_state == SensorState::NORMAL) {
                RCLCPP_INFO(node_ptr_->get_logger(),
                    "[%s] Sensor Recovered! Resuming monitor.",
                    monitor_name.c_str()
                );
            }
            current_sensor_state_ = new_state;
        }

        return current_sensor_state_;
    }
};
