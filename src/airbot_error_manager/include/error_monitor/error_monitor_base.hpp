#pragma once

#include <string>
#include <memory>
#include <initializer_list>
#include <algorithm>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "error_monitor/robot_state_blackboard.hpp"

enum class SensorState { NORMAL, DELAYED, TIMEOUT };

class ErrorMonitorBase
{
public:
    virtual ~ErrorMonitorBase() = default;

    virtual void loadParams(const std::string& ns) = 0;
    virtual void printParams() const = 0;
    virtual void startMonitor(std::shared_ptr<RobotStateBlackboard> blackboard) = 0;

    void setNode(rclcpp::Node* node) {
        node_ptr_ = node;
    }
protected:
    rclcpp::Node* node_ptr_ = nullptr;
    std::shared_ptr<RobotStateBlackboard> blackboard_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    SensorState current_sensor_state_ = SensorState::NORMAL;

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


