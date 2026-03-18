#pragma once

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "error_monitor/robot_state_blackboard.hpp"

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
};


