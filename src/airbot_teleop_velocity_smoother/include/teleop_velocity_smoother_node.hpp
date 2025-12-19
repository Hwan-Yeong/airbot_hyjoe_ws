#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "s_curve_profile.hpp"

namespace airbot {

class TeleopVelocitySmoother : public rclcpp::Node
{
public:
    TeleopVelocitySmoother();

private:
    void cmdVelTeleopCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void smootherTimer();

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool is_get_cmd_vel_teleop_ = false;
    std::chrono::steady_clock::time_point start_time_;
    double dt_, timeout_;
    SCurveProfile linear_;
    SCurveProfile angular_;

    double target_lin_{0.0};
    double target_ang_{0.0};
};

} // namespace airbot
