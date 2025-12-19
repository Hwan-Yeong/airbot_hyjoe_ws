#include "teleop_velocity_smoother_node.hpp"

namespace airbot {

TeleopVelocitySmoother::TeleopVelocitySmoother()
: Node("airbot_teleop_velocity_smoother_node"), is_get_cmd_vel_teleop_(false)
{
    declare_parameter("dt_sec", 0.01);
    declare_parameter("timeout_sec", 1.0);

    declare_parameter("linear.max_vel", 0.5);
    declare_parameter("linear.max_accel", 0.6);
    declare_parameter("linear.max_jerk", 2.0);

    declare_parameter("angular.max_vel", 1.57);
    declare_parameter("angular.max_accel", 2.5);
    declare_parameter("angular.max_jerk", 8.0);

    dt_ = get_parameter("dt_sec").as_double();
    timeout_ = get_parameter("timeout_sec").as_double();

    linear_.setLimits(
        get_parameter("linear.max_vel").as_double(),
        get_parameter("linear.max_accel").as_double(),
        get_parameter("linear.max_jerk").as_double());

    angular_.setLimits(
        get_parameter("angular.max_vel").as_double(),
        get_parameter("angular.max_accel").as_double(),
        get_parameter("angular.max_jerk").as_double());

    sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_teleop", 10,
        std::bind(&TeleopVelocitySmoother::cmdVelTeleopCallback, this, std::placeholders::_1));

    pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    timer_ = create_wall_timer(
        std::chrono::duration<double>(dt_),
        std::bind(&TeleopVelocitySmoother::smootherTimer, this));

    RCLCPP_INFO(get_logger(), "Time-Optimal S-Curve Velocity Smoother started");
}

void TeleopVelocitySmoother::cmdVelTeleopCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    is_get_cmd_vel_teleop_ = true;
    start_time_ = std::chrono::steady_clock::now();
    target_lin_ = msg->linear.x;
    target_ang_ = msg->angular.z;
}

void TeleopVelocitySmoother::smootherTimer()
{
    auto now = std::chrono::steady_clock::now();

    if (is_get_cmd_vel_teleop_) {
        auto time_diff = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_).count();
        if (time_diff > timeout_) {
            is_get_cmd_vel_teleop_ = false;
            target_lin_ = 0.0;
            target_ang_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "cmd_vel_teleop Time out, last topic received %.2f sec before", time_diff);
        }
    }

    double lin = linear_.update(target_lin_, dt_);
    double ang = angular_.update(target_ang_, dt_);

    if (!is_get_cmd_vel_teleop_ && std::abs(lin) < 1e-3 && std::abs(ang) < 1e-3) {
        return;
    }

    geometry_msgs::msg::Twist out;
    out.linear.x  = lin;
    out.angular.z = ang;
    pub_->publish(out);
}

} // namespace airbot
