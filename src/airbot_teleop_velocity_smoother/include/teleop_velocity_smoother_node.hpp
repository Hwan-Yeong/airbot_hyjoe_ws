#ifndef TELEOP_VELOCITY_SMOOTHER_HPP_
#define TELEOP_VELOCITY_SMOOTHER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace airbot {

class TeleopVelocitySmoother : public rclcpp::Node {
public:
    explicit TeleopVelocitySmoother(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~TeleopVelocitySmoother() = default;

private:
    // ROS 2 통신 설정
    void init_ros_interface();

    // 파라미터 로드
    void load_parameters();

    // 콜백 함수
    void input_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void timer_callback();

    // 유틸리티 함수
    double apply_limit(double current, double target, double max_accel, double dt);

    // ROS 2 Interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_input_vel_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_output_vel_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 데이터 변수
    geometry_msgs::msg::Twist target_vel_;
    geometry_msgs::msg::Twist current_vel_;
    rclcpp::Time last_update_time_;
    std::mutex data_mutex_;

    // 제어 파라미터
    double max_accel_linear_;
    double max_accel_angular_;
    double deadband_linear_;
    double deadband_angular_;
    double control_frequency_;
};

} // namespace robot_motion

#endif // TELEOP_VELOCITY_SMOOTHER_HPP_