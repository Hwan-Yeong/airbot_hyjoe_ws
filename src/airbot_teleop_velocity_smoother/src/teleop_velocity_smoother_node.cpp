#include "teleop_velocity_smoother_node.hpp"

namespace airbot {

TeleopVelocitySmoother::TeleopVelocitySmoother(const rclcpp::NodeOptions & options)
: Node("teleop_velocity_smoother_node", options)
{
    load_parameters();
    init_ros_interface();

    last_update_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "TeleopVelocitySmoother initialized.");
}

void TeleopVelocitySmoother::load_parameters() {
    this->declare_parameter("max_accel_linear", 0.8);      // m/s^2
    this->declare_parameter("max_accel_angular", 1.5);     // rad/s^2
    this->declare_parameter("deadband_linear", 0.001);
    this->declare_parameter("deadband_angular", 0.001);
    this->declare_parameter("control_frequency", 50.0);    // Hz

    max_accel_linear_ = this->get_parameter("max_accel_linear").as_double();
    max_accel_angular_ = this->get_parameter("max_accel_angular").as_double();
    deadband_linear_ = this->get_parameter("deadband_linear").as_double();
    deadband_angular_ = this->get_parameter("deadband_angular").as_double();
    control_frequency_ = this->get_parameter("control_frequency").as_double();
}

void TeleopVelocitySmoother::init_ros_interface() {
    // GUI/Teleop에서 들어오는 날것의 명령
    sub_input_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_teleop", 10, std::bind(&TeleopVelocitySmoother::input_callback, this, std::placeholders::_1));

    // 로봇으로 나가는 가공된 명령
    pub_output_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // 주기적인 속도 업데이트 타이머
    auto period = std::chrono::duration<double>(1.0 / control_frequency_);
    timer_ = this->create_wall_timer(period, std::bind(&TeleopVelocitySmoother::timer_callback, this));
}

void TeleopVelocitySmoother::input_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    target_vel_ = *msg;
}

void TeleopVelocitySmoother::timer_callback() {
    auto now = this->now();
    double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;

    // dt가 너무 크거나 비정상적이면 스킵
    if (dt <= 0.0 || dt > 0.5) return;

    geometry_msgs::msg::Twist out_vel;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // 선속도 제한 (X축)
        out_vel.linear.x = apply_limit(current_vel_.linear.x, target_vel_.linear.x, max_accel_linear_, dt);

        // 각속도 제한 (Z축)
        out_vel.angular.z = apply_limit(current_vel_.angular.z, target_vel_.angular.z, max_accel_angular_, dt);
    }

    // Deadband 처리 (매우 작은 값은 0으로 만들어 모터 떨림 방지)
    if (std::abs(out_vel.linear.x) < deadband_linear_) out_vel.linear.x = 0.0;
    if (std::abs(out_vel.angular.z) < deadband_angular_) out_vel.angular.z = 0.0;

    current_vel_ = out_vel;
    pub_output_vel_->publish(out_vel);
}

double TeleopVelocitySmoother::apply_limit(double current, double target, double max_accel, double dt) {
    double step = max_accel * dt;
    double error = target - current;

    if (std::abs(error) < step) {
        return target;
    }

    // 현재 속도에서 가속도 한계만큼만 목표 방향으로 가감속
    return current + (error > 0 ? step : -step);
}

} // namespace airbot

// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(airbot::TeleopVelocitySmoother)