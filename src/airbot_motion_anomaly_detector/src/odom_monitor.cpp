#include "airbot_motion_anomaly_detector/odom_monitor.hpp"

OdomMonitor::OdomMonitor() : Node("odom_monitor"), last_odom_time_(this->now()), last_imu_time_(this->now()) {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&OdomMonitor::odom_callback, this, std::placeholders::_1));
    
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&OdomMonitor::cmd_vel_callback, this, std::placeholders::_1));
    
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu_data", 10, std::bind(&OdomMonitor::imu_callback, this, std::placeholders::_1));

    // Timer to check if odom or IMU data is missing
    data_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&OdomMonitor::data_timer_callback, this));
}

void OdomMonitor::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_cmd_vel_ = *msg;
}

void OdomMonitor::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    auto now = this->now();
    rclcpp::Duration time_diff = now - last_odom_time_;
    double dt = time_diff.seconds();


    if (!first_odom_received_) {
        last_odom_ = *msg;
        first_odom_received_ = true;
        last_odom_time_ = now;
        return;
    }

    // Check if odom is being received at expected intervals (10ms = 0.01s)
    // const double expected_dt = 0.01; // Expected time interval (10ms)
    // const double max_dt_tolerance = 0.005; // 5ms tolerance

    //***Odom callback delay check */ 
    // if (std::abs(dt - expected_dt) > max_dt_tolerance) {
    //     RCLCPP_WARN(this->get_logger(), 
    //                 "[odom_monitor] Odom callback delay detected! Expected dt: %.3f s, Actual dt: %.3f s", 
    //                 expected_dt, dt);
    // }
    
    // Calculate position change (linear distance)
    double dx = msg->pose.pose.position.x - last_odom_.pose.pose.position.x;
    double dy = msg->pose.pose.position.y - last_odom_.pose.pose.position.y;
    double actual_distance = std::sqrt(dx * dx + dy * dy);

    // Get yaw values (orientation)
    double yaw1 = tf2::getYaw(last_odom_.pose.pose.orientation); //old odom orientation
    double yaw2 = tf2::getYaw(msg->pose.pose.orientation); //current odom orientatin

    // Calculate angular displacement with wrapping
    double angular_displacement = std::abs(angle_diff(yaw1, yaw2));

    // Compute dynamic position tolerance
    double expected_speed = last_cmd_vel_.linear.x; // Robot's last commanded speed
    double dynamic_position_tolerance = std::max(0.01, (std::abs(expected_speed) * dt) + 0.002); // Add 2mm buffer
    const double angular_tolerance = 0.03;  // 0.03 rad ≈ 1.7°


    // Check if the displacement exceeds the tolerance
    if (actual_distance > dynamic_position_tolerance || angular_displacement > angular_tolerance) {
        RCLCPP_WARN(this->get_logger(), 
                    "[odom_monitor] Unexpected odometry change detected! Actual linear distance: %.3f (Tolerance: %.3f), Angular displacement: %.3f", 
                    actual_distance, dynamic_position_tolerance, angular_displacement);
    }

    // Update the previous odom for the next callback
    last_odom_ = *msg;
    last_odom_time_ = now;
}

double OdomMonitor:: angle_diff(double angle1, double angle2) {
    double diff = angle2 - angle1;
    if (diff > M_PI) {
        diff -= 2 * M_PI;  // Wrap around
    } else if (diff < -M_PI) {
        diff += 2 * M_PI;  // Wrap around
    }
    return diff;
}

double OdomMonitor::round_to_decimal_places (double value, int decimal_places) {
    double factor = std::pow(10.0, decimal_places);
    return std::round(value * factor) / factor;
}

void OdomMonitor::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    auto now = this->now();
    rclcpp::Duration time_diff = now - last_imu_time_;
    double dt = time_diff.seconds();

    if (!first_imu_received_) {
        last_imu_ = *msg;
        first_imu_received_ = true;
        last_imu_time_ = now;
        return;
    }

    // Get current and previous yaw from IMU quaternion using tf2::getYaw
    double previous_yaw = tf2::getYaw(last_imu_.orientation);
    double current_yaw = tf2::getYaw(msg->orientation);

    // Round yaw values to 5 decimal places
    double rounded_previous_yaw = round_to_decimal_places(previous_yaw, 5);
    double rounded_current_yaw = round_to_decimal_places(current_yaw, 5);

    double yaw_change = std::abs(angle_diff(rounded_previous_yaw, rounded_current_yaw));

    // Calculate expected yaw change based on cmd_vel.angular.z
    double expected_yaw_change = std::abs(last_cmd_vel_.angular.z) * dt;

    // Set tolerance for yaw change comparison
    const double yaw_tolerance = 0.03;  // radians (to avoid small noises)
    double yaw_change_threshold = 1e-3;

    // Check if yaw change is negligible (same up to 5 decimal places)
    if (yaw_change < yaw_change_threshold) { /// not correct have to check
        // RCLCPP_WARN(this->get_logger(), "IMU anomaly detected! ! Yaw has not changed significantly. Previous Yaw: %.9f, Current Yaw: %.9f", previous_yaw, current_yaw);
    } 
    else {
        if (yaw_change > expected_yaw_change + yaw_tolerance) {
                RCLCPP_WARN(this->get_logger(),
                "[IMU_monitor]IMU anomaly detected! Unexpected yaw change. Actual Yaw Change: %.3f, Expected Yaw Change: %.3f (Tolerance: %.3f)",
                yaw_change, expected_yaw_change, yaw_tolerance);
        }   
    }

    // Update last IMU data
    last_imu_ = *msg;
    last_imu_time_ = now;
}

void OdomMonitor::data_timer_callback() {
    auto now = this->now();
    rclcpp::Duration odom_diff = now - last_odom_time_;
    rclcpp::Duration imu_diff = now - last_imu_time_;

    if (odom_diff.seconds() > 0.05) {
        RCLCPP_WARN(this->get_logger(), "No odometry data received for %.2f seconds.", odom_diff.seconds());
    }
    if (imu_diff.seconds() > 0.05) {
        RCLCPP_WARN(this->get_logger(), "No IMU data received for %.2f seconds.", imu_diff.seconds());
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomMonitor>());
    rclcpp::shutdown();
    return 0;
}
