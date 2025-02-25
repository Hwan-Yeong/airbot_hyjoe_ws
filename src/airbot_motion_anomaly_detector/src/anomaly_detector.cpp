#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "robot_custom_msgs/msg/motor_status.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

class AnomalyDetector : public rclcpp::Node {
public:
    AnomalyDetector() : Node("anomaly_detector") {
        acceleration_threshold_ = 12.5;
        pitch_duration_threshold_ = 0.5;
        motor_current_threshold_front_ = 6500.0; // == 650mA ,based on testing, may have to change later
        motor_current_threshold_back_ = 4000.0;//based on testing, may have to change later
        motor_current_exceed_limit_ = 50;  //based on testing, may have to change later
        linear_accl_threshold = -0.6; //based on testing, may have to change later

        pitch_start_time_ = 0.0;
        motor_current_exceed_count_ = 0;
        cmd_vel_x_ = 0.0;
        cmd_vel_z_ = 0.0;

        collision_pub_ = this->create_publisher<std_msgs::msg::Bool>("collision_detection", 10);
        slope_pub_ = this->create_publisher<std_msgs::msg::Bool>("slope_detection", 10);
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_data", 10, std::bind(&AnomalyDetector::imu_callback, this, std::placeholders::_1));
        motor_status_sub_ = this->create_subscription<robot_custom_msgs::msg::MotorStatus>(
            "/motor_status", 10, std::bind(&AnomalyDetector::motor_status_callback, this, std::placeholders::_1));
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&AnomalyDetector::cmd_vel_callback, this, std::placeholders::_1));
    }

private:
    double acceleration_threshold_;
    double pitch_duration_threshold_;
    double motor_current_threshold_front_;
    double motor_current_threshold_back_;
    int motor_current_exceed_limit_;
    double pitch_start_time_;
    int motor_current_exceed_count_;
    double linear_accl_threshold;
    double cmd_vel_x_;
    double cmd_vel_z_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr slope_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::MotorStatus>::SharedPtr motor_status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_vel_x_ = msg->linear.x;
        cmd_vel_z_ = std::abs(msg->angular.z);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double roll, pitch, yaw;
        get_rpy_from_quaternion(msg->orientation, roll, pitch, yaw);

        double ax = msg->linear_acceleration.x;
        double current_time = this->now().seconds();

        if (cmd_vel_x_ > 0 && cmd_vel_x_ < 0.2) {
            linear_accl_threshold = -0.3;
        }
        else{
            linear_accl_threshold = -0.6;
        }
        if (ax < linear_accl_threshold && pitch_start_time_ == 0.0) {
            pitch_start_time_ = current_time;
        }

        if (pitch_start_time_ != 0.0) {
            if (pitch < -0.03) {
                double pitch_duration = current_time - pitch_start_time_;
                if (pitch_duration >= pitch_duration_threshold_) {
                    RCLCPP_WARN(this->get_logger(), "Collision detected! Pitch stayed negative for %.3f seconds.", pitch_duration);
                    publish_collision(true);
                    pitch_start_time_ = 0.0;
                }
            } else {
                pitch_start_time_ = 0.0;
            }
        }
        
    }

    void motor_status_callback(const robot_custom_msgs::msg::MotorStatus::SharedPtr msg) {
        double motor_current_left = std::abs(msg->left_motor_current);

        if (cmd_vel_x_ > 0 && motor_current_left > motor_current_threshold_front_) {
            motor_current_exceed_count_++;
        } else if (cmd_vel_x_ < 0 && motor_current_left > motor_current_threshold_back_) {
            motor_current_exceed_count_++;
        } else {
            motor_current_exceed_count_ = 0;
        }

        // check_anomaly();
    }

    void check_collision() {
        if (motor_current_exceed_count_ >= motor_current_exceed_limit_) {
            RCLCPP_WARN(this->get_logger(), "Collision Detected: High Motor Current: %d!", motor_current_exceed_count_);
            publish_collision(true);
            motor_current_exceed_count_ = 0;
        } else {
            publish_collision(false);
        }
    }

    void publish_collision(bool detected) {
        std_msgs::msg::Bool collision_msg;
        collision_msg.data = detected;
        collision_pub_->publish(collision_msg);
    }

    void get_rpy_from_quaternion(const geometry_msgs::msg::Quaternion& quaternion, double& roll, double& pitch, double& yaw) {
        tf2::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AnomalyDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
