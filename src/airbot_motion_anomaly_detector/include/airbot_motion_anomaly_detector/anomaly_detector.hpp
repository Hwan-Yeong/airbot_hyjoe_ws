#ifndef ANOMALY_DETECTOR_HPP
#define ANOMALY_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include "robot_custom_msgs/msg/abnormal_event_data.hpp"
#include "robot_custom_msgs/msg/motor_status.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


class AnomalyDetector : public rclcpp::Node {
public:
    AnomalyDetector();

private:
    // ROS 2 Subscribers & Publishers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::MotorStatus>::SharedPtr motor_status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<robot_custom_msgs::msg::AbnormalEventData>::SharedPtr collision_pub_;
    rclcpp::Publisher<robot_custom_msgs::msg::AbnormalEventData>::SharedPtr slope_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
    

    // Parameters
    double pitch_duration_threshold_;
    double pitch_slope_duration_threshold_;
    double acceleration_threshold_low_;
    double acceleration_threshold_high_;
    double pitch_threshold_collision_;
    double pitch_threshold_min;
    double pitch_threshold_max;
    double motor_current_threshold_front_;
    double motor_current_threshold_back_;
    int motor_current_exceed_limit_;

    double robot_pose_x;
    double robot_pose_y;
    double robot_pose_angle;

    // Variables
    double pitch_start_time_;
    double pitch_slope_start_time_;
    int motor_current_exceed_count_;
    double cmd_vel_x_;

    // IMU Data Processing
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    // Motor Current Processing
    void motorStatusCallback(const robot_custom_msgs::msg::MotorStatus::SharedPtr msg);

    // Velocity Processing
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Collision Detection
    void detectCollision(double ax, double pitch, double current_time, double accel_threshold);

    // Slope Detection
    void detectSlope(double pitch, double current_time);

    // Publish Collision Alert
    void publishCollision(bool detected);

    // Publish Slope Alert
    void publishSlope(bool detected);

    // Convert Quaternion to Roll, Pitch, Yaw
    void getRPYFromQuaternion(const geometry_msgs::msg::Quaternion &q, double &roll, double &pitch, double &yaw);
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    double quaternion_to_euler(const geometry_msgs::msg::Quaternion& quat);

};

#endif // ANOMALY_DETECTOR_HPP
