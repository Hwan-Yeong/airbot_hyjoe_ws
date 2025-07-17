#ifndef ANOMALY_DETECTOR_HPP
#define ANOMALY_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <deque>
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
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<robot_custom_msgs::msg::AbnormalEventData>::SharedPtr collision_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr perception_climb_sub_;

    // Parameters
    double pitch_threshold_collision_;
    double pitch_diff_threshold_;
    int pitch_diff_window_size_;
    int pitch_collision_duration_threshold_;

    double robot_pose_x;
    double robot_pose_y;
    double robot_pose_angle;

    bool is_climb_detected_;
    bool is_collision_detected_;
    int collision_cnt_;
    std::deque<double> pitch_history_;

    // Variables
    double cmd_vel_x_;

    // IMU Data Processing
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    // Velocity Processing
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Collision Detection
    void detectCollision(double pitch);

    // Publish Collision Alert
    void publishCollision(bool detected);

    // Convert Quaternion to Roll, Pitch, Yaw
    void getRPYFromQuaternion(const geometry_msgs::msg::Quaternion &q, double &roll, double &pitch, double &yaw);
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void perceptionClimbCallback(const std_msgs::msg::Bool::SharedPtr msg);
    double quaternion_to_euler(const geometry_msgs::msg::Quaternion& quat);

};

#endif // ANOMALY_DETECTOR_HPP
