#ifndef ANOMALY_DETECTOR_HPP
#define ANOMALY_DETECTOR_HPP

#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include "robot_custom_msgs/msg/abnormal_event_data.hpp"
#include "robot_custom_msgs/msg/motor_status.hpp"
#include "robot_custom_msgs/msg/bottom_ir_data.hpp"
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
    rclcpp::Subscription<robot_custom_msgs::msg::BottomIrData>::SharedPtr bottom_ir_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr climb_state_sub_;

    rclcpp::TimerBase::SharedPtr collision_detection_timer_;

    // Parameters
    double pitch_collision_duration_threshold_;
    double pitch_slope_duration_threshold_;
    double acceleration_threshold_low_;
    double acceleration_threshold_high_;
    double pitch_threshold_collision_;
    double pitch_threshold_;
    double motor_current_threshold_front_;
    double motor_current_threshold_back_;
    int motor_current_exceed_limit_;

    double robot_pose_x_;
    double robot_pose_y_;
    double robot_pose_angle_;

    double ax_;
    double ay_;
    double roll_;
    double pitch_;
    double yaw_;
    robot_custom_msgs::msg::BottomIrData bottom_ir_;

    // Variables
    double pitch_start_time_;
    double pitch_slope_start_time_;
    int motor_current_exceed_count_;
    double cmd_vel_x_;

    bool bClimb_ = false;

    std::deque<int16_t> history_ff_;
    std::deque<int16_t> history_fl_;
    std::deque<int16_t> history_fr_;
    std::deque<int16_t> history_bb_;
    std::deque<int16_t> history_bl_;
    std::deque<int16_t> history_br_;

    // Collision Detection Monitor
    void collisionMonitor();

    // Climb State Callback
    void climbStateCallback(const std_msgs::msg::Bool::SharedPtr msg);

    // Bottom IR Procesing
    void bottomIrCallback(const robot_custom_msgs::msg::BottomIrData::SharedPtr msg);

    // IMU Data Processing
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    // Motor Current Processing
    void motorStatusCallback(const robot_custom_msgs::msg::MotorStatus::SharedPtr msg);

    // Velocity Processing
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Collision Detection
    void detectCollision(double ax, double ay, double pitch, robot_custom_msgs::msg::BottomIrData ir_data, double current_time, double accel_threshold);

    // Slope Detection
    void detectSlope(double pitch, double roll, double current_time);

    // Publish Collision Alert
    void publishCollision(bool detected);

    // Publish Slope Alert
    void publishSlope(bool detected);

    // Drop IR Moving Average Filter update Function
    robot_custom_msgs::msg::BottomIrData updateMovingAverageFilter(const robot_custom_msgs::msg::BottomIrData input);

    // Convert Quaternion to Roll, Pitch, Yaw
    void getRPYFromQuaternion(const geometry_msgs::msg::Quaternion &q, double &roll, double &pitch, double &yaw);
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    double quaternion_to_euler(const geometry_msgs::msg::Quaternion& quat);

};

#endif // ANOMALY_DETECTOR_HPP
