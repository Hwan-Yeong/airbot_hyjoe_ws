#pragma once
/**
 * @ NAMUHx A1 로봇 AMR부 ros2 시스템의 H/W <-> AP 인터페이스에서 파싱하는 토픽들을 dummy로 발행하는 노드
 *
 * @ 목적: H/W 종속성 없이 S/W 자체 검증이 용이하게 하기 위함
 */

#include <chrono>
#include <vector>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_custom_msgs/msg/battery_status.hpp"
#include "robot_custom_msgs/msg/motor_status.hpp"
#include "robot_custom_msgs/msg/tof_data.hpp"
#include "robot_custom_msgs/msg/imu_calibration.hpp"
#include "robot_custom_msgs/msg/station_data.hpp"
#include "robot_custom_msgs/msg/bottom_ir_data.hpp"


class NamuhxA1DummyNode : public rclcpp::Node
{
public:
    NamuhxA1DummyNode();
    ~NamuhxA1DummyNode();

private:
    void processor_callback();
    void uart_communication_processor();
    void lidar_processor();

    rclcpp::TimerBase::SharedPtr processor_;

    // parameters
    int uart_loop_cnt_;
    int lidar_loop_cnt_;
    int uart_loop_rate_ms_;
    int lidar_loop_rate_ms_;

    // #############################################
    // ############# uart communication #############
    // publisher
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr odom_status_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr bottom_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fw_version_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<robot_custom_msgs::msg::BatteryStatus>::SharedPtr battery_status_pub_;
    rclcpp::Publisher<robot_custom_msgs::msg::MotorStatus>::SharedPtr motor_status_pub_;
    rclcpp::Publisher<robot_custom_msgs::msg::TofData>::SharedPtr tof_data_pub_;
    rclcpp::Publisher<robot_custom_msgs::msg::ImuCalibration>::SharedPtr jig_imu_calibration_pub_;
    rclcpp::Publisher<robot_custom_msgs::msg::StationData>::SharedPtr station_data_pub_;
    rclcpp::Publisher<robot_custom_msgs::msg::BottomIrData>::SharedPtr bottom_ir_data_pub_;

    void pub_odom_status();
    void pub_bottom_status();
    void pub_fw_version();
    void pub_imu_data();
    void pub_odom();
    void pub_battery_status();
    void pub_motor_status();
    void pub_tof_data();
    void pub_jig_imu_calibration();
    void pub_station_data();
    void pub_bottom_ir_data();

    // subscriber
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr version_request_sub_;
    // fw version
    void req_fw_version_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    // imu
    double imu_prev_roll_ = 0.0;
    double imu_prev_pitch_ = 0.0;
    double imu_prev_yaw_ = 0.0;
    std::chrono::steady_clock::time_point imu_prev_time_;
    geometry_msgs::msg::Quaternion imu_orientation_;
    std::vector<double> computeAngularVelocity(double roll, double pitch, double roll_dot, double pitch_dot, double yaw_dot);
    // odom
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // #############################################


    // #############################################
    // ################### lidar ###################
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr scan_state_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    void pub_scan_state();
    void pub_scan();
    // #############################################
};