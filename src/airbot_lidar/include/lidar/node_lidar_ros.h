#ifndef NODE_LIDAR_ROS_HPP
#define NODE_LIDAR_ROS_HPP

#include "node_lidar.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include "std_msgs/msg/bool.hpp"
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>

#include <iostream>
#include <chrono>
#include <thread>

enum class LidarState {
    WAITING_TO_START,
    RUNNING,
    WAITING_TO_STOP,
    IDLE,
    ERROR_WHILE_RUNNING,
    ERROR_WHILE_STOPPING
};

inline std::string enumToString(LidarState state)
{
    switch (state) {
        case LidarState::WAITING_TO_START: return "WAITING_TO_START";
        case LidarState::RUNNING: return "RUNNING";
        case LidarState::WAITING_TO_STOP: return "WAITING_TO_STOP";
        case LidarState::IDLE: return "IDLE";
        case LidarState::ERROR_WHILE_RUNNING: return "ERROR_WHILE_RUNNING";
        case LidarState::ERROR_WHILE_STOPPING: return "ERROR_WHILE_STOPPING";
        default: return "UNKNOWN_STATE";
    }
}

class LidarManager
{
public:
    LidarManager(const rclcpp::Node::SharedPtr& node);

    void runLoop();

private:
    void initParams();

    void cmdLidarCallback(const std_msgs::msg::Bool::SharedPtr msg);

    LidarState procWaitingToStart();
    LidarState procRunning();
    LidarState procWaitingToStop();
    LidarState procIdle();
    LidarState procErrorWhileRunning();
    LidarState procErrorWhileStopping();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr scan_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr scan_dirty_error_pub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cmd_lidar_sub_;

    std_msgs::msg::Bool error_msg_;
    std_msgs::msg::Bool dirty_msg_;

    std::string frame_id_;
    LaserScan scan_;

    bool bLidarCmd;
    bool bLidarRun;
    bool bErrorState;

    LidarState state_;
    LidarState prev_state_;
    unsigned int start_cnt_;
    unsigned int lidar_run_cnt_;
    unsigned int dirty_points_;
    unsigned int dirty_cnt_;
    unsigned int dirty_reset_cnt_;
};

#endif // NODE_LIDAR_ROS_HPP