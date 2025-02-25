#ifndef ERROR_MANAGER_HPP
#define ERROR_MANAGER_HPP

#include <iostream>
#include <cmath>
#include <vector>
#include <chrono>
#include <array>
#include <algorithm>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robot_custom_msgs/msg/error_list.hpp"
#include "robot_custom_msgs/msg/error_list_array.hpp"


#define CLEAR_CNT 5
#define ERROR_LIST_SIZE 10

class ErrorManager : public rclcpp::Node
{
public:
    ErrorManager();
    ~ErrorManager();

private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_left_motor_stuck_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_right_motor_stuck_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_scan_dirty_front_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_scan_dirty_back_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_docking_station_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_charging_error_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr f_battery_charging_overcurrent_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr f_battery_discharging_overcurrent_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr f_top_tof_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr f_camera_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr f_right_motor_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr f_left_motor_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr f_scan_front_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr f_scan_back_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr f_battery_overheat_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr f_bot_tof_error_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr s_unreachable_goal_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr s_change_temporary_goal_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr s_fall_down_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr s_unable_to_docking_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr s_board_overheat_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr s_station_overheat_error_sub_;

    rclcpp::Publisher<robot_custom_msgs::msg::ErrorListArray>::SharedPtr error_list_pub_;

    rclcpp::TimerBase::SharedPtr pub_timer_;

    std::vector<std::shared_ptr<robot_custom_msgs::msg::ErrorList>> error_list_;

private:
    void publishErrorList();
    
    void leftMotorStuckErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void rightMotorStuckErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void scanDirtyFrontErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void scanDirtyBackErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void dockingStationErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void chargingErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void batChargingOverCurrentErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void batDischargingOverCurrentErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void topTofErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void cameraErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void rightMotorErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void leftMotorErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void scanFrontErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void scanBackErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void batOverHeatErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void botTofErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void unReachableGoalErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void changeTempGoalErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void fallDownErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void unableToDockingErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void boardOverHeatErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void stationOverHeatErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void updateErrorLists(int rank, std::string code);
    void addError(int rank, const std::string &error_code);
    void removeFromErrorLists(int rank);
    void printErrorList();
};

#endif // ERROR_MANAGER_HPP
