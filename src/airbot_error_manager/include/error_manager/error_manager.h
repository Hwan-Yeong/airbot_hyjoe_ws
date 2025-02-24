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


class ErrorManager : public rclcpp::Node
{
public:
    ErrorManager();
    ~ErrorManager();

private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr scan_front_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr scan_back_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr scan_dirty_front_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr scan_dirty_back_error_sub_;

    rclcpp::Publisher<robot_custom_msgs::msg::ErrorListArray>::SharedPtr error_list_pub_;

    rclcpp::TimerBase::SharedPtr pub_timer_;

    std::vector<std::shared_ptr<robot_custom_msgs::msg::ErrorList>> error_list_;

private:
    void scanFrontErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void scanBackErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void scanDirtyFrontErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void scanDirtyBackErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void updateErrorLists(int rank, std::string code);
    void publishErrorList();
    void addError(int rank, const std::string &error_code);
};

#endif // ERROR_MANAGER_HPP
