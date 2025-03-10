#ifndef APPLICATION_LOGGER_HPP
#define APPLICATION_LOGGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <actionlib_msgs/msg/goal_status_array.hpp>

#include "domain/parameter.hpp"
#include "domain/data.hpp"
#include "application/process.hpp"

#define LOG_BIG_LINE "=================================================================================="
#define LOG_SMALL_LINE "-------------------------------------------------------------------------------"

#define RCL_LOG_INFO(logger, format, ...)  RCLCPP_INFO(logger, "%s():%d:" format, __func__, __LINE__, ##__VA_ARGS__)
#define RCL_LOG_WARN(logger, format, ...)  RCLCPP_WARN(logger, "%s():%d:" format, __func__, __LINE__, ##__VA_ARGS__)
#define RCL_LOG_DEBUG(logger, format, ...) RCLCPP_DEBUG(logger, "%s():%d:" format, __func__, __LINE__, ##__VA_ARGS__)
#define RCL_LOG_ERROR(logger, format, ...) RCLCPP_ERROR(logger, "%s():%d:" format, __func__, __LINE__, ##__VA_ARGS__)
#define RCL_LOG_FATAL(logger, format, ...) RCLCPP_FATAL(logger, "%s():%d:" format, __func__, __LINE__, ##__VA_ARGS__)

using std::placeholders::_1;

class LoggerService
{
private:
    rclcpp::Node::SharedPtr node_;
    Parameter::SharedPtr parameter_;
    Data::SharedPtr data_;
    ProcessService::SharedPtr process_service_;

    rclcpp::CallbackGroup::SharedPtr logging_timer_cb_group_;
    rclcpp::TimerBase::SharedPtr logging_timer_;
    void logging_timer_cb();

    bool is_warm_up_started_;
    rclcpp::CallbackGroup::SharedPtr warm_up_status_subscription_cb_group_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr warm_up_status_subscription_;
    void warm_up_status_subscription_cb(const std_msgs::msg::Bool::SharedPtr status);

    rclcpp::CallbackGroup::SharedPtr cmd_vel_subscription_cb_group_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    void cmd_vel_subscription_cb(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);

    rclcpp::CallbackGroup::SharedPtr cmd_vel_nav_subscription_cb_group_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_nav_subscription_;
    void cmd_vel_nav_subscription_cb(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_nav);

    rclcpp::CallbackGroup::SharedPtr navigate_to_pose_status_subscription_cb_group_;
    rclcpp::Subscription<actionlib_msgs::msg::GoalStatusArray>::SharedPtr navigate_to_pose_status_subscription_;
    void navigate_to_pose_status_subscription_cb(const actionlib_msgs::msg::GoalStatusArray::SharedPtr navigate_to_pose_status);

    void log_big_line() const;
    void log_small_line() const;
    void log_topic_nullptr(const std::string &topic) const;
    void log_topic_data();
    void log_node_list() const;

public:
    explicit LoggerService(const rclcpp::Node::SharedPtr &node, const Parameter::SharedPtr &parameter);
    virtual ~LoggerService();

public:
    using SharedPtr = std::shared_ptr<LoggerService>;

};

#endif