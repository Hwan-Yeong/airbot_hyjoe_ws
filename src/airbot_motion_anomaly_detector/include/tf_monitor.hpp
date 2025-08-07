#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/msg/bool.hpp>

class TFMonitorNode : public rclcpp::Node {
public:
    TFMonitorNode();

private:
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tf_monitor_cmd_sub_;

    bool monitor_enabled_ = false;
    bool received_map_to_base = false;
    bool received_map_to_odom = false;
    bool received_odom_to_base = false;

    void cmdCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void checkTFs();
    void checkTF(const std::string &parent, const std::string &child);
};