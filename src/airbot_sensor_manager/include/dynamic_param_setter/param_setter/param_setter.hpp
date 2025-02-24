#ifndef __PARAM_SETTER_NODE_HPP__
#define __PARAM_SETTER_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <string>
#include <vector>
#include <thread>

class ParamSetterNode : public rclcpp::Node {
public:
    ParamSetterNode();
    ~ParamSetterNode();

private:
    void socCmdCallback(const std_msgs::msg::UInt8::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_;
    std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client_;
};

#endif // __PARAM_SETTER_NODE_HPP__
