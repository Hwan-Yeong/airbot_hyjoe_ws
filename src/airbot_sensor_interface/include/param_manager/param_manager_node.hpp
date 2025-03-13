#ifndef __PARAM_SETTER_NODE_HPP__
#define __PARAM_SETTER_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <string>
#include <vector>
#include <thread>

/**
 * @brief: 런타임에 sensor_interface_node의 파라미터를 동적으로 변경하는 노드입니다.
 */
class ParamManagerNode : public rclcpp::Node {
public:
    ParamManagerNode();
    ~ParamManagerNode();

private:
    void socCmdCallback(const std_msgs::msg::UInt8::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_;
    std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client_;
};

#endif // __PARAM_SETTER_NODE_HPP__
