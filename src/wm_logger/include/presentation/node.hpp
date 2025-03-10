#ifndef PRESENTATION_NODE_HPP
#define PRESENTATION_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include "domain/parameter.hpp"
#include "application/logger.hpp"

#define NODE_NAME "wm_logger"


class LoggerNode final : public rclcpp::Node
{
private:
    rclcpp::Node::SharedPtr node_;
    Parameter::SharedPtr parameter_;
    LoggerService::SharedPtr logger_service_;

    void declare_parameters();

public:
    explicit LoggerNode();
    virtual ~LoggerNode();

};  

#endif