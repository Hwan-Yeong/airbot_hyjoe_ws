#include "param_manager/param_manager_node.hpp"

ParamManagerNode::ParamManagerNode() : Node("param_manager_node") {
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/airbot_sensor_interface_node");

    while (!parameters_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_WARN(this->get_logger(), "Waiting for parameter server... please check 'airbot_sensor_interface_node' node is alive");
    }

    subscription_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/soc_cmd", 10,
        std::bind(&ParamManagerNode::socCmdCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "[param_manager_node] Node init finished!");
}

ParamManagerNode::~ParamManagerNode()
{
    parameters_client_.reset();
}

void ParamManagerNode::socCmdCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
    std::string frame_id;

    if (msg->data == 1) { // Auto Mapping
        frame_id = "map";
    } else if (msg->data == 3) { // Navigation
        frame_id = "map";
    } else {
        return;
    }

    if (frame_id.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty frame_id. Ignoring.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Setting parameter 'target_frame' to: %s", frame_id.c_str());

    auto future = parameters_client_->set_parameters(
        {rclcpp::Parameter("target_frame", frame_id)}
    );

    std::thread([future]() mutable {
        try {
            future.get();
            RCLCPP_INFO(rclcpp::get_logger("param_manager_node"), "Successfully set parameter!");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("param_manager_node"), "Exception in set_parameters(): %s", e.what());
        }
    }).detach();
}
