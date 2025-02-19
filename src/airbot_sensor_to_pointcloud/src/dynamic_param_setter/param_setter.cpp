#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <string>

class ParamSetterNode : public rclcpp::Node {
public:
    ParamSetterNode() : Node("param_setter") {
        parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "/airbot_sensor_to_pointcloud");

        while (!parameters_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for parameter server...");
        }

        subscription_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/soc_cmd", 10,
            std::bind(&ParamSetterNode::socCmdCallback, this, std::placeholders::_1));        
        RCLCPP_INFO(this->get_logger(), "[param_setter] Node init finished!");
    }

private:
    void socCmdCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
        std::string frame_id;

        if (msg->data == 1) { // Auto Mapping
            frame_id = "base_link";
        } else if (msg->data == 3) { // Navigation
            frame_id = "map";
        } else {
            frame_id = "map";
        }

        if (frame_id.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty frame_id. Ignoring.");
            return;
        }

        parameters_client_->set_parameters({rclcpp::Parameter("target_frame", frame_id)});
    }

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_;
    std::shared_ptr<rclcpp::SyncParametersClient> parameters_client_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParamSetterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}