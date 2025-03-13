#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <string>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("param_manager_node");

    // 노드 핸들러를 통해 파라미터 설정
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/airbot_sensor_interface_node");

    // 서비스가 활성화될 때까지 대기
    while (!parameters_client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_WARN(node->get_logger(), "Waiting for parameter server...");
    }

    static const unsigned int try_num = 5;
    for (int i = 0; i < try_num; i++) {
        std::string frame_id;
        std::cout << "Write the frame_id which you want to change ('map' or 'base_link'): ";
        std::cin >> frame_id;

        if (frame_id.empty()) {
            RCLCPP_WARN(node->get_logger(), "Invalid input. Please enter a valid frame_id.");
            continue;
        }

        parameters_client->set_parameters({rclcpp::Parameter("target_frame", frame_id)});

        auto param_value = parameters_client->get_parameters({"target_frame"})[0].as_string();

        if (param_value == frame_id) {
            RCLCPP_INFO(node->get_logger(), "Set parameter successful: target_frame = %s", param_value.c_str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to set parameter, current value: %s", param_value.c_str());
        }
    }

    rclcpp::shutdown();
    return 0;
}
