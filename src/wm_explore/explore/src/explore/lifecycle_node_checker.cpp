//
// Created by wavem on 25. 2. 23.
//

#include <lifecycle_msgs/srv/get_state.hpp>

#include "explore/lifecycle_node_checker.hpp"
#include "explore/logger.hpp"

namespace explore {
    LifecycleNodeChecker::LifecycleNodeChecker(const rclcpp::Node::WeakPtr &parent_node) : parent_node_(parent_node) {

    }

    bool LifecycleNodeChecker::is_node_activate(const std::string &node_name, const std::chrono::seconds &timeout) {
        auto node = parent_node_.lock();

        std::string service_name = "/" + node_name + "/get_state";
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client = node->create_client<lifecycle_msgs::srv::GetState>(service_name);

        // 서비스가 활성화될 때까지 대기
        if (!client->wait_for_service(timeout)) {
            RCLCPP_ERROR(node->get_logger(), "Service %s is not available.", service_name.c_str());
            return false;
        }

        // 요청 객체 생성 및 비동기 요청 전송
        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto future = client->async_send_request(request);

        // 응답이 올 때까지 대기
        auto status = future.wait_for(std::chrono::seconds(5));
        if (status == std::future_status::ready) {
            auto response = future.get();
            uint8_t state_id = response->current_state.id;

            RCL_LOG_INFO(node->get_logger(), "Service <%s> state code: %d", service_name.c_str(), state_id);

            return valid_states_.count(state_id) > 0;
        }

        return false;
    }

} // explore
