//
// Created by wavem on 25. 2. 27.
//

#include "explore/lifecycle_node_checker.hpp"
#include "explore/logger.hpp"

#include <iostream>
#include <ranges>

using namespace std::placeholders;

namespace explore {
    LifecycleNodeChecker::LifecycleNodeChecker(
        const rclcpp::Node::SharedPtr &node,
        const std::vector<std::string> &node_list
    ): node_(node), node_list_(node_list) {
        cbg_lifecycle_nodes_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        for (const std::string &node_name: node_list) {
            std::string service_name = node_name + "/get_state";

            auto client = node_->create_client<lifecycle_msgs::srv::GetState>(
                service_name,
                rmw_qos_profile_services_default,
                cbg_lifecycle_nodes_
            );

            get_state_clients_.push_back(client);
            node_activities_.insert(std::pair<std::string, bool>(node_name, false));

            if (client->wait_for_service()) {
                node_activities_[node_name] = true;
            }
            // SendGetStateRequest(client, node_name);
        }
    }

    void LifecycleNodeChecker::SendGetStateRequest(
        const rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr &client,
        const std::string &node_name
    ) {
        lifecycle_msgs::srv::GetState::Request::SharedPtr request_msg{};

        using ServiceResponseFuture = rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFutureWithRequest;
        auto response_received_callback = [this, node_name](ServiceResponseFuture future) {
            auto request_response_pair = future.get();

            lifecycle_msgs::srv::GetState::Response::SharedPtr response = request_response_pair.second;
            uint8_t state_id = response->current_state.id;

            bool is_activate = this->valid_states_.count(state_id);
            if (is_activate) {
                RCL_LOG_INFO(this->node_->get_logger(), "%s is ready!", node_name.c_str());
            } else {
                RCL_LOG_ERROR(this->node_->get_logger(), "%s is not ready...", node_name.c_str());
            }
            this->node_activities_[node_name] = is_activate;

            if (IsAllNodesAvailable()) {
                for (auto client : get_state_clients_) {
                    client.reset();
                }
            }
        };

        // wait service before async send request
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCL_LOG_ERROR(node_->get_logger(), "client interrupted while waiting for service to appear.");
                return;
            }
            RCL_LOG_INFO(node_->get_logger(), "Waiting for %s to appear", node_name.c_str());
        }
        client->async_send_request(request_msg, response_received_callback);
    }

    bool LifecycleNodeChecker::IsAllNodesAvailable() {
        // if a node is not activated, return false
        for (std::map<std::string, bool>::iterator it = node_activities_.begin(); it != node_activities_.end(); ++it) {
            if (!it->second) {
                return false;
            }
        }

        return true;
    }
}
