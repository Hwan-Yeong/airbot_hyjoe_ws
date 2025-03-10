//
// Created by wavem on 25. 2. 27.
//

#pragma once

#include "rclcpp/rclcpp.hpp"

#include "lifecycle_msgs/srv/get_state.hpp"

namespace explore {
    class LifecycleNodeChecker {
    public:
        LifecycleNodeChecker(const rclcpp::Node::SharedPtr &node, const std::vector<std::string> &node_list);

        bool IsAllNodesAvailable();
    private:
        void SendGetStateRequest(
            const rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr &client,
            const std::string &node_name
        );

    protected:
        rclcpp::Node::SharedPtr node_;

        std::vector<std::string> node_list_;
        std::map<std::string, bool> node_activities_;
        std::vector<rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> get_state_clients_;
        rclcpp::CallbackGroup::SharedPtr cbg_lifecycle_nodes_;

        const std::unordered_set<uint8_t> valid_states_ = {
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
            lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING,
            lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING
        };
    };
}
