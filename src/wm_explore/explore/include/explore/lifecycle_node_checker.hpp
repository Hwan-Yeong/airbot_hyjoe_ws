//
// Created by wavem on 25. 2. 23.
//

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/state.hpp>

namespace explore {
    class LifecycleNodeChecker: std::enable_shared_from_this<LifecycleNodeChecker> {
    public:
        LifecycleNodeChecker() = default;
        LifecycleNodeChecker(const rclcpp::Node::WeakPtr& parent_node);

        bool is_node_activate(const std::string &node_name, const std::chrono::seconds &timeout);
    private:
        rclcpp::Node::WeakPtr parent_node_;

    protected:
        const std::unordered_set<uint8_t> valid_states_ = {
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
            lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING,
            lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING
        };
    };
} // explore
