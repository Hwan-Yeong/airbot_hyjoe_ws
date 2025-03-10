//
// Created by wavem on 25. 3. 1.
//

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/client.hpp"

#include "explore_msgs/action/warmup.hpp"

#include "explore/frontier_search.hpp"

namespace explore {
    enum class WarmingUpState {
        IDLE,
        WARMING_UP,
        COMPLETED
    };

    class WarmUpClient {
    public:
        WarmUpClient(const rclcpp::Node::SharedPtr &node);
        ~WarmUpClient() = default;

        void StartWarmingUp();
        WarmingUpState GetWarmingUpState() const;

        void UpdateFrontiers(const std::vector<Frontier> &frontiers);

    private:
        using WarmUpAction = explore_msgs::action::Warmup;
        using WarmUpGoalHandle = rclcpp_action::ClientGoalHandle<explore_msgs::action::Warmup>;

        void SendWarmUpRequest(bool use_angle);

        void WarmUpResponseCallback(const WarmUpGoalHandle::SharedPtr &goal_handle);
        void WarmUpFeedbackCallback(WarmUpGoalHandle::SharedPtr, const std::shared_ptr<const WarmUpAction::Feedback> feedback);
        void WarmUpResultCallback(const WarmUpGoalHandle::WrappedResult &result);

        rclcpp::Node::SharedPtr node_;

        rclcpp_action::Client<explore_msgs::action::Warmup>::SharedPtr client_;
        rclcpp::CallbackGroup::SharedPtr cbg_warm_up_;

        std::vector<Frontier> frontiers_;

        int warm_up_count_;
        WarmingUpState state_;

        // parameters
        int max_warm_up_num_;
        int min_frontier_num_;
        bool use_angle_;
    };

}

