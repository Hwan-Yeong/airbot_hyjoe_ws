//
// Created by wavem on 25. 3. 1.
//

#include "explore/warm_up_client.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

#include "explore_msgs/action/warmup.hpp"
#include "explore/logger.hpp"

namespace explore {
    WarmUpClient::WarmUpClient(
        const rclcpp::Node::SharedPtr &node)
        : node_(node), state_(WarmingUpState::IDLE) {
        // create warm up server client
        cbg_warm_up_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        client_ = rclcpp_action::create_client<explore_msgs::action::Warmup>(node, "explore/warm_up", cbg_warm_up_);

        max_warm_up_num_ = 2;
        min_frontier_num_ = 2;
    }

    void WarmUpClient::StartWarmingUp() {
        // update warming up state
        state_ = WarmingUpState::WARMING_UP;

        SendWarmUpRequest(use_angle_);
    }


    void WarmUpClient::SendWarmUpRequest(bool use_angle) {
        // wait for warm up action server ready
        RCL_LOG_INFO(node_->get_logger(), "Waiting for warm up server ready...");
        client_->wait_for_action_server();
        RCL_LOG_INFO(node_->get_logger(), "Warm up server is ready. Sending warm up request!");

        // send warm up request
        WarmUpAction::Goal goal = WarmUpAction::Goal();
        goal.req_angle = use_angle;

        auto send_goal_option = rclcpp_action::Client<WarmUpAction>::SendGoalOptions();
        send_goal_option.goal_response_callback = std::bind(&WarmUpClient::WarmUpResponseCallback, this,
                                                            std::placeholders::_1);
        send_goal_option.feedback_callback = std::bind(&WarmUpClient::WarmUpFeedbackCallback, this,
                                                       std::placeholders::_1, std::placeholders::_2);
        send_goal_option.result_callback = std::bind(&WarmUpClient::WarmUpResultCallback, this, std::placeholders::_1);

        RCL_LOG_INFO(node_->get_logger(), "Sending warm up request");
        client_->async_send_goal(goal, send_goal_option);
    }

    void WarmUpClient::WarmUpResponseCallback(const WarmUpGoalHandle::SharedPtr &goal_handle) {
        if (!goal_handle) {
            RCL_LOG_ERROR(node_->get_logger(), "Warm up was rejected by server... Retry to sending warm up request!");
            SendWarmUpRequest(use_angle_);
        } else {
            RCL_LOG_INFO(node_->get_logger(), "Warm up accepted by server, waiting for result");
        }
    }

    void WarmUpClient::WarmUpFeedbackCallback(
        WarmUpGoalHandle::SharedPtr,
        const std::shared_ptr<const WarmUpAction::Feedback> feedback
    ) {
        RCL_LOG_INFO(node_->get_logger(), "Warm up sequence received: %d", feedback->sequence);
        state_ = WarmingUpState::WARMING_UP;
    }

    void WarmUpClient::WarmUpResultCallback(const WarmUpGoalHandle::WrappedResult &result) {
        if (state_ == WarmingUpState::COMPLETED) {
            RCL_LOG_WARN(node_->get_logger(), "Warm up already completed...");
            return;
        }

        switch (result.code) {
            case rclcpp_action::ResultCode::ABORTED:
                RCL_LOG_ERROR(node_->get_logger(), "Warm up was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCL_LOG_WARN(node_->get_logger(), "Warm up was canceled");
                break;
            case rclcpp_action::ResultCode::UNKNOWN:
                RCL_LOG_ERROR(node_->get_logger(), "Warm up was unknown");
                break;
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCL_LOG_INFO(node_->get_logger(), "Warm up was succeeded");
                // increase warm up count
                warm_up_count_++;
                break;
        }

        if (warm_up_count_ >= max_warm_up_num_) {
            sleep(1);
            state_ = WarmingUpState::COMPLETED;
            return;
        }

        if (frontiers_.size() < min_frontier_num_) {
            SendWarmUpRequest(use_angle_);
        } else {
            sleep(1);
            state_ = WarmingUpState::COMPLETED;
        }
    }

    void WarmUpClient::UpdateFrontiers(const std::vector<Frontier> &frontiers) {
        frontiers_ = frontiers;
    }

    WarmingUpState WarmUpClient::GetWarmingUpState() const {
        return state_;
    }
}
