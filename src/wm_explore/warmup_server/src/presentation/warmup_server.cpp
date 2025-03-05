//
// Created by changun on 25. 2. 13.
//

#include "presentation/warmup_server.hpp"

#define LOG_WARMUP_FUNC "warmup_goal_handle()"

explore::WarmupServer::WarmupServer() : Node(NODE_NAME){
    ros_constants_  = std::make_unique<RosConstants>();
    ros_init();
}

void explore::WarmupServer::ros_init() {

    cbg_action_server_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

    explore_warmup_server_ = rclcpp_action::create_server<WarmupAction>(
                this,
                ros_constants_->action_name_warmup_,
                std::bind(&explore::WarmupServer::warmup_goal_handle,this,std::placeholders::_1,std::placeholders::_2),
                std::bind(&explore::WarmupServer::warmup_cancel_handle,this,std::placeholders::_1),
                std::bind(&explore::WarmupServer::warmup_server_accepted_handle,this,std::placeholders::_1),
                rcl_action_server_get_default_options(),cbg_action_server_
            );

    cbg_topic_pub_cmdvel_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::PublisherOptions options_pub_cmdvel;
    options_pub_cmdvel.callback_group = cbg_topic_pub_cmdvel_;

    pub_cmdvel_ = this->create_publisher<geometry_msgs::msg::Twist>(
            ros_constants_->topic_name_cmdvel_,
            rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
            options_pub_cmdvel
            );
}

rclcpp_action::GoalResponse explore::WarmupServer::warmup_goal_handle(const rclcpp_action::GoalUUID &uuid,
                                                                      std::shared_ptr<const WarmupAction::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),"%s:%d:%s",LOG_WARMUP_FUNC,__LINE__,"start");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
explore::WarmupServer::warmup_cancel_handle(const std::shared_ptr<WarmUpActionServer> goal_handle) {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void explore::WarmupServer::warmup_server_accepted_handle(const std::shared_ptr<WarmUpActionServer> goal_handle) {
    std::thread{std::bind(&explore::WarmupServer::warmup_server_execute, this, std::placeholders::_1), goal_handle}.detach();
}

void explore::WarmupServer::warmup_server_execute(const std::shared_ptr<WarmUpActionServer> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<WarmupAction::Feedback>();
    auto& sequence = feedback->sequence;
    auto result = std::make_shared<WarmupAction::Result>();
    sleep(5);
    rclcpp::Time start_clock =  this->now();
    while(rclcpp::ok()){
        if(goal_handle->is_canceling()){
            result->status_code = static_cast<int>(explore::CodeWarmupStatus::kCancel);
            result->success = static_cast<bool>(explore::CodeWarmupResult::kFail);
            return;
        }
        sequence++;
        goal_handle->publish_feedback(feedback);

        geometry_msgs::msg::Twist temp_twist;
        temp_twist.linear.x = 0.2;
        temp_twist.angular.z = 0.0;
        pub_cmdvel_->publish(temp_twist);
        auto duration = this->now() - start_clock;
        if(duration.seconds() > 4){
            temp_twist.linear.x = 0.0;
            temp_twist.angular.z = 0.0;
            pub_cmdvel_->publish(temp_twist);
            break;
        }

        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }


    if(rclcpp::ok()){
        result->status_code = true;
        result->success = true;
        goal_handle->succeed(result);
    }
}

