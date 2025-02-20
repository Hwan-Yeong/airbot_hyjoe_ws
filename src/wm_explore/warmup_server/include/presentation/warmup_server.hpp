//
// Created by changun on 25. 2. 13.
//

#ifndef WARMUP_SERVER_WARMUP_SERVER_HPP
#define WARMUP_SERVER_WARMUP_SERVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "explore_msgs/action/warmup.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "domain/ros_define.hpp"


namespace explore {
    class WarmupServer : public rclcpp::Node {
        public :
            explicit WarmupServer();

        private :
            using WarmupAction = explore_msgs::action::Warmup;
            using WarmUpActionServer = rclcpp_action::ServerGoalHandle<WarmupAction>;

            rclcpp_action::Server<WarmupAction>::SharedPtr explore_warmup_server_;
            rclcpp::CallbackGroup::SharedPtr cbg_action_server_ ;
            rclcpp::CallbackGroup::SharedPtr cbg_topic_pub_cmdvel_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmdvel_;
            std::unique_ptr<RosConstants> ros_constants_;

            void ros_init();
            rclcpp_action::GoalResponse warmup_goal_handle(
                    const rclcpp_action::GoalUUID& uuid,
                    std::shared_ptr<const WarmupAction::Goal> goal);

            rclcpp_action::CancelResponse warmup_cancel_handle(
                    const std::shared_ptr<WarmUpActionServer> goal_handle);

            void warmup_server_accepted_handle(
                    const std::shared_ptr<WarmUpActionServer> goal_handle
                    );

            void warmup_server_execute(const std::shared_ptr<WarmUpActionServer> goal_handle);
    };
}


#endif //WARMUP_SERVER_WARMUP_SERVER_HPP
