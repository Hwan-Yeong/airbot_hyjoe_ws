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
#include "sensor_msgs/msg/laser_scan.hpp"
#include <mutex>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace explore {
    class WarmupServer : public rclcpp::Node {
        public :
            explicit WarmupServer();
            virtual ~WarmupServer();

        private :
            using WarmupAction = explore_msgs::action::Warmup;
            using WarmUpActionServer = rclcpp_action::ServerGoalHandle<WarmupAction>;
            using Lidar = sensor_msgs::msg::LaserScan;
            using NavToPose = nav2_msgs::action::NavigateToPose;
            using GoalHandleNavToPose = rclcpp_action::ClientGoalHandle<NavToPose>;
            using RobotPose = geometry_msgs::msg::PoseWithCovarianceStamped;

            rclcpp_action::Server<WarmupAction>::SharedPtr explore_warmup_server_;

            rclcpp::CallbackGroup::SharedPtr cbg_action_server_ ;
            rclcpp::CallbackGroup::SharedPtr cbg_action_client_nav_to_pose_;
            rclcpp::CallbackGroup::SharedPtr cbg_topic_pub_cmdvel_;
            rclcpp::CallbackGroup::SharedPtr cbg_topic_sub_robotpose_;
            rclcpp::CallbackGroup::SharedPtr cbg_topic_pub_start_warmup_;
            rclcpp::CallbackGroup::SharedPtr cbg_topic_pub_visuallization_;

            /**
             * @brief : Callback group for Lidar topic subscription.
             */
            rclcpp::CallbackGroup::SharedPtr cbg_topic_sub_scan_;

            //
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmdvel_;

            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_warmup_start_bool_;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_markers_;

            rclcpp_action::Client<NavToPose>::SharedPtr nav_to_pose_action_client_;
            rclcpp::Subscription<RobotPose>::SharedPtr sub_robotpose_;

            /**
             * @brief : ROS 2 subscriber for receiving Lidar data.
             */
            rclcpp::Subscription<Lidar>::SharedPtr sub_scan_;

            std::unique_ptr<RosConstants> ros_constants_;

            std::mutex mtx_lidar_;
            std::mutex mtx_robotpose_;
            std::shared_ptr<Lidar> cur_scan_;
            std::shared_ptr<RobotPose> cur_robotpose_;

            navigation::ResultCode cur_result_;

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
            void scan_callback(const std::shared_ptr<Lidar> scan);
            void robotpose_callback(const std::shared_ptr<RobotPose> rp);
            void goal_response_callback(const GoalHandleNavToPose::SharedPtr & goal_handle);
            void result_callback(const GoalHandleNavToPose::WrappedResult & result);
            void bt_navigator_action_goal(float map_x, float map_y);

            visualization_msgs::msg::Marker visualize_goal(float x, float y);

    };
}


#endif //WARMUP_SERVER_WARMUP_SERVER_HPP
