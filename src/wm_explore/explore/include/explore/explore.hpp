#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "explore/frontier_search.hpp"
#include "explore/costmap.hpp"
#include <tf2_ros/buffer.h>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2_ros/transform_listener.h"

#include "visualization_msgs/msg/marker_array.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "explore_msgs/action/warmup.hpp"
#include "explore/lifecycle_node_checker.hpp"
#include "explore/logger.hpp"

#include <map>
#include <string>
#include <mutex>
#include <lifecycle_msgs/srv/detail/get_state__builder.hpp>

namespace explore {
    enum class EXPLORE_STATE {
        NONE, // 초기화
        NAVI, // 움직이고 있어요.
        REACHED, // 도착 했어요
        NAVI_ERROR, // 움직이라고 했는데 에러 났어요
        MAKE, // 다음 목적지를 만드는 중이에요
        FINISH, // 탐색을 끝냈어요.
    };

    std::string enumToString(EXPLORE_STATE set);

    class Explore : public rclcpp::Node {
    public:
        using ClientT = nav2_msgs::action::NavigateToPose;
        using ActionClient = rclcpp_action::Client<ClientT>;

        Explore();

        void init();

    private:
        void mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

        void mapUpdateReceived(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg);

        void goalResponseCallback(
            // std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future);
            rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future);

        void goalResultCallback(const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult &result);

        bool wait_for_bt_navigator_active();

        bool wait_for_velocity_smoother_active();

        geometry_msgs::msg::Pose get_translated_pose();

        void makePlan();

        void visualizeFrontiers(const std::vector<Frontier> &frontiers);

        bool goalOnBlacklist(const geometry_msgs::msg::Point &goal);

        bool checkFrontier(geometry_msgs::msg::Point goal, std::vector<explore::Frontier> frontiers);

        void setState(EXPLORE_STATE set);
        EXPLORE_STATE getState();

        geometry_msgs::msg::Pose getRobotPose() const;

        void sendWarmupRequest(bool use_angle);
        void warmupResponseCallback(const rclcpp_action::ClientGoalHandle<explore_msgs::action::Warmup>::SharedPtr &response);
        void warmupFeedbackCallback(
            rclcpp_action::ClientGoalHandle<explore_msgs::action::Warmup>::SharedPtr,
            const std::shared_ptr<const explore_msgs::action::Warmup::Feedback> &feedback
        );
        void warmupResultCallback(
            const rclcpp_action::ClientGoalHandle<explore_msgs::action::Warmup>::WrappedResult &result);

    private:
        // publishers and subscribers
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::ConstSharedPtr costmap_sub_;
        rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::ConstSharedPtr costmap_updates_sub_;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr finish_pub_;

        // action client for nav2
        ActionClient::SharedPtr nav_to_pose_client_;
        std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future_goal_handle_;

        // action client for warm up
        rclcpp_action::Client<explore_msgs::action::Warmup>::SharedPtr explore_warmup_client_;
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr bt_navigator_state_client;
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr velocity_smoother_state_client;

        // tf
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // custom classes
        Costmap costmap_;
        FrontierSearch search_;
        std::shared_ptr<LifecycleNodeChecker> lifecycle_checker_;

        //state
        EXPLORE_STATE state_explore;

        // params
        double potential_scale_, orientation_scale_, gain_scale_, min_frontier_size_;
        bool visualize_;
        int timeout_;
        int check_finish_;
        bool state;

        // flag for check warm up completed
        bool is_warm_up_completed_ = false;

        // places you should stop trying to explore
        std::vector<geometry_msgs::msg::Point> frontier_blacklist_;

        // remember prev goal
        geometry_msgs::msg::Point prev_goal_;
        double prev_distance_;
        rclcpp::Time last_progress_;
        rclcpp::Duration progress_timeout_{0, 0};

        std::mutex mutex_; // protects sending of actions to nav2

        int debug_same_goal_cnt; // same goal debug
        int warmup_cnt_ = 0;

        std::vector<explore::Frontier> frontiers;
        geometry_msgs::msg::Point target_position;

    protected:
        const std::unordered_set<uint8_t> valid_states_ = {
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
            lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING,
            lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING
        };
    };
} // namespace explore
