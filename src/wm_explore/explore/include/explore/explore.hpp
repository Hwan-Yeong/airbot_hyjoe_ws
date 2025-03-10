//
// Created by wavem on 25. 2. 25.
//

#pragma once

#include "explore/costmap_client.hpp"
#include "explore/frontier_search.hpp"
#include "explore/lifecycle_node_checker.hpp"
#include "explore/warm_up_client.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_util/lifecycle_node.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "explore_msgs/action/warmup.hpp"

#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>

#define ACTION_NAME "navigate_to_pose"

namespace explore {
    enum class ExplorationState {
        IDLE, // 탐색이 시작되지 않음
        SEND_GOAL,
        MOVING_TO_GOAL, // 이동 명령 결과 대기 중
        GOAL_REACHED, // 목표 도달 성공
        FAILED_TO_REACH, // 목표 도달 실패
        COMPLETE // 탐색 완료
    };

    static const std::map<ExplorationState, std::string> ExplorationStateStr = {
        std::pair<ExplorationState, std::string>(ExplorationState::IDLE, "IDLE"),
        std::pair<ExplorationState, std::string>(ExplorationState::SEND_GOAL, "SEND_GOAL"),
        std::pair<ExplorationState, std::string>(ExplorationState::MOVING_TO_GOAL, "MOVING_TO_GOAL"),
        std::pair<ExplorationState, std::string>(ExplorationState::GOAL_REACHED, "GOAL_REACHED"),
        std::pair<ExplorationState, std::string>(ExplorationState::FAILED_TO_REACH, "FAILED_TO_REACH"),
        std::pair<ExplorationState, std::string>(ExplorationState::COMPLETE, "COMPLETE")
    };

    class Explore : public rclcpp::Node {
    public:
        Explore();

        ~Explore();

        void Init();

        using NavAction = nav2_msgs::action::NavigateToPose;
        using NavGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
        using NavCancelResponse = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::CancelResponse;

    private:
        void MakePlan();

        void VisualizeFrontiers(const std::vector<Frontier> &frontiers);

        void RemoveGoalFromAbortedList(const geometry_msgs::msg::Point &goal);

        bool GoalOnAbortedList(const geometry_msgs::msg::Point &goal);
        bool GoalOnBlacklist(const geometry_msgs::msg::Point &goal);

        void NavFeedbackCallback(NavGoalHandle::SharedPtr, const std::shared_ptr<const NavAction::Feedback> feedback);
        void ReachedGoal(const NavGoalHandle::WrappedResult &result,
                         const geometry_msgs::msg::Point &frontier_goal);

        void UpdateExplorationState(const ExplorationState &state);

        geometry_msgs::msg::Point GetNearestPoint(
            const std::vector<geometry_msgs::msg::Point> &points,
            const geometry_msgs::msg::Pose &current_pose
        ) const;

        // publishers & subscribers
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_marker_publisher_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr explore_complete_publisher_;

        // action client
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;

        // tf2_ros objects
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // explore objects
        std::shared_ptr<FrontierSearch> frontier_search_;
        std::shared_ptr<Costmap2DClient> costmap_client_;
        rclcpp::TimerBase::SharedPtr exploring_timer_;
        rclcpp::CallbackGroup::SharedPtr cbg_exploring_timer_;

        // for warming up objects
        std::shared_ptr<WarmUpClient> warm_up_client_;
        std::shared_ptr<LifecycleNodeChecker> node_checker_;

        // cache datas
        ExplorationState state_;
        std::vector<geometry_msgs::msg::Point> frontier_aborted_list_;
        std::vector<geometry_msgs::msg::Point> frontier_blacklist_;
        geometry_msgs::msg::Point current_goal_;
        rclcpp::Time last_progress_;
        double prev_distance_remaining_ = std::numeric_limits<double>::max();
        size_t last_markers_count_;
        int empty_frontier_count_;

        // parameters
        double planner_frequency_;
        double min_frontier_size_;
        double potential_scale_, gain_scale_;
        double progress_timeout_;
        bool visualize_;
    };
} // explore
