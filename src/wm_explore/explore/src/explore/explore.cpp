//
// Created by wavem on 25. 2. 25.
//

#include "explore/explore.hpp"
#include "explore/logger.hpp"

#include <algorithm>

inline static bool SamePoint(const geometry_msgs::msg::Point &one, const geometry_msgs::msg::Point &two) {
    double dx = one.x - two.x;
    double dy = one.y - two.y;
    double dist = sqrt(dx * dx + dy * dy);
    return dist < 0.1;
}

inline static bool SameRobotPose(const geometry_msgs::msg::Pose &one, const geometry_msgs::msg::Pose &two) {
    double dx = one.position.x - two.position.x;
    double dy = one.position.y - two.position.y;
    double dist = sqrt(dx * dx + dy * dy);
    return dist < 0.1;
}

namespace explore {
    Explore::Explore()
        : rclcpp::Node("explore_node"),
          tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),
           last_markers_count_(0), state_(ExplorationState::IDLE) {
        RCL_LOG_INFO(get_logger(), "explore node launched!");

        this->declare_parameter<float>("planner_frequency", 2.0);
        this->declare_parameter<float>("progress_timeout", 15.0);
        this->declare_parameter<bool>("visualize", true);
        this->declare_parameter<float>("potential_scale", 1.0);
        this->declare_parameter<float>("gain_scale", 0.0);
        this->declare_parameter<float>("min_frontier_size", 0.1);

        this->get_parameter("planner_frequency", planner_frequency_);
        this->get_parameter("progress_timeout", progress_timeout_);
        this->get_parameter("visualize", visualize_);
        this->get_parameter("potential_scale", potential_scale_);
        this->get_parameter("gain_scale", gain_scale_);
        this->get_parameter("min_frontier_size", min_frontier_size_);

        RCL_LOG_INFO(get_logger(), "planner_frequency: %f", planner_frequency_);
        RCL_LOG_INFO(get_logger(), "progress_timeout: %f", progress_timeout_);
        RCL_LOG_INFO(get_logger(), "visualize: %i", visualize_);
        RCL_LOG_INFO(get_logger(), "potential_scale: %f", potential_scale_);
        RCL_LOG_INFO(get_logger(), "gain_scale: %f", gain_scale_);
        RCL_LOG_INFO(get_logger(), "min_frontier_size: %f", min_frontier_size_);

        // initialize action clients
        RCL_LOG_INFO(get_logger(), "Waiting for navigation server ready...");
        nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, ACTION_NAME);
        nav_to_pose_client_->wait_for_action_server();

        // initialize publishers & subscribers
        explore_complete_publisher_ = this->create_publisher<std_msgs::msg::Empty>("explore_finish", 1);
        if (visualize_) {
            frontier_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("frontiers", 1);
        }
    }

    Explore::~Explore() {
    }

    void Explore::Init() {
        node_checker_ = std::make_shared<LifecycleNodeChecker>(
            this->shared_from_this(),
            std::vector<std::string>{"bt_navigator", "velocity_smoother"}
        );

        costmap_client_ = std::make_shared<Costmap2DClient>(this->shared_from_this(), &tf_buffer_);

        //
        warm_up_client_ = std::make_shared<WarmUpClient>(this->shared_from_this());

        // initialize frontier search
        frontier_search_ = std::make_shared<FrontierSearch>(
            costmap_client_->GetCostmap(),
            potential_scale_,
            gain_scale_,
            min_frontier_size_
        );

        // initialize timer
        cbg_exploring_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        exploring_timer_ = create_wall_timer(
            std::chrono::milliseconds((uint16_t) (1000.0 / planner_frequency_)),
            std::bind(&Explore::MakePlan, this),
            cbg_exploring_timer_
        );
    }


    void Explore::UpdateExplorationState(const ExplorationState &state) {
        if (state_ != state && state_ != ExplorationState::COMPLETE) {
            RCL_LOG_INFO(get_logger(), "Explore State: %s", ExplorationStateStr.at(state).c_str());
            state_ = state;
        }
    }

    geometry_msgs::msg::Point Explore::GetNearestPoint(
        const std::vector<geometry_msgs::msg::Point> &points,
        const geometry_msgs::msg::Pose &current_pose
    ) const {
        geometry_msgs::msg::Point result;
        if (points.empty()) {
            RCL_LOG_WARN(get_logger(), "Point list is empty. Returning default point.");
            return result;
        }

        double min_distance = std::numeric_limits<double>::infinity();
        double current_x = current_pose.position.x;
        double current_y = current_pose.position.y;

        for (const auto &point: points) {
            double distance = std::hypot(point.x - current_x, point.y - current_y);
            if (distance < min_distance) {
                min_distance = distance;
                result = point;
            }
        }

        return result;
    }


    void Explore::VisualizeFrontiers(const std::vector<Frontier> &frontiers) {
        std_msgs::msg::ColorRGBA blue;
        blue.r = 0;
        blue.g = 0;
        blue.b = 1.0;
        blue.a = 1.0;
        std_msgs::msg::ColorRGBA red;
        red.r = 1.0;
        red.g = 0;
        red.b = 0;
        red.a = 1.0;
        std_msgs::msg::ColorRGBA green;
        green.r = 0;
        green.g = 1.0;
        green.b = 0;
        green.a = 1.0;
        std_msgs::msg::ColorRGBA yellow;
        yellow.r = 1.0;
        yellow.g = 1.0;
        yellow.b = 0;
        yellow.a = 1.0;
        std_msgs::msg::ColorRGBA gray;
        gray.r = 0.5;
        gray.g = 0.5;
        gray.b = 0.5;
        gray.a = 1.0;

        visualization_msgs::msg::MarkerArray markers_msg;
        int id = 0;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = costmap_client_->GetGlobalFrameID();
        marker.header.stamp = this->now();
        marker.ns = "frontiers";
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.lifetime = rclcpp::Duration::from_seconds(0);
        marker.frame_locked = true;

        // frontier cells
        for (const auto &frontier : frontiers) {
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::POINTS;

            // scale
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            // color
            if (GoalOnBlacklist(frontier.centroid)) {
                marker.color = red;
            } else if (GoalOnAbortedList(frontier.centroid)) {
                marker.color = yellow;
            } else {
                marker.color = blue;
            }

            // add frontier cell points
            for (const auto &point : frontier.points) {
                marker.points.push_back(point);
            }

            markers_msg.markers.push_back(marker);
            id++;
        }

        // frontier centroids
        for (const auto &frontier : frontiers) {
            if (GoalOnBlacklist(frontier.centroid) || GoalOnAbortedList(frontier.centroid)) {
                continue;
            }

            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::SPHERE;

            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            marker.pose.position = frontier.centroid;

            marker.color = gray;

            markers_msg.markers.push_back(marker);
            id++;
        }

        // aborted list and black list
        for (const auto &point : frontier_blacklist_) {
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::SPHERE;

            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;

            marker.pose.position = point;
            marker.color = red;

            markers_msg.markers.push_back(marker);
            id++;
        }

        for (const auto &point : frontier_aborted_list_) {
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::SPHERE;

            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;

            marker.pose.position = point;
            marker.color = yellow;

            markers_msg.markers.push_back(marker);
            id++;
        }

        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;

        marker.scale.x = 0.35;
        marker.scale.y = 0.35;
        marker.scale.z = 0.35;

        marker.pose.position = current_goal_;
        marker.color = green;

        markers_msg.markers.push_back(marker);
        id++;

        size_t current_markers_count = markers_msg.markers.size();

        // delete previous markers, which are now unused
        for (; id < last_markers_count_; id++) {
            marker.action = visualization_msgs::msg::Marker::DELETE;
            marker.id = int(id);
            markers_msg.markers.push_back(marker);
        }

        last_markers_count_ = current_markers_count;
        frontier_marker_publisher_->publish(markers_msg);
    }

    void Explore::MakePlan() {
        if (state_ == ExplorationState::COMPLETE) {
            RCL_LOG_INFO(get_logger(), "Exploring completed!");
            return;
        }

        // check required nodes ready
        if (!node_checker_->IsAllNodesAvailable()) {
            RCL_LOG_INFO(this->get_logger(), "Waiting for all required node activated...");
            return;
        }

        // check costmap updated
        if (!costmap_client_->IsCostmapInitialized()) {
            RCL_LOG_INFO(this->get_logger(), "Waiting for costmap initialized...");
            return;
        }

        // search frontiers
        auto current_robot_pose = costmap_client_->GetTFRobotPose();
        std::vector<Frontier> frontiers = frontier_search_->SearchFrom(current_robot_pose.position);

        // publish frontiers for rviz
        if (visualize_) {
            VisualizeFrontiers(frontiers);
        }

        // update warm up client's frontier data
        warm_up_client_->UpdateFrontiers(frontiers);

        // if warm up not started, start warming up
        if (warm_up_client_->GetWarmingUpState() == WarmingUpState::IDLE) {
            RCL_LOG_INFO(this->get_logger(), "start warming up!");
            warm_up_client_->StartWarmingUp();
        }

        // check warming up completed
        if (warm_up_client_->GetWarmingUpState() != WarmingUpState::COMPLETED) {
            RCL_LOG_INFO(this->get_logger(), "Waiting for warming up completed...");
            return;
        }

        // remove frontier if in aborted list or black list
        frontiers.erase(
            std::remove_if(frontiers.begin(), frontiers.end(), [&](const Frontier &f) {
                return GoalOnBlacklist(f.centroid) || GoalOnAbortedList(f.centroid);
            }),
            frontiers.end()
        );

        // check frontier list is empty
        geometry_msgs::msg::Point target_position;
        if (frontiers.empty()) {
            // if frontier is not found and aborted list exist, set target position
            if (!frontier_aborted_list_.empty()) {
                target_position = GetNearestPoint(frontier_aborted_list_, current_robot_pose);
            } else {
                empty_frontier_count_++;
                RCL_LOG_INFO(this->get_logger(), "No frontier found: %d", empty_frontier_count_);

                // if frontier empty occurred higher 20, complete exploring
                if (empty_frontier_count_ >= 10) {
                    if (state_ != ExplorationState::COMPLETE) {
                        UpdateExplorationState(ExplorationState::COMPLETE);

                        std_msgs::msg::Empty msg = std_msgs::msg::Empty();
                        RCL_LOG_INFO(get_logger(), "publish explore_finish topic");
                        explore_complete_publisher_->publish(msg);
                    }
                }

                return;
            }
        } else {
            target_position = frontiers[0].centroid;
            empty_frontier_count_ = 0;
        }

        // if robot is not driving, send new goal
        if (state_ != ExplorationState::SEND_GOAL && state_ != ExplorationState::MOVING_TO_GOAL) {
            // update exploration state
            current_goal_ = target_position;
            UpdateExplorationState(ExplorationState::SEND_GOAL);

            // sending goal
            nav_to_pose_client_->async_cancel_all_goals(
                [this, target_position](const NavCancelResponse::SharedPtr &response) {
                    auto goal = nav2_msgs::action::NavigateToPose::Goal();
                    goal.pose.pose.position = target_position;
                    goal.pose.pose.orientation.w = 1.;
                    goal.pose.header.frame_id = costmap_client_->GetGlobalFrameID();
                    goal.pose.header.stamp = this->now();

                    RCL_LOG_INFO(get_logger(), "Send Goal x: %f, y: %f", goal.pose.pose.position.x,goal.pose.pose.position.y);
                    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
                    send_goal_options.feedback_callback = std::bind(&Explore::NavFeedbackCallback, this,std::placeholders::_1, std::placeholders::_2);
                    send_goal_options.result_callback = [this,target_position](const NavGoalHandle::WrappedResult &result) {
                        ReachedGoal(result, target_position);
                    };
                    nav_to_pose_client_->async_send_goal(goal, send_goal_options);

                    last_progress_ = this->now();
                });
        }
    }

    void Explore::RemoveGoalFromAbortedList(const geometry_msgs::msg::Point &goal) {
        frontier_aborted_list_.erase(
            std::remove_if(
                frontier_aborted_list_.begin(),
                frontier_aborted_list_.end(),
                [&](const geometry_msgs::msg::Point &aborted_pose) {
                    return SamePoint(aborted_pose, goal);
                }
            ),
            frontier_aborted_list_.end()
        );
    }


    bool Explore::GoalOnAbortedList(const geometry_msgs::msg::Point &goal) {
        constexpr static size_t tolerace = 5;
        nav2_costmap_2d::Costmap2D *costmap2d = costmap_client_->GetCostmap();
        double ref_value = tolerace * costmap2d->getResolution();

        // check if a goal is on the blacklist for goals that we're pursuing
        for (auto &frontier_goal: frontier_aborted_list_) {
            double x_diff = fabs(goal.x - frontier_goal.x);
            double y_diff = fabs(goal.y - frontier_goal.y);

            if (x_diff < tolerace * ref_value && y_diff < ref_value)
                return true;
        }

        return false;
    }

    bool Explore::GoalOnBlacklist(const geometry_msgs::msg::Point &goal) {
        constexpr static size_t tolerace = 5;
        nav2_costmap_2d::Costmap2D *costmap2d = costmap_client_->GetCostmap();
        double ref_value = tolerace * costmap2d->getResolution();

        // check if a goal is on the blacklist for goals that we're pursuing
        for (auto &frontier_goal: frontier_blacklist_) {
            double x_diff = fabs(goal.x - frontier_goal.x);
            double y_diff = fabs(goal.y - frontier_goal.y);

            if (x_diff < tolerace * ref_value && y_diff < ref_value)
                return true;
        }

        return false;
    }

    void Explore::NavFeedbackCallback(
        NavGoalHandle::SharedPtr,
        const std::shared_ptr<const NavAction::Feedback> feedback
    ) {
        UpdateExplorationState(ExplorationState::MOVING_TO_GOAL);
        double current_distance = feedback->distance_remaining;

        if (current_distance < prev_distance_remaining_) {
            last_progress_ = this->now(); // 마지막 진행 시간 업데이트
        }

        prev_distance_remaining_ = current_distance;

        // check timeout
        if ((this->now() - last_progress_).seconds() > progress_timeout_) {
            RCL_LOG_ERROR(get_logger(), "Robot is stuck... cancel current goal!");
            nav_to_pose_client_->async_cancel_all_goals();

            // if a goal is aborted twice, remove it from aborted list and add to black list
            if (GoalOnAbortedList(current_goal_)) {
                RCL_LOG_WARN(get_logger(), "Goal was canceled twice... add current goal to black list!");
                frontier_blacklist_.push_back(current_goal_);
                RemoveGoalFromAbortedList(current_goal_);
            } else {
                RCL_LOG_WARN(get_logger(), "Goal was canceled once... add current goal to aborted list!");
                frontier_aborted_list_.push_back(current_goal_);
            }
        }

    }


    void Explore::ReachedGoal(
        const NavGoalHandle::WrappedResult &result,
        const geometry_msgs::msg::Point &frontier_goal
    ) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED: {
                RCL_LOG_INFO(get_logger(), "Goal was successful");
                UpdateExplorationState(ExplorationState::GOAL_REACHED);

                // if reached aborted goal, remove it from aborted list
                RemoveGoalFromAbortedList(frontier_goal);
                frontier_blacklist_.push_back(frontier_goal);

                break;
            }
            case rclcpp_action::ResultCode::ABORTED: {
                UpdateExplorationState(ExplorationState::FAILED_TO_REACH);

                // if a goal is aborted twice, remove it from aborted list and add to black list
                if (GoalOnAbortedList(frontier_goal)) {
                    RCL_LOG_WARN(get_logger(), "Goal was aborted twice... add current goal to black list!");
                    frontier_blacklist_.push_back(frontier_goal);
                    RemoveGoalFromAbortedList(frontier_goal);
                } else {
                    RCL_LOG_WARN(get_logger(), "Goal was aborted once... add current goal to aborted list!");
                    frontier_aborted_list_.push_back(frontier_goal);
                }

                break;
            }
            case rclcpp_action::ResultCode::CANCELED: {
                RCL_LOG_INFO(get_logger(), "Goal was canceled!");
                UpdateExplorationState(ExplorationState::FAILED_TO_REACH);

                break;
            }
            case rclcpp_action::ResultCode::UNKNOWN: {
                RCL_LOG_WARN(get_logger(), "Unknown result code from nav2");
                UpdateExplorationState(ExplorationState::FAILED_TO_REACH);

                break;
            }
        }
    }
} // explore
