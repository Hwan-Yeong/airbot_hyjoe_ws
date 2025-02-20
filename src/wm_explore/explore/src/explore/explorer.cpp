#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>

#include "explore/explore.hpp"

using std::placeholders::_1;

namespace explore {
    std::string enumToString(EXPLORE_STATE set) {
        static const std::map<EXPLORE_STATE, std::string> eMap = {
            {EXPLORE_STATE::NONE, "NONE"},
            {EXPLORE_STATE::NAVI, "NAVI"},
            {EXPLORE_STATE::REACHED, "REACHED"},
            {EXPLORE_STATE::REACHED, "NAVI_ERROR"},
            {EXPLORE_STATE::MAKE, "MAKE"},
            {EXPLORE_STATE::FINISH, "FINISH"}
        };

        auto it = eMap.find(set);
        if (it != eMap.end()) {
            return it->second;
        }
        return "Unknown";
    }

    Explore::Explore() : Node("explore") {
        // read params
        this->get_parameter_or<double>("potential_scale", potential_scale_, 1.0);
        this->get_parameter_or<double>("gain_scale", gain_scale_, 0.5);
        this->get_parameter_or<double>("min_frontier_size", min_frontier_size_, 0.5);
        this->get_parameter_or<double>("orientation_scale", orientation_scale_, 0.0);
        this->get_parameter_or<int>("progress_timeout", timeout_, 5);
        this->get_parameter_or<bool>("visualize", visualize_, true);

        RCLCPP_INFO(this->get_logger(), "potential_scale : %f ", potential_scale_);
        RCLCPP_INFO(this->get_logger(), "gain_scale : %f ", gain_scale_);
        RCLCPP_INFO(this->get_logger(), "min_frontier_size : %f ", min_frontier_size_);
        RCLCPP_INFO(this->get_logger(), "orientation_scale : %f ", orientation_scale_);
        RCLCPP_INFO(this->get_logger(), "timeout_ : %d ", timeout_);
        RCLCPP_INFO(this->get_logger(), "visualize : %d ", visualize_);

        progress_timeout_ = rclcpp::Duration(timeout_, 0.0);


        check_finish_ = 0;
        state = false;
        // init transform server
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // init frontier_exploration
        search_ = FrontierSearch(costmap_.getCostmap(),
                                 potential_scale_, gain_scale_,
                                 min_frontier_size_);

        RCLCPP_INFO(this->get_logger(), "potential_scale_: %f", potential_scale_);
        // timed callback to make plan
        unsigned int planner_period = 200; // in milli secs
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(planner_period),
            std::bind(&Explore::makePlan, this));

        // map subscribers
        // ros2 has no waitForMessage yet
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10, std::bind(&Explore::mapReceived, this, _1));

        costmap_updates_sub_ = this->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
            "map_updates", 10, std::bind(&Explore::mapUpdateReceived, this, _1));

        // visualization publisher
        marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("frontiers", 1);
        finish_pub_ = this->create_publisher<std_msgs::msg::Empty>("explore_finish", 1);
        // action client
        nav_to_pose_client_ = rclcpp_action::create_client<ClientT>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "navigate_to_pose");
        setState(EXPLORE_STATE::NONE);

        // warmup server service client
        explore_warmup_client_ = rclcpp_action::create_client<explore_msgs::action::Warmup>(this, "/explore/warm_up");

        debug_same_goal_cnt = 0; //debug.

        sendWarmupRequest();
    }

    void Explore::setState(EXPLORE_STATE set) {
        if (state_explore != set) {
            std::string strSet = enumToString(set);
            std::string strState = enumToString(state_explore);
            RCLCPP_INFO(this->get_logger(), "setState : %s -> %s", strState.c_str(), strSet.c_str());
        }
        state_explore = set;
    }

    EXPLORE_STATE Explore::getState() {
        return state_explore;
    }

    void Explore::mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "Got new map");
        costmap_.updateFullMap(msg, state);
    }

    void Explore::mapUpdateReceived(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Got new partial map");
        costmap_.updatePartialMap(msg);
    }

    void Explore::makePlan() {
        if (is_warm_up_completed_) {
            // RCLCPP_INFO(this->get_logger(), "Making Plan");

            // get current pose
            // geometry_msgs::msg::Pose curr_pose = getRobotPose();
            geometry_msgs::msg::TransformStamped map_to_base_link;
            try {
                map_to_base_link = tf_buffer_->lookupTransform(
                    "map", "base_link", tf2::TimePointZero);
                // RCLCPP_INFO(this->get_logger(), "Got pose");
            } catch (tf2::TransformException &ex) {
                // RCLCPP_INFO(this->get_logger(), "No pose");
            }

            geometry_msgs::msg::Pose pose;
            pose.position.x = map_to_base_link.transform.translation.x;
            pose.position.y = map_to_base_link.transform.translation.y;
            pose.position.z = map_to_base_link.transform.translation.z;

            frontiers = search_.searchFrom(pose.position);

            EXPLORE_STATE currentState = getState();

            if (currentState == EXPLORE_STATE::NAVI || currentState == EXPLORE_STATE::FINISH) {
#if 1 //debug
                std::string str = enumToString(currentState);
                RCLCPP_INFO(this->get_logger(), "currentState : %s ", str.c_str());
#endif
                return; // 도착할때 까지는 프론티어 뽑지 말아봐.
            } else {
                setState(EXPLORE_STATE::MAKE);
            }

            // RCLCPP_ERROR(this->get_logger(), "Frontiers size: %d", frontiers.size() );
            // stop system when frontiers are empty
            if (frontiers.empty()) {
                // stop();
                check_finish_++;
                std_msgs::msg::Empty msgs;
                if (check_finish_ > 2) {
                    if (!state) {
                        RCLCPP_INFO(this->get_logger(), "finish : frontiers.empty()");
                        finish_pub_->publish(msgs);
                        setState(EXPLORE_STATE::FINISH);
                    } else
                        check_finish_ = 0;
                    state = false;
                }
                return;
            }

            // publish
            if (visualize_) {
                visualizeFrontiers(frontiers);
            }

            // try to find non blacklisted position to go to
            auto frontier = std::find_if_not(frontiers.begin(), frontiers.end(),
                                             [this](const Frontier &f) {
                                                 return goalOnBlacklist(f.centroid);
                                             });
            if (frontier == frontiers.end()) {
                std_msgs::msg::Empty msgs;
                RCLCPP_INFO(this->get_logger(), "finish : All goals is blacklisted.");
                finish_pub_->publish(msgs);
                setState(EXPLORE_STATE::FINISH);

                return;
            }

            target_position = frontier->centroid;

            std::lock_guard<std::mutex> lck(mutex_);

            // time out if we are not making any progress
            bool same_goal = prev_goal_ == target_position;
            prev_goal_ = target_position;
            if (!same_goal || prev_distance_ > frontier->min_distance) {
                // we have different goal or we made some progress
                last_progress_ = this->now();
                prev_distance_ = frontier->min_distance;
            }
            // black list if we've made no progress for a long time
            if (this->now() - last_progress_ > progress_timeout_) {
                frontier_blacklist_.push_back(target_position);
                RCLCPP_INFO(this->get_logger(), "Adding current goal to black list");
                // makePlan();
                return;
            }

            // we don't need to do anything if we still pursuing the same goal
            if (same_goal) {
                debug_same_goal_cnt++;
                RCLCPP_INFO(this->get_logger(), "same goal cnt : %d", debug_same_goal_cnt);
                return;
            }
            debug_same_goal_cnt = 0;

            // send goal to nav2
                ClientT::Goal client_goal;
                client_goal.pose.pose.position = target_position;
                client_goal.pose.pose.orientation.w = 1.;
                client_goal.pose.header.frame_id = "map";
                client_goal.pose.header.stamp = this->now();

                auto send_goal_options = rclcpp_action::Client<ClientT>::SendGoalOptions();

                send_goal_options.goal_response_callback =
                        std::bind(&Explore::goalResponseCallback, this, std::placeholders::_1);

                send_goal_options.result_callback =
                        std::bind(&Explore::goalResultCallback, this, std::placeholders::_1);

                future_goal_handle_ =
                        nav_to_pose_client_->async_send_goal(client_goal, send_goal_options);
        }
    }

    void Explore::goalResponseCallback(
        // std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future)
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Goal was rejected by the server.");
            setState(EXPLORE_STATE::NAVI_ERROR);
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal was accepted by the server.");
            setState(EXPLORE_STATE::NAVI);
        }
    }

    /**
     * nav2 에 action 을 보내고 도착을 판단할려고 만든 함수
     */
    void Explore::goalResultCallback(
        const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult &result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
                setState(EXPLORE_STATE::REACHED);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_INFO(this->get_logger(), "Goal was aborted!");
                setState(EXPLORE_STATE::NAVI_ERROR);
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "Goal was canceled!");
                setState(EXPLORE_STATE::NAVI_ERROR);
                break;
            default:
                RCLCPP_INFO(this->get_logger(), "Unknown result code!");
                setState(EXPLORE_STATE::NAVI_ERROR);
                break;
        }
    }


    bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point &goal) {
        // RCLCPP_INFO(this->get_logger(), "goalOnBlacklist");
        nav2_costmap_2d::Costmap2D *costmap2d = costmap_.getCostmap();
        const double tolerace = 20 * costmap2d->getResolution(); // 1m

        // check if a goal is on the blacklist for goals that we're pursuing
        for (auto &frontier_goal: frontier_blacklist_) {
            double x_diff = fabs(goal.x - frontier_goal.x);
            double y_diff = fabs(goal.y - frontier_goal.y);

            if (x_diff < tolerace && y_diff < tolerace) {
                RCLCPP_INFO(this->get_logger(), "this goal is black list !");
                return true;
            }
        }
        return false;
    }

    void Explore::visualizeFrontiers(const std::vector<Frontier> &frontiers) {
        static size_t prev_marker_count = 0;

        visualization_msgs::msg::MarkerArray markers_msg;

        // recycle m object when adding to marker array
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = this->now();
        m.ns = "frontier";
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.1;
        m.scale.y = 0.1;
        m.scale.z = 0.1;
        m.color.a = 1.0;
        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 0.0;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(
            0, 1); // uniform distribution between 0 and 1

        size_t id = 0;
        // add centroid of wavefronts to marker array
        for (auto &wavefront: frontiers) {
            m.color.r = dis(gen);
            m.color.g = dis(gen);
            m.color.b = dis(gen);
            for (auto &pt: wavefront.points) {
                m.id = id++;
                m.pose.position.x = pt.x;
                m.pose.position.y = pt.y;
                m.pose.position.z = 0.0;
                markers_msg.markers.push_back(m);
            }
        }
        size_t curr_marker_count = markers_msg.markers.size();

        // delete prev. markers
        m.action = visualization_msgs::msg::Marker::DELETE;
        for (; id < prev_marker_count; ++id) {
            m.id = int(id);
            markers_msg.markers.push_back(m);
        }

        prev_marker_count = curr_marker_count;
        marker_array_publisher_->publish(markers_msg);
    }

    bool Explore::checkFrontier(geometry_msgs::msg::Point goal, std::vector<explore::Frontier> frontiers) {
        double th = 1.0;
        return true;
        for (auto frontier: frontiers) {
            for (auto point: frontier.points) {
                double dist = std::hypot(goal.x - point.x, goal.y - point.y);
                if (dist < 1.0)
                    return false;
            }
        }
        return true;
    }

    void Explore::sendWarmupRequest() {
        while (rclcpp::ok() && !explore_warmup_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_INFO(this->get_logger(), "%s():%d:Waiting for Warmup server...", __func__, __LINE__);
        }

        explore_msgs::action::Warmup::Goal goal_msg;
        goal_msg.req_angle = false;

        RCLCPP_INFO(this->get_logger(), "%s():%d:Send warm up request.", __func__, __LINE__);

        auto send_goal_options = rclcpp_action::Client<explore_msgs::action::Warmup>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&Explore::warmupResponseCallback, this,std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&Explore::warmupFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&Explore::warmupResultCallback, this, std::placeholders::_1);
        this->explore_warmup_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void Explore::warmupResponseCallback(
        const rclcpp_action::ClientGoalHandle<explore_msgs::action::Warmup>::SharedPtr &response) {
        if (!response) {
            RCLCPP_ERROR(this->get_logger(), "%s():%d:Warmup request was rejected by warmup server", __func__,
                         __LINE__);
        } else {
            RCLCPP_INFO(this->get_logger(), "%s():%d:Warmup request accepted by warmup server, waiting for result...",
                        __func__, __LINE__);
        }
    }

    void Explore::warmupFeedbackCallback(rclcpp_action::ClientGoalHandle<explore_msgs::action::Warmup>::SharedPtr,
                                         const std::shared_ptr<const explore_msgs::action::Warmup::Feedback> &
                                         feedback) {
        RCLCPP_INFO(this->get_logger(), "%s():%d:Warmup in progress. {sequence: %d}\n", __func__, __LINE__,
                    feedback->sequence);
    }

    void Explore::warmupResultCallback(
        const rclcpp_action::ClientGoalHandle<explore_msgs::action::Warmup>::WrappedResult &result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                is_warm_up_completed_ = true;
                RCLCPP_INFO(this->get_logger(), "%s():%d:Warmup request succeeded!", __func__, __LINE__);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "%s():%d:Warmup request aborted!", __func__, __LINE__);
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "%s():%d:Warmup request canceled!", __func__, __LINE__);
                break;
        }
        sleep(2);
    }
} // namespace explore
