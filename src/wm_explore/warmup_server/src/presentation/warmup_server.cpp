//
// Created by changun on 25. 2. 13.
//

#include "presentation/warmup_server.hpp"
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>

#include "application/search/nav_goal_trans.hpp"
#include "application/search/safe_area.hpp"
#include "domain/df_logger.hpp"
#include "presentation/visualization_warmup.hpp"

#define LOG_WARMUP_FUNC "warmup_goal_handle()"

explore::WarmupServer::WarmupServer() : Node(NODE_NAME), cur_robotpose_(std::make_shared<RobotPose>()), cur_result_(navigation::ResultCode::kUnknown), result_try_check_(0){
    ros_constants_  = std::make_unique<RosConstants>();
    ros_init();
}

void explore::WarmupServer::ros_init() {
  //  Action Server Warmup Setting
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

    cbg_action_client_nav_to_pose_ =  this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    nav_to_pose_action_client_  = rclcpp_action::create_client<NavToPose>(this,
                                                                         ros_constants_->action_name_nav_to_pose_,
                                                                         cbg_action_client_nav_to_pose_);

    //  Topic Cmd_vel Setting
    cbg_topic_pub_cmdvel_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::PublisherOptions options_pub_cmdvel;
    options_pub_cmdvel.callback_group = cbg_topic_pub_cmdvel_;

    pub_cmdvel_ = this->create_publisher<geometry_msgs::msg::Twist>(
            ros_constants_->topic_name_cmdvel_,
            rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
            options_pub_cmdvel
            );
    // Topic Scan Setting
    cbg_topic_sub_scan_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options_sub_scan;
    options_sub_scan.callback_group = cbg_topic_sub_scan_;

    sub_scan_ = this->create_subscription<Lidar>(
        ros_constants_->topic_name_scan_,
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        std::bind(&explore::WarmupServer::scan_callback,
                                      this,
                                      std::placeholders::_1),
        options_sub_scan );

    cbg_topic_sub_robotpose_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options_sub_robotpose;
    options_sub_robotpose.callback_group = cbg_topic_sub_robotpose_;
    sub_robotpose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        ros_constants_->topic_name_robot_pose,
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        std::bind(&explore::WarmupServer::robotpose_callback,this,std::placeholders::_1),
        options_sub_robotpose
        );

    cbg_topic_pub_start_warmup_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::PublisherOptions options_pub_start_warmup;
    options_pub_start_warmup.callback_group = cbg_topic_pub_start_warmup_;
    pub_warmup_start_bool_ = this->create_publisher<std_msgs::msg::Bool>(
            "/warmup/start",
            rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
            options_pub_start_warmup);

    cbg_topic_pub_visuallization_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::PublisherOptions options_pub_visualization;
    options_pub_visualization.callback_group = cbg_topic_pub_visuallization_;
    pub_markers_ = this->create_publisher<visualization_msgs::msg::Marker>("Warmup/marker",rclcpp::QoS(rclcpp::SystemDefaultsQoS()),options_pub_visualization);

    // ==
    cbg_topic_pub_visuallization_list_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::PublisherOptions options_pub_visuallization_list;
    options_pub_visuallization_list.callback_group = cbg_topic_pub_visuallization_list_;
    pub_markers_list_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("Warmup/visualization_list",rclcpp::QoS(rclcpp::SystemDefaultsQoS()),options_pub_visuallization_list);
}

rclcpp_action::GoalResponse explore::WarmupServer::warmup_goal_handle(const rclcpp_action::GoalUUID &uuid,
                                                                      std::shared_ptr<const WarmupAction::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),"%s:%d:%s %s",LOG_WARMUP_FUNC,__LINE__,"start ",goal->req_angle ? "true":"false");
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
    result_try_check_ = 0;

    auto result = std::make_shared<WarmupAction::Result>();
    //sleep(5);
    rclcpp::Time start_clock =  this->now();

    search_area_ = std::make_shared<search::SafeArea>();


  if (goal_handle->is_canceling()) {
    result->status_code =
        static_cast<int>(explore::CodeWarmupStatus::kCancel);
    result->success = static_cast<bool>(explore::CodeWarmupResult::kFail);
    return;
  }
    auto start_time = this->now();
    while (rclcpp::ok()) {
        if (cur_scan_.get() == nullptr) {
            RCLCPP_INFO(this->get_logger(), "cur_scan_ is nullptr, waiting...");
        } else {
            RCLCPP_INFO(this->get_logger(), "cur_scan_ is set!");
            break;
        }
        // 5초 초과 시 루프 종료
        if ((this->now() - start_time) >= rclcpp::Duration::from_seconds(5)) {
            RCLCPP_ERROR(this->get_logger(), "Timeout reached! Exiting loop.");
            result->status_code =
            static_cast<int>(explore::CodeWarmupStatus::kSensorFail);
            result->success = static_cast<bool>(explore::CodeWarmupResult::kFail);
            return;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(500));  // 0.5초 대기 후 재시도
    }

   RCLCPP_INFO(this->get_logger(), "Warmup server start!");


  entity::Vertex map_origin;
  {
    std::lock_guard<std::mutex> lock(mtx_lidar_);
    map_origin = search_area_->search_goal(cur_scan_);
        explore::VisualizationWarmup visualization_warmup;
        explore::Color color = explore::Color::kBlue;
        visualization_msgs::msg::MarkerArray vma =visualization_warmup.visualize_list(search_area_->get_obs_area_vector(),"visualize_goal_warmup_list",color);
        pub_markers_list_->publish(vma);

  }
    sleep(1);

  bt_navigator_action_goal(map_origin.GetX(),map_origin.GetY());

    /*
    if(rclcpp::ok()) {
        if ()
    }
    */


    auto nav_start_time = std::chrono::steady_clock::now();
    int goal_respons_lp = 0;
    while (rclcpp::ok()) {
        sequence = goal_respons_lp++;
        goal_handle->publish_feedback(feedback);
        auto elapsed_time = std::chrono::steady_clock::now() - nav_start_time;
        if (elapsed_time > std::chrono::seconds(30)) {
            RCL_LOG_WARN(get_logger(), "Timeout reached, goal failed");
            result->status_code = false;
            result->success = false;
            goal_handle->succeed(result);
            cur_result_ = navigation::ResultCode::kUnknown;
            break; // 루프 종료
        }

        if (cur_result_ != navigation::ResultCode::kUnknown) {
            if (cur_result_ == navigation::ResultCode::kAborted) {
                result->status_code = static_cast<int>(CodeWarmupStatus::kMoveFail);
                result->success = false;
            }
            else if (cur_result_ == navigation::ResultCode::kSuccess) {
                result->status_code = static_cast<int>(CodeWarmupStatus::kSuccess);
                result->success = true;
            }
            else if (cur_result_ == navigation::ResultCode::kFailed) {
                result->status_code = static_cast<int>(CodeWarmupStatus::kMoveFail);
                result->success = false;
            }

            RCL_LOG_INFO(get_logger(), "Recive goal");
            goal_handle->succeed(result);
            cur_result_ = navigation::ResultCode::kUnknown;
            break; // 성공 시 루프 종료
        }
        sleep(1);
        //rclcpp::spin_some(this->get_node_base_interface());
    }
}


void explore::WarmupServer::scan_callback(const std::shared_ptr<Lidar> scan) {
    (void)scan;
    std::lock_guard<std::mutex> lock(mtx_lidar_);
    cur_scan_ = scan;
    //RCL_LOG_INFO(this->get_logger(),"scan callback");
}



void explore::WarmupServer::goal_response_callback(const GoalHandleNavToPose::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void explore::WarmupServer::result_callback(
    const rclcpp_action::ClientGoalHandle<
        explore::WarmupServer::NavToPose>::WrappedResult& result) {

    if (result_try_check_>5) {
        cur_result_ = navigation::ResultCode::kAborted;
        return;
    }
    entity::Vertex goal;
    switch(result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
            cur_result_ = navigation::ResultCode::kSuccess;
            return;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted!");
            // priority false
            search_area_->area_check_map_false_update();
            // obstacle 선정
            goal = search_area_->select_pose();
            if (goal.GetX() == 0 && goal.GetY() == 0) {
                cur_result_ = navigation::ResultCode::kAborted;
                return ;
            }
            bt_navigator_action_goal(goal.GetX(),goal.GetY());
            // goal 송신
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal was canceled!");
            cur_result_ = navigation::ResultCode::kCancelled;
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code: %d", static_cast<int>(result.code));
            cur_result_ = navigation::ResultCode::kFailed;
            break;
    }
}
void explore::WarmupServer::bt_navigator_action_goal(float map_x, float map_y) {
  while (!nav_to_pose_action_client_->wait_for_action_server(
      std::chrono::seconds(300))) {
    if (!rclcpp::ok()) {
      RCL_LOG_INFO(get_logger(),
                   "Interrupted while waiting for the automapping bt_navigator "
                   "action server...");
      return;
    }
    RCLCPP_INFO(
        get_logger(),
        "Automapping bt_navigator action server not found. Waiting again...");
  }
  auto bt_navigator_send_goal_options =
      rclcpp_action::Client<NavToPose>::SendGoalOptions();

  bt_navigator_send_goal_options.goal_response_callback =
      std::bind(&explore::WarmupServer::goal_response_callback, this,
                std::placeholders::_1);

  bt_navigator_send_goal_options.result_callback = std::bind(
      &explore::WarmupServer::result_callback, this, std::placeholders::_1);

  NavToPose::Goal goal_msg = NavToPose::Goal();
  search::NavGoalTrans nav_goal_trans;
  nav_goal_trans.set_origin_map_pose(map_x,map_y);
  geometry_msgs::msg::PoseStamped temp_goal_pose;
  temp_goal_pose.header.frame_id = "map";
  temp_goal_pose.header.stamp = this->now();
  temp_goal_pose.pose = nav_goal_trans.get_goal_pose(cur_robotpose_->pose.pose);

  goal_msg.pose = temp_goal_pose;
  std_msgs::msg::Bool warmup_start;
  warmup_start.data = true;
  pub_warmup_start_bool_->publish(warmup_start);
    explore::VisualizationWarmup visualization_warmup;
    visualization_msgs::msg::Marker warm_visualization =
        visualization_warmup.visualize_goal(
            goal_msg.pose.pose.position.x,
            goal_msg.pose.pose.position.y,
            "warmup_goal/visual",
            explore::Color::kYellow,
            0);

    pub_markers_->publish(warm_visualization);

  nav_to_pose_action_client_->async_send_goal(goal_msg,
                                              bt_navigator_send_goal_options);

}

void explore::WarmupServer::robotpose_callback(
    const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> rp) {
    cur_robotpose_ = rp;
}

explore::WarmupServer::~WarmupServer() = default;
