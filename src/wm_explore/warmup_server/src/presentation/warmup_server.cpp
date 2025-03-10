//
// Created by changun on 25. 2. 13.
//

#include "presentation/warmup_server.hpp"
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>

#include "application/search/nav_goal_trans.hpp"
#include "application/search/safe_area.hpp"
#include "domain/df_logger.hpp"

#define LOG_WARMUP_FUNC "warmup_goal_handle()"

explore::WarmupServer::WarmupServer() : Node(NODE_NAME), cur_robotpose_(std::make_shared<RobotPose>()), cur_result_(navigation::ResultCode::kUnknown){
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
    std::cout <<sequence <<'\n';
    auto result = std::make_shared<WarmupAction::Result>();
    //sleep(5);
    rclcpp::Time start_clock =  this->now();

    search::SafeArea search_area;


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


  //sequence++;
  //goal_handle->publish_feedback(feedback);
  entity::Vertex map_origin;
  {
    std::lock_guard<std::mutex> lock(mtx_lidar_);
    map_origin = search_area.search_goal(cur_scan_);
  }
    sleep(1);

  bt_navigator_action_goal(map_origin.GetX(),map_origin.GetY());

    /*
    if(rclcpp::ok()) {
        if ()
    }
    */


    auto nav_start_time = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
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
            RCL_LOG_INFO(get_logger(), "Success goal");
            result->status_code = true;
            result->success = true;
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
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
            cur_result_ = navigation::ResultCode::kSuccess;
        return;

        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted!");
            cur_result_ = navigation::ResultCode::kAborted;
        break;

        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal was canceled!");
            cur_result_ = navigation::ResultCode::kCancelled;
        break;

        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code: %d", static_cast<int>(result.code));
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
    visualization_msgs::msg::Marker warm_visualization = visualize_goal(goal_msg.pose.pose.position.x,goal_msg.pose.pose.position.y);

    pub_markers_->publish(warm_visualization);

  nav_to_pose_action_client_->async_send_goal(goal_msg,
                                              bt_navigator_send_goal_options);
}

visualization_msgs::msg::Marker explore::WarmupServer::visualize_goal(float x, float y) {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "visualize_goal_warmup";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.lifetime=rclcpp::Duration::from_seconds(10);
    marker.frame_locked = true;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 1.0;
    return marker;
}

void explore::WarmupServer::robotpose_callback(
    const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> rp) {
    cur_robotpose_ = rp;
}

explore::WarmupServer::~WarmupServer() = default;
