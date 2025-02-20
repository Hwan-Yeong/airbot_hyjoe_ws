// 2025-02-10 clabil v1.3
#include "localization_service/localization_service.hpp"

#define PRINT_INFO false

LocalizationService::LocalizationService() : Node("A1_localization") {
  this->declare_parameter("reqeust_time_ms", 100);
  this->declare_parameter("min_particle", 1000);
  this->declare_parameter("print_log", false);

  reqeust_time_ms_ = this->get_parameter("reqeust_time_ms").as_int();
  min_particle_ = this->get_parameter("min_particle").as_int();
  print_log_ = this->get_parameter("print_log").as_bool();

  // nomotion_update 클라이언트 생성
  initpose_srv_ =
      this->create_client<std_srvs::srv::Empty>("request_nomotion_update");

  // global_costmap/clear_entirely_global_costmap 클라이언트 생성
  clear_costmap_global_srv_ =
      this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
          "global_costmap/clear_entirely_global_costmap");

  // local_costmap/clear_entirely_local_costmap 클라이언트 생성
  clear_costmap_local_srv_ =
      this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
          "local_costmap/clear_entirely_local_costmap");

  // Ros2 에서 QoS 설정을 하지 않으면 데이터가 안넘어오는 경우가 생김
  rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
  qos_profile.best_effort();

  // initpose 구독, (Rviz2)
  initialpose_subscription_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", qos_profile,
          std::bind(&LocalizationService::initpose_callback, this,
                    std::placeholders::_1));

  // 목적지 지정 (Rviz2)
  goalpose_subscription_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/goal_pose", qos_profile,
          std::bind(&LocalizationService::goalpose_callback, this,
                    std::placeholders::_1));

  // 파티클 개수 확인 구독(Nav2 Nomotion)
  particlecloud_subscription_ =
      this->create_subscription<nav2_msgs::msg::ParticleCloud>(
          "/particle_cloud", qos_profile,
          std::bind(&LocalizationService::particlecloud_callback, this,
                    std::placeholders::_1));

  // initpose 발행, (UDP_Interface)
  initialpose_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", 1);

  // Costmap clear 명령 구독, (UDP_Interface)
  costmap_clear_req_subscription_ =
      this->create_subscription<std_msgs::msg::Empty>(
          "/localization/clear/costmap", qos_profile,
          std::bind(&LocalizationService::costmap_clear_callback, this,
                    std::placeholders::_1));

  // 초기 위치 설정, (UDP_Interface)
  nomotion_req_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
      "/localization/request", qos_profile,
      std::bind(&LocalizationService::nomotion_callback, this,
                std::placeholders::_1));

  // 초기 위치 설정, (UDP_Interface)
  nomotion_localization_req_subscription_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/localization/request/pose", qos_profile,
          std::bind(&LocalizationService::nomotion_localization_callback, this,
                    std::placeholders::_1));

  // 로컬라이제이션 완료, (UDP_Interface)
  localization_comp_publisher_ =
      this->create_publisher<std_msgs::msg::Empty>("/localization/complete", 1);

  if (print_log_) {
    RCLCPP_INFO(this->get_logger(), "%s():%d: A1 Localization service start!",
                __FUNCTION__, __LINE__);
  }
}

void LocalizationService::initpose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  (void)msg;
  if (print_log_) {
    RCLCPP_INFO(this->get_logger(), "%s():%d: Pose estimate start.",
                __FUNCTION__, __LINE__);
  }
  initpose_req_ = true;
  initpose_end_ = false;
}

void LocalizationService::particlecloud_callback(
    const nav2_msgs::msg::ParticleCloud::SharedPtr msg) {
  if (initpose_req_) {
    size_t particles = msg->particles.size();
    if (particles <= min_particle_ + 1) {
      initpose_end_ = true;
    }
  }
}

// 2025-01-02, clabil, 강성준 선임
void LocalizationService::nomotion_callback(
    const std_msgs::msg::Empty::SharedPtr msg) {
  (void)msg;
  geometry_msgs::msg::PoseWithCovarianceStamped init_pose_msg;
  init_pose_msg.header.stamp = this->get_clock()->now();
  init_pose_msg.header.frame_id = "map";

  init_pose_msg.pose.pose.position.x = 0;
  init_pose_msg.pose.pose.position.y = 0;
  init_pose_msg.pose.pose.position.z = 0;

  init_pose_msg.pose.pose.orientation.x = 0.0;
  init_pose_msg.pose.pose.orientation.y = 0.0;
  init_pose_msg.pose.pose.orientation.z = 0.0;
  init_pose_msg.pose.pose.orientation.w = 1.0;

  for (size_t i = 0; i < 36; ++i) {
    if (i % 7 == 0)
      init_pose_msg.pose.covariance[i] = 1.0;
    else
      init_pose_msg.pose.covariance[i] = 0.0;
  }
  if (print_log_) {
    RCLCPP_INFO(this->get_logger(), "%s():%d: Station pose estimate start.",
                __FUNCTION__, __LINE__);
  }
  initialpose_publisher_->publish(init_pose_msg);
}

void LocalizationService::nomotion_localization_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {

  geometry_msgs::msg::PoseWithCovarianceStamped init_pose_msg;
  init_pose_msg.header.stamp = this->get_clock()->now();
  init_pose_msg.header.frame_id = msg->header.frame_id;

  init_pose_msg.pose.pose.position.x = msg->pose.pose.position.x;
  init_pose_msg.pose.pose.position.y = msg->pose.pose.position.y;
  init_pose_msg.pose.pose.position.z = msg->pose.pose.position.z;

  init_pose_msg.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  init_pose_msg.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  init_pose_msg.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  init_pose_msg.pose.pose.orientation.w = msg->pose.pose.orientation.w;

  for (size_t i = 0; i < 36; ++i) {
    init_pose_msg.pose.covariance[i] = msg->pose.covariance[i];
  }

  initialpose_publisher_->publish(init_pose_msg);
  if (print_log_) {
    RCLCPP_INFO(this->get_logger(),
                "%s():%d: "
                "Localization with amcl_pose(x: %f,y: %f,z: %f) , "
                "orientation(x:%f,y:%f,z:%f,w:%f)",
                __FUNCTION__, __LINE__, msg->pose.pose.position.x,
                msg->pose.pose.position.y, msg->pose.pose.position.z,
                msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  }
}

void LocalizationService::costmap_clear_callback(
    const std_msgs::msg::Empty::SharedPtr msg) {
  (void)msg;
  call_clear_costmap_services();
}

void LocalizationService::goalpose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  (void)msg;
  call_clear_costmap_services();
}

void LocalizationService::call_clear_costmap_services(void) {
  auto request =
      std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
  clear_costmap_global_srv_->async_send_request(request);
  clear_costmap_local_srv_->async_send_request(request);
  if (print_log_) {
    RCLCPP_INFO(this->get_logger(),
                "%s():%d: Costmap clear service called successfully..",
                __FUNCTION__, __LINE__);
  }
}

void LocalizationService::run() {
  auto timer_ = this->create_wall_timer(
      std::chrono::milliseconds(reqeust_time_ms_),
      std::bind(&LocalizationService::timer_callback, this));
  rclcpp::spin(shared_from_this());
}

void LocalizationService::timer_callback() {
  if (initpose_req_) {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    // nomotion update 요청
    initpose_srv_->async_send_request(request);
    // 끝나면
    if (initpose_end_) {
      if (print_log_) {
        RCLCPP_INFO(this->get_logger(), "%s():%d: Pose estimate finished.",
                    __FUNCTION__, __LINE__);
      }
      // 1초 대기
      std::this_thread::sleep_for(std::chrono::seconds(1));
      // 클리어 코스트맵
      call_clear_costmap_services();
      initpose_req_ = false;

      std_msgs::msg::Empty empty_msg;
      localization_comp_publisher_->publish(empty_msg);
    }
  }
}