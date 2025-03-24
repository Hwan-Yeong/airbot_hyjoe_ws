// 2025-02-10 clabil v1.3
#include "localization_service/localization_service.hpp"

#define PRINT_INFO false

geometry_msgs::msg::Twist getTwistDefault()
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return cmd_vel;
}

LocalizationService::LocalizationService() : Node("A1_localization")
{
    this->declare_parameter("reqeust_time_ms", 100);
    this->declare_parameter("min_particle", 1000);
    this->declare_parameter("print_log", false);
    this->declare_parameter("fail_check_sec", 10.0);
    this->declare_parameter("particle_check_sec", 2.0);

    this->reqeust_time_ms_ = this->get_parameter("reqeust_time_ms").as_int();
    this->min_particle_ = this->get_parameter("min_particle").as_int();
    this->print_log_ = this->get_parameter("print_log").as_bool();

    this->get_parameter("fail_check_sec", fail_check_sec_);
    this->get_parameter("particle_check_sec", particle_check_sec_);

    // Services
    // nomotion_update 클라이언트 생성
    this->initpose_srv_ = this->create_client<std_srvs::srv::Empty>("request_nomotion_update");

    // global_costmap/clear_entirely_global_costmap 클라이언트 생성
    this->clear_costmap_global_srv_ =
        this->create_client<nav2_msgs::srv::ClearEntireCostmap>("global_costmap/clear_entirely_global_costmap");

    // local_costmap/clear_entirely_local_costmap 클라이언트 생성
    this->clear_costmap_local_srv_ =
        this->create_client<nav2_msgs::srv::ClearEntireCostmap>("local_costmap/clear_entirely_local_costmap");

    // Subscribers
    this->init_subscribers();

    // Publishers
    // initpose 발행, (UDP_Interface)
    this->initialpose_publisher_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);
    // 로컬라이제이션 완료, (UDP_Interface)
    // this->localization_comp_publisher_ = this->create_publisher<std_msgs::msg::Empty>("/localization/complete", 1);
    this->localization_comp_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/localization/complete", 1);
    // cmd , (MCU)
    this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    if (this->print_log_)
    {
        RCLCPP_INFO(this->get_logger(), "A1 Localization service start!");
    }
}

void LocalizationService::init_subscribers(void)
{
    // Ros2 에서 QoS 설정을 하지 않으면 데이터가 안넘어오는 경우가 생김
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.best_effort();

    // initpose 구독, (Rviz2)
    this->subscribers["init_pose"] = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", qos_profile, std::bind(&LocalizationService::initpose_callback, this, std::placeholders::_1));

    // 목적지 지정 (Rviz2)
    this->subscribers["goal_pose"] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", qos_profile, std::bind(&LocalizationService::goalpose_callback, this, std::placeholders::_1));

    // 파티클 개수 확인 구독(Nav2 Nomotion)
    this->subscribers["particle_cloud"] = this->create_subscription<nav2_msgs::msg::ParticleCloud>(
        "/particle_cloud",
        qos_profile,
        std::bind(&LocalizationService::particlecloud_callback, this, std::placeholders::_1));

    // Costmap clear 명령 구독, (UDP_Interface)
    this->subscribers["costmap_clear_request"] = this->create_subscription<std_msgs::msg::Empty>(
        "/localization/clear/costmap",
        qos_profile,
        std::bind(&LocalizationService::costmap_clear_callback, this, std::placeholders::_1));

    // 초기 위치 설정, (UDP_Interface)
    this->subscribers["nomotion_zero_request"] = this->create_subscription<std_msgs::msg::Empty>(
        "/localization/request",
        qos_profile,
        std::bind(&LocalizationService::nomotion_callback, this, std::placeholders::_1));

    // 초기 위치 설정, (UDP_Interface)
    this->subscribers["nomotion_pose_request"] =
        this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localization/request/pose",
            qos_profile,
            std::bind(&LocalizationService::nomotion_with_pose_callback, this, std::placeholders::_1));

    // 라이다 검출 확인
    this->subscribers["lidar_scan"] = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos_profile, std::bind(&LocalizationService::scan_callback, this, std::placeholders::_1));
}

void LocalizationService::initpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    (void)msg;
    if (this->print_log_)
    {
        RCLCPP_INFO(this->get_logger(), "Pose estimate start.");
    }
    this->initpose_req_ = true;
    this->initpose_end_ = false;
    this->initpose_start_time_ = this->now();
    this->particle_time_ = this->now();
}

void LocalizationService::particlecloud_callback(const nav2_msgs::msg::ParticleCloud::SharedPtr msg)
{
    if (this->initpose_req_)
    {
        size_t particles = msg->particles.size();
        this->particle_time_ = this->now();
        // RCLCPP_INFO(this->get_logger(), "Particle size -> %ld.", particles);
        if (particles <= this->min_particle_ + 1)
        {
            this->initpose_end_ = true;
        }
    }
}

void LocalizationService::nomotion_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
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

    for (size_t i = 0; i < 36; ++i)
    {
        if (i % 7 == 0)
            init_pose_msg.pose.covariance[i] = 1.0;
        else
            init_pose_msg.pose.covariance[i] = 0.0;
    }
    if (this->print_log_)
    {
        RCLCPP_INFO(this->get_logger(), "Station pose estimate start.");
    }
    initialpose_publisher_->publish(init_pose_msg);
}

void LocalizationService::nomotion_with_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
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

    for (size_t i = 0; i < 36; ++i)
    {
        init_pose_msg.pose.covariance[i] = msg->pose.covariance[i];
    }
    if (this->print_log_)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Localization with amcl_pose(x: %f,y: %f,z: %f) , "
            "orientation(x:%f,y:%f,z:%f,w:%f)",
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
    }
    this->initialpose_publisher_->publish(init_pose_msg);
}

void LocalizationService::costmap_clear_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg;
    call_clear_costmap_services();
}

void LocalizationService::goalpose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    (void)msg;
    call_clear_costmap_services();
}

void LocalizationService::call_clear_costmap_services(void)
{
    auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
    this->clear_costmap_global_srv_->async_send_request(request);
    this->clear_costmap_local_srv_->async_send_request(request);
    if (print_log_)
    {
        RCLCPP_INFO(this->get_logger(), "Costmap clear service called successfully.");
    }
}

void LocalizationService::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    (void)msg;
    this->lidar_check_time = this->now();
}

void LocalizationService::run()
{
    auto timer_ = this->create_wall_timer(
        std::chrono::milliseconds(this->reqeust_time_ms_), std::bind(&LocalizationService::timer_callback, this));
    rclcpp::spin(shared_from_this());
}

bool LocalizationService::checking_lidar(rclcpp::Time now)
{
    if (now - this->lidar_check_time > rclcpp::Duration::from_seconds(1.0))
    {
        if (this->print_log_)
        {
            RCLCPP_ERROR(this->get_logger(), "No lidar data. cannot use nomotion localization");
            return true;
        }
    }
    return false;
}
void LocalizationService::checking_particle(rclcpp::Time now)
{
    if (now - this->particle_time_ > rclcpp::Duration::from_seconds(this->particle_check_sec_))
    {
        if (this->print_log_)
        {
            RCLCPP_INFO(this->get_logger(), "Cannot use nomotion localization. move rotate.");
        }
        auto cmd_vel = getTwistDefault();
        if (!this->turn_left_)
        {
            this->turn_left_ = true;
            cmd_vel.angular.z = 0.1;
        }
        else
        {
            this->turn_left_ = false;
            cmd_vel.angular.z = -0.1;
        }

        this->cmd_vel_publisher_->publish(cmd_vel);
    }
}
void LocalizationService::timer_callback()
{
    if (this->initpose_req_)
    {
        auto now = this->now();
        // 라이다 안들어오면 동작 안함
        if (this->checking_lidar(now))
        {
            return;
        }
        // 파티클 클라우드가 안올라오면 회전 처리
        this->checking_particle(now);

        // 설정된 시간 이상 로컬이 걸리면 실패 처리
        if (now - this->initpose_start_time_ > rclcpp::Duration::from_seconds(this->fail_check_sec_))
        {
            if (this->initpose_req_ && !this->initpose_end_)
            {
                this->initpose_end_ = true;
                this->initpose_req_  = false;
                RCLCPP_WARN(this->get_logger(), "Fail to localization. time limit : %f sec.", this->fail_check_sec_);

                // std_msgs::msg::Empty empty_msg;
                // this->localization_comp_publisher_->publish(empty_msg);

                std_msgs::msg::Bool result_msg;
                result_msg.data = false;
                this->localization_comp_publisher_->publish(result_msg);
                return;
            }
        }

        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        // nomotion update 요청
        this->initpose_srv_->async_send_request(request);
        // 끝나면
        if (this->initpose_end_)
        {
            if (this->print_log_)
            {
                RCLCPP_INFO(this->get_logger(), "Pose estimate finished.");
            }

            auto cmd_vel = getTwistDefault();
            this->cmd_vel_publisher_->publish(cmd_vel);

            // 1초 대기
            std::this_thread::sleep_for(std::chrono::seconds(1));
            // 클리어 코스트맵
            // call_clear_costmap_services();
            this->initpose_req_ = false;

            // std_msgs::msg::Empty empty_msg;
            // this->localization_comp_publisher_->publish(empty_msg);

            std_msgs::msg::Bool result_msg;
            result_msg.data = true;
            this->localization_comp_publisher_->publish(result_msg);
        }
    }
}