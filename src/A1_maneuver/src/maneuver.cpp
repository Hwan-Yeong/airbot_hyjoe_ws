#include "maneuver.hpp"

namespace A1::maneuver
{

/**
 * @brief Maneuver 생성자
 * Parameter 설정과 subscribes, publishers 설정을 진행한다.
 */
ManeuverNode::ManeuverNode() : Node("A1_maneuver")
{
    this->declare_parameter("time_ms.timer", 100);
    int timer_ms = this->get_parameter("time_ms.timer").as_int();

    // this->last_state_pub_time_ = this->now();
    this->last_state_pub_time_ = std::chrono::steady_clock::now();
    this->init_params();
    this->init_subs();
    this->current_velocity_ = 0;
    this->cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    this->force_escape_pub_ = this->create_publisher<std_msgs::msg::Bool>("/maneuver/request/escape", 1);

    this->state_str_pub_ = this->create_publisher<std_msgs::msg::String>("/maneuver/state/string", 1);
    this->state_int_pub_ = this->create_publisher<std_msgs::msg::Int8MultiArray>("/maneuver/state", 1);

    this->timer_ =
        this->create_wall_timer(std::chrono::milliseconds(timer_ms), std::bind(&ManeuverNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "[Maneuver] setting finished.");
}

/***
 * @brief 파라미터 설정 및 변수 초기화
 */
void ManeuverNode::init_params(void)
{
    this->main_state_ = MAIN_STATE::IDLE;
    this->navi_state_ = NAVI_STATE::IDLE;
    this->sub_state_ = SUB_STATE::IDLE;
    this->explore_state_ = MAPPING_STATE::PREPARATION_STATE;
    this->costmap_escape_sampling_ = 0;

    this->back_object_detect_ = false;
    this->front_escape_detect_ = false;

    this->declare_parameter("time_ms.wait.normal", 1000);
    this->declare_parameter("time_ms.wait.drop_off", 2000);
    this->declare_parameter("time_ms.back.1d", 1500);
    this->declare_parameter("time_ms.back.collision", 1500);

    this->declare_parameter("costmap.escape.value", 90);
    this->declare_parameter("costmap.escape.release_value", 70);
    this->declare_parameter("costmap.escape.angular_z_speed", 0.25);
    this->declare_parameter("costmap.escape.sampling_threshold", 30);
    this->declare_parameter("costmap.escape.candidate.threshold", 80);
    this->declare_parameter("costmap.escape.candidate.min_dist", 0.3);
    this->declare_parameter("costmap.escape.candidate.max_dist", 0.5);
    this->declare_parameter("costmap.escape.time_multiplier_for_dist", 10000);
    this->declare_parameter("costmap.no_plan.sampling_threshold", 30);
    this->declare_parameter("costmap.no_plan.release_degree", 30.0);
    this->declare_parameter("costmap.no_plan.global_plan_timeout_sec", 2.0);
    this->declare_parameter("costmap.no_plan.timeout_sec", 5.0);

    this->declare_parameter("lidar.escape.front.diagonal_distance", 0.25);
    this->declare_parameter("lidar.escape.front.min_distance", 0.25);
    this->declare_parameter("lidar.escape.front.max_distance", 0.35);
    this->declare_parameter("lidar.escape.front.angle", 60.0);
    this->declare_parameter("lidar.escape.front.max_vel", 0.40);
    this->declare_parameter("lidar.escape.front.min_vel", 0.01);

    this->declare_parameter("lidar.escape.back.distance", 0.3);
    this->declare_parameter("lidar.escape.back.angle", 45.0);

    this->declare_parameter("lidar.drop_ir.collision.front.distance", 0.24);
    this->declare_parameter("lidar.drop_ir.collision.front.angle", 75.0);
    this->declare_parameter("lidar.drop_ir.collision.back.distance", 0.24);
    this->declare_parameter("lidar.drop_ir.collision.back.angle", 75.0);

    this->declare_parameter("velocity_scaling_factor", 0.05);

    this->get_parameter("time_ms.wait.normal", this->wait_ms_normal_);
    this->get_parameter("time_ms.wait.drop_off", this->wait_ms_drop_off_);
    this->get_parameter("time_ms.back.1d", this->back_ms_1d_);
    this->get_parameter("time_ms.back.collision", this->back_ms_collision_);

    this->get_parameter("costmap.escape.value", this->costmap_escape_value_);
    this->get_parameter("costmap.escape.release_value", this->costmap_escape_release_value_);
    this->get_parameter("costmap.escape.sampling_threshold", this->costmap_escape_sampling_threshold_);
    this->get_parameter("costmap.escape.candidate.threshold", this->costmap_escape_candidate_threshold_);
    this->get_parameter("costmap.escape.candidate.min_dist", this->costmap_escape_candidate_min_dist_);
    this->get_parameter("costmap.escape.candidate.max_dist", this->costmap_escape_candidate_max_dist_);
    this->get_parameter("costmap.escape.time_multiplier_for_dist", this->costmap_escape_time_multiplier_for_dist);
    this->get_parameter("costmap.escape.angular_z_speed", this->costmap_escape_angular_z_speed);

    this->get_parameter("costmap.no_plan.sampling_threshold", this->no_plan_sampling_threshold_);
    this->get_parameter("costmap.no_plan.release_degree", this->no_plan_release_degree_);
    this->get_parameter("costmap.no_plan.global_plan_timeout_sec", this->no_plan_global_timeout_sec_);
    this->get_parameter("costmap.no_plan.timeout_sec", this->no_plan_timeout_sec_);

    this->get_parameter("lidar.escape.front.diagonal_distance", this->lidar_front_escape_diagonal_dist_);
    this->get_parameter("lidar.escape.front.min_distance", this->lidar_front_escape_min_dist_);
    this->get_parameter("lidar.escape.front.max_distance", this->lidar_front_escape_max_dist_);
    this->get_parameter("lidar.escape.front.angle", this->lidar_front_escape_angle_);
    this->get_parameter("lidar.escape.front.max_vel", this->max_vel_);
    this->get_parameter("lidar.escape.front.min_vel", this->min_vel_);

    this->get_parameter("lidar.escape.back.distance", this->lidar_back_escape_dist_);
    this->get_parameter("lidar.escape.back.angle", this->lidar_back_escape_angle_);

    this->get_parameter("lidar.drop_ir.collision.front.distance", this->drop_ir_lidar_collision_front_distance_);
    this->get_parameter("lidar.drop_ir.collision.front.angle", this->drop_ir_lidar_collision_front_angle_);
    this->get_parameter("lidar.drop_ir.collision.back.distance", this->drop_ir_lidar_collision_back_distance_);
    this->get_parameter("lidar.drop_ir.collision.back.angle", this->drop_ir_lidar_collision_back_angle_);

    this->get_parameter("velocity_scaling_factor", this->velocity_scaling_factor_);
}

/***
 * @brief subscribes 설정 함수
 */
void ManeuverNode::init_subs(void)
{
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.best_effort();
    rclcpp::QoS wm_qos_profile = rclcpp::QoS(1).reliable().transient_local();

    this->subscribers["explore_status"] = this->create_subscription<std_msgs::msg::UInt8>(
        "/explore_status",
        wm_qos_profile,
        [this](const std_msgs::msg::UInt8::SharedPtr msg)
        {
            if (!this->use_maneuver_action_)
                return;
            this->explore_state_ = static_cast<MAPPING_STATE>(msg->data);
        });

    this->subscribers["navi_state"] = this->create_subscription<robot_custom_msgs::msg::NaviState>(
        "/navi_datas", qos_profile, std::bind(&ManeuverNode::navi_state_callback, this, std::placeholders::_1));

    // [EB]Motor status
    this->subscribers["motor_status"] = this->create_subscription<robot_custom_msgs::msg::MotorStatus>(
        "/motor_status",
        qos_profile,
        [this](const robot_custom_msgs::msg::MotorStatus::SharedPtr msg)
        {
            this->motor_status_.setLeftRpm(msg->left_motor_rpm);
            this->motor_status_.setRightRpm(msg->right_motor_rpm);
            this->current_velocity_ = this->motor_status_.getVelocity();
        });

    // [EB]Drop off IR sensors
    this->subscribers["bottom_lr_data"] = this->create_subscription<robot_custom_msgs::msg::BottomIrData>(
        "/bottom_ir_data", qos_profile, std::bind(&ManeuverNode::bottom_ir_callback, this, std::placeholders::_1));

    // [CB]Perception action command
    this->subscribers["perception_action_stop"] = this->create_subscription<std_msgs::msg::Int8>(
        "/perception/action/stop", 10, std::bind(&ManeuverNode::action_stop_callback, this, std::placeholders::_1));

    // [CB]Perception climb state
    this->subscribers["perception_climb"] = this->create_subscription<std_msgs::msg::Bool>(
        "/perception/climb", 10, std::bind(&ManeuverNode::perception_climb_callback, this, std::placeholders::_1));

    // [EB]turn on/off maneuver action
    this->subscribers["maneuver_use"] = this->create_subscription<std_msgs::msg::Bool>(
        "/maneuver/use",
        qos_profile,
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            this->use_maneuver_action_ = msg->data;
            RCLCPP_INFO(this->get_logger(), "[Maneuver] %s , State init", msg->data ? "ON" : "OFF");
            this->main_state_ = MAIN_STATE::IDLE;
            this->sub_state_ = SUB_STATE::IDLE;
            if (!msg->data)
            {
                geometry_msgs::msg::Twist cmd_vel{};
                this->cmd_vel_pub_->publish(cmd_vel);
            }
        });

    // [EB]Lidar
    this->subscribers["scan"] = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos_profile, std::bind(&ManeuverNode::scan_callback, this, std::placeholders::_1));

    // [EB]Collision pcd
    this->subscribers["collision"] = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/sensor_to_pointcloud/collision",
        qos_profile,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            if (msg->width > 0 && this->use_maneuver_action_)
            {
                if (this->log_time_diff("collision_detect", 1000))
                {
                    RCLCPP_INFO(this->get_logger(), "[Collision] Collision detecting. move back.");
                    this->state_log(MAIN_STATE::BACK, SUB_STATE::COLLISION);
                }

                this->sub_state_ = SUB_STATE::COLLISION;
                this->robot_state(this->back_ms_collision_, MAIN_STATE::BACK);
            }
        });

    this->subscribers["collision_back"] = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/sensor_to_pointcloud/collision/rear",
        qos_profile,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            if (msg->width > 0 && this->use_maneuver_action_)
            {
                if (this->log_time_diff("collision_back_detect", 1000))
                {
                    RCLCPP_INFO(this->get_logger(), "[Collision] Collision back detecting. stop.");
                    this->state_log(MAIN_STATE::STOP, SUB_STATE::COLLISION_BACK);
                }

                this->sub_state_ = SUB_STATE::COLLISION_BACK;
                this->robot_state(this->wait_ms_normal_, MAIN_STATE::STOP);
            }
        });

    // [NAV2] cmd_vel
    this->subscribers["cmd_vel_nav"] = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_smoothed", qos_profile, std::bind(&ManeuverNode::cmd_vel_nav_callback, this, std::placeholders::_1));

    // [NAV2] global costmap
    this->subscribers["costmap"] = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap",
        10,
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        {
            // if (!this->use_maneuver_action_)
            //     return;

            auto x = this->robot_pose_.x;
            auto y = this->robot_pose_.y;
            int8_t cost = this->getCostXY(msg, x, y);
            if (cost != -1)
            {
                this->latest_costmap_ = msg;
            }
        });

    // [NAV2] acml pose
    this->subscribers["amcl_pose"] = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
        {
            this->robot_pose_.x = msg->pose.pose.position.x;
            this->robot_pose_.y = msg->pose.pose.position.y;
            this->robot_pose_.z = msg->pose.pose.position.z;
            tf2::Quaternion q;
            q.setX(msg->pose.pose.orientation.x);
            q.setY(msg->pose.pose.orientation.y);
            q.setZ(msg->pose.pose.orientation.z);
            q.setW(msg->pose.pose.orientation.w);
            this->robot_pose_.setFromQuaternion(q);
        });

    // [NAV2] local plan
    this->subscribers["local_plan"] = this->create_subscription<nav_msgs::msg::Path>(
        "/local_plan", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { this->local_plan_ = msg; });

    // [NAV2] global plan
    this->subscribers["global_plan"] = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { this->global_plan_ = msg; });

    // [EB] tof data
    this->subscribers["tof_data"] = this->create_subscription<robot_custom_msgs::msg::TofData>(
        "/tof_data", 10, [this](const robot_custom_msgs::msg::TofData::SharedPtr msg) { this->tof_msg_ = msg; });

    // [EB] lidar direction
    this->subscribers["force_escape_heading"] = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/forced_escape_heading",
        10,
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {
            if (this->sub_state_ == SUB_STATE::FORCE_ESCAPE_WAIT)
            {
                float front_rad = msg->data[0];
                float front_dist = msg->data[1];
                float rear_rad = msg->data[2];
                float rear_dist = msg->data[3];
                // 전방이 더 멀다. 전진
                double given_yaw = front_dist >= rear_dist ? front_rad : rear_rad;
                this->force_escape_target_angle_ = given_yaw;
                this->force_escape_given_yaw_ = this->robot_pose_.yaw + given_yaw;
                this->state_log(MAIN_STATE::TURN, SUB_STATE::FORCE_ESCAPE_TURN);
                this->main_state_ = MAIN_STATE::TURN;
                this->sub_state_ = SUB_STATE::FORCE_ESCAPE_TURN;
                RCLCPP_INFO(
                    this->get_logger(),
                    "[Force escape][front] ang: %f, dist: %f, [rear] ang: %f, dist: %f, given_yaw: %f",
                    front_rad,
                    front_dist,
                    rear_rad,
                    rear_dist,
                    given_yaw);
            }
        });

    // [NAV2] static map
    this->subscribers["static_map"] = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { this->latest_static_map_ = msg; });
}

/***
 * @brief 라이다 스캔 콜백
 * 정면/후면 라이다 거리 검사하는 콜백
 */
void ManeuverNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (!this->use_maneuver_action_)
        return;

    bool front_obstacle_detect = false;

    float back_threshold = this->lidar_back_escape_dist_;
    float back_angle_threshold = this->lidar_back_escape_angle_;
    bool back_obstacle_detect = false;

    bool drop_ir_lidar_front_detect = false;
    bool drop_ir_lidar_back_detect = false;

    int num_readings = msg->ranges.size();
    float angle = msg->angle_min;

    float variable_const =
        (this->lidar_front_escape_max_dist_ - this->lidar_front_escape_min_dist_) / (this->max_vel_ - this->min_vel_);
    float b = this->lidar_front_escape_min_dist_ - (variable_const * this->min_vel_);
    float speed = this->motor_status_.getVelocity();
    if (speed < 0)
    {
        speed = 0;
    }
    float front_threshold = b + variable_const * speed;
    for (int i = 0; i < num_readings; i++, angle += msg->angle_increment)
    {
        double dist = msg->ranges[i];

        if (std::isnan(dist) || std::isinf(dist) || dist <= msg->range_min || dist >= msg->range_max)
        {
            continue;  // 무효한 데이터는 건너뜀
        }

        float angle_deg = angle * 180.0f / M_PI;  // 라디안 -> 도(degree) 변환

        if (std::abs(angle_deg) <= this->drop_ir_lidar_collision_front_angle_ &&
            dist <= this->drop_ir_lidar_collision_front_distance_)
        {
            drop_ir_lidar_front_detect = true;
        }
        else if (
            std::abs(angle_deg) >= (180.0f - this->drop_ir_lidar_collision_back_angle_) &&
            dist <= this->drop_ir_lidar_collision_back_distance_)
        {
            drop_ir_lidar_back_detect = true;
        }

        // 전방 장애물 감지 (-front_angle_threshold ~ +front_angle_threshold)
        if (std::abs(angle_deg) <= this->lidar_front_escape_angle_ && dist <= front_threshold)
        {
            front_obstacle_detect = true;

            if (this->log_time_diff("lidar_front_detect", 1000))
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "[Lidar] front -> speed: %f, threshold: %f, obj dist: %f, angle_threshold: %f, angle: %f",
                    speed,
                    front_threshold,
                    dist,
                    this->lidar_front_escape_angle_,
                    angle_deg);
            }
        }
        // 전방 대각선 장애물 감지 (-90 ~ -front_angle_threshold || +front_angle_threshold ~ 90)
        else if (
            std::abs(angle_deg) > this->lidar_front_escape_angle_ && std::abs(angle_deg) < 90.0f &&
            dist < this->lidar_front_escape_diagonal_dist_)
        {
            front_obstacle_detect = true;

            if (this->log_time_diff("lidar_front_diagonal_detect", 1000))
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "[Lidar] front diagonal -> speed: %f, threshold: %f, obj dist: %f, angle_threshold: %f, angle: %f",
                    speed,
                    this->lidar_front_escape_diagonal_dist_,
                    dist,
                    this->lidar_front_escape_angle_,
                    angle_deg);
            }
        }
        // 후방 장애물 감지 (180도 방향 기준으로 back_angle_threshold 내에 있음)
        else if (std::abs(angle_deg) >= (180.0f - back_angle_threshold) && dist <= back_threshold)
        {
            back_obstacle_detect = true;
        }
    }

    this->front_escape_detect_ = front_obstacle_detect;
    this->back_object_detect_ = back_obstacle_detect;

    this->drop_ir_lidar_front_detect_ = drop_ir_lidar_front_detect;
    this->drop_ir_lidar_back_detect_ = drop_ir_lidar_back_detect;
}

/***
 * @brief Navigation의 cmd_vel을 구독하여 IDLE 상태일 때 명령어를 bypass 해주는 콜백
 */
void ManeuverNode::cmd_vel_nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (!this->use_maneuver_action_)
        return;

    if (this->is_main_idle() && this->is_sub_idle())
    {
        double speed = this->current_velocity_ + this->velocity_scaling_factor_;
        if (msg->linear.x > speed && msg->linear.x > 0.1)
        {
            msg->linear.x = speed;
        }
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = msg->linear.x;
        cmd_vel.linear.y = msg->linear.y;
        cmd_vel.linear.z = msg->linear.z;
        cmd_vel.angular.x = msg->angular.x;
        cmd_vel.angular.y = msg->angular.y;
        cmd_vel.angular.z = msg->angular.z;
        cmd_vel_pub_->publish(cmd_vel);
    }
}

/***
 * @brief state_manager에서 navigation의 상태를 보내줌
 */
void ManeuverNode::navi_state_callback(const robot_custom_msgs::msg::NaviState::SharedPtr msg)
{
    if (!this->use_maneuver_action_)
        return;
    this->navi_state_ = static_cast<NAVI_STATE>(msg->state);
}

/***
 * @brief 추락 IR sensors 로직 적용
 *
 *        FF
 *     / - 0 - \
 * FL 5         1 FR
 *    |base_link|
 * BL 4         2 BR
 *     \ - 3 - /
 *        BB
 */
void ManeuverNode::bottom_ir_callback(const robot_custom_msgs::msg::BottomIrData::SharedPtr msg)
{
    if (!this->use_maneuver_action_)
        return;

    bool ff = msg->ff;
    bool fl = msg->fl;
    bool fr = msg->fr;
    bool bb = msg->bb;
    bool bl = msg->bl;
    bool br = msg->br;
    geometry_msgs::msg::Twist cmd_vel{};
    bool pub_cmd = false;
    if (ff || fr || fl || bb || bl || br)
    {
        if (!this->is_climb_from_perception)
        {
            pub_cmd = true;
        }
        else if (this->log_time_diff("drop_ir_climb_log", 1000))
        {
            RCLCPP_INFO(
                this->get_logger(),
                "[Climb] DROP IR ff:%s fr:%s fl:%s bl:%s bb:%s br:%s detected. but climb state.",
                ff ? "true" : "false",
                fr ? "true" : "false",
                fl ? "true" : "false",
                bb ? "true" : "false",
                bl ? "true" : "false",
                br ? "true" : "false");
            return;
        }

        if (this->tof_msg_)
        {
            float drop_off_param_limit = 0.59 - 0.19;
            if (fr)
            {
                float bot_right_end = this->tof_msg_->bot_right[15];
                if (drop_off_param_limit > bot_right_end)
                {
                    if (this->log_time_diff("ir_fr_ignore", 1000))
                        RCLCPP_WARN(this->get_logger(), "[DROP IR] fr but ignored.");
                    return;
                }
            }
            else if (fl)
            {
                float bot_left_end = this->tof_msg_->bot_left[13];
                if (drop_off_param_limit > bot_left_end)
                {
                    if (this->log_time_diff("ir_fl_ignore", 1000))
                        RCLCPP_WARN(this->get_logger(), "[DROP IR] fl but ignored.");
                    return;
                }
            }
            else if (ff)
            {
                float bot_left_center = this->tof_msg_->bot_left[15];
                float bot_right_center = this->tof_msg_->bot_right[13];
                if (drop_off_param_limit > bot_left_center || drop_off_param_limit > bot_right_center)
                {
                    if (this->log_time_diff("ir_ff_ignore", 1000))
                        RCLCPP_WARN(this->get_logger(), "[DROP IR] ff but ignored.");
                    return;
                }
            }
        }
    }
    else
    {
        if (this->main_state_ == MAIN_STATE::STOP && this->sub_state_ == SUB_STATE::DROP_IR)
        {
            if (this->log_time_diff("drop_ir_release_log", 1000))
            {
                this->state_log(MAIN_STATE::WAIT, SUB_STATE::DROP_IR);
            }

            this->sub_state_ = SUB_STATE::DROP_IR;
            this->robot_state(500, MAIN_STATE::WAIT);
        }
        return;
    }

    float speed = 0.1;
    float angle = 0.1;
    if (this->drop_ir_lidar_back_detect_ && (ff || fr || fl))
    {  // stop //
        if (this->log_time_diff("ir_front_detect_no_move", 1000))
        {
            RCLCPP_WARN(this->get_logger(), "[Bottom IR] request move back. but something exist. stop.");
        }
    }
    else if (this->drop_ir_lidar_front_detect_ && (bb || br || bl))
    {  // stop //
        if (this->log_time_diff("ir_back_detect_no_move", 1000))
        {
            RCLCPP_WARN(this->get_logger(), "[Bottom IR] request move forward. but something exist. stop.");
        }
    }
    else if (ff && fr)
    {
        if (this->log_time_diff("drop_ir_ff_fr_detect", 1000))
        {
            RCLCPP_WARN(this->get_logger(), "[DROP IR] ff fr detected.");
        }
        // 0, 1
        cmd_vel.linear.x = -speed;
        cmd_vel.angular.z = angle;
    }
    else if (ff && fl)
    {
        // 0, 5
        if (this->log_time_diff("drop_ir_ff_fl_detect", 1000))
        {
            RCLCPP_WARN(this->get_logger(), "[DROP IR] ff fl detected.");
        }
        cmd_vel.linear.x = -speed;
        cmd_vel.angular.z = -angle;
    }
    else if ((fr && br) || (fl && bl))
    {
        if ((fr && br) && this->log_time_diff("drop_ir_fr_br_detect", 1000))
        {
            RCLCPP_WARN(this->get_logger(), "[DROP IR] fr br detected.");
        }
        if ((fl && bl) && this->log_time_diff("drop_ir_fl_bl_detect", 1000))
        {
            RCLCPP_WARN(this->get_logger(), "[DROP IR] fl bl detected.");
        }
        // 1, 2
        // 4, 5
        // stop //
    }
    else if (bb && br)
    {
        if (this->log_time_diff("drop_ir_bb_br_detect", 1000))
        {
            RCLCPP_WARN(this->get_logger(), "[DROP IR] bb br detected.");
        }
        // 2, 3
        cmd_vel.linear.x = speed;
        cmd_vel.angular.z = -angle;
    }
    else if (bb && bl)
    {
        if (this->log_time_diff("drop_ir_bb_bl_detect", 1000))
        {
            RCLCPP_WARN(this->get_logger(), "[DROP IR] bb bl detected.");
        }
        // 3, 4
        cmd_vel.linear.x = speed;
        cmd_vel.angular.z = angle;
    }
    else if (ff)
    {
        if (this->log_time_diff("drop_ir_ff_detect", 1000))
        {
            RCLCPP_WARN(this->get_logger(), "[DROP IR] ff detected.");
        }
        cmd_vel.linear.x = -speed;
    }
    else if (fl)
    {
        if (this->log_time_diff("drop_ir_fl_detect", 1000))
        {
            RCLCPP_WARN(this->get_logger(), "[DROP IR] fl detected.");
        }
        cmd_vel.linear.x = -speed;
        cmd_vel.angular.z = -angle;
    }
    else if (fr)
    {
        if (this->log_time_diff("drop_ir_fr_detect", 1000))
        {
            RCLCPP_WARN(this->get_logger(), "[DROP IR] fr detected.");
        }
        cmd_vel.linear.x = -speed;
        cmd_vel.angular.z = angle;
    }
    else if (br)
    {
        if (this->log_time_diff("drop_ir_br_detect", 1000))
        {
            RCLCPP_WARN(this->get_logger(), "[DROP IR] br detected.");
        }
        cmd_vel.linear.x = speed;
        cmd_vel.angular.z = -angle;
    }
    else if (bb)
    {
        if (this->log_time_diff("drop_ir_bb_detect", 1000))
        {
            RCLCPP_WARN(this->get_logger(), "[DROP IR] bb detected.");
        }
        cmd_vel.linear.x = speed;
    }
    else if (bl)
    {
        if (this->log_time_diff("drop_ir_bl_detect", 1000))
        {
            RCLCPP_WARN(this->get_logger(), "[DROP IR] bl detected.");
        }
        cmd_vel.linear.x = speed;
        cmd_vel.angular.z = angle;
    }

    if (pub_cmd)
    {
        if (this->log_time_diff("drop_ir_detect_log", 100))
        {
            this->state_log(MAIN_STATE::STOP, SUB_STATE::DROP_IR);
        }

        this->main_state_ = MAIN_STATE::STOP;
        this->sub_state_ = SUB_STATE::DROP_IR;
        this->cmd_vel_pub_->publish(cmd_vel);
    }
}

/**
 * @brief perception에서 climb state를 전송함.
 *
 * @param[in] msg Boolean 메세지. 로봇이 승월상태인지 아닌지. 승월이면 true.
 */
void ManeuverNode::perception_climb_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    auto climb = msg->data;
    if (climb != this->is_climb_from_perception)
    {
        if (climb)
        {
            RCLCPP_INFO(this->get_logger(), "[Climb] Enter climb state from perception.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "[Climb] Release climb state from perception.");
        }
    }
    this->is_climb_from_perception = climb;
}
/***
 * @brief perception에서 특정 상태로 데이터를 보냄. 그 데이터를 기반으로 액션 로직 적용
 * 0 : SUB_STATE::IDLE
 * 1 : SUB_STATE::DROP_OFF
 * 2 : SUB_STATE::ONE_D_TOF
 */
void ManeuverNode::action_stop_callback(const std_msgs::msg::Int8::SharedPtr msg)
{
    if (!this->use_maneuver_action_)
        return;

    if (this->is_main_idle() && this->is_sub_idle())
    {
        if (msg->data == 1)
        {
            this->state_log(MAIN_STATE::STOP, SUB_STATE::DROP_OFF);
            this->main_state_ = MAIN_STATE::STOP;
            this->sub_state_ = SUB_STATE::DROP_OFF;
        }
        else if (msg->data == 2)
        {
            this->state_log(MAIN_STATE::STOP, SUB_STATE::ONE_D_TOF);
            this->main_state_ = MAIN_STATE::STOP;
            this->sub_state_ = SUB_STATE::ONE_D_TOF;
        }
        else
        {
            this->state_log(MAIN_STATE::IDLE, SUB_STATE::IDLE);
            this->main_state_ = MAIN_STATE::IDLE;
            this->sub_state_ = SUB_STATE::IDLE;
        }
    }
}

/**
 * @brief 일정 주기로 동작하는 Timer Callback
 */
void ManeuverNode::timer_callback()
{
    auto current_time = std::chrono::steady_clock::now();
    if (current_time - this->last_state_pub_time_ > std::chrono::seconds(1))
    {
        this->last_state_pub_time_ = std::chrono::steady_clock::now();
        this->pub_states();
    }
    else if (current_time - this->last_state_pub_time_ < std::chrono::seconds(0))
    {
        this->last_state_pub_time_ = std::chrono::steady_clock::now();
    }

    if (this->use_maneuver_action_ == false)
        return;

    bool is_publish = false;
    geometry_msgs::msg::Twist cmd_vel{};

    // 현재 위치 코스트맵 체크
    this->current_postion_costmap_check();
    // 현재 위치가 코스트맵에 갇혔으면
    bool isForceEscape =
        (this->sub_state_ == SUB_STATE::FORCE_ESCAPE_WAIT || this->sub_state_ == SUB_STATE::FORCE_ESCAPE_TURN ||
         this->sub_state_ == SUB_STATE::FORCE_ESCAPE_MOVE);

    if (this->costmap_escape_sampling_ > this->costmap_escape_sampling_threshold_ &&
        (this->sub_state_ != SUB_STATE::ESCAPE_TURN && this->sub_state_ != SUB_STATE::ESCAPE_MOVE && !isForceEscape))
    {
        this->lethal_escape_process();
    }

    if (this->main_state_ == MAIN_STATE::BACK)
    {
        if (this->is_processing(current_time))
        {
            if (this->sub_state_ == SUB_STATE::COLLISION)
            {
                this->robot_action_back(cmd_vel, -0.2);
            }
            if (this->sub_state_ == SUB_STATE::FORCE_ESCAPE_MOVE)
            {
                this->robot_action_front(cmd_vel, -0.05);
            }
            else
            {
                this->robot_action_back(cmd_vel, -0.1);
            }
            is_publish = true;
        }
        else
        {
            this->state_log(MAIN_STATE::WAIT, this->sub_state_);
            this->robot_state(this->wait_ms_normal_, MAIN_STATE::WAIT);
        }
    }
    else if (this->main_state_ == MAIN_STATE::FRONT)
    {
        if (this->is_processing(current_time))
        {
            if (this->sub_state_ == SUB_STATE::FORCE_ESCAPE_MOVE)
            {
                this->robot_action_front(cmd_vel, 0.05);
            }
            else
            {
                this->robot_action_front(cmd_vel, 0.1);
            }
            is_publish = true;
        }
        else
        {
            this->state_log(MAIN_STATE::WAIT, this->sub_state_);
            this->robot_state(this->wait_ms_normal_, MAIN_STATE::WAIT);
        }
    }
    else if (this->main_state_ == MAIN_STATE::STOP)
    {
        bool isMinRpm = this->motor_status_.isLowRpm();
        bool isRotate = this->motor_status_.isRotate();

        if (isMinRpm || isRotate)
        {
            if (this->sub_state_ == SUB_STATE::ONE_D_TOF)
            {
                if (this->log_time_diff("timer_1d_tof_move_back", 1000))
                {
                    RCLCPP_INFO(this->get_logger(), "[1D ToF] 1D detect. Go to back.");
                }
                this->state_log(MAIN_STATE::BACK, SUB_STATE::ONE_D_TOF);
                this->robot_state(this->back_ms_1d_, MAIN_STATE::BACK);
                this->sub_state_ = SUB_STATE::ONE_D_TOF;
            }
            else if (this->sub_state_ == SUB_STATE::DROP_OFF)
            {
                this->state_log(MAIN_STATE::WAIT, SUB_STATE::DROP_OFF);
                this->robot_state(this->wait_ms_drop_off_, MAIN_STATE::WAIT);
                this->sub_state_ = SUB_STATE::DROP_OFF;
            }
            else if (this->sub_state_ == SUB_STATE::COLLISION_BACK)
            {
                this->state_log(MAIN_STATE::WAIT, SUB_STATE::COLLISION_BACK);
                this->robot_state(this->wait_ms_normal_, MAIN_STATE::WAIT);
                this->sub_state_ = SUB_STATE::COLLISION_BACK;
            }
        }
        else if (this->sub_state_ != SUB_STATE::DROP_IR)
        {
            // 정지 커맨드 전송, drop ir은 자체적으로 작동함
            is_publish = true;
        }
    }
    else if (this->main_state_ == MAIN_STATE::WAIT)
    {
        if (this->is_processing(current_time))
        {
            cmd_vel.linear.x = 0.0;
            is_publish = true;
        }
        else
        {
            this->state_log(MAIN_STATE::IDLE, SUB_STATE::IDLE);
            this->main_state_ = MAIN_STATE::IDLE;
            this->sub_state_ = SUB_STATE::IDLE;
        }
    }
    else if (this->main_state_ == MAIN_STATE::TURN)
    {
        if (this->sub_state_ == SUB_STATE::ESCAPE_TURN)
        {
            this->escape_turn_action(cmd_vel, is_publish);
        }
        else if (this->sub_state_ == SUB_STATE::NO_LOCAL_PLAN)
        {
            this->process_no_local_plan(cmd_vel, is_publish);
        }
        else if (this->sub_state_ == SUB_STATE::FORCE_ESCAPE_TURN)
        {
            this->force_escape_turn_action(cmd_vel, is_publish);
        }
    }

    // 탈출 상태 확인
    bool isNotEscapeState = !(this->sub_state_ == SUB_STATE::ESCAPE_TURN || this->sub_state_ == SUB_STATE::ESCAPE_MOVE);
    isForceEscape =
        (this->sub_state_ == SUB_STATE::FORCE_ESCAPE_WAIT || this->sub_state_ == SUB_STATE::FORCE_ESCAPE_TURN ||
         this->sub_state_ == SUB_STATE::FORCE_ESCAPE_MOVE);

    if (isNotEscapeState && !isForceEscape)
    {
        // Nav 주행중(전진, 회전) 전방 라이다 검출 발생시, 후진
        // 2025.05.29 승월 상태에서는 작동 하지 않기로 함
        if (this->front_escape_detect_ && (this->motor_status_.isMoveToForward() || this->motor_status_.isRotate()))
        {
            if (this->is_climb_from_perception)
            {
                if (this->log_time_diff("timer_front_escape_climb", 1000))
                    RCLCPP_WARN(this->get_logger(), "[Lidar] Front escape detected. but climb state now.");
            }
            else
            {
                if (this->log_time_diff("timer_front_escape_detect", 1000))
                {
                    RCLCPP_WARN(this->get_logger(), "[Lidar] Front escape detected.");
                    this->state_log(MAIN_STATE::BACK, SUB_STATE::LIDAR_FRONT);
                }
                this->robot_state(2000, MAIN_STATE::BACK);
                this->sub_state_ = SUB_STATE::LIDAR_FRONT;
            }
        }

        // Nav 주행중(후진) 후방 라이다 검출시 정지
        if (this->back_object_detect_ && this->motor_status_.isMoveToBack())
        {
            if (this->log_time_diff("timer_back_object_detect", 1000))
            {
                RCLCPP_WARN(this->get_logger(), "[Lidar] Robot move back. but something exists. wait .");
                this->state_log(MAIN_STATE::WAIT, SUB_STATE::LIDAR_BACK);
            }
            this->robot_state(1000, MAIN_STATE::WAIT);
            this->sub_state_ = SUB_STATE::LIDAR_BACK;
            cmd_vel.linear.x = 0.0;
            is_publish = true;
        }

        /**
        2025-06-20, hjkim(everybot)
        자동매핑 상태에서 NAVI_STATE 상태가 없어 로컬 플랜 동작하지 않는 문제를 해결하기 위해
        explore_state 가 running 인 경우 로컬플랜 동작하도록 수정
        @See A1_maneuver/include/states.hpp MAPPING_STATES

        # No valid trajectories out of 420!
        1. 코스트맵에 갇힌 상태가 아니다
        2. 네비게이션의 현재 상태가 이동중이다 (/navi_datas topic), this->navi_state_ == NAVI_STATE::MOVE_GOAL
        3. /plan (global plan)의 경로 상태가 있을 경우
        4. /local_plan의 poses가 [] 즉 empty 상태면 위에 나타난 상태임
        */
        if ((this->navi_state_ == NAVI_STATE::MOVE_GOAL || this->explore_state_ == MAPPING_STATE::EXPLORING_STATE ||
             this->explore_state_ == MAPPING_STATE::COMPLETE_STATE) &&
            is_main_idle())
        {
            this->check_no_local_plan();
        }
        else
        {
            this->no_plan_sampling_ = 0;
        }
    }
    if (this->no_plan_sampling_ > this->no_plan_sampling_threshold_ && is_main_idle() && is_sub_idle())
    {
        this->no_plan_start_time_ = this->get_clock()->now();
        this->enter_no_local_plan(cmd_vel, is_publish);
    }

    // 상태 퍼블리시해야하면 퍼블리시
    if (is_publish)
    {
        this->cmd_vel_pub_->publish(cmd_vel);
    }
}

void ManeuverNode::force_escape_turn_action(geometry_msgs::msg::Twist& cmd_vel, bool& is_publish)
{
    // 판별용
    double target_rad = this->force_escape_target_angle_;
    double target_deg = A1::maneuver::rad2deg(target_rad);

    double rad_diff = A1::maneuver::angleDiff(this->force_escape_given_yaw_, this->robot_pose_.yaw);
    auto angle_diff = A1::maneuver::rad2deg(rad_diff);
    double angular_z = this->costmap_escape_angular_z_speed;

    if (this->log_time_diff("force_escape_turn_target_angle", 1000))
    {
        RCLCPP_INFO(
            this->get_logger(),
            "[Force escape] Turn. yaw:%f, target angle %f, diff %f",
            this->robot_pose_.yaw,
            target_rad,
            angle_diff);
    }
    if (std::fabs(target_deg) > 90)
    {
        // 후진
        if (angle_diff < -90 && angle_diff > -170)
        {
            cmd_vel.angular.z = angular_z;
            is_publish = true;
        }
        else if (angle_diff > 90 && angle_diff < 170)
        {
            cmd_vel.angular.z = -angular_z;
            is_publish = true;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "[Force escape] Turn finished. Move back.");
            this->state_log(MAIN_STATE::BACK, SUB_STATE::FORCE_ESCAPE_MOVE);
            this->robot_state(100.0 * this->costmap_escape_time_multiplier_for_dist, MAIN_STATE::BACK);
            this->sub_state_ = SUB_STATE::FORCE_ESCAPE_MOVE;
        }
    }
    else
    {
        // 각도 맞추고 전진
        if (angle_diff < -3)
        {
            cmd_vel.angular.z = -angular_z;
            is_publish = true;
        }
        else if (angle_diff > 3)
        {
            cmd_vel.angular.z = angular_z;
            is_publish = true;
        }
        else
        {
            cmd_vel.angular.z = 0.0;
            is_publish = true;
            RCLCPP_INFO(this->get_logger(), "[Force escape] Turn finished. Move forward.");
            this->state_log(MAIN_STATE::FRONT, SUB_STATE::FORCE_ESCAPE_MOVE);
            this->robot_state(100.0 * this->costmap_escape_time_multiplier_for_dist, MAIN_STATE::FRONT);
            this->sub_state_ = SUB_STATE::FORCE_ESCAPE_MOVE;
        }
    }
}
/**
 * @brief [Costmap escape] (Turn, mov forward, move back) for escape
 * @param cmd_vel publish할 cmd_vel
 * @param is_publish publish해야하면 true
 *
 * 이 기능은 로봇이 MAIN_STATE::TURN 및 SUB_STATE::ESCAPE_TURN 상태일 때 호출됩니다.
 * 1. 현재 위치와 탈출점과의 각도 차이를 계샨
 * 2. 각도 차이에 따라 좌회전할지 우회전할지 결정합니다.
 * 3. 목표점이 전진일 때와 후진일 때를 구분하여 액션을 취합니다.
 */
void ManeuverNode::escape_turn_action(geometry_msgs::msg::Twist& cmd_vel, bool& is_publish)
{
    double cell_rad = std::atan2(escape_target_.my, escape_target_.mx);
    double rad_diff = A1::maneuver::angleDiff(cell_rad, this->robot_pose_.yaw);
    auto angle_diff = A1::maneuver::rad2deg(rad_diff);
    double angular_z = this->costmap_escape_angular_z_speed;

    if (this->log_time_diff("escape_turn_target_angle", 1000))
    {
        RCLCPP_INFO(this->get_logger(), "[Costmap escape] Turn for escape. target angle diff : %f", angle_diff);
    }
    if (std::fabs(escape_target_.angle_diff) > 90)
    {
        // 후진
        if (angle_diff < -90 && angle_diff > -170)
        {
            cmd_vel.angular.z = angular_z;
            is_publish = true;
        }
        else if (angle_diff > 90 && angle_diff < 170)
        {
            cmd_vel.angular.z = -angular_z;
            is_publish = true;
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "[Costmap escape] Turn finished(Angle of rotation: %f). Move back.",
                escape_target_.angle_diff - angle_diff);
            this->state_log(MAIN_STATE::BACK, SUB_STATE::ESCAPE_MOVE);
            this->robot_state(
                (escape_target_.distance * this->costmap_escape_time_multiplier_for_dist), MAIN_STATE::BACK);
            this->sub_state_ = SUB_STATE::ESCAPE_MOVE;
        }
    }
    else
    {
        // 각도 맞추고 전진
        if (angle_diff < -3)
        {
            cmd_vel.angular.z = -angular_z;
            is_publish = true;
        }
        else if (angle_diff > 3)
        {
            cmd_vel.angular.z = angular_z;
            is_publish = true;
        }
        else
        {
            cmd_vel.angular.z = 0.0;
            is_publish = true;
            RCLCPP_INFO(
                this->get_logger(),
                "[Costmap escape] Turn finished(Angle of rotation: %f). Move forward.",
                escape_target_.angle_diff - angle_diff);
            this->state_log(MAIN_STATE::FRONT, SUB_STATE::ESCAPE_MOVE);
            this->robot_state(
                (escape_target_.distance * this->costmap_escape_time_multiplier_for_dist), MAIN_STATE::FRONT);
            this->sub_state_ = SUB_STATE::ESCAPE_MOVE;
        }
    }
}

/**
 * @brief 로컬 플랜이 없는지 확인하고 샘플링 횟수를 업데이트
 *
 * 이 함수는 로봇이 유효한 로컬 플랜을 가지고 있는지 여부를 다음과 같이 평가
 * global plan에서 포즈의 최신성과 존재 여부 확인
 * local plan에 유효한 plan 없는 경우 no_plan_sampling_이 증가
 * 로봇이 이전에 NO_LOCAL_PLAN 상태였다면 IDLE 상태로 전환
    # No valid trajectories out of 420!
    1. 코스트맵에 갇힌 상태가 아니다
    2. 네비게이션의 현재 상태가 이동중이다 (/navi_datas topic), this->navi_state_ == NAVI_STATE::MOVE_GOAL
    3. /plan (global plan)의 경로 상태가 있을 경우
    4. /local_plan의 poses가 [] 즉 empty 상태면 위에 나타난 상태임
 */

void ManeuverNode::check_no_local_plan(void)
{
    if (!this->global_plan_ || !this->local_plan_)
    {
        this->no_plan_sampling_ = 0;
    }
    else
    {
        // 현재 시간 획득 (노드 내에서 rclcpp::Clock 사용)
        auto now = this->now();

        // 5초 이내인지 확인
        bool global_plan_recent = (now - rclcpp::Time(this->global_plan_->header.stamp)).seconds() <= 5.0;
        bool local_plan_recent = (now - rclcpp::Time(this->local_plan_->header.stamp)).seconds() <= 5.0;

        // 경로 개수 확인
        bool global_plan_has_path = this->global_plan_->poses.size() > 0;
        bool local_plan_empty = this->local_plan_->poses.size() == 0;
        if (global_plan_recent && local_plan_recent && global_plan_has_path && local_plan_empty)
        {
            // 조건을 만족할 때 실행할 코드
            this->no_plan_sampling_++;
            if (this->log_time_diff("no_plan_sampling_upcount", 1000))
            {
                RCLCPP_WARN(this->get_logger(), "[No plan] sampling counting: %d.", this->no_plan_sampling_);
            }
        }
        else
        {
            this->no_plan_sampling_ = 0;
        }
    }
}

/**
 * @brief local_plan이 없는 상황에 대한 플랜에 진입합니다.
 *
 * 이 기능은 로컬 플랜이 다음과 같을 때 로봇의 각속도를 조정합니다
 * 1. 코스트맵에 갇힌 상태가 아니다
 * 2. 네비게이션의 현재 상태가 이동중이다 (/navi_datas topic), this->navi_state_ == NAVI_STATE::MOVE_GOAL
 * 3. /plan (global plan)의 경로 상태가 있을 경우
 * 4. /local_plan의 poses가 [] 즉 empty 상태면 위에 나타난 상태임
 * 로봇을 원하는 방향으로 회전시키기 위해 각속도를 설정합니다
 * 방향. 샘플링 횟수가 임계값을 초과할 경우 경고를 기록합니다.
 *
 * @param cmd_vel 트위스트 메시지를 참조하여 각속도를 설정합니다.
 * @param is_publish true if the velocity command should be published, false otherwise
 */
void ManeuverNode::enter_no_local_plan(geometry_msgs::msg::Twist& cmd_vel, bool& is_publish)
{
    if (!this->global_plan_ || !this->local_plan_)
        return;
    if (this->global_plan_->poses.size() < 1 || this->local_plan_->poses.size() >= 1)
        return;

    auto pose = this->robot_pose_;
    auto start_pose = this->global_plan_->poses[0];
    double pose_angle = std::atan2(start_pose.pose.position.y - pose.y, start_pose.pose.position.x - pose.x);
    double diff = A1::maneuver::angleDiff(pose_angle, pose.yaw);
    auto degree = rad2deg(diff);
    double angular_z = this->costmap_escape_angular_z_speed;
    // turn left angular.z +
    // turn right angular.z -
    if (degree <= -0.0)
    {
        cmd_vel.angular.z = angular_z;
    }
    else if (degree > 0.0)
    {
        cmd_vel.angular.z = -angular_z;
    }

    this->main_state_ = MAIN_STATE::TURN;
    this->sub_state_ = SUB_STATE::NO_LOCAL_PLAN;
    if (this->log_time_diff("no_plan_sampling_threshold", 1000))
    {
        this->state_log(MAIN_STATE::TURN, SUB_STATE::NO_LOCAL_PLAN);
        RCLCPP_WARN(
            this->get_logger(),
            "[No plan] No valid trajectory. sampling count %d.Turn start.",
            this->no_plan_sampling_);
    }

    is_publish = true;
}

/**
 * @brief no local_plan 해소하기 위한 작업을 시도합니다.
 *
 * @param cmd_vel 트위스트 메시지를 참조하여 각속도를 설정합니다.
 * @param is_publish true if the velocity command should be published, false otherwise
 */
void ManeuverNode::process_no_local_plan(geometry_msgs::msg::Twist& cmd_vel, bool& is_publish)
{
    auto current_time = this->get_clock()->now();
    if (!this->global_plan_ || !this->local_plan_)
        return;
    if (this->global_plan_)
    {
        auto global_plan_time = current_time - rclcpp::Time(this->global_plan_->header.stamp);
        if (global_plan_time > rclcpp::Duration::from_seconds(this->no_plan_global_timeout_sec_))
        {
            this->resolve_no_local_plan();
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            is_publish = true;
            RCLCPP_INFO(this->get_logger(), "[No plan] global plan timeout! release no local plan state.");
            return;
        }
        else if (this->global_plan_->poses.size() > 1)
        {
            auto global_first_pose = this->global_plan_->poses[0];
            double global_pose_angle = std::atan2(global_first_pose.pose.position.y, global_first_pose.pose.position.x);
            double diff = A1::maneuver::angleDiff(global_pose_angle, this->robot_pose_.yaw);
            auto robot_global_pose_diff = rad2deg(diff);
            if (std::abs(robot_global_pose_diff) < 5.0)
            {
                cmd_vel.linear.x = 0.05;
                cmd_vel.angular.z = 0.0;
                is_publish = true;
                RCLCPP_INFO(
                    this->get_logger(),
                    "[No plan] force move to forward! global path and robot yaw diff %.3f",
                    robot_global_pose_diff);
            }
        }
        // RCLCPP_INFO(this->get_logger(), "[No plan] global plan time check! %f", global_plan_time.seconds());
    }

    if (current_time - this->no_plan_start_time_ > rclcpp::Duration::from_seconds(this->no_plan_timeout_sec_))
    {
        this->resolve_no_local_plan();
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        is_publish = true;
        RCLCPP_INFO(
            this->get_logger(),
            "[No plan] no plan timeout! %.3fs",
            (current_time - this->no_plan_start_time_).seconds());
        return;
    }

    if (this->global_plan_->poses.size() < 1 || this->local_plan_->poses.size() < 1)
        return;

    auto global_first_pose = this->global_plan_->poses[0];
    auto local_first_pose = this->local_plan_->poses[0];

    double global_pose_angle = std::atan2(global_first_pose.pose.position.y, global_first_pose.pose.position.x);
    double local_pose_angle = std::atan2(local_first_pose.pose.position.y, local_first_pose.pose.position.x);
    double diff = A1::maneuver::angleDiff(global_pose_angle, local_pose_angle);
    auto degree = rad2deg(diff);
    if (this->log_time_diff("resolve_no_local_plan", 1000))
        RCLCPP_WARN(
            this->get_logger(),
            "[No plan] resolve_no_local_plan.(deg) global_angle: %f, local_angle: %f, angle diff : %f",
            rad2deg(global_pose_angle),
            rad2deg(local_pose_angle),
            degree);

    if (std::abs(degree) < this->no_plan_release_degree_)
    {
        this->resolve_no_local_plan();
        RCLCPP_INFO(this->get_logger(), "[No plan] Angle diff between Global-path And Local-path : %f", degree);
        RCLCPP_INFO(this->get_logger(), "[No plan] resolve no valid trajectory! ");
    }
}

void ManeuverNode::resolve_no_local_plan()
{
    this->state_log(MAIN_STATE::IDLE, SUB_STATE::IDLE);
    this->main_state_ = MAIN_STATE::IDLE;
    this->sub_state_ = SUB_STATE::IDLE;
    this->no_plan_sampling_ = 0;
}

/***
 * @brief Maneuver의 현재 상태를 topic으로 출력해주는 함수
 */
void ManeuverNode::pub_states()
{
    std::ostringstream formatted;
    formatted << "Use:" << (use_maneuver_action_ ? "True" : "False") << "/";
    formatted << "Main:" << enumToString(main_state_).c_str() << "/";
    formatted << "Sub:" << enumToString(sub_state_).c_str() << "/";
    formatted << "Navi:" << enumToString(navi_state_).c_str() << "/";
    formatted << "Mapping:" << enumToString(explore_state_).c_str() << "/";
    std_msgs::msg::String msg;
    msg.data = formatted.str();
    std_msgs::msg::Int8MultiArray int_msg;
    int_msg.data = {
        static_cast<signed char>(use_maneuver_action_ ? 1 : 0),
        static_cast<signed char>(main_state_),
        static_cast<signed char>(sub_state_),
        static_cast<signed char>(navi_state_),
        static_cast<signed char>(explore_state_)};
    if (this->current_maneuver_state != this->use_maneuver_action_)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "[Maneuver] maneuver use state change. use: %s.",
            this->use_maneuver_action_ ? "True" : "False");
        this->current_maneuver_state = this->use_maneuver_action_;
    }
    this->state_str_pub_->publish(msg);
    this->state_int_pub_->publish(int_msg);
}

/**
 * @brief 현재 maneuver의 액션 상태가 IDLE인지 확인하는 함수
 */
bool ManeuverNode::is_main_idle()
{
    return this->main_state_ == MAIN_STATE::IDLE;
}

/**
 * @brief 현재 상태가 stop이 적용된 상태인지 확인하는 함수
 */
bool ManeuverNode::is_sub_idle()
{
    return this->sub_state_ == SUB_STATE::IDLE;
}

/**
 * @brief 로봇이 뒤로 이동을 제어하는 함수
 */
void ManeuverNode::robot_action_back(geometry_msgs::msg::Twist& cmd_vel, const double speed)
{
    bool isEscapeState = (this->sub_state_ == SUB_STATE::ESCAPE_TURN || this->sub_state_ == SUB_STATE::ESCAPE_MOVE);

    bool isForceEscape =
        (this->sub_state_ == SUB_STATE::FORCE_ESCAPE_WAIT || this->sub_state_ == SUB_STATE::FORCE_ESCAPE_TURN ||
         this->sub_state_ == SUB_STATE::FORCE_ESCAPE_MOVE);

    if (this->back_object_detect_ == false || isEscapeState || isForceEscape)
    {
        cmd_vel.linear.x = speed;
    }
    else
    {
        cmd_vel.linear.x = 0.0;
        if (this->log_time_diff("move_back_lidar_detect", 1000))
        {
            RCLCPP_WARN(this->get_logger(), "[Lidar] Cannot move back. stop.");
        }
    }
}

/**
 * @brief 로봇이 앞으로 이동을 제어하는 함수
 */
void ManeuverNode::robot_action_front(geometry_msgs::msg::Twist& cmd_vel, const double speed)
{
    bool isForceEscape =
        (this->sub_state_ == SUB_STATE::FORCE_ESCAPE_WAIT || this->sub_state_ == SUB_STATE::FORCE_ESCAPE_TURN ||
         this->sub_state_ == SUB_STATE::FORCE_ESCAPE_MOVE);
    if (this->front_escape_detect_ == false || isForceEscape)
    {
        cmd_vel.linear.x = speed;
    }
    else
    {
        cmd_vel.linear.x = 0.0;
        if (this->log_time_diff("move_forward_lidar_detect", 1000))
        {
            RCLCPP_WARN(this->get_logger(), "[Lidar] Cannot move forward. stop.");
        }
    }
}

/**
 * @brief 로봇의 상태를 변화하고, 그 상태를 유지하는 시간을 지정하는 함수
 */
void ManeuverNode::robot_state(const double process_time_ms, const MAIN_STATE state)
{
    // std::chrono 사용
    this->action_start_time_ = std::chrono::steady_clock::now();
    this->action_process_time_ = std::chrono::milliseconds(static_cast<int64_t>(process_time_ms));
    this->main_state_ = state;
}

/**
 * @brief 로봇의 액션이 진행되고 있는지 확인하는 함수
 */
bool ManeuverNode::is_processing(std::chrono::steady_clock::time_point current_time)
{
    auto diff = current_time - this->action_start_time_;
    return diff <= this->action_process_time_;
}

int8_t ManeuverNode::getCostXY(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, float x, float y)
{
    auto width = msg->info.width;
    auto height = msg->info.height;
    auto resolution = msg->info.resolution;
    auto origin = msg->info.origin;

    int mx = static_cast<int>((x - origin.position.x) / resolution);
    int my = static_cast<int>((y - origin.position.y) / resolution);
    if (mx >= 0 && mx < static_cast<int>(width) && my >= 0 && my < static_cast<int>(height))
    {
        int index = my * width + mx;
        return msg->data[index];
    }
    return -2;
}
/**
 * @brief  로봇의 현재 위치를 cost map에서 확인하고 high-cost 영역에 갇혀 있는지 확인합니다.
 *
 * 이 함수는 비용 지도에서 로봇의 위치를 평가하여 로봇이 고비용 영역 내에 있는지 확인합니다.
 * 로봇이 high cost에 갇혀 있는 것이 감지되면 탈출 샘플링 카운터를 늘리고 이벤트를 기록합니다.
 * 탈출 샘플링 카운터가 특정 임계값에 도달하면 탈출 프로세스가 시작됩니다.
 */
void ManeuverNode::current_postion_costmap_check(void)
{
    if (!this->latest_costmap_)
    {
        return;
    }

    auto x = this->robot_pose_.x;
    auto y = this->robot_pose_.y;

    int8_t cost = this->getCostXY(this->latest_costmap_, x, y);
    int8_t static_cost = -2;
    if (this->latest_static_map_)
        static_cost = this->getCostXY(this->latest_static_map_, x, y);
    if (this->log_time_diff("robot_costmap_value", 1000))
    {
        if (static_cost == -2)
        {
            RCLCPP_INFO(this->get_logger(), "[Costmap check] Robots(%.2f, %.2f) cost(%d)", x, y, cost);
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(), "[Costmap check] Robots(%.2f, %.2f) cost(%d), static(%d)", x, y, cost, static_cost);
        }
    }
    if (cost > this->costmap_escape_value_ || static_cost == -1)
    {
        if ((this->main_state_ == MAIN_STATE::IDLE) || (this->sub_state_ == SUB_STATE::NO_LOCAL_PLAN))
        {
            if (this->costmap_escape_sampling_ == 0)
            {
                this->escape_start_time = std::chrono::steady_clock::now();
            }
            this->costmap_escape_sampling_++;
            if (this->log_time_diff("costmap_stuck_log", 1000))
            {
                if (static_cost == -2)
                {
                    RCLCPP_INFO(
                        this->get_logger(), "[Costmap escape] Robots(%.2f, %.2f) stuck at high cost(%d)", x, y, cost);
                }
                else
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "[Costmap escape] Robots(%.2f, %.2f) stuck at high cost(%d)-static(%d)",
                        x,
                        y,
                        cost,
                        static_cost);
                }
            }
        }
    }
    else if (cost <= this->costmap_escape_release_value_ && static_cost != -1)
    {
        this->costmap_escape_sampling_ = 0;
        if (this->sub_state_ == SUB_STATE::ESCAPE_MOVE || this->sub_state_ == SUB_STATE::ESCAPE_TURN)
        {
            if (this->log_time_diff("costmap_escape_idle", 1000))
            {
                RCLCPP_INFO(this->get_logger(), "[Costmap escape] escaped from high value cost(%d)", cost);
                this->state_log(MAIN_STATE::WAIT, SUB_STATE::IDLE);
            }
            this->robot_state(500, MAIN_STATE::WAIT);
            this->sub_state_ = SUB_STATE::IDLE;
        }

        if (this->sub_state_ == SUB_STATE::FORCE_ESCAPE_WAIT || this->sub_state_ == SUB_STATE::FORCE_ESCAPE_TURN ||
            this->sub_state_ == SUB_STATE::FORCE_ESCAPE_MOVE)
        {
            if (this->log_time_diff("force_escape_idle", 1000))
            {
                RCLCPP_INFO(this->get_logger(), "[Force escape] escaped complete");
                this->state_log(MAIN_STATE::WAIT, SUB_STATE::IDLE);
            }

            std_msgs::msg::Bool msg;
            msg.data = false;
            this->force_escape_pub_->publish(msg);
            this->robot_state(500, MAIN_STATE::WAIT);
            this->sub_state_ = SUB_STATE::IDLE;
        }
    }

    if (this->sub_state_ == SUB_STATE::FORCE_ESCAPE_WAIT || this->sub_state_ == SUB_STATE::FORCE_ESCAPE_TURN ||
        this->sub_state_ == SUB_STATE::FORCE_ESCAPE_MOVE)
    {
        auto current_time = std::chrono::steady_clock::now();
        if (current_time - this->force_escape_start_time_ > std::chrono::seconds(30))
        {
            RCLCPP_INFO(this->get_logger(), "[Force escape] force escape timeout(30s) !! ");
            this->state_log(MAIN_STATE::WAIT, SUB_STATE::IDLE);
            this->robot_state(500, MAIN_STATE::WAIT);
            this->sub_state_ = SUB_STATE::IDLE;
        }
    }
}

/**
 * @brief Costmap의 높은 코스트에 갇혔을 때의 탈출 프로세스
 *
 * 이 기능은 로봇이 high cost 영역에 갇혀 있는지 확인합니다.
 * 이동할 최적의 후보를 찾아 탈출합니다.
 *
 * @note timer_callback에 의해 호출됨.
 */
void ManeuverNode::lethal_escape_process(void)
{
    if (this->log_time_diff("high_cost_detect_log", 1000))
    {
        RCLCPP_INFO(
            this->get_logger(),
            "[Costmap escape] Escape from high cost, Detect Count(%d)",
            this->costmap_escape_sampling_);
    }
    if (this->log_time_diff("escape_start_time", 1000))
    {
        auto now = std::chrono::steady_clock::now();
        auto diff = now - this->escape_start_time;
        double diff_sec = std::chrono::duration_cast<std::chrono::duration<double>>(diff).count();
        RCLCPP_INFO(this->get_logger(), "[Costmap escape] Robot in high cost %.3f secs", diff_sec);
    }

    std::vector<EscapeCandidate> candidates = findBestCandidate(
        this->latest_costmap_,
        this->latest_static_map_,
        this->robot_pose_,
        this->costmap_escape_candidate_threshold_,
        this->costmap_escape_candidate_min_dist_,
        this->costmap_escape_candidate_max_dist_);
    // int count = 0;
    // RCLCPP_INFO(this->get_logger(), "===================");
    // for (const auto& c : candidates)
    // {
    //     RCLCPP_INFO(
    //         this->get_logger(),
    //         "%d.Candidate x:%f, y:%f, dist: %f, angle_diff:%f",
    //         ++count,
    //         c.mx,
    //         c.my,
    //         c.distance,
    //         c.angle_diff);
    // }
    if (candidates.size() > 0)
    {
        EscapeCandidate candidate = candidates[0];
        if (this->log_time_diff("high_cost_escape_turn_log", 1000))
        {
            auto x = candidate.mx + this->robot_pose_.x;
            auto y = candidate.my + this->robot_pose_.y;
            int8_t cost = this->getCostXY(this->latest_costmap_, x, y);
            RCLCPP_INFO(
                this->get_logger(),
                "[Costmap escape] Selected escape candidate x:%f, y:%f, dist: %f, angle_diff:%f, cost: %d",
                candidate.mx + this->robot_pose_.x,
                candidate.my + this->robot_pose_.y,
                candidate.distance,
                candidate.angle_diff,
                cost);
        }
        escape_target_ = candidate;
        this->state_log(MAIN_STATE::TURN, SUB_STATE::ESCAPE_TURN);
        this->main_state_ = MAIN_STATE::TURN;
        this->sub_state_ = SUB_STATE::ESCAPE_TURN;
    }
    else
    {
        std_msgs::msg::Bool msg;
        msg.data = true;
        this->force_escape_pub_->publish(msg);
        this->state_log(MAIN_STATE::STOP, SUB_STATE::FORCE_ESCAPE_WAIT);
        this->main_state_ = MAIN_STATE::STOP;
        this->sub_state_ = SUB_STATE::FORCE_ESCAPE_WAIT;
        this->force_escape_start_time_ = std::chrono::steady_clock::now();
        if (this->log_time_diff("escape_impossible_log", 1000))
        {
            RCLCPP_ERROR(this->get_logger(), "[Costmap escape] No available candidate for escape.");
        }
    }
}

/**
 * @brief state의 변경을 표시하는 함수
 */
void ManeuverNode::state_log(MAIN_STATE a_after, SUB_STATE b_after)
{
    std::ostringstream log_msg;
    log_msg << "[State change] => main_state : [" << enumToString(this->main_state_) << "] -> ["
            << enumToString(a_after) << "], sub_state : [" << enumToString(this->sub_state_) << "]";

    if (this->sub_state_ != b_after)
    {
        log_msg << " -> [" << enumToString(b_after) << "]";
    }

    RCLCPP_INFO(this->get_logger(), "%s", log_msg.str().c_str());
}

/**
 * @brief 동일한 이름을 가진 마지막 로그 이후로 주어진 시간 간격이 경과하면 true를 반환합니다.
 *
 * 이 기능은 로그 메시지의 빈도를 제한하는 데 사용됩니다. map을 사용하여 마지막 시간을 추적합니다.
 * 지정된 이름의 메시지가 기록되었습니다. 마지막 로그 이후의 시간이 지정된 간격보다 길면,
 * 함수는 true를 반환하고 현재 시간으로 map를 업데이트합니다.
 *
 * @param name 로그 메시지의 이름입니다.
 * @param interval_ms 밀리초 단위의 시간 간격.
 * 주어진 시간 간격이 경과하면 true을 반환하고, 그렇지 않으면 false을 반환합니다.
 */
bool ManeuverNode::log_time_diff(std::string name, double interval_ms)
{
    auto now = std::chrono::steady_clock::now();
    auto it = logTimeMap.find(name);
    bool result = false;
    auto interval = std::chrono::milliseconds(static_cast<int64_t>(interval_ms));

    if (it != logTimeMap.end())
    {
        auto diff = now - it->second;
        if (diff >= interval)
        {
            result = true;
            logTimeMap[name] = now;
        }
    }
    else
    {
        result = true;
        logTimeMap[name] = now;
    }
    return result;
}

}  // namespace A1::maneuver