#include "maneuver.hpp"

namespace A1::maneuver
{

ManeuverNode::ManeuverNode() : Node("A1_maneuver")
{
    this->declare_parameter("time_ms.timer", 100);
    int timer_ms = this->get_parameter("time_ms.timer").as_int();

    this->init_params();
    this->init_subs();
    this->current_velocity_ = 0;
    this->cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    this->status_pub_ = this->create_publisher<std_msgs::msg::String>("/maneuver/status", 1);

    this->timer_ =
        this->create_wall_timer(std::chrono::milliseconds(timer_ms), std::bind(&ManeuverNode::timer_callback, this));
}

void ManeuverNode::init_params(void)
{
    this->action_state_ = ACTION_STATE::IDLE;
    this->navi_state_ = NAVI_STATE::IDLE;
    this->stop_state_ = STOP_STATE::NO_STOP;

    this->back_object_detect_ = false;
    this->front_escape_detect_ = false;

    this->declare_parameter("time_ms.wait.normal", 1000);
    this->declare_parameter("time_ms.wait.drop_off", 2000);
    this->declare_parameter("time_ms.back.1d", 1500);
    this->declare_parameter("time_ms.back.abort", 1500);

    this->declare_parameter("lidar.escape.front.distance", 0.3);
    this->declare_parameter("lidar.escape.front.angle", 60.0);
    this->declare_parameter("lidar.escape.back.distance", 0.3);
    this->declare_parameter("lidar.escape.back.angle", 45.0);

    this->declare_parameter("velocity_scaling_factor", 0.05);

    // this->declare_parameter("speed.back", 0.1);
    // this->declare_parameter("speed.front", 0.1);

    this->get_parameter("time_ms.wait.normal", this->wait_ms_normal_);
    this->get_parameter("time_ms.wait.drop_off", this->wait_ms_drop_off_);
    this->get_parameter("time_ms.back.1d", this->back_ms_1d_);
    this->get_parameter("time_ms.back.abort", this->back_ms_abort_);

    this->get_parameter("lidar.escape.front.distance", this->lidar_front_escape_dist_);
    this->get_parameter("lidar.escape.front.angle", this->lidar_front_escape_angle_);
    this->get_parameter("lidar.escape.back.distance", this->lidar_back_escape_dist_);
    this->get_parameter("lidar.escape.back.angle", this->lidar_back_escape_angle_);

    this->get_parameter("velocity_scaling_factor", this->velocity_scaling_factor_);
}

void ManeuverNode::init_subs(void)
{
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.best_effort();

    // [EB]Navigation status
    this->subscribers["navi_state"] = this->create_subscription<robot_custom_msgs::msg::NaviState>(
        "/navi_datas", qos_profile, std::bind(&ManeuverNode::navi_state_callback, this, std::placeholders::_1));

    // [EB]Motor status
    this->subscribers["motor_status"] = this->create_subscription<robot_custom_msgs::msg::MotorStatus>(
        "/motor_status", qos_profile, std::bind(&ManeuverNode::motor_status_callback, this, std::placeholders::_1));

    // [EB]Drop off IR sensors
    this->subscribers["bottom_lr_data"] = this->create_subscription<robot_custom_msgs::msg::BottomIrData>(
        "/bottom_ir_data", qos_profile, std::bind(&ManeuverNode::bottom_ir_callback, this, std::placeholders::_1));

    // [CB]Perception command
    this->subscribers["action_stop"] = this->create_subscription<std_msgs::msg::Int8>(
        "/perception/action/stop",
        qos_profile,
        std::bind(&ManeuverNode::action_stop_callback, this, std::placeholders::_1));

    // [EB] turn on/off maneuver action
    this->subscribers["maneuver_use"] = this->create_subscription<std_msgs::msg::Bool>(
        "/maneuver/use", qos_profile, std::bind(&ManeuverNode::maneuver_use_callback, this, std::placeholders::_1));

    // [EB]Lidar
    this->subscribers["scan"] = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos_profile, std::bind(&ManeuverNode::scan_callback, this, std::placeholders::_1));

    // [EB]Collision pcd
    this->subscribers["collision"] = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/sensor_to_pointcloud/collision",
        qos_profile,
        std::bind(&ManeuverNode::collision_callback, this, std::placeholders::_1));

    // [NAV2] cmd_vel
    this->subscribers["cmd_vel_nav"] = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_smoothed", qos_profile, std::bind(&ManeuverNode::cmd_vel_nav_callback, this, std::placeholders::_1));
}

void ManeuverNode::maneuver_use_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    this->use_maneuver_action_ = msg->data;

    if (this->use_maneuver_action_)
    {
        if (!this->timer_ || !this->timer_->is_canceled())
        {
            int timer_ms = this->get_parameter("time_ms.timer").as_int();
            this->timer_ = this->create_wall_timer(
                std::chrono::milliseconds(timer_ms), std::bind(&ManeuverNode::timer_callback, this));
        }
    }
    else
    {
        this->timer_.reset();
    }
}

void ManeuverNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    float front_threshold = this->lidar_front_escape_dist_;
    float front_angle_threshold = this->lidar_front_escape_angle_;
    bool front_obstacle_detect = false;
    float back_threshold = this->lidar_back_escape_dist_;
    float back_angle_threshold = this->lidar_back_escape_angle_;
    bool back_obstacle_detect = false;

    int num_readings = msg->ranges.size();
    float angle = msg->angle_min;

    for (int i = 0; i < num_readings; i++, angle += msg->angle_increment)
    {
        double dist = msg->ranges[i];

        if (std::isnan(dist) || std::isinf(dist) || dist <= msg->range_min || dist >= msg->range_max)
        {
            continue;  // 무효한 데이터는 건너뜀
        }

        float angle_deg = angle * 180.0f / M_PI;  // 라디안 -> 도(degree) 변환

        // 전방 장애물 감지 (-front_angle_threshold ~ +front_angle_threshold)
        if (std::abs(angle_deg) <= front_angle_threshold && dist <= front_threshold)
        {
            front_obstacle_detect = true;
        }
        // 후방 장애물 감지 (180도 방향 기준으로 back_angle_threshold 내에 있음)
        if (std::abs(angle_deg) >= (180.0f - back_angle_threshold) && dist <= back_threshold)
        {
            back_obstacle_detect = true;
        }
    }

    this->front_escape_detect_ = front_obstacle_detect;
    this->back_object_detect_ = back_obstacle_detect;
}

void ManeuverNode::cmd_vel_nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (is_action_idle())
    {
        double speed = this->current_velocity_ + this->velocity_scaling_factor_;
        if (msg->linear.x > speed)
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

void ManeuverNode::navi_state_callback(const robot_custom_msgs::msg::NaviState::SharedPtr msg)
{
    NAVI_STATE current_state = static_cast<NAVI_STATE>(msg->state);
    if (this->navi_state_ != NAVI_STATE::FAIL && current_state == NAVI_STATE::FAIL)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Navi State: %s, Fail_reason: %s",
            enumToString(current_state).c_str(),
            enumToString(static_cast<NAVI_FAIL_REASON>(msg->fail_reason)).c_str());
        if (this->back_object_detect_ == false)
        {
            RCLCPP_INFO(this->get_logger(), "Start abort process. move back.");
            // robot_back(this->back_ms_abort_);
            robot_state(this->back_ms_abort_, ACTION_STATE::BACK);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Start abort process. But cant move.");
            // robot_wait(1500);
            robot_state(1500, ACTION_STATE::WAIT);
        }
    }
    this->navi_state_ = current_state;
}

/**
 * IR sensors
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
    bool ff = msg->ff;
    bool fl = msg->fl;
    bool fr = msg->fr;
    bool bb = msg->bb;
    bool bl = msg->bl;
    bool br = msg->br;
    auto cmd_vel = this->robot_action_default();
    bool pub_cmd = false;
    if (ff || fr || fl || bb || bl || br)
    {
        pub_cmd = true;
    }
    else
    {
        if (this->action_state_ == ACTION_STATE::DROP_IR)
        {
            this->state_log(ACTION_STATE::DROP_IR, ACTION_STATE::WAIT, STOP_STATE::NO_STOP, STOP_STATE::NO_STOP);
            this->robot_state(500, ACTION_STATE::WAIT);
            // robot_wait(500);
        }
        return;
    }

    float speed = 0.1;
    float angle = 0.1;
    if (ff && fr)
    {
        // 0, 1
        cmd_vel.linear.x = -speed;
        cmd_vel.angular.z = angle;
    }
    else if (ff && fl)
    {
        // 0, 5
        cmd_vel.linear.x = -speed;
        cmd_vel.angular.z = -angle;
    }
    else if ((fr && br) || (fl && bl))
    {
        // 1, 2
        // 4, 5
        // stop //
    }
    else if (bb && br)
    {
        // 2, 3
        cmd_vel.linear.x = speed;
        cmd_vel.angular.z = -angle;
    }
    else if (bb && bl)
    {
        // 3, 4
        cmd_vel.linear.x = speed;
        cmd_vel.angular.z = angle;
    }
    else if (ff)
    {
        cmd_vel.linear.x = -speed;
    }
    else if (fl)
    {
        cmd_vel.linear.x = -speed;
        cmd_vel.angular.z = -angle;
    }
    else if (fr)
    {
        cmd_vel.linear.x = -speed;
        cmd_vel.angular.z = angle;
    }
    else if (br)
    {
        cmd_vel.linear.x = speed;
        cmd_vel.angular.z = -angle;
    }
    else if (bb)
    {
        cmd_vel.linear.x = speed;
    }
    else if (bl)
    {
        cmd_vel.linear.x = speed;
        cmd_vel.angular.z = angle;
    }

    if (pub_cmd)
    {
        this->state_log(action_state_, ACTION_STATE::DROP_IR, STOP_STATE::NO_STOP, STOP_STATE::NO_STOP);
        this->action_state_ = ACTION_STATE::DROP_IR;
        this->cmd_vel_pub_->publish(cmd_vel);
    }
}

/***
 0 : STOP_STATE::NO_STOP
 1 : STOP_STATE::DROP_OFF
 2 : STOP_STATE::ONE_D_TOF
 */
void ManeuverNode::action_stop_callback(const std_msgs::msg::Int8::SharedPtr msg)
{
    if (is_action_idle())  // && (motor_status_.isMoveToFoward() || motor_status_.isRotate()))
    {
        this->state_log(
            ACTION_STATE::IDLE, ACTION_STATE::STOP, STOP_STATE::NO_STOP, static_cast<STOP_STATE>(msg->data));
        if (msg->data == 1)
        {
            this->action_state_ = ACTION_STATE::STOP;
            this->stop_state_ = STOP_STATE::DROP_OFF;
        }
        else if (msg->data == 2)
        {
            this->action_state_ = ACTION_STATE::STOP;
            this->stop_state_ = STOP_STATE::ONE_D_TOF;
        }
        else
        {
            this->action_state_ = ACTION_STATE::IDLE;
            this->stop_state_ = STOP_STATE::NO_STOP;
        }
    }
}

/***
 * Motor status -> rpm
 */
void ManeuverNode::motor_status_callback(const robot_custom_msgs::msg::MotorStatus::SharedPtr msg)
{
    this->motor_status_.setLeftRpm(msg->left_motor_rpm);
    this->motor_status_.setRightRpm(msg->right_motor_rpm);
    this->current_velocity_ = this->motor_status_.getVelocity();
}

/***
 * airbot_sensor_manager
 * collision -> pointcloud
 */
void ManeuverNode::collision_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (msg->width > 0)
    {
        RCLCPP_INFO(this->get_logger(), "Collision ! move back.");
        // robot_back(2000);
        robot_state(2000, ACTION_STATE::BACK);
    }
}

void ManeuverNode::timer_callback()
{
    rclcpp::Time current_time = this->get_clock()->now();
    // 뒤로가기 상태
    bool is_publish = false;
    geometry_msgs::msg::Twist cmd_vel = this->robot_action_default();
    if (this->action_state_ == ACTION_STATE::BACK)
    {
        if (current_time - this->action_start_time_ < this->action_process_time_)
        {
            robot_action_back(cmd_vel, -0.1);
            is_publish = true;
        }
        else
        {
            this->state_log(ACTION_STATE::BACK, ACTION_STATE::WAIT, STOP_STATE::NO_STOP, STOP_STATE::NO_STOP);
            this->robot_state(this->wait_ms_normal_, ACTION_STATE::WAIT);
            // this->robot_wait(this->wait_ms_normal_);
        }
    }
    else if (this->action_state_ == ACTION_STATE::FRONT)
    {
        if (current_time - this->action_start_time_ < this->action_process_time_)
        {
            robot_action_front(cmd_vel, -0.1);
            is_publish = true;
        }
        else
        {
            this->state_log(ACTION_STATE::FRONT, ACTION_STATE::WAIT, STOP_STATE::NO_STOP, STOP_STATE::NO_STOP);
            this->robot_state(this->wait_ms_normal_, ACTION_STATE::WAIT);
            // this->robot_wait(this->wait_ms_normal_);
        }
    }
    else if (this->action_state_ == ACTION_STATE::STOP && is_stop_state())
    {
        bool isMinRpm = this->motor_status_.isLowRpm();
        bool isRotate = this->motor_status_.isRotate();

        if (isMinRpm || isRotate)
        {
            if (this->stop_state_ == STOP_STATE::ONE_D_TOF)
            {
                RCLCPP_INFO(this->get_logger(), "1D detect. Go to back.");
                this->state_log(ACTION_STATE::STOP, ACTION_STATE::BACK, STOP_STATE::ONE_D_TOF, STOP_STATE::NO_STOP);
                // this->robot_back(this->back_ms_1d_);
                this->robot_state(this->back_ms_1d_, ACTION_STATE::BACK);
            }
            else
            {
                this->state_log(ACTION_STATE::STOP, ACTION_STATE::WAIT, STOP_STATE::DROP_OFF, STOP_STATE::NO_STOP);
                // this->robot_wait(this->wait_ms_drop_off_);
                this->robot_state(this->wait_ms_drop_off_, ACTION_STATE::WAIT);
            }
        }
        else
        {
            is_publish = true;
        }
    }
    else if (this->action_state_ == ACTION_STATE::WAIT)
    {
        if (current_time - this->action_start_time_ > this->action_process_time_)
        {
            this->state_log(ACTION_STATE::WAIT, ACTION_STATE::IDLE, STOP_STATE::NO_STOP, STOP_STATE::NO_STOP);
            this->action_state_ = ACTION_STATE::IDLE;
            this->stop_state_ = STOP_STATE::NO_STOP;
        }
        else
        {
            is_publish = true;
        }
    }

    if (this->front_escape_detect_ && (motor_status_.isMoveToFoward() || motor_status_.isRotate()))
    {
        RCLCPP_INFO(this->get_logger(), "Front escape detected");
        this->robot_state(2000, ACTION_STATE::BACK);
        // this->robot_back(2000);
    }

    if (is_publish)
    {
        this->cmd_vel_pub_->publish(cmd_vel);
    }
    this->pub_states();
}

void ManeuverNode::pub_states()
{
    std::ostringstream formatted;
    formatted << "Use:" << (use_maneuver_action_ ? "Use" : "Not use") << "/";
    formatted << " Action:" << enumToString(action_state_).c_str() << "/";
    formatted << " Stop:" << enumToString(stop_state_).c_str() << "/";
    formatted << " Navi:" << enumToString(navi_state_).c_str();
    std_msgs::msg::String msg;
    msg.data = formatted.str();
    this->status_pub_->publish(msg);
}

bool ManeuverNode::is_action_idle()
{
    return this->action_state_ == ACTION_STATE::IDLE && !is_stop_state();
}

bool ManeuverNode::is_stop_state()
{
    return this->stop_state_ != STOP_STATE::NO_STOP;
}

geometry_msgs::msg::Twist ManeuverNode::robot_action_default(void)
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

void ManeuverNode::robot_action_back(geometry_msgs::msg::Twist& cmd_vel, const double speed)
{
    if (this->back_object_detect_ == false)
    {
        cmd_vel.linear.x = speed;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Cannot move back. stop.");
    }
}

void ManeuverNode::robot_action_front(geometry_msgs::msg::Twist& cmd_vel, const double speed)
{
    cmd_vel.linear.x = speed;
}

void ManeuverNode::robot_state(const double process_time_ms, const ACTION_STATE state)
{
    this->action_start_time_ = this->get_clock()->now();
    this->action_process_time_ = rclcpp::Duration::from_seconds(process_time_ms / 1000.0);
    this->action_state_ = state;
}

void ManeuverNode::state_log(ACTION_STATE a_before, ACTION_STATE a_after, STOP_STATE b_before, STOP_STATE b_after)
{
    std::ostringstream log_msg;
    log_msg << "state change => action_state : [" << enumToString(a_before) << "] -> [" << enumToString(a_after)
            << "], stop_state : [" << enumToString(b_before) << "]";

    if (b_before != b_after)
    {
        log_msg << " -> [" << enumToString(b_after) << "]";
    }

    RCLCPP_INFO(this->get_logger(), "%s", log_msg.str().c_str());
}

}  // namespace A1::maneuver