#include "maneuver.hpp"

namespace A1::maneuver
{

ManeuverNode::ManeuverNode() : Node("A1_maneuver")
{
    action_state_ = ACTION_STATE::IDLE;
    navi_state_ = NAVI_STATE::IDLE;
    stop_state_ = STOP_STATE::NO_STOP;

    back_object_detect_ = false;
    front_escape_detect_ = false;

    this->declare_parameter("time_ms.timer", 100);
    this->declare_parameter("time_ms.wait.normal", 1000);
    this->declare_parameter("time_ms.wait.drop_off", 2000);
    this->declare_parameter("time_ms.back.1d", 1500);
    this->declare_parameter("time_ms.back.abort", 1500);

    // this->declare_parameter("speed.back", 0.1);
    // this->declare_parameter("speed.front", 0.1);

    int timer_ms = this->get_parameter("time_ms.timer").as_int();
    wait_ms_normal_ = this->get_parameter("time_ms.wait.normal").as_int();
    wait_ms_drop_off_ = this->get_parameter("time_ms.wait.drop_off").as_int();
    back_ms_1d_ = this->get_parameter("time_ms.back.1d").as_int();
    back_ms_abort_ = this->get_parameter("time_ms.back.abort").as_int();

    this->declare_parameter("lidar.distance.escape.front", 0.1);
    this->get_parameter("lidar.distance.escape.front", lidar_front_escape_dist_);

    this->declare_parameter("lidar.distance.abort.back", 0.3);
    this->get_parameter("lidar.distance.abort.back", lidar_back_abort_dist_);

    this->declare_parameter("velocity_scaling_factor", 0.05);
    this->get_parameter("velocity_scaling_factor", velocity_scaling_factor_);

    this->init_subs();
    current_velocity_ = 0;
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(timer_ms), std::bind(&ManeuverNode::timer_callback, this));
}

void ManeuverNode::init_subs(void)
{
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.best_effort();
    // From UDP_Interface /navi_datas topic
    this->subscribers["navi_state"] = this->create_subscription<robot_custom_msgs::msg::NaviState>(
        "/navi_datas", qos_profile, std::bind(&ManeuverNode::navi_state_callback, this, std::placeholders::_1));

    this->subscribers["motor_status"] = this->create_subscription<robot_custom_msgs::msg::MotorStatus>(
        "/motor_status", qos_profile, std::bind(&ManeuverNode::motor_status_callback, this, std::placeholders::_1));

    this->subscribers["action_stop"] = this->create_subscription<std_msgs::msg::Int32>(
        "/perception/action/stop",
        qos_profile,
        std::bind(&ManeuverNode::action_stop_callback, this, std::placeholders::_1));

    this->subscribers["scan_back"] = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan_back", qos_profile, std::bind(&ManeuverNode::scan_back_callback, this, std::placeholders::_1));

    this->subscribers["scan_front"] = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan_front", qos_profile, std::bind(&ManeuverNode::scan_front_callback, this, std::placeholders::_1));

    this->subscribers["cmd_vel_nav"] = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_smoothed", qos_profile, std::bind(&ManeuverNode::cmd_vel_nav_callback, this, std::placeholders::_1));
}

void ManeuverNode::scan_back_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    float angle_increment = msg->angle_increment;
    std::vector<float> ranges = msg->ranges;
    int total_readings = ranges.size();
    int mid_index = total_readings / 2;

    float angle_range = M_PI / 12;  // +- 30도
    int range_offset = angle_range / angle_increment;

    int start_index = std::max(0, mid_index - range_offset);
    int end_index = std::min(total_readings - 1, mid_index + range_offset);

    bool obstacle_detected = false;
    for (int i = start_index; i <= end_index; i++)
    {
        if (ranges[i] < lidar_back_abort_dist_ && ranges[i] > 0)
        {
            obstacle_detected = true;
            break;
        }
    }
    back_object_detect_ = obstacle_detected;
}

void ManeuverNode::scan_front_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    float angle_increment = msg->angle_increment;
    std::vector<float> ranges = msg->ranges;
    int total_readings = ranges.size();
    int mid_index = total_readings / 2;

    float angle_range = M_PI / 6;  // +- 30도
    int range_offset = angle_range / angle_increment;

    int start_index = std::max(0, mid_index - range_offset);
    int end_index = std::min(total_readings - 1, mid_index + range_offset);
    bool obstacle_escape_detected = false;
    for (int i = start_index; i <= end_index; i++)
    {
        if (ranges[i] < lidar_front_escape_dist_ && ranges[i] > 0)
        {
            obstacle_escape_detected = true;
            break;
        }
    }
    front_escape_detect_ = obstacle_escape_detected;
}

void ManeuverNode::cmd_vel_nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (is_action_idle())
    {
        double speed = current_velocity_ + velocity_scaling_factor_;
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
    if (navi_state_ != NAVI_STATE::FAIL && current_state == NAVI_STATE::FAIL)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "%s():%d: Navi State: %s, Fail_reason: %s",
            __FUNCTION__,
            __LINE__,
            enumToString(current_state).c_str(),
            enumToString(static_cast<NAVI_FAIL_REASON>(msg->fail_reason)).c_str());
        if (back_object_detect_ == false)
        {
            RCLCPP_INFO(this->get_logger(), "%s():%d: Start abort process. move back.", __FUNCTION__, __LINE__);
            robot_back(back_ms_abort_);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "%s():%d: Start abort process. But cant move.", __FUNCTION__, __LINE__);
            robot_wait(1500);
        }
    }
    navi_state_ = current_state;
}

/***
 0 : STOP_STATE::NO_STOP
 1 : STOP_STATE::DROP_OFF
 2 : STOP_STATE::ONE_D_TOF
 */
void ManeuverNode::action_stop_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (is_action_idle())  // && (motor_status_.isMoveToFoward() || motor_status_.isRotate()))
    {
        RCLCPP_INFO(
            this->get_logger(),
            "%s():%d: state change => action_state : [Idle]-> "
            "[Stop], stop_state : [No stop] -> [%s]",
            __FUNCTION__,
            __LINE__,
            enumToString(static_cast<STOP_STATE>(msg->data)).c_str());
        if (msg->data == 1)
        {
            action_state_ = ACTION_STATE::STOP;
            stop_state_ = STOP_STATE::DROP_OFF;
        }
        else if (msg->data == 2)
        {
            action_state_ = ACTION_STATE::STOP;
            stop_state_ = STOP_STATE::ONE_D_TOF;
        }
        else
        {
            action_state_ = ACTION_STATE::IDLE;
            stop_state_ = STOP_STATE::NO_STOP;
        }
    }
}

void ManeuverNode::motor_status_callback(const robot_custom_msgs::msg::MotorStatus::SharedPtr msg)
{
    this->motor_status_.setLeftRpm(msg->left_motor_rpm);
    this->motor_status_.setRightRpm(msg->right_motor_rpm);
    current_velocity_ = this->motor_status_.getVelocity();
}

void ManeuverNode::timer_callback()
{
    rclcpp::Time current_time = this->get_clock()->now();
    // 뒤로가기 상태
    if (action_state_ == ACTION_STATE::BACK)
    {
        if (current_time - action_start_time_ < action_process_time_)
        {
            robot_action_back(-0.1);
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "%s():%d: state change => action_state : [Back]-> "
                "[Wait], stop_state : [No stop]",
                __FUNCTION__,
                __LINE__);
            robot_wait(wait_ms_normal_);
        }
    }
    else if (action_state_ == ACTION_STATE::FRONT)
    {
        if (current_time - action_start_time_ < action_process_time_)
        {
            robot_action_front(0.1);
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "%s():%d: state change => action_state : [Front]-> "
                "[Wait], stop_state : [No stop]",
                __FUNCTION__,
                __LINE__);
            robot_wait(wait_ms_normal_);
        }
    }
    else if (action_state_ == ACTION_STATE::STOP && is_stop_state())
    {
        bool isMinRpm = this->motor_status_.isLowRpm();
        bool isRotate = this->motor_status_.isRotate();

        if (isMinRpm || isRotate)
        {
            if (stop_state_ == STOP_STATE::ONE_D_TOF)
            {
                robot_back(back_ms_1d_);
                RCLCPP_INFO(this->get_logger(), "%s():%d: 1D detect. Go to back.", __FUNCTION__, __LINE__);
                RCLCPP_INFO(
                    this->get_logger(),
                    "%s():%d: state change => action_state : [Stop]-> "
                    "[Back], stop_state : [1D ToF] -> [No stop]",
                    __FUNCTION__,
                    __LINE__);
            }
            else
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "%s():%d: state change => action_state : [Stop]-> "
                    "[Wait], stop_state : [Drop off] -> [No stop]",
                    __FUNCTION__,
                    __LINE__);
                robot_wait(wait_ms_drop_off_);
            }
        }
        else
        {
            robot_action_stop();
        }
    }
    else if (action_state_ == ACTION_STATE::WAIT)
    {
        if (current_time - action_start_time_ > action_process_time_)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "%s():%d: state change => action_state : [Wait]-> "
                "[Idle], stop_state : [No stop]",
                __FUNCTION__,
                __LINE__);
            action_state_ = ACTION_STATE::IDLE;
            stop_state_ = STOP_STATE::NO_STOP;
        }
        else
        {
            robot_action_stop();
        }
    }

    if (front_escape_detect_ && (motor_status_.isMoveToFoward() || motor_status_.isRotate()))
    {
        RCLCPP_INFO(this->get_logger(), "%s():%d: Front escape detected", __FUNCTION__, __LINE__);
        robot_back(2000);
    }
}

bool ManeuverNode::is_action_idle()
{
    return action_state_ == ACTION_STATE::IDLE && !is_stop_state();
}

bool ManeuverNode::is_stop_state()
{
    return stop_state_ != STOP_STATE::NO_STOP;
}

void ManeuverNode::robot_action_stop(void)
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel);
}

void ManeuverNode::robot_action_back(const double speed)
{
    if (back_object_detect_ == false)
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = speed;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "%s():%d: Cannot move back.", __FUNCTION__, __LINE__);
    }
}

void ManeuverNode::robot_action_front(const double speed)
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = speed;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel);
}

void ManeuverNode::robot_back(const double process_time_ms)
{
    action_start_time_ = this->get_clock()->now();
    action_process_time_ = rclcpp::Duration::from_seconds(process_time_ms / 1000.0);
    action_state_ = ACTION_STATE::BACK;
}

void ManeuverNode::robot_front(const double process_time_ms)
{
    action_start_time_ = this->get_clock()->now();
    action_process_time_ = rclcpp::Duration::from_seconds(process_time_ms / 1000.0);
    action_state_ = ACTION_STATE::FRONT;
}

void ManeuverNode::robot_wait(const double process_time_ms)
{
    action_start_time_ = this->get_clock()->now();
    action_process_time_ = rclcpp::Duration::from_seconds(process_time_ms / 1000.0);
    action_state_ = ACTION_STATE::WAIT;
}

}  // namespace A1::maneuver