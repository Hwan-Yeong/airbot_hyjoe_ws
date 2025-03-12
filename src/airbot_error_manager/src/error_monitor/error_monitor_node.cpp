#include "error_monitor/error_monitor_node.hpp"

ErrorMonitorNode::ErrorMonitorNode()
    : Node("airbot_error_monitor")
{
    initVariables();

    addMonitor<LowBatteryErrorMonitor::InputType, LowBatteryErrorMonitor>(std::make_shared<LowBatteryErrorMonitor>());
    addMonitor<FallDownErrorMonitor::InputType, FallDownErrorMonitor>(std::make_shared<FallDownErrorMonitor>());
    addMonitor<BoardOverheatErrorMonitor::InputType, BoardOverheatErrorMonitor>(std::make_shared<BoardOverheatErrorMonitor>());

    // Subscriber
    bottom_status_sub_ = this->create_subscription<robot_custom_msgs::msg::BottomIrData>(
        "bottom_status", 10, std::bind(&ErrorMonitorNode::bottomStatusCallback, this, std::placeholders::_1)
    );
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu_data", 10, std::bind(&ErrorMonitorNode::imuCallback, this, std::placeholders::_1)
    );
    battery_status_sub_ = this->create_subscription<robot_custom_msgs::msg::BatteryStatus>(
        "/battery_status", 10, std::bind(&ErrorMonitorNode::batteryCallback, this, std::placeholders::_1)
    );

    // Publisher
    board_overheat_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/board_overheat", 10);
    fall_down_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/fall_down", 20);
    low_battery_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/low_battery", 10);

    // Timer
    timer_ = this->create_wall_timer(
        10ms,
        std::bind(&ErrorMonitorNode::errorMonitor, this));
}

ErrorMonitorNode::~ErrorMonitorNode()
{
}
void ErrorMonitorNode::initVariables()
{
    isBottomStatusUpdate = false;
    isImuUpdate = false;
    isBatteryUpdate = false;

    publish_cnt_low_battery_error_ = 0;
    publish_cnt_fall_down_error_ = 0;
    publish_cnt_board_overheat_error_ = 0;

    bottom_status_data = robot_custom_msgs::msg::BottomIrData();
    imu_data = sensor_msgs::msg::Imu();
    battery_data = robot_custom_msgs::msg::BatteryStatus();
}

void ErrorMonitorNode::errorMonitor()
{
    std_msgs::msg::Bool error_msg;

    publish_cnt_low_battery_error_ +=10;
    publish_cnt_fall_down_error_ +=10;
    publish_cnt_board_overheat_error_ +=10;

    // low battery monitor
    if (isBatteryUpdate && (publish_cnt_low_battery_error_ >= 30000)) { // 30초
        bool low_battery_error = this->checkAllErrors(battery_data);
        if (low_battery_error) {
            RCLCPP_INFO(this->get_logger(), "low_battery_error : %s", low_battery_error ? "true" : "false");
            error_msg.data = true;
            low_battery_error_pub_->publish(error_msg);
        } else {
            error_msg.data = false;
            low_battery_error_pub_->publish(error_msg);
        }

        isBatteryUpdate = false; // 상태 초기화
    }

    // fall down monitor
    if (isBottomStatusUpdate && isImuUpdate && (publish_cnt_fall_down_error_ >= 1000)) { // 1초
        bool fall_down_error = this->checkAllErrors(std::make_pair(bottom_status_data, imu_data));
        if (fall_down_error) {
            RCLCPP_INFO(this->get_logger(), "fall_down_error : %s", fall_down_error ? "true" : "false");
            error_msg.data = true;
            fall_down_error_pub_->publish(error_msg);
        } else {
            error_msg.data = false;
            fall_down_error_pub_->publish(error_msg);
        }

        isBottomStatusUpdate = false;
        isImuUpdate = false;
    }

    // board overheat monitor
    if (publish_cnt_board_overheat_error_ >= 1000) { // 1초
        bool board_overheat_error = this->checkAllErrors(std::nullptr_t());
        if (board_overheat_error) {
            RCLCPP_INFO(this->get_logger(), "board_overheat_error : %s", board_overheat_error ? "true" : "false");
            error_msg.data = true;
            board_overheat_error_pub_->publish(error_msg);
        } else {
            error_msg.data = false;
            board_overheat_error_pub_->publish(error_msg);
        }
    }

    if (publish_cnt_low_battery_error_ >= 10000) publish_cnt_low_battery_error_ = 0;
    if (publish_cnt_fall_down_error_ >= 10000) publish_cnt_fall_down_error_ = 0;
    if (publish_cnt_board_overheat_error_ >= 10000) publish_cnt_board_overheat_error_ = 0;
}

void ErrorMonitorNode::batteryCallback(const robot_custom_msgs::msg::BatteryStatus::SharedPtr msg)
{
    battery_data = *msg;
    isBatteryUpdate = true;
}

void ErrorMonitorNode::bottomStatusCallback(const robot_custom_msgs::msg::BottomIrData::SharedPtr msg)
{
    bottom_status_data = *msg;
    isBottomStatusUpdate = true;
}

void ErrorMonitorNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_data = *msg;
    isImuUpdate = true;
}
