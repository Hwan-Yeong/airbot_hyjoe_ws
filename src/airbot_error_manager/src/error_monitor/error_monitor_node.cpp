#include "error_monitor/error_monitor_node.hpp"

ErrorMonitorNode::ErrorMonitorNode()
    : Node("airbot_error_monitor")
{
    initVariables();

    addMonitor<BatteryErrorMonitor::InputType, BatteryErrorMonitor>(std::make_shared<BatteryErrorMonitor>());
    addMonitor<FallDownErrorMonitor::InputType, FallDownErrorMonitor>(std::make_shared<FallDownErrorMonitor>());
    addMonitor<BoardTemperatureErrorMonitor::InputType, BoardTemperatureErrorMonitor>(std::make_shared<BoardTemperatureErrorMonitor>());

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
    board_temperature_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/board_temp", 10);
    fall_down_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/fall_down", 20);
    battery_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/battery", 10);

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

    bottom_status_data = robot_custom_msgs::msg::BottomIrData();
    imu_data = sensor_msgs::msg::Imu();
    battery_data = robot_custom_msgs::msg::BatteryStatus();
}

void ErrorMonitorNode::errorMonitor()
{
    std_msgs::msg::Bool error_msg;

    // battery monitor
    if (isBatteryUpdate) {
        bool battery_error = this->checkAllErrors(battery_data);
        if (battery_error) {
            RCLCPP_INFO(this->get_logger(), "battery_error : %s", battery_error ? "true" : "false");
            error_msg.data = true;
            battery_error_pub_->publish(error_msg);
        } else {
            error_msg.data = false;
            battery_error_pub_->publish(error_msg);
        }

        isBatteryUpdate = false; // 상태 초기화
    }

    // fall down monitor
    if (isBottomStatusUpdate && isImuUpdate) {
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

    // board temperature monitor
    bool board_temperature_eror = this->checkAllErrors(std::nullptr_t());
    if (board_temperature_eror) {
        RCLCPP_INFO(this->get_logger(), "board_temperature_eror : %s", board_temperature_eror ? "true" : "false");
        error_msg.data = true;
        board_temperature_error_pub_->publish(error_msg);
    } else {
        error_msg.data = false;
        board_temperature_error_pub_->publish(error_msg);
    }
}

void ErrorMonitorNode::batteryCallback(const robot_custom_msgs::msg::BatteryStatus::SharedPtr msg)
{
    count++;
    if (count >= 3000) { // 30초 이상일 경우
        battery_data = *msg;
        isBatteryUpdate = true;
        count = 0;
    } else {
        isBatteryUpdate = false;
    }
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
