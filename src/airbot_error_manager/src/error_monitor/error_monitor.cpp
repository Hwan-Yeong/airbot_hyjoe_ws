#include "error_monitor/error_monitor.hpp"

ErrorMonitor::ErrorMonitor()
    : Node("airbot_error_monitor")
{
    initVariables();

    // Subscriber
    bottom_status_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "bottom_status", 10, std::bind(&ErrorMonitor::bottomStatusCallback, this, std::placeholders::_1)
    );
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu_data", 10, std::bind(&ErrorMonitor::imuCallback, this, std::placeholders::_1)
    );
    
    // Publisher
    board_temperature_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/board_temp", 10);
    fall_down_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/fall_down", 10);

    // Timer
    timer_ = this->create_wall_timer(
        50ms,
        std::bind(&ErrorMonitor::errorMonitor, this));
}

ErrorMonitor::~ErrorMonitor()
{
}
void ErrorMonitor::initVariables()
{
    isBottomStatusUpdate = false;
    isImuUpdate = false;

    bottom_status_data = std_msgs::msg::UInt8();
    imu_data = sensor_msgs::msg::Imu();
}

void ErrorMonitor::errorMonitor()
{
    std_msgs::msg::Bool error_msg;

    // board temperature monitor
    if(board_temperature_monitor_.errorMonitor()) {
        error_msg.data = true;
        fall_down_error_pub_->publish(error_msg);
    } else {
        // 해제 조건 결정되면 false로 보내든가...
    }

    // fall down monitor
    if (isBottomStatusUpdate && isImuUpdate) {
        if (fall_down_monitor_.errorMonitor(bottom_status_data, imu_data)) {
            error_msg.data = true;
            fall_down_error_pub_->publish(error_msg);
        } else {
            // 해제 조건 결정되면 false로 보내든가...
        }
        isBottomStatusUpdate = false;
        isImuUpdate = false;
    }
}

void ErrorMonitor::bottomStatusCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    bottom_status_data = *msg;
    isBottomStatusUpdate = true;
}

void ErrorMonitor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_data = *msg;
    isImuUpdate = true;
}