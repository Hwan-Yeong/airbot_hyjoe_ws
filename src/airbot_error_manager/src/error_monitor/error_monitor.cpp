#include "error_monitor/error_monitor.hpp"

ErrorMonitor::ErrorMonitor()
    : Node("airbot_error_monitor")
{
    initVariables();

    // Subscriber
    bottom_status_sub_ = this->create_subscription<robot_custom_msgs::msg::BottomIrData>(
        "bottom_status", 10, std::bind(&ErrorMonitor::bottomStatusCallback, this, std::placeholders::_1)
    );
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu_data", 10, std::bind(&ErrorMonitor::imuCallback, this, std::placeholders::_1)
    );
    battery_status_sub_ = this->create_subscription<robot_custom_msgs::msg::BatteryStatus>(
        "/battery_status", 10, std::bind(&ErrorMonitor::batteryCallback, this, std::placeholders::_1)
    );

    // Publisher
    board_temperature_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/board_temp", 10);
    fall_down_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/fall_down", 20);
    battery_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/battery", 10);

    // Timer
    timer_ = this->create_wall_timer(
        10ms,
        std::bind(&ErrorMonitor::errorMonitor, this));
}

ErrorMonitor::~ErrorMonitor()
{
}
void ErrorMonitor::initVariables()
{
    isBottomStatusUpdate = false;
    isImuUpdate = false;

    bottom_status_data = robot_custom_msgs::msg::BottomIrData();
    imu_data = sensor_msgs::msg::Imu();
    battery_data = robot_custom_msgs::msg::BatteryStatus();
}

void ErrorMonitor::errorMonitor()
{
    std_msgs::msg::Bool error_msg;

/*
    // board temperature monitor
    if(board_temperature_monitor_.errorMonitor()) {
        error_msg.data = false;  // true
        board_temperature_error_pub_->publish(error_msg);
    } else {
        // 해제 조건 결정되면 false로 보내든가...
    }
*/

    // fall down monitor
    if (isBottomStatusUpdate && isImuUpdate) {
        if (fall_down_monitor_.errorMonitor(bottom_status_data, imu_data))
        {
            error_msg.data = true;    // 원래 true
            fall_down_error_pub_->publish(error_msg);
        } else {
            // 해제 조건 결정되면 false로 보내든가... 추후 수정
            error_msg.data = false;
            fall_down_error_pub_->publish(error_msg);
        }
        isBottomStatusUpdate = false;
        isImuUpdate = false;
    }

    // battery monitor
    if(isBatteryUpdate)
    {
        if(battery_monitor_.batteryMonitor(battery_data))
        {
            error_msg.data = true;
            battery_error_pub_ -> publish(error_msg);
        }
        else
        {
            error_msg.data = false;
            battery_error_pub_ -> publish(error_msg);
        }
    }
}

void ErrorMonitor::bottomStatusCallback(const robot_custom_msgs::msg::BottomIrData::SharedPtr msg)
{
    bottom_status_data = *msg;
    isBottomStatusUpdate = true;
}

void ErrorMonitor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_data = *msg;
    isImuUpdate = true;
}

void ErrorMonitor::batteryCallback(const robot_custom_msgs::msg::BatteryStatus::SharedPtr msg)
{
    battery_data = *msg;
    count++;
    if(count >= 3000) // 30초 이상일 경우
    {
        isBatteryUpdate = true;
        count = 0;
    }
    else
    {
        isBatteryUpdate = false;
    }

    // isImuUpdate = true;
}