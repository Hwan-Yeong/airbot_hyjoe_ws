#include "error_monitor/error_monitor.hpp"

ErrorMonitor::ErrorMonitor()
    : Node("airbot_error_monitor")
{
    initVariables();

    addMonitor<BatteryErrorMonitor::InputType>(std::make_shared<BatteryErrorMonitor>());

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
    battery_data = robot_custom_msgs::msg::BatteryStatus::SharedPtr();
}

void ErrorMonitor::errorMonitor()
{
    std_msgs::msg::Bool error_msg;

    // battery monitor
    if (isBatteryUpdate)
    {
        bool battery_error = this->checkAllErrors(*battery_data);
        if (battery_error) {
            error_msg.data = true;
            battery_error_pub_->publish(error_msg);
        } else {
            error_msg.data = false;
            battery_error_pub_->publish(error_msg);
        }

        isBatteryUpdate = false; // 상태 초기화
    }
}

void ErrorMonitor::batteryCallback(const robot_custom_msgs::msg::BatteryStatus::SharedPtr msg)
{
    count++;
    RCLCPP_INFO(this->get_logger(), "count : %d", count);
    if(count >= 3000) // 30초 이상일 경우
    {
        battery_data = std::make_shared<robot_custom_msgs::msg::BatteryStatus>(*msg);
        isBatteryUpdate = true;
        count = 0;
    }
    else
    {
        isBatteryUpdate = false;
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

bool BatteryErrorMonitor::checkError(const InputType& input)
{
    // 배터리 잔량 표시  // 15% 이하일 경우
    double battery_remaining_amount;
    // int count=0;  // 30초 유지관련 카운트 값
    // int maintenance = 30000;  // 단위 mms
    // 베터리 잔량 관련되서 15%이하이면  복귀 불가능
    battery_remaining_amount = input.battery_percent;

    if(battery_remaining_amount <= 15 && battery_remaining_amount >= 10)
    {
        return true;    // 베터리가 10프로 이상 15프로 이하일 경우
    }
    else
    {
        return false;      // 베터리가 10프로 이상 15프로 이상일 경우
    }

    // 배터리 잔량 표시  // 10% 이하일 경우
}