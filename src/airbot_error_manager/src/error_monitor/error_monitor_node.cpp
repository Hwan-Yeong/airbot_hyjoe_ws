#include "error_monitor/error_monitor_node.hpp"

ErrorMonitorNode::ErrorMonitorNode()
    : Node("airbot_error_monitor")
{
    initVariables();
    setParams();

    addMonitor<LowBatteryErrorMonitor>(std::make_shared<LowBatteryErrorMonitor>());
    addMonitor<FallDownErrorMonitor>(std::make_shared<FallDownErrorMonitor>());
    addMonitor<BoardOverheatErrorMonitor>(std::make_shared<BoardOverheatErrorMonitor>());
    addMonitor<BatteryDischargingErrorMonitor>(std::make_shared<BatteryDischargingErrorMonitor>());
    addMonitor<ChargingErrorMonitor>(std::make_shared<ChargingErrorMonitor>());
    addMonitor<LiftErrorMonitor>(std::make_shared<LiftErrorMonitor>());

    // Subscriber
    bottom_status_sub_ = this->create_subscription<robot_custom_msgs::msg::BottomIrData>(
        "bottom_ir_data", 10, std::bind(&ErrorMonitorNode::bottomStatusCallback, this, std::placeholders::_1)
    );
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu_data", 10, std::bind(&ErrorMonitorNode::imuCallback, this, std::placeholders::_1)
    );
    battery_status_sub_ = this->create_subscription<robot_custom_msgs::msg::BatteryStatus>(
        "/battery_status", 10, std::bind(&ErrorMonitorNode::batteryCallback, this, std::placeholders::_1)
    );
    station_data_sub_ = this->create_subscription<robot_custom_msgs::msg::StationData>(
        "/station_data", 10, std::bind(&ErrorMonitorNode::stationDataCallback, this, std::placeholders::_1)
    );

    // Publisher
    fall_down_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/s_code/fall_down", 20);
    low_battery_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/s_code/low_battery", 10);
    board_overheat_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/s_code/board_overheat", 10);
    battery_discharge_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/s_code/discharging_battery", 10);
    charging_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("error/e_code/charging", 10);
    // lift_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("/", 10);

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
    isStationDataUpdate = false;

    publish_cnt_low_battery_error_ = 0;
    publish_cnt_fall_down_error_ = 0;
    publish_cnt_board_overheat_error_ = 0;
    publish_cnt_battery_discharge_error_ = 0;
    publish_cnt_charging_error_ = 0;
    publish_cnt_lift_error_ = 0;

    bottom_status_data = robot_custom_msgs::msg::BottomIrData();
    imu_data = sensor_msgs::msg::Imu();
    battery_data = robot_custom_msgs::msg::BatteryStatus();
    station_data = robot_custom_msgs::msg::StationData();
}

void ErrorMonitorNode::setParams()
{
    this->declare_parameter<int>("publish_rate.low_battery_rate_ms", 1000);
    this->declare_parameter<int>("publish_rate.fall_down_rate_ms", 1000);
    this->declare_parameter<int>("publish_rate.board_overheat_rate_ms", 1000);
    this->declare_parameter<int>("publish_rate.battery_discharge_rate_ms", 1000);
    this->declare_parameter<int>("publish_rate.charging_rate_ms", 1000);
    this->declare_parameter<int>("publish_rate.lift_error_rate_ms", 10);

    this->get_parameter("publish_rate.low_battery_rate_ms", publish_cnt_low_battery_error_rate_);
    this->get_parameter("publish_rate.fall_down_rate_ms", publish_cnt_fall_down_error_rate_);
    this->get_parameter("publish_rate.board_overheat_rate_ms", publish_cnt_board_overheat_error_rate_);
    this->get_parameter("publish_rate.battery_discharge_rate_ms", publish_cnt_battery_discharge_error_rate_);
    this->get_parameter("publish_rate.charging_rate_ms", publish_cnt_charging_error_rate_);
    this->get_parameter("publish_rate.lift_error_rate_ms", publish_cnt_lift_error_rate_);

    RCLCPP_INFO(this->get_logger(), "=================== ERROR MONITOR PARAMETER ===================");
    RCLCPP_INFO(this->get_logger(), "Low Battery Rate: %d ms", publish_cnt_low_battery_error_rate_);
    RCLCPP_INFO(this->get_logger(), "Fall Down Rate: %d ms", publish_cnt_fall_down_error_rate_);
    RCLCPP_INFO(this->get_logger(), "Board Overheat Rate: %d ms", publish_cnt_board_overheat_error_rate_);
    RCLCPP_INFO(this->get_logger(), "Battery Discharge Rate: %d ms", publish_cnt_battery_discharge_error_rate_);
    RCLCPP_INFO(this->get_logger(), "Charging Rate: %d ms", publish_cnt_charging_error_rate_);
    RCLCPP_INFO(this->get_logger(), "Lift Error Rate: %d ms", publish_cnt_lift_error_rate_);
    RCLCPP_INFO(this->get_logger(), "===============================================================");
}

void ErrorMonitorNode::errorMonitor()
{
    std_msgs::msg::Bool error_msg;

    publish_cnt_low_battery_error_ +=10;
    publish_cnt_fall_down_error_ +=10;
    publish_cnt_board_overheat_error_ +=10;
    publish_cnt_battery_discharge_error_ += 10;
    publish_cnt_charging_error_ += 10;
    publish_cnt_lift_error_ += 10;

    // fall down monitor
    if (isBottomStatusUpdate && isImuUpdate && (publish_cnt_fall_down_error_ >= publish_cnt_fall_down_error_rate_)) {
        bool fall_down_error = this->runMonitor<FallDownErrorMonitor>(std::make_pair(bottom_status_data, imu_data));
        if (fall_down_error) {
            RCLCPP_INFO(this->get_logger(), "fall_down_error : %s", fall_down_error ? "true" : "false");
            error_msg.data = true;
            fall_down_error_pub_->publish(error_msg);
        } else {
            error_msg.data = false;
            fall_down_error_pub_->publish(error_msg);
        }
        publish_cnt_fall_down_error_ = 0;
    }

    // low battery monitor
    if (isBatteryUpdate && (publish_cnt_low_battery_error_ >= publish_cnt_low_battery_error_rate_)) {
        bool low_battery_error = this->runMonitor<LowBatteryErrorMonitor>(battery_data);
        if (low_battery_error) {
            // RCLCPP_INFO(this->get_logger(), "low_battery_error : %s", low_battery_error ? "true" : "false");
            // error_msg.data = true;
            // low_battery_error_pub_->publish(error_msg);
        } else {
            // icbaek, 2025.03.19 : false 여도 publish하지 않게 하였음.
            // error_msg.data = false;
            // low_battery_error_pub_->publish(error_msg);
        }
        publish_cnt_low_battery_error_ = 0;
    }

    // board overheat monitor
    if (publish_cnt_board_overheat_error_ >= publish_cnt_board_overheat_error_rate_) {
        bool board_overheat_error = this->runMonitor<BoardOverheatErrorMonitor>(std::nullptr_t());
        if (board_overheat_error) {
            // RCLCPP_INFO(this->get_logger(), "board_overheat_error : %s", board_overheat_error ? "true" : "false");
            // error_msg.data = true;
            // board_overheat_error_pub_->publish(error_msg);
        } else {
            // icbaek, 2025.03.19 : false 여도 publish하지 않게 하였음.
            // error_msg.data = false;
            // board_overheat_error_pub_->publish(error_msg);
        }
        publish_cnt_board_overheat_error_ = 0;
    }

    // battery discharging monitor
    if (isBatteryUpdate && (publish_cnt_battery_discharge_error_ >= publish_cnt_battery_discharge_error_rate_)) {
        bool battery_discharge_error = this->runMonitor<BatteryDischargingErrorMonitor>(battery_data);
        if (battery_discharge_error) {
            // RCLCPP_INFO(this->get_logger(), "battery_discharge_error : %s", battery_discharge_error ? "true" : "false");
            // error_msg.data = true;
            // battery_discharge_error_pub_->publish(error_msg);
        } else {
            // icbaek, 2025.03.19 : false 여도 publish하지 않게 하였음.
            // error_msg.data = false;
            // battery_discharge_error_pub_->publish(error_msg);
        }
        publish_cnt_battery_discharge_error_ = 0;
    }

    // charging monitor
    if (isStationDataUpdate && isBatteryUpdate && (publish_cnt_charging_error_ >= publish_cnt_charging_error_rate_)) {
        bool charging_error = this->runMonitor<ChargingErrorMonitor>(std::make_pair(battery_data, station_data));
        if (charging_error) {
            // RCLCPP_INFO(this->get_logger(), "charging_error : %s", charging_error ? "true" : "false");
            // error_msg.data = true;
            // charging_error_pub_->publish(error_msg);
        } else {
            // icbaek, 2025.03.19 : false 여도 publish하지 않게 하였음.
            // error_msg.data = false;
            // charging_error_pub_->publish(error_msg);
        }
        publish_cnt_charging_error_ = 0;
    }

    // lift monitor
    if (isBottomStatusUpdate && isImuUpdate && (publish_cnt_lift_error_ >= publish_cnt_lift_error_rate_)) {
        bool lift_error = this->runMonitor<LiftErrorMonitor>(std::make_pair(bottom_status_data, imu_data));
        if (lift_error) {
            // RCLCPP_INFO(this->get_logger(), "lift_error : %s", lift_error ? "true" : "false");
            // error_msg.data = true;
            // lift_error_pub_->publish(error_msg);
        } else {
            // error_msg.data = false;
            // lift_error_pub_->publish(error_msg);
        }
        publish_cnt_lift_error_ = 0;
    }

    isImuUpdate = false;
    isBatteryUpdate = false;
    isStationDataUpdate = false;
    isBottomStatusUpdate = false;

    // publish_cnt_* 변수 오버플로우 방지
    if (publish_cnt_low_battery_error_ >= 100000) publish_cnt_low_battery_error_ = 0;
    if (publish_cnt_fall_down_error_ >= 100000) publish_cnt_fall_down_error_ = 0;
    if (publish_cnt_board_overheat_error_ >= 100000) publish_cnt_board_overheat_error_ = 0;
    if (publish_cnt_battery_discharge_error_ >= 100000) publish_cnt_battery_discharge_error_ = 0;
    if (publish_cnt_charging_error_ >= 100000) publish_cnt_charging_error_ = 0;
    if (publish_cnt_lift_error_ >= 100000) publish_cnt_lift_error_ = 0;
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

void ErrorMonitorNode::stationDataCallback(const robot_custom_msgs::msg::StationData::SharedPtr msg)
{
    station_data = *msg;
    isStationDataUpdate = true;
}