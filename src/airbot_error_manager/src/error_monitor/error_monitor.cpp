#include "error_monitor/error_monitor.hpp"

ErrorMonitor::ErrorMonitor()
    : Node("airbot_error_monitor")
{
    initVariables();

    addMonitor<BatteryErrorMonitor::InputType>(std::make_shared<BatteryErrorMonitor>());
    addMonitor<FallDownErrorMonitor::InputType>(std::make_shared<FallDownErrorMonitor>());
    addMonitor<BoardTemperatureErrorMonitor::InputType>(std::make_shared<BoardTemperatureErrorMonitor>());

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

    // battery monitor
    if (isBatteryUpdate) {
        bool battery_error = this->checkAllErrors(battery_data);
        if (battery_error) {
            RCLCPP_INFO(this->get_logger(), "battery_error : %d", battery_error);
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
            RCLCPP_INFO(this->get_logger(), "fall_down_error : %d", fall_down_error);
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
        RCLCPP_INFO(this->get_logger(), "board_temperature_eror : %d", board_temperature_eror);
        error_msg.data = true;
        board_temperature_error_pub_->publish(error_msg);
    } else {
        error_msg.data = false;
        board_temperature_error_pub_->publish(error_msg);
    }
}

void ErrorMonitor::batteryCallback(const robot_custom_msgs::msg::BatteryStatus::SharedPtr msg)
{
    count++;
    RCLCPP_INFO(this->get_logger(), "count : %d", count); //////////////////////////////////////////////////////////////////////////////
    if (count >= 1) { // 30초 이상일 경우 //////////////////////////////////////////////////////////////////////////////
        battery_data = *msg;
        isBatteryUpdate = true;
        count = 0;
    } else {
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

    if(battery_remaining_amount <= 15 && battery_remaining_amount >= 10) { // 베터리가 10프로 이상 15프로 이하일 경우
        return true;
    } else {// 베터리가 10프로 이상 15프로 이상일 경우
        return false;
    }
}

bool FallDownErrorMonitor::checkError(const InputType& input)
{
    bool bottomdata_range = false;
    bool imu_range = false;
    int count = 0;

    // 밑 센서값 조정
    // 값이 방향에 따라서 일정하게 변하지 않기 때문에
    // 방향을 나누어서 값 변화에 대해서 전도 현상값의 범위를 조정해야 함
    // front - front_L - back_L - back - back_R - front_R
    if (input.first.ff == 1) {
        count++;
        return false;
    }
    if (input.first.fl == 1) {
        count++;
    }
    if (input.first.fr == 1) {
        count++;
    }
    if (input.first.bb == 1) {
        count++;
    }
    if (input.first.bl == 1) {
        count++;
    }
    if (input.first.br == 1) {
        count++;
    }

    if (count >= 3) {
        bottomdata_range = true;
    } else {
        bottomdata_range = false;
    }

    // imu 센서값 조정
    // 선형 가속도 값을 roll, pitch 각도값으로 변환
    double deg_pitch, deg_roll;  // 각도 값
    double roll, pitch, yaw;
    get_rpy_from_quaternion(input.second.orientation, roll, pitch, yaw);

    deg_pitch = pitch * 180.0 / M_PI;
    deg_roll = roll * 180.0 / M_PI;

    if (abs(deg_pitch) >= 60 || abs(deg_roll) >= 60) {
        imu_range = true;
    }

    if (imu_range && bottomdata_range) { // 데이터 값에 따른 결정
        return true;        // 전도가 일어남
        RCLCPP_WARN(rclcpp::get_logger("fall_down_monitor"), "Detected (ff : %d)  (fl : %d) (fr :%d) (bb : %d) (bl : %d) (br : %d)  (pich : %.3f) (roll : %.3f)", input.first.ff, input.first.fr, input.first.fr, input.first.bb, input.first.bl, input.first.br, deg_pitch, deg_roll);
    } else {
        return false;       // 전도가 일어나지 않음
        RCLCPP_WARN(rclcpp::get_logger("fall_down_monitor"), "Not Detected (ff : %d)  (fl : %d) (fr :%d) (bf : %d) (bl : %d) (br : %d)  (pich : %.3f) (roll : %.3f)", input.first.ff, input.first.fr, input.first.fr, input.first.bb, input.first.bl, input.first.br, deg_pitch, deg_roll);
    }
}

void FallDownErrorMonitor::get_rpy_from_quaternion(const geometry_msgs::msg::Quaternion& quaternion, double& roll, double& pitch, double& yaw)
{
    tf2::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}

bool BoardTemperatureErrorMonitor::checkError(const InputType& input)
{
    if (input == nullptr) {
        return false;
    }
    // AP 보드 온도 에러 판단 (Board temperature error check)
    for (const auto& file_path : temp_files) {
        std::ifstream file(file_path);
        if (!file) {
            RCLCPP_ERROR(rclcpp::get_logger("temp_monitor"),
                        "Failed to read temperature file: %s", file_path.c_str());
            continue;
        }

        std::string line;
        try {
            std::getline(file, line);
            file.close();

            float temp_value = std::stof(line) / 1000.0; // Convert from millidegrees to degrees

            // Check for high temperature warning
            if (temp_value > 70.0) {
                RCLCPP_WARN(rclcpp::get_logger("temp_monitor"),
                            "Warning: High temperature detected!,File: %s, Temperature: %.2f°C",
                            file_path.c_str(), temp_value);
                return true; // Return immediately if any temperature exceeds 70°C
            }
        }
        catch (const std::invalid_argument& e) {
            RCLCPP_ERROR(rclcpp::get_logger("temp_monitor"),
                        "Invalid temperature data in file %s: %s",
                        file_path.c_str(), e.what());
        }
    }

    return false; // No high temperature detected
}
