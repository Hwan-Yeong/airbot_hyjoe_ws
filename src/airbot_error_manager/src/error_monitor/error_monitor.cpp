#include "error_monitor/error_monitor.hpp"


bool LowBatteryErrorMonitor::checkError(const InputType& input)
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
    // 배터리 잔량 표시  // 10% 이하일 경우
}

bool BatteryDischargingErrorMonitor::checkError(const InputType &input)
{
    // 배터리 잔량 표시 // 10% 이하일 경우
    static rclcpp::Clock clock(RCL_ROS_TIME);
    static rclcpp::Time prev_time = clock.now();
    rclcpp::Time current_time = clock.now();
    double time_diff = (current_time - prev_time).seconds();
    RCLCPP_INFO(rclcpp::get_logger("BatteryDischargingErrorMonitor"), "[ time diff: %.3f ]", time_diff);

    double battery_remaining_amount = input.battery_percent;
    if (battery_remaining_amount <= 10) {
        if (time_diff >= 30) {
            RCLCPP_ERROR(rclcpp::get_logger("BatteryDischargingErrorMonitor"), "Time diff: %.3f sec, Battery amount : %.3f", time_diff, battery_remaining_amount);
            prev_time = clock.now();
            return true;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("BatteryDischargingErrorMonitor"), "Time diff: %.3f sec, Battery amount : %.3f", time_diff, battery_remaining_amount);
            return false;
        }
    } else { // 베터리가 10프로 이상
        prev_time = clock.now();
        return false;
    }
    return false;
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

    if (imu_range && bottomdata_range) { // 전도가 일어남, 데이터 값에 따른 결정
        return true;
        RCLCPP_WARN(rclcpp::get_logger("FallDownErrorMonitor"), "Detected (ff : %d)  (fl : %d) (fr :%d) (bb : %d) (bl : %d) (br : %d)  (pich : %.3f) (roll : %.3f)",
                    input.first.ff, input.first.fr, input.first.fr, input.first.bb, input.first.bl, input.first.br, deg_pitch, deg_roll);
    } else { // 전도가 일어나지 않음
        return false;
        RCLCPP_WARN(rclcpp::get_logger("FallDownErrorMonitor"), "Not Detected (ff : %d)  (fl : %d) (fr :%d) (bf : %d) (bl : %d) (br : %d)  (pich : %.3f) (roll : %.3f)",
                    input.first.ff, input.first.fr, input.first.fr, input.first.bb, input.first.bl, input.first.br, deg_pitch, deg_roll);
    }
}

void FallDownErrorMonitor::get_rpy_from_quaternion(const geometry_msgs::msg::Quaternion& quaternion, double& roll, double& pitch, double& yaw)
{
    tf2::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}

bool BoardOverheatErrorMonitor::checkError(const InputType& input)
{
    if (input != nullptr) {
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
                RCLCPP_WARN(rclcpp::get_logger("BoardOverheatErrorMonitor"),
                            "Warning: High temperature detected!,File: %s, Temperature: %.2f°C",
                            file_path.c_str(), temp_value);
                return true; // Return immediately if any temperature exceeds 70°C
            }
        }
        catch (const std::invalid_argument& e) {
            RCLCPP_ERROR(rclcpp::get_logger("BoardOverheatErrorMonitor"),
                        "Invalid temperature data in file %s: %s",
                        file_path.c_str(), e.what());
        }
    }

    return false; // No high temperature detected
}
