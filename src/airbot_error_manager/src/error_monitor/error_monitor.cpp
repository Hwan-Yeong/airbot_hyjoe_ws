#include "error_monitor/error_monitor.hpp"


bool LowBatteryErrorMonitor::checkError(const InputType& input)
{
    // 배터리 잔량 표시  // 15% 이하일 경우
    static rclcpp::Clock clock(RCL_ROS_TIME);
    double battery_remaining_amount;
    double current_time, time;
    static double prev_time=0;
    static bool prev_status=false;
    static bool prev_no_low_battery=false;
    static bool pre_setting=false;

    current_time = clock.now().seconds();

    if (!pre_setting) { // 이전 시간에 대해서 초기시간 설정
        prev_time = current_time;
        pre_setting = true;
    }
    time = current_time - prev_time;

    RCLCPP_WARN(rclcpp::get_logger("LowBatteryErrorMonitor"), "lowerbatter ===> current_time %.3f, prev_time %.3f === %.3f ==",
                current_time, prev_time, time);
    battery_remaining_amount = input.battery_percent;
    if (battery_remaining_amount <= 96 && battery_remaining_amount >= 94) {
        prev_no_low_battery = true;
        if (time >= 30) {
            if (!prev_status) {
                RCLCPP_WARN(rclcpp::get_logger("low battery"), "(time : %.3f), (battery amount : %.3f)", time, battery_remaining_amount);
                prev_status = true;
            }
            return true;    // 베터리가 10프로 이상 15프로 이하 동시에 30초 이상일 경우
        } else {
            if (prev_status) {
                RCLCPP_WARN(rclcpp::get_logger("no low battery"), "(time : %.3f), (battery amount : %.3f)", time, battery_remaining_amount);
                prev_status=false;
            }
            return false;
        }
    } else {
        prev_time = clock.now().seconds();
        prev_status=true;
        if (prev_no_low_battery) {
            RCLCPP_WARN(rclcpp::get_logger("no low battery"), "(time : %.3f), (battery amount : %.3f)", time, battery_remaining_amount);
            prev_no_low_battery = false;
        }
        return false;      // 베터리가 10프로 이하 15프로 이상일 경우
    }
}

bool BatteryDischargingErrorMonitor::checkError(const InputType &input)
{
    // 배터리 잔량 표시 // 10% 이하일 경우
    static rclcpp::Clock clock(RCL_ROS_TIME);
    double battery_remaining_amount;
    double current_time, time;
    static double prev_time=0;
    static bool prev_status=false;
    static bool prev_no_low_battery=false;
    static bool pre_Setting=false;

    current_time = clock.now().seconds();

    if (!pre_Setting) { // 이전 시간에 대해서 초기시간 설정
        prev_time = current_time;
        pre_Setting = true;
    }
    time = current_time - prev_time;

    battery_remaining_amount = input.battery_percent;
    if (battery_remaining_amount <= 10) {
        prev_no_low_battery = true;
        current_time = clock.now().seconds();
        if (time >= 30) {
            if (!prev_status) {
                RCLCPP_WARN(rclcpp::get_logger("discharge battery"), "(time : %.f), (battery amount : %.3f)", time, battery_remaining_amount);
                prev_status = true;
            }
            return true;    // 베터리가 10프로 이하일 경우, 동시에 30초 이상일 경우
        } else {
            if (prev_status) {
                RCLCPP_WARN(rclcpp::get_logger("no discharge battery"), "(time : %.f), (battery amount : %.3f)", time, battery_remaining_amount);
                prev_status=false;
            }
            return false;
        }
    } else {
        prev_time = clock.now().seconds();
        prev_status=true;

        if (prev_no_low_battery) {
            RCLCPP_WARN(rclcpp::get_logger("no discharge battery"), "(battery amount : %.3f)", battery_remaining_amount);
            prev_no_low_battery = false;
        }
        return false;      // 베터리가 10프로 이상
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

bool ChargingErrorMonitor::checkError(const InputType &input)
{
    /*
        Docking Station에서 충전이 불가능한 경우
        도킹 상태 --> 1 일때 도킹이 됨,  0 일때 도킹이 안된다고 생각함
        도킹이 안되었을때는 충전이 불가능하기 때문에 false를 반환한다.
        도킹을 했을 경우에 밧데리가 어느정도 이상일때는 도킹이 안된것으로 간주하고 false를 반환한다.
        도킹을 했을 경우에 충전기간이 10분간 충전량이 2%이하일 경우에 충전이 안된 것으로 간주하고 false를 반환한다.
    */

    static int8_t battery_current_remaining_amount;
    static int8_t battery_remaining_amount;
    double time;
    int8_t charging_amount;
    int docking_mode;
    static double prev_time;
    static bool isstatus=false;
    static bool pre_nodocking=false;
    static bool pre_status=false;
    static bool pre_charging=false;
    static bool pre_setting=false;
    static bool pre_docking=false;

    static rclcpp::Clock clock(RCL_ROS_TIME);
    rclcpp::Time current_time = clock.now();

    if (!pre_setting) { // 이전 시간에 대해서 초기시간 설정
        prev_time = current_time.seconds();
        pre_setting = true;
    }
    time = current_time.seconds() - prev_time;

    docking_mode = input.second.docking_status;

    RCLCPP_WARN(rclcpp::get_logger("ChargingErrorMonitor"), " docking_mode ===> %d ==", docking_mode);

    if (docking_mode != 48) { // 무슨 기준인지?
        if (!pre_nodocking) {
            RCLCPP_WARN(rclcpp::get_logger("ChargingErrorMonitor"), "docking false" );
            pre_nodocking = true;
        }
        return false;
    } else { // 도킹이 48일때 도킹이 된 상태
        pre_docking=false; // 도킹 0일때 플래그
        battery_current_remaining_amount = input.first.battery_percent;
        if (!pre_charging) { // 초기 배터리 퍼센트값 정의
            battery_remaining_amount = input.first.battery_percent;
            RCLCPP_WARN(rclcpp::get_logger("ChargingErrorMonitor"), "battery_remaining_amount === %d ==", battery_remaining_amount);
            pre_charging = true;
        }

        // RCLCPP_WARN(rclcpp::get_logger(""), " === %d ==", battery_remaining_amount);
        if (battery_current_remaining_amount <= 60) { // 배터리가 95퍼센트가 남았을경우 충전 에러
            if (!pre_docking) {
                RCLCPP_WARN(rclcpp::get_logger("ChargingErrorMonitor"), "battery_current_remaining_amount : %d) ---> charging false", battery_current_remaining_amount);
                pre_docking=true;
            }
            return false;
        } else {
            // RCLCPP_WARN(rclcpp::get_logger("2"), " === %f ==   %f  ===  %f  =====", current_time,  prev_time, current_time - prev_time);
            // RCLCPP_WARN(rclcpp::get_logger(""), " === 1 ==");

            pre_nodocking=false;
            pre_docking=false;
            charging_amount = battery_current_remaining_amount - battery_remaining_amount;
            // RCLCPP_WARN(rclcpp::get_logger("2-2"), " === %f  ======  %f ==   %f", time, prev_time, current_time);
            // RCLCPP_WARN(rclcpp::get_logger("2-3"), " === %d  ======  %d ==   %d", battery_current_remaining_amount, battery_remaining_amount, charging_amount);
            if (time >= 100) {
                RCLCPP_WARN(rclcpp::get_logger("ChargingErrorMonitor"), "=== 1 ==");
                prev_time = clock.now().seconds();
                if (charging_amount >= 0) { // 배터리 충전이 안되고 있는 경우
                    if (!pre_status) {
                        RCLCPP_WARN(rclcpp::get_logger("ChargingErrorMonitor"), "charging false --> (time : %.3f), %d, %d (charging_amount : %d)",
                                    time, battery_current_remaining_amount, battery_remaining_amount ,charging_amount);
                        pre_status=true;
                        isstatus = false;
                    }
                    battery_remaining_amount = input.first.battery_percent;  // 초기 배터리 퍼센트값 충전이 않되는 경우 업데이트
                    return false;
                } else {
                    pre_status=false;
                    if (!isstatus) {
                        RCLCPP_WARN(rclcpp::get_logger("ChargingErrorMonitor"), "charging true --> (time : %.3f), %d, %d (charging_amount : %d)",
                                    time, battery_current_remaining_amount, battery_remaining_amount ,charging_amount);
                        isstatus=true;
                    }
                    battery_remaining_amount = input.first.battery_percent;  // 초기 배터리 퍼센트값 충전이 될 경우 업데이트
                    return true;
                }
            }
            RCLCPP_WARN(rclcpp::get_logger("ChargingErrorMonitor"), " === %.3f ==  === %d ==   === %d ==  === %d ==", time, battery_current_remaining_amount, battery_remaining_amount, charging_amount);
            return true;
        }
    }
    return false;
}