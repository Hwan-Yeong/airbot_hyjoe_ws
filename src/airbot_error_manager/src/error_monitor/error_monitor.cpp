#include "error_monitor/error_monitor.hpp"


bool LowBatteryErrorMonitor::checkError(const InputType& input)
{
    // 베터리가 10프로 이상 15프로 이하 동시에 30초 이상일 경우
    static rclcpp::Clock clock(RCL_STEADY_TIME);
    int battery_remaining_amount;
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

    battery_remaining_amount = input.battery_percent;
    if (battery_remaining_amount <= 15 && battery_remaining_amount > 10) {

        prev_no_low_battery = true;
        if (time >= 30) {
            if (!prev_status) {
                RCLCPP_WARN(rclcpp::get_logger("low battery"), "(time : %.3f), (battery amount : %d)", time, battery_remaining_amount);
                prev_status = true;
            }
            return true;    
        } else {
            if (prev_status) {
                RCLCPP_WARN(rclcpp::get_logger("battery checking start"), "(time : %.3f), (battery amount : %d)", time, battery_remaining_amount);
                prev_status=false;
            }
            return false;
        }
    } else {
        prev_time = clock.now().seconds();
        prev_status=true;
        if (prev_no_low_battery) {
            RCLCPP_WARN(rclcpp::get_logger("no low battery"), "(time : %.3f), (battery amount : %d)", time, battery_remaining_amount);
            prev_no_low_battery = false;
        }
        return false;      // 베터리가 10프로 이하 15프로 이상일 경우
    }
}

bool BatteryDischargingErrorMonitor::checkError(const InputType &input)
{
    // 베터리가 10프로 이하일 경우, 동시에 30초 이상일 경우
    static rclcpp::Clock clock(RCL_STEADY_TIME);
    int battery_remaining_amount;
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
        if (time >= 30) {
            if (!prev_status) {
                RCLCPP_WARN(rclcpp::get_logger("discharge battery"), "(time : %.f), (battery amount : %d)", time, battery_remaining_amount);
                prev_status = true;
            }
            return true;    
        } else {
            if (prev_status) {
                RCLCPP_WARN(rclcpp::get_logger("discharge battery check start"), "(time : %.f), (battery amount : %d)", time, battery_remaining_amount);
                prev_status=false;
            }
            return false;
        }
    } else {
        prev_time = clock.now().seconds();
        prev_status=true;

        if (prev_no_low_battery) {
            RCLCPP_WARN(rclcpp::get_logger("no discharge battery"), "(battery amount : %d)", battery_remaining_amount);
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
    static bool prev_status=false;
    
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
        if(!prev_status){
            RCLCPP_WARN(rclcpp::get_logger("FallDownErrorMonitor"), "Detected (ff : %d)  (fl : %d) (fr :%d) (bb : %d) (bl : %d) (br : %d)  (pich : %.3f) (roll : %.3f)",
                    input.first.ff, input.first.fr, input.first.fr, input.first.bb, input.first.bl, input.first.br, deg_pitch, deg_roll);
            prev_status=true;
        }
        return true;
    } else { // 전도가 일어나지 않음
        if(prev_status){
            RCLCPP_WARN(rclcpp::get_logger("FallDownErrorMonitor"), "Not Detected (ff : %d)  (fl : %d) (fr :%d) (bf : %d) (bl : %d) (br : %d)  (pich : %.3f) (roll : %.3f)",
                    input.first.ff, input.first.fr, input.first.fr, input.first.bb, input.first.bl, input.first.br, deg_pitch, deg_roll);
            prev_status=false;
        }
        return false;
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
    return false;
    /*
        < 충전 에러 검사 >
        해당 모니터는 10분마다 에러 발생/해제를 체크하지만, 충전중이 아닐때 혹은 충전중 95%가 넘어가는 시점부터는 바로 해제를 반환합니다.
        체크하는 10분 사이의 간격에는 이전에 판단된 상태를 계속해서 반환합니다.

        1. 배터리 95% 이하일때만 에러 체크
        2. isCharging상태가 유지된 상태에서 10분마다 상태 체크
        3. chargeDiff = initialCharge - finalCharge 의 크기가 2% 이하이면 에러 발생.
        4. chargeDiff의 크기가 2% 이하가 아니면 에러 해제. (에러 발생/해제는 10분마다 판단해서 결과를 알려준다)
    */

    static rclcpp::Clock clock(RCL_STEADY_TIME);
    double currentTime = clock.now().seconds();
    // RCLCPP_INFO(rclcpp::get_logger("ChargingErrorMonitor"), "cur time: %.3f", currentTime);
    uint8_t currentCharge = input.first.battery_percent;

    bool isCharging = input.second.docking_status & 0X30; // charger found || start charging

    if (!isCharging) { // charger나 charging 상태가 모두 아니면 무조건 에러 해제.
        lastCheckTime = currentTime;
        initialCharge = currentCharge;
        errorState = false;
        isFirstCheck = true;
        return false;
    }

    if (currentCharge <= 90) {
        if (isFirstCheck) { // 측정 주기 타이머 시작
            lastCheckTime = currentTime;
            initialCharge = currentCharge;
            isFirstCheck = false;
            // RCLCPP_INFO(rclcpp::get_logger("ChargingErrorMonitor"), "last time: %.3f", lastCheckTime);
        }
        double timediff = currentTime - lastCheckTime;
        // RCLCPP_INFO(rclcpp::get_logger("ChargingErrorMonitor"), "time diff: %.3f", timediff);
        if (timediff >= 600) { // 10분 경과 시 평가
            int chargeDiff = static_cast<int>(currentCharge) - static_cast<int>(initialCharge);
            if (chargeDiff <= 2) { // 현재 충전량이 2퍼센트 이하인 경우
                errorState = true;
            } else {
                errorState = false;
            }
            isFirstCheck = true;
        } else {
            // 아무 처리도 하지 않음으로서 이전 errorState를 유지하도록 한다.
        }
    } else { // 충전중인데 배터리 90% 이하가 아니면 그냥 에러 해제.
        errorState = false;
        isFirstCheck = true;
    }

    return errorState;
}

bool LiftErrorMonitor::checkError(const InputType &input)
{
    return false;
    /*
        < 들림 에러 검사 >
        해당 모니터는 10ms마다 호출한다는 것을 가정합니다.
        IR, imu 데이터로 각각 들림 의심을 하고 두 플래그가 동시에 10번 발생하면 들림 에러를 띄웁니다. (반응속도로 봤을 때 큰 차이 없어서...)
        낙하 IR이 모두 false로 바뀔때만 에러를 해제하고, 그 전까지는 에러로 간주합니다.

        1. 낙하 IR 에서 최소 4개 이상이 true인 경우 들림 의심.
        2. z축 가속도(m/s^2)가 10.5이상 or 9.2 이하인 경우 들림 의심.
        3. 1,2번 의심이 모두 발생한 상황이 10번 반복되면 에러를 발생시킨다.
        4. 모든 IR 센서가 false일 경우 에러를 해제시킨다.
    */

    static rclcpp::Clock clock(RCL_STEADY_TIME);
    static int count = 0;
    bool irLiftFlag = false;
    bool imuLiftFlag = false;

    count = (input.first.ff == true) + (input.first.fl == true) + (input.first.fr == true) +
            (input.first.bb == true) + (input.first.bl == true) + (input.first.br == true);

    if (count == 0) { // 모든 IR 센서가 false일 경우 에러 해제.
        errorCount = 0;
        errorState = false;
        return errorState;
    } else if (count >= 4) { // ir 센서 true개수 4개 이상이면 ir 들림 의심
        irLiftFlag = true;
    } else {
        irLiftFlag = false;
    }

    double acc_z = input.second.linear_acceleration.z;

    // acc_z가 10.5이상이고 acc_z가 9.2이하이면 imu 들림 의심 (기준값 수정 필요할 수도 있음)
    if (acc_z <= 9.2 || acc_z >= 10.5) {
        imuLiftFlag = true;
    } else {
        imuLiftFlag = false;
    }

    if (irLiftFlag && imuLiftFlag) {
        errorCount++;
    } else if (errorCount > 0) {
        errorCount--; // Imu나 IR 값이 잠깐 튀어도 카운트로 바로 초기화 해 버리지 않기 위해.
    }/* else {
        errorCount = 0;
    }*/

    if (errorCount >= 10) {
        errorState = true;
    }

    return errorState;
}
