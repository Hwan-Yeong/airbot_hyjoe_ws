#include "error_monitor/error_monitor.hpp"


bool LowBatteryErrorMonitor::checkError(const InputType& input)
{
    if (input.second.state == 4 || input.second.state == 5) { // 4: RETURN_CHARGER, 5: DOCKING
        return false;
    }

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

    battery_remaining_amount = input.first.battery_percent;
    if (battery_remaining_amount <= 15 && battery_remaining_amount > 10) {

        prev_no_low_battery = true;
        if (time >= 30) {
            if (!prev_status) {
                // [250407] hyjoe : low battery 에러 발생시 모니터 체크 시간(sec), 배터리 상태 1번만 로깅
                RCLCPP_WARN(rclcpp::get_logger("LowBatteryErrorMonitor"),
                    "elapsed time since error check started: %.3f, remaining capacity: %d mAh, battery percent: %d %%, battery current: %.3f mA",
                    time, input.first.remaining_capacity, static_cast<int>(input.first.battery_percent), input.first.battery_current
                );
                prev_status = true;
            }
            return true;
        } else {
            if (prev_status) {
                // [250407] hyjoe : low battery 에러 조건에 들어왔을 때 시간 체크 시작 시점에 1번만 배터리 상태 로깅
                RCLCPP_WARN(rclcpp::get_logger("LowBatteryErrorMonitor"),
                    "start to low battery monitor (remaining capacity: %d mAh, battery percent: %d %%, battery current: %.3f mA)",
                    input.first.remaining_capacity, static_cast<int>(input.first.battery_percent), input.first.battery_current
                );
                prev_status=false;
            }
            return false;
        }
    } else {
        prev_time = clock.now().seconds();
        prev_status=true;
        if (prev_no_low_battery) {
            // [250407] hyjoe : low battery 에러 발생 한적이 있었던 경우, 해제시 1번만 배터리 상태 로깅
            RCLCPP_INFO(rclcpp::get_logger("LowBatteryErrorMonitor"),
                "Low Battery error released (remaining capacity: %d mAh, battery percent: %d %%, battery current: %.3f mA)",
                input.first.remaining_capacity, static_cast<int>(input.first.battery_percent), input.first.battery_current
            );
            prev_no_low_battery = false;
        }
        return false;      // 베터리가 10프로 이하 15프로 이상일 경우
    }
}

bool BatteryDischargingErrorMonitor::checkError(const InputType &input)
{
    if (input.second.state == 4 || input.second.state == 5) { // 4: RETURN_CHARGER, 5: DOCKING
        return false;
    }

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

    battery_remaining_amount = input.first.battery_percent;
    if (battery_remaining_amount <= 10) {
        prev_no_low_battery = true;
        if (time >= 30) {
            if (!prev_status) {
                // [250407] hyjoe : 배터리 방전 에러 발생시 모니터 체크 시간(sec), 배터리 충전량 상태 1번만 로깅
                RCLCPP_WARN(rclcpp::get_logger("BatteryDischargingErrorMonitor"),
                    "elapsed time since error check started: %.3f, remaining capacity: %d mAh, battery percent: %d %%, battery current: %.3f mA",
                    time, input.first.remaining_capacity, static_cast<int>(input.first.battery_percent), input.first.battery_current
                );
                prev_status = true;
            }
            return true;
        } else {
            if (prev_status) {
                // [250407] hyjoe : 배터리 방전 에러 조건에 들어왔을 때 시간 체크 시작 시점에 1번만 배터리 충전량 상태 로깅
                RCLCPP_WARN(rclcpp::get_logger("BatteryDischargingErrorMonitor"),
                    "start to battery discharging monitor (remaining capacity: %d mAh, battery percent: %d %%, battery current: %.3f mA)",
                    input.first.remaining_capacity, static_cast<int>(input.first.battery_percent), input.first.battery_current
                );
                prev_status=false;
            }
            return false;
        }
    } else {
        prev_time = clock.now().seconds();
        prev_status=true;

        if (prev_no_low_battery) {
            // [250407] hyjoe : 배터리 방전 에러 발생 한적이 있었던 경우, 해제시 1번만 배터리 상태 로깅
            RCLCPP_INFO(rclcpp::get_logger("BatteryDischargingErrorMonitor"),
                "Battery Discharging error released (remaining capacity: %d mAh, battery percent: %d %%, battery current: %.3f mA)",
                input.first.remaining_capacity, static_cast<int>(input.first.battery_percent), input.first.battery_current
            );
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
            // [250407] hyjoe : 전도 에러 발생시 낙하IR상태, roll, pitch 정보 1번만 로깅
            RCLCPP_WARN(rclcpp::get_logger("FallDownErrorMonitor"),
                "Occured (ff : %d)  (fl : %d) (fr :%d) (bb : %d) (bl : %d) (br : %d)  (pich : %.3f deg) (roll : %.3f deg)",
                input.first.ff, input.first.fr, input.first.fr, input.first.bb, input.first.bl, input.first.br, deg_pitch, deg_roll
            );
            prev_status=true;
        }
        return true;
    } else { // 전도가 일어나지 않음
        if(prev_status){
            // [250407] hyjoe : 전도 에러 발생 한적이 있었던 경우, 해제시 1번만 낙하IR상태, roll, pitch 정보 1번만 로깅
            RCLCPP_INFO(rclcpp::get_logger("FallDownErrorMonitor"),
                "Released (ff : %d)  (fl : %d) (fr :%d) (bf : %d) (bl : %d) (br : %d)  (pich : %.3f deg) (roll : %.3f deg)",
                input.first.ff, input.first.fr, input.first.fr, input.first.bb, input.first.bl, input.first.br, deg_pitch, deg_roll
            );
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
            RCLCPP_ERROR(rclcpp::get_logger("BoardOverheatErrorMonitor"),
                "Failed to read temperature file: %s",
                file_path.c_str()
            );
            continue;
        }

        std::string line;
        try {
            std::getline(file, line);
            file.close();

            float temp_value = std::stof(line) / 1000.0; // Convert from millidegrees to degrees

            // Check for high temperature warning
            if (temp_value > 70.0) {
                // [250407] hyjoe : 보드 과열 에러 발생 시 파일 위치, 보드 온도 로깅 (계속)
                RCLCPP_INFO(rclcpp::get_logger("BoardOverheatErrorMonitor"),
                    "Warning: High temperature detected!,File: %s, Temperature: %.2f°C",
                    file_path.c_str(), temp_value
                );
                return true; // Return immediately if any temperature exceeds 70°C
            }
        }
        catch (const std::invalid_argument& e) {
            RCLCPP_ERROR(rclcpp::get_logger("BoardOverheatErrorMonitor"),
                "Invalid temperature data in file %s: %s",
                file_path.c_str(), e.what()
            );
        }
    }

    return false; // No high temperature detected
}

bool ChargingErrorMonitor::checkError(const InputType &input)
{
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

    // [250329] KKS : 80프로 이하일 때 시작으로 변경
    if (currentCharge <= 80) {
        if (isFirstCheck) { // 측정 주기 타이머 시작
            lastCheckTime = currentTime;
            initialCharge = currentCharge;
            isFirstCheck = false;
            // [250407] hyjoe : 충전 에러 체크 시작할 때 배터리 상태 로그 출력
            RCLCPP_WARN(
                rclcpp::get_logger("ChargingErrorMonitor"),
                "remaining capacity: %d mAh, battery percent: %d %%, battery current: %.3f mA",
                input.first.remaining_capacity, static_cast<int>(input.first.battery_percent), input.first.battery_current
            );
        }
        double timediff = currentTime - lastCheckTime;
        // RCLCPP_INFO(rclcpp::get_logger("ChargingErrorMonitor"), "time diff: %.3f", timediff);
        if (timediff >= 1200) { // [250329] KKS : 20분 변경 // 10분 경과 시 평가
            int chargeDiff = static_cast<int>(currentCharge) - static_cast<int>(initialCharge);
            if (chargeDiff <= 2) { // 현재 충전량이 2퍼센트 이하인 경우
                errorState = true;
                // [250407] hyjoe : 충전 에러 발생 시 충전단자 상태, 배터리량 로그 출력
                RCLCPP_WARN(
                    rclcpp::get_logger("ChargingErrorMonitor"),
                    "docking status: 0x%02X, remaining capacity: %d mAh, battery percent: %d %%, battery current: %.3f mA",
                    input.second.docking_status, input.first.remaining_capacity, static_cast<int>(input.first.battery_percent), input.first.battery_current
                );
                // [250407] hyjoe : 충전 에러 발생 시 에러 체크 시작으로부터 경과시간 로그 출력
                RCLCPP_WARN(rclcpp::get_logger("ChargingErrorMonitor"),
                    "elapsed time since error check started: %.3f sec, chargeDiff: %d %%",
                    timediff, chargeDiff
                );
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
        // [250407] hyjoe : 들림 에러 발생 의심시 IR센서 상태 로깅 (로봇이 들리는 경우가 자주 발생하지는 않을 것 같아 상태 지속 시 계속 로깅)
        RCLCPP_WARN(rclcpp::get_logger("FallDownErrorMonitor"),
            "Over 4 IR sensors Lift Detected! (ff : %d)  (fl : %d) (fr :%d) (bb : %d) (bl : %d) (br : %d)",
            input.first.ff, input.first.fr, input.first.fr, input.first.bb, input.first.bl, input.first.br
        );
    } else {
        irLiftFlag = false;
    }

    double acc_z = input.second.linear_acceleration.z;

    // acc_z가 10.5이상이고 acc_z가 9.2이하이면 imu 들림 의심 (기준값 수정 필요할 수도 있음)
    if (acc_z <= 9.2 || acc_z >= 10.5) {
        imuLiftFlag = true;
        // [250407] hyjoe : 들림 에러 발생 의심시 imu z축 가속도값 로깅 (승월이나 전도시에도 해당 로그 나올 수 있지만, 추후 정확한 상태 진단을 위해 일단 로깅)
        RCLCPP_WARN(rclcpp::get_logger("FallDownErrorMonitor"),
            "Imu z axis acceleration Lift Detected! (acc_z: %.3f m/s^2)",
            acc_z
        );
    } else {
        imuLiftFlag = false;
    }

    if (irLiftFlag && imuLiftFlag) {
        errorCount++;
    } else if (errorCount > 0) { // Imu나 IR 값이 잠깐 튀어도 카운트 바로 초기화 하지 않기 위해.
        errorCount--;
    }/* else {
        errorCount = 0;
    }*/

    if (errorCount >= 10) {
        errorState = true;
        // [250407] hyjoe : 들림 에러 발생 시 에러 체크 카운터 로깅 (얼마나 오래 지속되고있는지 계속 로깅 - 해제 시점 알 수 있음)
        RCLCPP_WARN(rclcpp::get_logger("FallDownErrorMonitor"),
            "IR & IMU both Lift Detected! (error count: %d)",
            static_cast<int>(errorCount)
        );
    }

    return errorState;
}

bool CliffDetectionErrorMonitor::checkError(const InputType &input)
{
    /*
        < 낙하 감지 에러 검사 >
        해당 모니터는 10ms마다 호출한다는 것을 가정합니다.
        1. 낙하센서가 지속적으로 감지되었음에도 불구하고, odometry변화가 있을 때 에러 검출.
            (즉, 로봇이 이동을 시도할 때)
            1) 바퀴가 빠져 헛도는 상황,
            2) IR센서와 maneuver동작의 오류로 인한 낙하센서 감지 와 로봇 이동이 동시에 발생하는 경우 등
    */
    static rclcpp::Clock clock(RCL_STEADY_TIME);
    static double startErrorCheckTimeArray[6]={}, prePositionXArray[6]={}, prePositionYArray[6]={}, accumDist[6]={};
    static bool isFirstCheckArray[6] = {true, true, true, true, true, true};
    static bool preErrorState[6] = {false, false, false, false, false, false};
    double curDist, curPositionX, curPositionY, timeDiff;
    bool cliff[6]={false}, errorState = false;

    auto bottomIrData = std::get<0>(input);
    auto odom = std::get<1>(input);
    auto robotState = std::get<2>(input);

    // ROBOT_STATE::IDLE, ONSTATION, ERROR (로봇 정지상태에서는 판단 X)
    if (robotState.state == 0 || robotState.state == 7 || robotState.state == 9) {
        isFirstCheckArray[6]={true};
        return false;
    }

    cliff[0]= bottomIrData.ff;    cliff[1]= bottomIrData.fl;    cliff[2]= bottomIrData.bl;
    cliff[3]= bottomIrData.bb;    cliff[4]= bottomIrData.br;    cliff[5]= bottomIrData.fr;

    for (int i=0; i<6; i++) {
        if (cliff[i] == false) {
            isFirstCheckArray[i] = true;
            if (preErrorState[i] == true) { // 낙하 에러 해제시 로깅
                RCLCPP_INFO(rclcpp::get_logger("CliffDetectionErrorMonitor"),
                    "Cliff IR #[%d] : %s, IR Detection Error Released",
                    i+1, cliff[i] ? "true" : "false"
                );
            }
            preErrorState[i] = false;
            continue;
        } else {
            if (isFirstCheckArray[i]) {
                startErrorCheckTimeArray[i]=clock.now().seconds();
                prePositionXArray[i] = odom.pose.pose.position.x;
                prePositionYArray[i] = odom.pose.pose.position.y;
                accumDist[i] = 0.0;
                isFirstCheckArray[i] = false;
                // 낙하 에러 체크 시작 시 최초 한번 로깅
                RCLCPP_INFO(rclcpp::get_logger("CliffDetectionErrorMonitor"),
                    "Initial check => Cliff IR #[%d] : %s, pre_position (X, Y): (%.3f, %.3f)",
                    i+1, cliff[i] ? "true" : "false", prePositionXArray[i], prePositionYArray[i]
                );
            }

            timeDiff = clock.now().seconds() - startErrorCheckTimeArray[i];

            curPositionX = odom.pose.pose.position.x;
            curPositionY = odom.pose.pose.position.y;

            double dx = curPositionX - prePositionXArray[i];
            double dy = curPositionY - prePositionYArray[i];

            curDist = std::sqrt(dx*dx + dy*dy);
            accumDist[i] += curDist;

            prePositionXArray[i] = curPositionX;
            prePositionYArray[i] = curPositionY;

            if (timeDiff >= 3.0 || accumDist[i] >= 0.3) { // 낙하센서 3초 지속 감지 or 낙하센서 감지된 상태에서 30cm 이동
                if (preErrorState[i] == false) {
                    RCLCPP_INFO(rclcpp::get_logger("CliffDetectionErrorMonitor"),
                        "Cliff IR #[%d] : %s, timediff: %.3f sec, Accumulated Distance: %.3f m",
                        i+1, cliff[i] ? "true" : "false", timeDiff, accumDist[i]
                    );
                }
                errorState |= true;
                preErrorState[i] = true;
            }
        }
    }

    return errorState;
}