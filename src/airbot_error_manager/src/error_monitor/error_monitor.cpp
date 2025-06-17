#include "error_monitor/error_monitor.hpp"


#define CLIFF_IR_ADC_THRESHOLD 900

bool LowBatteryErrorMonitor::checkError(const InputType& input)
{
    static rclcpp::Clock clock(RCL_STEADY_TIME);
    current_time = clock.now().seconds();

    //check off station
    if( input.second.docking_status & 0x10 ){
        if( !station_flag ){
            RCLCPP_INFO(node_ptr_->get_logger(), "[LowBatteryErrorMonitor]CHECK AMR ON STATION ==> dockingstatus[%02x] ",input.second.docking_status);
        }
        station_flag = true;
    } else{
        if( station_flag ){
            RCLCPP_INFO(node_ptr_->get_logger(), "[LowBatteryErrorMonitor]CHECK AMR OFF STATION ==> dockingstatus[%02x] ",input.second.docking_status);
        }
        station_flag = false;
    }

    //check error
    // 조건 : OFF STATION 상태, 배터리 잔여 15%이하
    if( !error_state ){ //Error 가 아닐 경우
        if( station_flag == false ){ //OFF STATION일 경우
            if (input.first.battery_percent <= 15 && input.first.battery_percent > 10) {
                if (!prev_state) {
                        // [250407] hyjoe : low battery 에러 발생시 모니터 체크 시간(sec), 배터리 상태 1번만 로깅
                        RCLCPP_INFO(node_ptr_->get_logger(),
                        "[LowBatteryErrorMonitor] OCCUR LOW BATTERY ERROR!!!\n"
                        "Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / Percentage:[%d %%] / Current:[%.1f mA] / Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
                        "Battery Cell Voltage:[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d",
                        
                        input.first.battery_manufacturer,
                        input.first.remaining_capacity, static_cast<int>(input.first.battery_percent), input.first.battery_current, input.first.battery_voltage, input.first.battery_temperature1, input.first.battery_temperature2,
                        input.first.cell_voltage1, input.first.cell_voltage2, input.first.cell_voltage3, input.first.cell_voltage4, input.first.cell_voltage5
                    );
                }
                error_state = true;
            }
        }
    } else{ //check release error
        // 조건 :  배터리 잔여 20%초과 30초 유지
        if(input.first.battery_percent > 20 ){
            //check time
            if (!init_setting) { // release 체크 시간에 대해서 초기시간 설정
                prev_time = current_time;
                init_setting = true;
            }
            release_time_diff = current_time - prev_time;
            if( release_time_diff >= 30){
                if (prev_state) {
                    // [250407] hyjoe : low battery 에러 발생 한적이 있었던 경우, 해제시 1번만 배터리 상태 로깅
                    RCLCPP_INFO(node_ptr_->get_logger(),
                        "[LowBatteryErrorMonitor] [RELEASED] Low Battery error \n"
                        "elapsed time since release check started: %.3f\n"
                        "Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / Percentage:[%d %%] / Current:[%.1f mA] / Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
                        "Battery Cell Voltage:[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d",
                        release_time_diff,
                        input.first.battery_manufacturer, input.first.remaining_capacity, static_cast<int>(input.first.battery_percent), input.first.battery_current, input.first.battery_voltage, input.first.battery_temperature1, input.first.battery_temperature2,
                        input.first.cell_voltage1, input.first.cell_voltage2, input.first.cell_voltage3, input.first.cell_voltage4, input.first.cell_voltage5
                    );
                }
                prev_time = current_time;
                is_first_logging = true;
                error_state = false;
                init_setting = false;
            } 
            else {
                if (is_first_logging) {
                    // [250407] hyjoe : low battery 에러 조건에 들어왔을 때 시간 체크 시작 시점에 1번만 배터리 상태 로깅
                    RCLCPP_INFO(node_ptr_->get_logger(),
                        "[LowBatteryErrorMonitor] [START RELEASE] low battery monitor\n"
                        "Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / Percentage:[%d %%] / Current:[%.1f mA] / Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
                        "Battery Cell Voltage:[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d",
                        input.first.battery_manufacturer, input.first.remaining_capacity, static_cast<int>(input.first.battery_percent), input.first.battery_current, input.first.battery_voltage, input.first.battery_temperature1, input.first.battery_temperature2,
                        input.first.cell_voltage1, input.first.cell_voltage2, input.first.cell_voltage3, input.first.cell_voltage4, input.first.cell_voltage5
                    );
                    is_first_logging = false;
                }
            }
        } else{
            //time reset
            prev_time = current_time;
            is_first_logging = true;
            init_setting = false;
        }
    }
    
    prev_state = error_state;

    return error_state;
}

bool BatteryDischargingErrorMonitor::checkError(const InputType &input)
{

    // 베터리가 10프로 이하일 경우, 동시에 30초 이상일 경우
    static rclcpp::Clock clock(RCL_STEADY_TIME);
    double current_time, time_diff;
    static double prev_time = 0;
    static bool prev_state = false;
    static bool init_setting = false;
    static bool is_first_logging = true;

    //발생 조건1: 충전중이 아닐때,
    if( input.second.docking_status & 0x70 ){
        if( !charge_flag ){
            RCLCPP_INFO(node_ptr_->get_logger(), "[DischargingErrorMonitor]CHECK AMR CHARGING ==> dockingstatus[%02x] ",input.second.docking_status);
        }
        charge_flag = true;
    } else{
        if( charge_flag ){
            RCLCPP_INFO(node_ptr_->get_logger(), "[DischargingErrorMonitor]CHECK AMR DISCHARGING==> dockingstatus[%02x] ",input.second.docking_status);
        }
        charge_flag = false;
    }
    
    current_time = clock.now().seconds();
    
    if( !error_state ){ //에러가 아닐떄.
        if( !charge_flag && input.first.battery_percent <= 10) {// 충전 중이 아닐때.
            
            if (!init_setting) { // 이전 시간에 대해서 초기시간 설정
                prev_time = current_time;
                init_setting = true;
            }
            time_diff = current_time - prev_time;

            if (time_diff >= 10) {
                if (!prev_state) {
                    RCLCPP_INFO(node_ptr_->get_logger(),
                        "[BatteryDischargingErrorMonitor] elapsed time since error check started: %.3f\n"
                        "Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / Percentage:[%d %%] / Current:[%.1f mA] / Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
                        "Battery Cell Voltage:[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d",
                        time_diff,
                        input.first.battery_manufacturer, input.first.remaining_capacity, static_cast<int>(input.first.battery_percent), input.first.battery_current, input.first.battery_voltage, input.first.battery_temperature1, input.first.battery_temperature2,
                        input.first.cell_voltage1, input.first.cell_voltage2, input.first.cell_voltage3, input.first.cell_voltage4, input.first.cell_voltage5
                    );
                }
                error_state = true;
            } else {
                if (is_first_logging) {
                    RCLCPP_INFO(node_ptr_->get_logger(),
                        "[BatteryDischargingErrorMonitor] start to battery discharging monitor\n"
                        "Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / Percentage:[%d %%] / Current:[%.1f mA] / Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
                        "Battery Cell Voltage:[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d",
                        input.first.battery_manufacturer, input.first.remaining_capacity, static_cast<int>(input.first.battery_percent), input.first.battery_current, input.first.battery_voltage, input.first.battery_temperature1, input.first.battery_temperature2,
                        input.first.cell_voltage1, input.first.cell_voltage2, input.first.cell_voltage3, input.first.cell_voltage4, input.first.cell_voltage5
                    );
                    is_first_logging = false;
                }
                error_state = false;
            }
        } else{
            if( init_setting ){
                init_setting = false;
                prev_time = current_time;
                is_first_logging = true;
            }
        }
    } else { // 조건: 베터리 15프로 초과 & 30초 유지
        if(input.first.battery_percent > 15 ){
            //check time
            if (!init_setting) { // release 체크 시간에 대해서 초기시간 설정
                release_start_time = current_time;
                init_setting = true;
            }
            release_time_diff = current_time - release_start_time;

            if( release_time_diff >= 30){
                if (prev_state) {
                    // [250407] hyjoe : battery discharging 에러 발생 한적이 있었던 경우, 해제시 1번만 배터리 상태 로깅
                    RCLCPP_INFO(node_ptr_->get_logger(),
                        "[BatteryDischargingErrorMonitor] [RELEASED] battery discharging error \n"
                        "elapsed time since release check started: %.3f\n"
                        "Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / Percentage:[%d %%] / Current:[%.1f mA] / Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
                        "Battery Cell Voltage:[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d",
                        release_time_diff,
                        input.first.battery_manufacturer, input.first.remaining_capacity, static_cast<int>(input.first.battery_percent), input.first.battery_current, input.first.battery_voltage, input.first.battery_temperature1, input.first.battery_temperature2,
                        input.first.cell_voltage1, input.first.cell_voltage2, input.first.cell_voltage3, input.first.cell_voltage4, input.first.cell_voltage5
                    );
                }
                release_start_time = current_time;
                is_first_logging = true;
                error_state = false;
                init_setting = false;
            } 
            else {
                if (is_first_logging) {
                    // [250407] hyjoe : battery discharging 에러 조건에 들어왔을 때 시간 체크 시작 시점에 1번만 배터리 상태 로깅
                    RCLCPP_INFO(node_ptr_->get_logger(),
                        "[BatteryDischargingErrorMonitor] [START RELEASE] battery discharging monitor\n"
                        "Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / Percentage:[%d %%] / Current:[%.1f mA] / Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
                        "Battery Cell Voltage:[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d",
                        input.first.battery_manufacturer, input.first.remaining_capacity, static_cast<int>(input.first.battery_percent), input.first.battery_current, input.first.battery_voltage, input.first.battery_temperature1, input.first.battery_temperature2,
                        input.first.cell_voltage1, input.first.cell_voltage2, input.first.cell_voltage3, input.first.cell_voltage4, input.first.cell_voltage5
                    );
                    is_first_logging = false;
                }
            }
        } else{
            //time reset
            release_start_time = current_time;
            is_first_logging = true;
            init_setting = false;
        }
    }
    prev_state = error_state;

    return error_state;
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
    if (input.first.adc_ff < CLIFF_IR_ADC_THRESHOLD) {
        count++;
    }
    if (input.first.adc_fl < CLIFF_IR_ADC_THRESHOLD) {
        count++;
    }
    if (input.first.adc_fr < CLIFF_IR_ADC_THRESHOLD) {
        count++;
    }
    if (input.first.adc_bb < CLIFF_IR_ADC_THRESHOLD) {
        count++;
    }
    if (input.first.adc_bl < CLIFF_IR_ADC_THRESHOLD) {
        count++;
    }
    if (input.first.adc_br < CLIFF_IR_ADC_THRESHOLD) {
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
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[FallDownErrorMonitor] Occured (adc_ff : %d)  (adc_fl : %d) (adc_fr :%d) (adc_bb : %d) (adc_bl : %d) (adc_br : %d)  (pich : %.3f deg) (roll : %.3f deg)",
                input.first.adc_ff, input.first.adc_fr, input.first.adc_fr, input.first.adc_bb, input.first.adc_bl, input.first.adc_br, deg_pitch, deg_roll
            );
            prev_status=true;
        }
        return true;
    } else { // 전도가 일어나지 않음
        if(prev_status){
            // [250407] hyjoe : 전도 에러 발생 한적이 있었던 경우, 해제시 1번만 낙하IR상태, roll, pitch 정보 1번만 로깅
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[FallDownErrorMonitor] Released (adc_ff : %d)  (adc_fl : %d) (adc_fr :%d) (adc_bf : %d) (adc_bl : %d) (adc_br : %d)  (pich : %.3f deg) (roll : %.3f deg)",
                input.first.adc_ff, input.first.adc_fr, input.first.adc_fr, input.first.adc_bb, input.first.adc_bl, input.first.adc_br, deg_pitch, deg_roll
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
        error_state = false;
        return error_state;
    }

    constexpr double OVERHEAT_TEMP_C = 70.0;
    constexpr double OVERHEAT_DURATION_S = 30.0;

    static rclcpp::Clock clock(RCL_STEADY_TIME);
    static std::unordered_map<std::string, double> overheat_occured_times_;
    static std::unordered_map<std::string, bool> overheat_logged_;
    error_state = false;

    // AP 보드 온도 에러 판단 (Board temperature error check)
    for (const auto& file_path : temp_files) {
        std::ifstream file(file_path);
        if (!file) {
            RCLCPP_ERROR(node_ptr_->get_logger(),
                "[BoardOverheatErrorMonitor] Failed to read temperature file: %s",
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
            if (temp_value > OVERHEAT_TEMP_C) {
                // [250407] hyjoe : 보드 과열 에러 발생 시 파일 위치, 보드 온도 로깅 (계속)
                RCLCPP_INFO(node_ptr_->get_logger(),
                    "[BoardOverheatErrorMonitor] Warning: High temperature detected!,File: %s, Temperature: %.2f°C",
                    file_path.c_str(), temp_value
                );

                auto now_sec = clock.now().seconds();
                auto it = overheat_occured_times_.find(file_path);
                if (it == overheat_occured_times_.end()) {
                    overheat_occured_times_[file_path] = now_sec;
                    overheat_logged_[file_path] = false;
                } else if (now_sec - it->second >= OVERHEAT_DURATION_S) {
                    if (!overheat_logged_[file_path]) {
                        RCLCPP_INFO(node_ptr_->get_logger(),
                            "[BoardOverheatErrorMonitor] Error: High temperature detected Over 30sec!,File: %s, Temperature: %.2f°C",
                            file_path.c_str(), temp_value
                        );
                        overheat_logged_[file_path] = true;
                    }
                    error_state = true;
                }
            } else {
                if (overheat_occured_times_.find(file_path) != overheat_occured_times_.end()) {
                    overheat_occured_times_.erase(file_path);
                    overheat_logged_.erase(file_path);
                }
            }
        }
        catch (const std::invalid_argument& e) {
            RCLCPP_ERROR(node_ptr_->get_logger(),
                "[BoardOverheatErrorMonitor] Invalid temperature data in file %s: %s",
                file_path.c_str(), e.what()
            );
        }
    }

    return error_state;
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

    auto battery = std::get<0>(input);
    auto station = std::get<1>(input);
    auto robot_state = std::get<2>(input);

    static rclcpp::Clock clock(RCL_STEADY_TIME);
    double currentTime = clock.now().seconds();
    // RCLCPP_INFO(node_ptr_->get_logger(), "cur time: %.3f", currentTime);
    uint8_t currentChargePercentage = battery.battery_percent;

    bool isCharging = station.docking_status & 0X30; // charger found || start charging


    // 배터리 정보 로깅용
    if (currentChargePercentage != prevChargePercentage) {
        // 최초 한번은 무조건 로깅함 (prev 초기값: 0)
        RCLCPP_INFO(
            node_ptr_->get_logger(),
            "[ChargingErrorMonitor] Docking status: 0x%02X\n"
            "Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / Percentage:[%d %%] / Current:[%.1f mA] / Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
            "Battery Cell Voltage:[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d",
            station.docking_status,
            battery.battery_manufacturer, battery.remaining_capacity, static_cast<int>(battery.battery_percent), battery.battery_current, battery.battery_voltage, battery.battery_temperature1, battery.battery_temperature2,
            battery.cell_voltage1, battery.cell_voltage2, battery.cell_voltage3, battery.cell_voltage4, battery.cell_voltage5
        );
        prevChargePercentage = currentChargePercentage;
    }

    if (robot_state.state == 4 || robot_state.state == 5) { // 스테이션 복귀 명령시 해제 사양( 4: RETURN_CHARGER, 5: DOCKING )
        lastCheckTime = currentTime;
        initialCharge = currentChargePercentage;
        errorState = false;
        isFirstCheck = true;
        return false;
    }
    
    // if (!isCharging) { // charger나 charging 상태가 모두 아니면 무조건 에러 해제.
    //     lastCheckTime = currentTime;
    //     initialCharge = currentChargePercentage;
    //     errorState = false;
    //     isFirstCheck = true;
    //     return false;
    // }

    // [250411] KKS : 충전에러 지속 발생으로 60프로 이하일 때 시작으로 변경 //TBD: 충전 테이블 확인 후 재설정 예정
    // [250329] KKS : 80프로 이하일 때 시작으로 변경
    
    if ( !errorState && isCharging ){
        if ( currentChargePercentage <= 60) {
            if (isFirstCheck) { // 측정 주기 타이머 시작
                lastCheckTime = currentTime;
                initialCharge = currentChargePercentage;
                isFirstCheck = false;
                // [250407] hyjoe : 충전 에러 체크 시작할 때 배터리 상태 로그 출력
                RCLCPP_INFO(
                    node_ptr_->get_logger(),
                    "[ChargingErrorMonitor] Docking status: 0x%02X\n"
                    "[ChargingErrorMonitor] Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / Percentage:[%d %%] / Current:[%.1f mA] / Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
                    "Battery Cell Voltage:[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d",
                    station.docking_status,
                    battery.battery_manufacturer, battery.remaining_capacity, static_cast<int>(battery.battery_percent), battery.battery_current, battery.battery_voltage, battery.battery_temperature1, battery.battery_temperature2,
                    battery.cell_voltage1, battery.cell_voltage2, battery.cell_voltage3, battery.cell_voltage4, battery.cell_voltage5
                );
            }
            double timediff = currentTime - lastCheckTime;
            // RCLCPP_INFO(node_ptr_->get_logger(), "time diff: %.3f", timediff);
            if (timediff >= 1200) { // [250329] KKS : 20분 변경 // 10분 경과 시 평가
                int chargeDiff = static_cast<int>(currentChargePercentage) - static_cast<int>(initialCharge);
                if (chargeDiff <= 2) { // 현재 충전량이 2퍼센트 이하인 경우
                    errorState = true;
                    // [250407] hyjoe : 충전 에러 발생 시 충전단자 상태, 배터리량 로그 출력
                    RCLCPP_INFO(
                        node_ptr_->get_logger(),
                        "[ChargingErrorMonitor] Docking status: 0x%02X\n"
                        "Manufacturer:[%d] / Remaining capacity:[%d mAh] / Percentage:[%d %%] / Current:[%.1f mA] / Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
                        "Battery Cell Voltage:[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d",
                        station.docking_status,
                        battery.battery_manufacturer, battery.remaining_capacity, static_cast<int>(battery.battery_percent), battery.battery_current, battery.battery_voltage, battery.battery_temperature1, battery.battery_temperature2,
                        battery.cell_voltage1, battery.cell_voltage2, battery.cell_voltage3, battery.cell_voltage4, battery.cell_voltage5
                    );
                    // [250407] hyjoe : 충전 에러 발생 시 에러 체크 시작으로부터 경과시간 로그 출력
                    RCLCPP_INFO(node_ptr_->get_logger(),
                        "[ChargingErrorMonitor] elapsed time since error check started: %.3f sec, chargeDiff: %d %%",
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
    } else{
        lastCheckTime = currentTime;
        initialCharge = currentChargePercentage;
        isFirstCheck = true;
    }
    prevChargePercentage = currentChargePercentage;

    return errorState;
}

bool LiftErrorMonitor::checkError(const InputType &input)
{
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
    static bool irLiftFlag = false;
    static bool imuLiftFlag = false;

    count = (input.first.adc_ff < CLIFF_IR_ADC_THRESHOLD) + (input.first.adc_fl < CLIFF_IR_ADC_THRESHOLD) + (input.first.adc_fr < CLIFF_IR_ADC_THRESHOLD) +
            (input.first.adc_bb < CLIFF_IR_ADC_THRESHOLD) + (input.first.adc_bl < CLIFF_IR_ADC_THRESHOLD) + (input.first.adc_br < CLIFF_IR_ADC_THRESHOLD);

    if (count == 0) { // 모든 IR 센서가 false일 경우 에러 해제.
        if (errorState) {
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[LiftErrorMonitor] LiftError Released!"
            );
        }
        errorCount = 0;
        errorState = false;
        irLiftFlag = false;
    } else if (count >= 4) { // ir 센서 true개수 4개 이상이면 ir 들림 의심
        // [250407] hyjoe : 들림 에러 발생 의심시 IR센서 상태 로깅
        if (!irLiftFlag) {
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[LiftErrorMonitor] Over 4 IR sensors Lift Detected! (adc_ff : %d)  (adc_fl : %d) (adc_fr :%d) (adc_bb : %d) (adc_bl : %d) (adc_br : %d)",
                input.first.adc_ff, input.first.adc_fr, input.first.adc_fr, input.first.adc_bb, input.first.adc_bl, input.first.adc_br
            );
        }
        irLiftFlag = true;
    } else {
        irLiftFlag = false;
    }

    double acc_z = input.second.linear_acceleration.z;

    //250521 KKS : 낙하가 감지되지 않을 경우 z축 검사하지 않음
    // acc_z가 10.5이상이고 acc_z가 9.2이하이면 imu 들림 의심 (기준값 수정 필요할 수도 있음)
    if (irLiftFlag && (acc_z <= 9.2 || acc_z >= 10.5)) {
        // [250407] hyjoe : 들림 에러 발생 의심시 imu z축 가속도값 로깅 (승월이나 전도시에도 해당 로그 나올 수 있지만, 추후 정확한 상태 진단을 위해 일단 로깅)
        if (!imuLiftFlag) {
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[LiftErrorMonitor] Imu z axis acceleration Lift Detected! (acc_z: %.3f m/s^2)",
                acc_z
            );
        }
        imuLiftFlag = true;
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
        // [250407] hyjoe : 들림 에러 발생 시 에러 체크 카운터 로깅
        if (!errorState) {
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[LiftErrorMonitor] IR & IMU both Lift Detected! (error count: %d)",
                static_cast<int>(errorCount)
            );
        }
        errorState = true;
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
    static bool preErrorState[6] = {}; // false로 초기화
    double curDist, curPositionX, curPositionY, timeDiff;
    bool cliff[6]={}, errorState = false;

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
                RCLCPP_INFO(node_ptr_->get_logger(),
                    "[CliffDetectionErrorMonitor] Cliff IR #[%d] : %s, IR Detection Error Released",
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
                RCLCPP_INFO(node_ptr_->get_logger(),
                    "[CliffDetectionErrorMonitor] Initial check => Cliff IR #[%d] : %s, pre_position (X, Y): (%.3f, %.3f)",
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

            if (/*timeDiff >= 3.0 || */accumDist[i] >= 0.3) { // 낙하센서 3초 지속 감지 or 낙하센서 감지된 상태에서 30cm 이동
                if (preErrorState[i] == false) {
                    RCLCPP_INFO(node_ptr_->get_logger(),
                        "[CliffDetectionErrorMonitor] Cliff IR #[%d] : %s, timediff: %.3f sec, Accumulated Distance: %.3f m",
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