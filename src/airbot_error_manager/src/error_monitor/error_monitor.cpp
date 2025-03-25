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
    int check_amount;
    int check_start_remaining_amount;
    int current_remaining_amount;
    static bool pre_mode=false;
    static bool curr_mode;
    static bool start_checking_charging_error=false;

    curr_mode = input.second.docking_status & 48;
    if((!pre_mode && curr_mode))   // charger 혹은 charging 상태일때(처음 도킹했을때)
    {
        check_start_remaining_amount = input.first.battery_percent;
        start_checking_charging_error=true;
        pre_mode=curr_mode;
    }

    if(!curr_mode)  // charger와 charging 상태가 아닐때(도킹하지 않았을때)
    {
        start_checking_charging_error=false;
        pre_mode=curr_mode;
    }
    // start_checking_charging_error --> 도킹 상태 체크 --> 도킹 되었으면 true, 도킹 안되었으면 false
    if(start_checking_charging_error && check_start_remaining_amount < 90)
    {

        current_remaining_amount = input.first.battery_percent;
        check_amount = current_remaining_amount - check_start_remaining_amount;  // input.first.battery_percent는 현재 배터리 양을 의미함
        if(check_amount <= 3)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else{
        return false;
    }
}


bool LiftErrorMonitor::checkError(const InputType &input)
{

    static rclcpp::Clock clock(RCL_STEADY_TIME);
    bool lift_bottom_range, gravity_range;
    static int count=0, gravity_count=0;
    double deg_pitch, deg_roll, deg_yaw;   // 각도 값
    double roll, pitch, yaw;               // 라디안 값
    double  acc_x, acc_y, acc_z;           // 선형 가속독
    static double gravity_time=0;
    double time, current_time;
    static bool check_gravity=false;
    static bool status=false;

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

    if (count >= 4) {    // 센서의 변화의 갯수에 따라서 true, false 값 설정 : 여기서는 4개 이상을 설정함
        RCLCPP_WARN(rclcpp::get_logger("LiftErrorMonitor") , "lift_bottom_range ==> result true ,  count : %d  ", count);
        lift_bottom_range = true;
    } else {
        RCLCPP_WARN(rclcpp::get_logger("LiftErrorMonitor") , "lift_bottom_range ==> result false ,  count : %d  ", count);
        lift_bottom_range = false;
    }

    acc_x = input.second.linear_acceleration.x;    // 선형 가속도
    acc_y = input.second.linear_acceleration.y;    // 선형 가속도
    acc_z = input.second.linear_acceleration.z;    // 선형 가속도

    // acc_z가 10.5이상이고 acc_z가 9.2이하이면 들림 조건임

    if((acc_z <= 9.2 || acc_z >= 10.5) && !check_gravity)
    {
        gravity_time = clock.now().seconds();
        check_gravity=true;
    }
    else
    {
    }

    // 들림 조건이면 카운트 시작
    if((acc_z <= 9.2 || acc_z >= 10.5))
    {
        gravity_count++;
    }

    current_time = clock.now().seconds();
    time = current_time - gravity_time;

    // 시간이 200ms 안에서 acc_z 조건이 5개 이상이면 들림의 경우이며, 5미만이면 들림의 경우가 아니다
    if(time >= 0.2)
    {
        gravity_time = 0;
        check_gravity=false;

        if(gravity_count >= 5)
        {
            gravity_count=0;
            gravity_range=true;
            // return true;
        }
        else
        {
            gravity_count=0;
            gravity_range=false;
            // return false;
        }
    }

    if((lift_bottom_range && gravity_range) && !status)
    {
        status=true;
        return true;
    }
    else{
        status=false;
        return false;
    }
}
