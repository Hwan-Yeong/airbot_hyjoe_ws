#include "error_monitor/monitors/battery_discharging.hpp"

void BatteryDischargingErrorMonitor::loadParams(const YAML::Node& config) {
    if (!node_ptr_) return;

    // Default values
    params.occure_percentage_min = 0;
    params.occure_percentage_max = 5;
    params.occure_duration_sec = 10.0;
    params.release_percentage_th = 10;
    params.release_duration_sec = 30.0;
    params.monitoring_rate_ms = 1000;

    if (config["occure"]) {
        if (config["occure"]["battery_percentage_min"]) params.occure_percentage_min = config["occure"]["battery_percentage_min"].as<int>();
        if (config["occure"]["battery_percentage_max"]) params.occure_percentage_max = config["occure"]["battery_percentage_max"].as<int>();
        if (config["occure"]["duration_sec"]) params.occure_duration_sec = config["occure"]["duration_sec"].as<double>();
    }
    if (config["release"]) {
        if (config["release"]["battery_percentage_th"]) params.release_percentage_th = config["release"]["battery_percentage_th"].as<int>();
        if (config["release"]["duration_sec"]) params.release_duration_sec = config["release"]["duration_sec"].as<double>();
    }
    if (config["monitoring_rate_ms"]) params.monitoring_rate_ms = config["monitoring_rate_ms"].as<int>();
}

void BatteryDischargingErrorMonitor::printParams() const {
    if (!node_ptr_) return;
    RCLCPP_INFO(node_ptr_->get_logger(),
        "[%s] occure_min: %d, occure_max: %d, occure_duration: %.1f, "
        "release_th: %d, release_duration: %.1f, rate: %d",
        paramNamespace().c_str(),
        params.occure_percentage_min,
        params.occure_percentage_max,
        params.occure_duration_sec,
        params.release_percentage_th,
        params.release_duration_sec,
        params.monitoring_rate_ms);
}

void BatteryDischargingErrorMonitor::startMonitor(std::shared_ptr<RobotStateBlackboard> blackboard) {
    blackboard_ = blackboard;
    error_pub_ = node_ptr_->create_publisher<std_msgs::msg::Bool>(
        "error/s_code/discharging_battery", 10);
    timer_ = node_ptr_->create_wall_timer(
        std::chrono::milliseconds(params.monitoring_rate_ms),
        [this](){ timerCallback(); }
    );
}

void BatteryDischargingErrorMonitor::timerCallback()
{
    std::pair<robot_custom_msgs::msg::BatteryStatus, robot_custom_msgs::msg::StationData> input;
    {
        auto bat = blackboard_->getBatteryData();
        auto st = blackboard_->getStationData();
        
        if (checkSensorState(paramNamespace(), 10, {bat.last_update_time, st.last_update_time})
            != SensorState::NORMAL) {
            return;
        }

        if (!bat.is_updated || !st.is_updated) return;
        input = std::make_pair(bat.data, st.data);
    }

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
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[DischargingErrorMonitor]CHECK AMR CHARGING ==> dockingstatus[%02x] ",
                input.second.docking_status);
        }
        charge_flag = true;
    } else{
        if( charge_flag ){
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[DischargingErrorMonitor]CHECK AMR DISCHARGING==> dockingstatus[%02x] ",
                input.second.docking_status);
        }
        charge_flag = false;
    }
    
    current_time = clock.now().seconds();

    if( !error_state ){ //에러가 아닐떄.
            if( !charge_flag && input.first.battery_percent > 0 && input.first.battery_percent <= params.occure_percentage_max) {// 10 % //250730 KKS : 5%로 변경  // 0%초과 5%이하
            
            if (!init_setting) { // 이전 시간에 대해서 초기시간 설정
                prev_time = current_time;
                init_setting = true;
            }
            time_diff = current_time - prev_time;

            if (time_diff >= params.occure_duration_sec) { // 10 sec
                if (!prev_state) {
                    RCLCPP_INFO(node_ptr_->get_logger(),
                        "[BatteryDischargingErrorMonitor] elapsed time since error check started: %.3f\n"
                        "Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / Percentage:[%d %%] /"
                        "Current:[%.1f mA] / Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
                        "Battery Cell Voltage:[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d\n"
                        "Battery Version: 0x%02X",
                        time_diff,
                        input.first.battery_manufacturer,
                        input.first.remaining_capacity,
                        static_cast<int>(input.first.battery_percent),
                        input.first.battery_current,
                        input.first.battery_voltage,
                        input.first.battery_temperature1,
                        input.first.battery_temperature2,
                        input.first.cell_voltage1,
                        input.first.cell_voltage2,
                        input.first.cell_voltage3,
                        input.first.cell_voltage4,
                        input.first.cell_voltage5,
                        input.first.battery_version
                    );
                }
                error_state = true;
            } else {
                if (is_first_logging) {
                    RCLCPP_INFO(node_ptr_->get_logger(),
                        "[BatteryDischargingErrorMonitor] start to battery discharging monitor\n"
                        "Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / Percentage:[%d %%] /"
                        "Current:[%.1f mA] / Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
                        "Battery Cell Voltage:[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d\n"
                        "Battery Version: 0x%02X",
                        input.first.battery_manufacturer,
                        input.first.remaining_capacity,
                        static_cast<int>(input.first.battery_percent),
                        input.first.battery_current,
                        input.first.battery_voltage,
                        input.first.battery_temperature1,
                        input.first.battery_temperature2,
                        input.first.cell_voltage1,
                        input.first.cell_voltage2,
                        input.first.cell_voltage3,
                        input.first.cell_voltage4,
                        input.first.cell_voltage5,
                        input.first.battery_version
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
    } else { // 조건: 베터리 15프로 초과 & 30초 유지 //250730 KKS : S03에러가 5%로 변경됨에 따라 10%초과로 변경
        if(input.first.battery_percent > params.release_percentage_th ){ // 15 %
            //check time
            if (!init_setting) { // release 체크 시간에 대해서 초기시간 설정
                release_start_time = current_time;
                init_setting = true;
            }
            release_time_diff = current_time - release_start_time;

            if( release_time_diff >= params.release_duration_sec){ // 30 sec
                if (prev_state) {
                    // [250407] hyjoe : battery discharging 에러 발생 한적이 있었던 경우, 해제시 1번만 배터리 상태 로깅
                    RCLCPP_INFO(node_ptr_->get_logger(),
                        "[BatteryDischargingErrorMonitor] [RELEASED] battery discharging error \n"
                        "elapsed time since release check started: %.3f\n"
                        "Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / Percentage:[%d %%] /"
                        "Current:[%.1f mA] / Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
                        "Battery Cell Voltage:[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d\n"
                        "Battery Version: 0x%02X",
                        release_time_diff,
                        input.first.battery_manufacturer,
                        input.first.remaining_capacity,
                        static_cast<int>(input.first.battery_percent),
                        input.first.battery_current,
                        input.first.battery_voltage,
                        input.first.battery_temperature1,
                        input.first.battery_temperature2,
                        input.first.cell_voltage1,
                        input.first.cell_voltage2,
                        input.first.cell_voltage3,
                        input.first.cell_voltage4,
                        input.first.cell_voltage5,
                        input.first.battery_version
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
                        "Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / Percentage:[%d %%] /"
                        "Current:[%.1f mA] / Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
                        "Battery Cell Voltage:[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d\n"
                        "Battery Version: 0x%02X",
                        input.first.battery_manufacturer,
                        input.first.remaining_capacity,
                        static_cast<int>(input.first.battery_percent),
                        input.first.battery_current,
                        input.first.battery_voltage,
                        input.first.battery_temperature1,
                        input.first.battery_temperature2,
                        input.first.cell_voltage1,
                        input.first.cell_voltage2,
                        input.first.cell_voltage3,
                        input.first.cell_voltage4,
                        input.first.cell_voltage5,
                        input.first.battery_version
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

    std_msgs::msg::Bool msg;
    msg.data = error_state;
    error_pub_->publish(msg);
}
