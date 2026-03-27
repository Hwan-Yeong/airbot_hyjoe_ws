#include "error_monitor/monitors/low_battery.hpp"

void LowBatteryErrorMonitor::loadParams(const YAML::Node& config) {
    if (!node_ptr_) return;
    
    // Default values
    params.monitoring_rate_ms = 1000;
    params.occure_percentage_min = 5;
    params.occure_percentage_max = 15;
    params.release_percentage_th = 20;
    params.release_duration_sec = 30.0;

    if (config["monitoring_rate_ms"]) {
        params.monitoring_rate_ms = config["monitoring_rate_ms"].as<int>();
    }
    if (config["occure"]) {
        if (config["occure"]["battery_percentage_min"]) {
            params.occure_percentage_min = config["occure"]["battery_percentage_min"].as<int>();
        }
        if (config["occure"]["battery_percentage_max"]) {
            params.occure_percentage_max = config["occure"]["battery_percentage_max"].as<int>();
        }
    }
    if (config["release"]) {
        if (config["release"]["battery_percentage_th"]) {
            params.release_percentage_th = config["release"]["battery_percentage_th"].as<int>();
        }
        if (config["release"]["duration_sec"]) {
            params.release_duration_sec = config["release"]["duration_sec"].as<double>();
        }
    }
}

void LowBatteryErrorMonitor::printParams() const {
    if (!node_ptr_) return;
    RCLCPP_INFO(node_ptr_->get_logger(),
        "\n[%s] rate: %d\n"
        "occure_min: %d, occure_max: %d, release_th: %d, release_duration: %.1f",
        paramNamespace().c_str(),
        params.monitoring_rate_ms,
        params.occure_percentage_min,
        params.occure_percentage_max,
        params.release_percentage_th,
        params.release_duration_sec
    );
}

void LowBatteryErrorMonitor::startMonitor(std::shared_ptr<RobotStateBlackboard> blackboard) {
    blackboard_ = blackboard;
    error_pub_ = node_ptr_->create_publisher<std_msgs::msg::Bool>(
        "error/s_code/low_battery", 10);
    timer_ = node_ptr_->create_wall_timer(
        std::chrono::milliseconds(params.monitoring_rate_ms),
        [this](){ timerCallback(); }
    );
}

void LowBatteryErrorMonitor::timerCallback()
{
    auto bat = blackboard_->getBatteryData();
    auto st = blackboard_->getStationData();
    
    if (checkSensorState(paramNamespace(), 10, {bat.last_update_time, st.last_update_time})
        != SensorState::NORMAL) {
        return;
    }

    if (!bat.is_updated || !st.is_updated) return;

    static rclcpp::Clock clock(RCL_STEADY_TIME);
    current_time = clock.now().seconds();

    //check off station
    if( st.data.docking_status & 0x10 ){
        if( !station_flag ){
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[LowBatteryErrorMonitor]CHECK AMR ON STATION ==> dockingstatus[%02x] ",
                st.data.docking_status);
        }
        station_flag = true;
    } else{
        if( station_flag ){
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[LowBatteryErrorMonitor]CHECK AMR OFF STATION ==> dockingstatus[%02x] ",
                st.data.docking_status);
        }
        station_flag = false;
    }

    //250730 KKS : S03 발생 조건 5%로 min값 수정
    //check error
    // 조건 : OFF STATION 상태, 배터리 잔여 15%이하
    if( !error_state ){ //Error 가 아닐 경우
        if( station_flag == false ){ //OFF STATION일 경우
            if (bat.data.battery_percent <= params.occure_percentage_max &&
                bat.data.battery_percent > params.occure_percentage_min) {
                if (!prev_state) {
                    // [250407] hyjoe : low battery 에러 발생시 모니터 체크 시간(sec), 배터리 상태 1번만 로깅
                    RCLCPP_INFO(node_ptr_->get_logger(),
                        "[LowBatteryErrorMonitor] OCCUR LOW BATTERY ERROR!!!\n"
                        "Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / Percentage:[%d %%] /"
                        "Current:[%.1f mA] / Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
                        "Battery Cell Voltage:[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d\n"
                        "Battery Version: 0x%02X",
                        bat.data.battery_manufacturer,
                        bat.data.remaining_capacity,
                        static_cast<int>(bat.data.battery_percent),
                        bat.data.battery_current,
                        bat.data.battery_voltage,
                        bat.data.battery_temperature1,
                        bat.data.battery_temperature2,
                        bat.data.cell_voltage1,
                        bat.data.cell_voltage2,
                        bat.data.cell_voltage3,
                        bat.data.cell_voltage4,
                        bat.data.cell_voltage5,
                        bat.data.battery_version
                    );
                }
                error_state = true;
            }
        }
    } else{ //check release error
        // 조건 :  배터리 잔여 20%초과 30초 유지
        if(bat.data.battery_percent > params.release_percentage_th ){ // 20 %
            //check time
            if (!init_setting) { // release 체크 시간에 대해서 초기시간 설정
                prev_time = current_time;
                init_setting = true;
            }
            release_time_diff = current_time - prev_time;
            if( release_time_diff >= params.release_duration_sec){ // 30 sec
                if (prev_state) {
                    // [250407] hyjoe : low battery 에러 발생 한적이 있었던 경우, 해제시 1번만 배터리 상태 로깅
                    RCLCPP_INFO(node_ptr_->get_logger(),
                        "[LowBatteryErrorMonitor] [RELEASED] Low Battery error \n"
                        "elapsed time since release check started: %.3f\n"
                        "Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / Percentage:[%d %%] /"
                        "Current:[%.1f mA] / Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
                        "Battery Cell Voltage:[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d\n"
                        "Battery Version: 0x%02X",
                        release_time_diff,
                        bat.data.battery_manufacturer,
                        bat.data.remaining_capacity,
                        static_cast<int>(bat.data.battery_percent),
                        bat.data.battery_current,
                        bat.data.battery_voltage,
                        bat.data.battery_temperature1,
                        bat.data.battery_temperature2,
                        bat.data.cell_voltage1,
                        bat.data.cell_voltage2,
                        bat.data.cell_voltage3,
                        bat.data.cell_voltage4,
                        bat.data.cell_voltage5,
                        bat.data.battery_version
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
                        "Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / Percentage:[%d %%] /"
                        "Current:[%.1f mA] / Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
                        "Battery Cell Voltage:[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d\n"
                        "Battery Version: 0x%02X",
                        bat.data.battery_manufacturer,
                        bat.data.remaining_capacity,
                        static_cast<int>(bat.data.battery_percent),
                        bat.data.battery_current,
                        bat.data.battery_voltage,
                        bat.data.battery_temperature1,
                        bat.data.battery_temperature2,
                        bat.data.cell_voltage1,
                        bat.data.cell_voltage2,
                        bat.data.cell_voltage3,
                        bat.data.cell_voltage4,
                        bat.data.cell_voltage5,
                        bat.data.battery_version
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

    std_msgs::msg::Bool msg;
    msg.data = error_state;
    error_pub_->publish(msg);
}
