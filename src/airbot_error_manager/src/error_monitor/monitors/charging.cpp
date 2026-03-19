#include "error_monitor/monitors/charging.hpp"

void ChargingErrorMonitor::loadParams(const std::string& ns) {
    if (!node_ptr_) return;
    node_ptr_->declare_parameter<int>(ns + ".occure.battery_percentage_min", 1);
    node_ptr_->declare_parameter<int>(ns + ".occure.battery_percentage_max", 80);
    node_ptr_->declare_parameter<double>(ns + ".occure.duration_sec", 1200.0);
    node_ptr_->declare_parameter<int>(ns + ".monitoring_rate_ms", 1000);

    node_ptr_->get_parameter(ns + ".occure.battery_percentage_min", params.percentage_min_th);
    node_ptr_->get_parameter(ns + ".occure.battery_percentage_max", params.percentage_max_th);
    node_ptr_->get_parameter(ns + ".occure.duration_sec", params.duration_sec);
    node_ptr_->get_parameter(ns + ".monitoring_rate_ms", params.monitoring_rate_ms);
}

void ChargingErrorMonitor::printParams() const {
    if (!node_ptr_) return;
    RCLCPP_INFO(node_ptr_->get_logger(),
        "[%s] percentage_min_th: %d, percentage_max_th: %d, duration_sec: %.1f, rate: %d",
        paramNamespace().c_str(),
        params.percentage_min_th,
        params.percentage_max_th,
        params.duration_sec,
        params.monitoring_rate_ms);
}

void ChargingErrorMonitor::startMonitor(std::shared_ptr<RobotStateBlackboard> blackboard) {
    blackboard_ = blackboard;
    error_pub_ = node_ptr_->create_publisher<std_msgs::msg::Bool>(
        "error/s_code/charging_battery", 10);
    timer_ = node_ptr_->create_wall_timer(
        std::chrono::milliseconds(params.monitoring_rate_ms),
        [this](){ timerCallback(); }
    );
}

void ChargingErrorMonitor::timerCallback()
{
    std::tuple<robot_custom_msgs::msg::BatteryStatus, robot_custom_msgs::msg::StationData, robot_custom_msgs::msg::RobotState> input;
    {
        auto bat = blackboard_->getBatteryData();
        auto st = blackboard_->getStationData();
        
        if (checkSensorState(paramNamespace(), 10, {bat.last_update_time, st.last_update_time})
            != SensorState::NORMAL) {
            return;
        }

        auto state = blackboard_->getRobotStateData();
        if (!bat.is_updated || !st.is_updated || !state.is_updated) return;
        input = std::make_tuple(bat.data, st.data, state.data);
    }
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
    uint8_t currentChargePercentage = battery.battery_percent;

    bool isCharging = station.docking_status & 0X30; // charger found || start charging

    // 배터리 정보 로깅용
    if (currentChargePercentage != prevChargePercentage) {
        RCLCPP_INFO(
            node_ptr_->get_logger(),
            "[ChargingErrorMonitor] Docking status: 0x%02X\n"
            "Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / Percentage:[%d %%] /"
            "Current:[%.1f mA] / Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
            "Battery Cell Voltage:[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d\n"
            "Battery Version: 0x%02X",
            station.docking_status,
            battery.battery_manufacturer,
            battery.remaining_capacity,
            static_cast<int>(battery.battery_percent),
            battery.battery_current,
            battery.battery_voltage,
            battery.battery_temperature1,
            battery.battery_temperature2,
            battery.cell_voltage1,
            battery.cell_voltage2,
            battery.cell_voltage3,
            battery.cell_voltage4,
            battery.cell_voltage5,
            battery.battery_version
        );
        prevChargePercentage = currentChargePercentage;
    }

    if (robot_state.state == 4 || robot_state.state == 5) { // 스테이션 복귀 명령시 해제 사양( 4: RETURN_CHARGER, 5: DOCKING )
        lastCheckTime = currentTime;
        initialCharge = currentChargePercentage;
        errorState = false;
        isFirstCheck = true;
        std_msgs::msg::Bool msg;
        msg.data = false;
        error_pub_->publish(msg);
        return;
    }
    
    if ( !errorState && isCharging ){
        if (params.percentage_min_th <= currentChargePercentage && currentChargePercentage <= params.percentage_max_th) { // 1 ~ 60 %
            if (isFirstCheck) { // 측정 주기 타이머 시작
                lastCheckTime = currentTime;
                initialCharge = currentChargePercentage;
                isFirstCheck = false;
                RCLCPP_INFO(
                    node_ptr_->get_logger(),
                    "[ChargingErrorMonitor] Docking status: 0x%02X\n"
                    "[ChargingErrorMonitor] Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] /"
                    "Percentage:[%d %%] / Current:[%.1f mA] / Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
                    "Battery Cell Voltage:[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d\n"
                    "Battery Version: 0x%02X",
                    station.docking_status,
                    battery.battery_manufacturer,
                    battery.remaining_capacity,
                    static_cast<int>(battery.battery_percent),
                    battery.battery_current,
                    battery.battery_voltage,
                    battery.battery_temperature1,
                    battery.battery_temperature2,
                    battery.cell_voltage1,
                    battery.cell_voltage2,
                    battery.cell_voltage3,
                    battery.cell_voltage4,
                    battery.cell_voltage5,
                    battery.battery_version
                );
            }
            double timediff = currentTime - lastCheckTime;
            if (timediff >= params.duration_sec) { 
                int chargeDiff = static_cast<int>(currentChargePercentage) - static_cast<int>(initialCharge);
                if (chargeDiff <= 2) { 
                    errorState = true;
                    RCLCPP_INFO(
                        node_ptr_->get_logger(),
                        "[ChargingErrorMonitor] Docking status: 0x%02X\n"
                        "Manufacturer:[%d] / Remaining capacity:[%d mAh] /"
                        "Percentage:[%d %%] / Current:[%.1f mA] / Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
                        "Battery Cell Voltage:[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d\n"
                        "Battery Version: 0x%02X",
                        station.docking_status,
                        battery.battery_manufacturer,
                        battery.remaining_capacity,
                        static_cast<int>(battery.battery_percent),
                        battery.battery_current,
                        battery.battery_voltage,
                        battery.battery_temperature1,
                        battery.battery_temperature2,
                        battery.cell_voltage1,
                        battery.cell_voltage2,
                        battery.cell_voltage3,
                        battery.cell_voltage4,
                        battery.cell_voltage5,
                        battery.battery_version
                    );
                    RCLCPP_INFO(node_ptr_->get_logger(),
                        "[ChargingErrorMonitor] elapsed time since error check started: %.3f sec, chargeDiff: %d %%",
                        timediff, chargeDiff
                    );
                } else {
                    errorState = false;
                }
                isFirstCheck = true;
            } else {
                // 이전 errorState를 유지
            }
        } else { 
            errorState = false;
            isFirstCheck = true;
        }
    } else{
        lastCheckTime = currentTime;
        initialCharge = currentChargePercentage;
        isFirstCheck = true;
    }
    prevChargePercentage = currentChargePercentage;

    std_msgs::msg::Bool msg;
    msg.data = errorState;
    error_pub_->publish(msg);
}
