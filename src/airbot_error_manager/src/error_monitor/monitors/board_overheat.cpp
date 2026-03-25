#include "error_monitor/monitors/board_overheat.hpp"

void BoardOverheatErrorMonitor::loadParams(const YAML::Node& config) {
    if (!node_ptr_) return;
    
    // Default values matched with yaml for fallback
    params.temperature_th = 85.0;
    params.duration_sec = 30.0;
    params.monitoring_rate_ms = 1000;

    if (config["occure"]) {
        if (config["occure"]["temperature_th_c"]) params.temperature_th = config["occure"]["temperature_th_c"].as<double>();
        else if (config["occure"]["temperature_th"]) params.temperature_th = config["occure"]["temperature_th"].as<double>(); // Fallback to old name just in case
        if (config["occure"]["duration_sec"]) params.duration_sec = config["occure"]["duration_sec"].as<double>();
    }
    if (config["monitoring_rate_ms"]) params.monitoring_rate_ms = config["monitoring_rate_ms"].as<int>();
}

void BoardOverheatErrorMonitor::printParams() const {
    if (!node_ptr_) return;
    RCLCPP_INFO(node_ptr_->get_logger(),
        "[%s] temperature_th: %.1f, duration_sec: %.1f, rate: %d",
        paramNamespace().c_str(),
        params.temperature_th,
        params.duration_sec,
        params.monitoring_rate_ms
    );
}

void BoardOverheatErrorMonitor::startMonitor(std::shared_ptr<RobotStateBlackboard> blackboard) {
    blackboard_ = blackboard;
    error_pub_ = node_ptr_->create_publisher<std_msgs::msg::Bool>(
        "error/s_code/board_overheat", 10);
    timer_ = node_ptr_->create_wall_timer(
        std::chrono::milliseconds(params.monitoring_rate_ms),
        [this](){ timerCallback(); }
    );
}

void BoardOverheatErrorMonitor::timerCallback()
{
    auto ap = blackboard_->getApTemperatureData();
    
    if (checkSensorState(paramNamespace(), 100, {ap.last_update_time})
        != SensorState::NORMAL) {
        return;
    }

    static rclcpp::Clock clock(RCL_STEADY_TIME);

    std::vector<std::pair<std::string, float>> temps = {
        {"ap", ap.data.ap},
        {"bigcore0", ap.data.bigcore0},
        {"bigcore1", ap.data.bigcore1},
        {"littlecore", ap.data.littlecore},
        {"center", ap.data.center},
        {"gpu", ap.data.gpu},
        {"npu", ap.data.npu}
    };

    bool any_error = false;
    for (const auto& temp_pair : temps) {
        const std::string& zone_name = temp_pair.first;
        float temp_value = temp_pair.second;

        if (temp_value > params.temperature_th) {
            auto it = overheat_occured_times_.find(zone_name);
            if (it == overheat_occured_times_.end()) {
                // 온도가 처음으로 threshold 넘었을 때 시간 체크 시작.
                overheat_occured_times_[zone_name] = clock.now().seconds();
                RCLCPP_WARN(node_ptr_->get_logger(),
                    "[BoardOverheat] Warning: [%s] / Temp [%.1f]°C > "
                    "threshold [%.1f]°C. Starting %.0fs timer.",
                    zone_name.c_str(),
                    temp_value,
                    params.temperature_th,
                    params.duration_sec);
            } else {
                // threshold 넘은 상태가 30초 이상 지속되었는지 확인.
                if (clock.now().seconds() - it->second >= params.duration_sec) {
                    any_error = true; // 하나라도 30초 이상 지속되면 에러 발생.

                    if( static_cast<int>(clock.now().seconds() - it->second) % 31 == 0){ 
                        RCLCPP_WARN(node_ptr_->get_logger(),
                            "[BoardOverheat] Error: [%s] / Temp [%.1f]°C > "
                            "threshold [%.1f]°C. Over Time %.0fs.",
                            zone_name.c_str(),
                            temp_value,
                            params.temperature_th,
                            clock.now().seconds() - it->second);
                    }
                }
            }
        } else {
            // 온도가 threshold 아래로 떨어지면..
            if (overheat_occured_times_.count(zone_name)) {
                auto it_release = overheat_release_start_times_.find(zone_name);
                if (it_release == overheat_release_start_times_.end()) {
                    // release 시작 시간 등록
                    overheat_release_start_times_[zone_name] = clock.now().seconds();
                } else {
                    // 5초 이상 유지 확인.
                    if (clock.now().seconds() - it_release->second >= 5.0) {
                        overheat_occured_times_.erase(zone_name);
                        overheat_release_start_times_.erase(zone_name);
                        RCLCPP_WARN(node_ptr_->get_logger(),
                            "[BoardOverheat] Release: [%s] / Temp [%.1f]°C "
                            "< threshold [%.1f]°C for 5 seconds.",
                            zone_name.c_str(),
                            temp_value,
                            params.temperature_th);
                    }
                }
            }
        }
    }

    if(any_error){
        error_state = true;
    } else {
        if( overheat_occured_times_.empty()){
            error_state = false;
        }
    }

    std_msgs::msg::Bool msg;
    msg.data = error_state;
    error_pub_->publish(msg);
}
