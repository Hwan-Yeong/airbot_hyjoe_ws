#include "error_monitor/monitors/ai_communication.hpp"

void AICommunicationErrorMonitor::loadParams(const YAML::Node& config) {
    if (!node_ptr_) return;

    // Default values matched with yaml for fallback
    params.duration_cnt = 10;
    params.duration_cnt_first = 180;
    params.monitoring_rate_ms = 1000;

    if (config["occure"]) {
        if (config["occure"]["duration_cnt"]) params.duration_cnt = config["occure"]["duration_cnt"].as<int>();
        if (config["occure"]["duration_cnt_first_sec"]) params.duration_cnt_first = config["occure"]["duration_cnt_first_sec"].as<int>();
        else if (config["occure"]["duration_cnt_first"]) params.duration_cnt_first = config["occure"]["duration_cnt_first"].as<int>(); // fallback
    }
    if (config["monitoring_rate_ms"]) params.monitoring_rate_ms = config["monitoring_rate_ms"].as<int>();
}

void AICommunicationErrorMonitor::printParams() const {
    if (!node_ptr_) return;
    RCLCPP_INFO(node_ptr_->get_logger(),
        "[%s] duration_cnt_first_sec: %d, duration_cnt_sec: %d, monitoring_rate_ms: %d",
        paramNamespace().c_str(),
        params.duration_cnt_first,
        params.duration_cnt,
        params.monitoring_rate_ms
    );
}

void AICommunicationErrorMonitor::startMonitor(std::shared_ptr<RobotStateBlackboard> blackboard) {
    blackboard_ = blackboard;
    error_pub_ = node_ptr_->create_publisher<std_msgs::msg::Bool>(
        "error/s_code/ai_communication", 10);
    timer_ = node_ptr_->create_wall_timer(
        std::chrono::milliseconds(params.monitoring_rate_ms),
        [this](){ timerCallback(); }
    );
}

void AICommunicationErrorMonitor::timerCallback()
{    
    auto ai_v = blackboard_->getAiVersionData();
    auto ai_t = blackboard_->getAiTemperatureData();

    static std::chrono::steady_clock::time_point recorded_latest_time;
    auto current_latest_time = std::max(ai_v.last_update_time, ai_t.last_update_time);

    bool bUpdated = (current_latest_time > recorded_latest_time);
    if (bUpdated) {
        recorded_latest_time = current_latest_time;
    }
    
    int duration_cnt = firstReceiveCheck == false ? params.duration_cnt_first : params.duration_cnt;

    if (bUpdated) {
        errorState = false;
        firstReceiveCheck = true;
        monitorCnt = 0;
    } else {
        monitorCnt++;
        if (monitorCnt >= duration_cnt) {
            if (!errorState) {
                RCLCPP_INFO(node_ptr_->get_logger(),
                    "[AICommunicationErrorMonitor] AI disconnect Error Occured! Timeout %d sec",
                    monitorCnt
                );
            }
            errorState = true;
            monitorCnt = 0;
        } else {
            if (!(monitorCnt%30)) {
                RCLCPP_INFO(node_ptr_->get_logger(),
                    "[AICommunicationErrorMonitor] AI still disconnected... during %d sec",
                    monitorCnt
                );
            }
        }
    }

    std_msgs::msg::Bool msg;
    msg.data = errorState;
    error_pub_->publish(msg);
}
