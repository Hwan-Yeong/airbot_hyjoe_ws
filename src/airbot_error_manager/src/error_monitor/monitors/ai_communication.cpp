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
        "error/f_code/ai_connect", 10);
    timer_ = node_ptr_->create_wall_timer(
        std::chrono::milliseconds(params.monitoring_rate_ms),
        [this](){ timerCallback(); }
    );
}

void AICommunicationErrorMonitor::timerCallback()
{
    auto ai_v = blackboard_->getAiVersionData();
    auto ai_t = blackboard_->getAiTemperatureData();
    auto now = std::chrono::steady_clock::now();

    if (!ai_v.is_updated || !ai_t.is_updated) return;

    bool bVersionUpdate = (ai_v.last_update_time > recorded_v_time);
    bool bTemperatureDataUpdate = (ai_t.last_update_time > recorded_t_time);

    if (bVersionUpdate) recorded_v_time = ai_v.last_update_time;
    if (bTemperatureDataUpdate) recorded_t_time = ai_t.last_update_time;

    double disconnect_time = std::chrono::duration_cast<std::chrono::seconds>(now - ai_t.last_update_time).count();

    if (!firstReceiveCheck) { 
        if (bVersionUpdate || bTemperatureDataUpdate) {
            firstReceiveCheck = true;
            errorState = false;
            monitorCnt = 0;
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[AICommunicationErrorMonitor] First AI topic received. Start monitoring.");
        } else {
            monitorCnt++;
            if (monitorCnt >= params.duration_cnt_first) {
                if (!errorState) {
                    RCLCPP_INFO(node_ptr_->get_logger(),
                        "[AICommunicationErrorMonitor] AI disconnect Error Occured! "
                        "Initial timeout %d sec",
                        params.duration_cnt_first);
                }
                errorState = true;
            }
        }
    } else { // firstReceiveCheck == true
        if (bTemperatureDataUpdate) {
            if (errorState) {
                errorState = false;
                monitorCnt = 0;
                RCLCPP_INFO(node_ptr_->get_logger(), 
                        "[AICommunicationErrorMonitor] AI disconnect Error Released!");
            }
        } else {
            monitorCnt++;
            if (monitorCnt >= params.duration_cnt || disconnect_time >= 15.0) {
                if (!errorState) {
                    RCLCPP_INFO(node_ptr_->get_logger(), 
                        "[AICommunicationErrorMonitor] AI disconnect Error Occured! "
                        "Timeout [%d sec] | disconnect_time [%.2f sec]",
                        params.duration_cnt,
                        disconnect_time);
                }
                errorState = true;
            }
        }
    }

    if (errorState && monitorCnt > 0) {
        if ((monitorCnt % 30) == 0) {
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[AICommunicationErrorMonitor] AI still disconnected... during %d sec",
                monitorCnt
            );
        }
    }

    std_msgs::msg::Bool msg;
    msg.data = errorState;
    error_pub_->publish(msg);
}
