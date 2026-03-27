#include "error_monitor/monitors/ai_communication.hpp"

void AICommunicationErrorMonitor::loadParams(const YAML::Node& config) {
  if (!node_ptr_)
    return;

  // Default values
  params.monitoring_rate_ms = 1000;
  params.duration_cnt_first = 180;
  params.duration_cnt = 10;

  if (config["monitoring_rate_ms"]) {
    params.monitoring_rate_ms = config["monitoring_rate_ms"].as<int>();
  }
  if (config["occur"]) {
    if (config["occur"]["duration_cnt_first_sec"]) {
      params.duration_cnt_first =
          config["occur"]["duration_cnt_first_sec"].as<int>();
    }
    if (config["occur"]["duration_cnt_sec"]) {
      params.duration_cnt = config["occur"]["duration_cnt_sec"].as<int>();
    }
  }
}

void AICommunicationErrorMonitor::printParams() const {
  if (!node_ptr_)
    return;
  RCLCPP_INFO(node_ptr_->get_logger(),
              "\n[%s] rate: %d\n"
              "duration_cnt_first_sec: %d, duration_cnt_sec: %d, ",
              paramNamespace().c_str(), params.monitoring_rate_ms,
              params.duration_cnt_first, params.duration_cnt);
}

void AICommunicationErrorMonitor::startMonitor(
    std::shared_ptr<RobotStateBlackboard> blackboard) {
  blackboard_ = blackboard;
  error_pub_ = node_ptr_->create_publisher<std_msgs::msg::Bool>(
      "error/f_code/ai_connect", 10);
  timer_ = node_ptr_->create_wall_timer(
      std::chrono::milliseconds(params.monitoring_rate_ms),
      [this]() { timerCallback(); });
}

/*
 * [AI 통신 에러 (F09-3)]
 * - 모니터링 주기:
 *    1000ms
 * - 발생 조건: 
 *    1) 최초: AI 버전/온도 데이터 미수신 상태에서 180초 경과 시
 *    2) 운영 중: 수신되던 온도 데이터가 마지막 갱신 15초 경과 또는 타이머 10회 갱신 누락 시
 * - 해제 조건:
 *    AI 온도 데이터가 새롭게 갱신되어 수신될 경우 즉시 해제
 */
void AICommunicationErrorMonitor::timerCallback() {
  auto ai_v = blackboard_->getAiVersionData();
  auto ai_t = blackboard_->getAiTemperatureData();
  auto now = std::chrono::steady_clock::now();

  if (!ai_v.is_updated || !ai_t.is_updated)
    return;

  bool bVersionUpdate = (ai_v.last_update_time > recorded_v_time);
  bool bTemperatureDataUpdate = (ai_t.last_update_time > recorded_t_time);

  if (bVersionUpdate)
    recorded_v_time = ai_v.last_update_time;
  if (bTemperatureDataUpdate)
    recorded_t_time = ai_t.last_update_time;

  double disconnect_time = std::chrono::duration_cast<std::chrono::seconds>(
                               now - ai_t.last_update_time)
                               .count();

  if (!firstReceiveCheck) {
    if (bVersionUpdate || bTemperatureDataUpdate) {
      firstReceiveCheck = true;
      errorState = false;
      monitorCnt = 0;
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[%s] First AI topic received. Start monitoring.",
                  paramNamespace().c_str());
    } else {
      monitorCnt++;
      if (monitorCnt >= params.duration_cnt_first) {
        if (!errorState) {
          RCLCPP_INFO(node_ptr_->get_logger(),
                      "[%s] AI disconnect Error Occured! "
                      "Initial timeout %d sec",
                      paramNamespace().c_str(), params.duration_cnt_first);
        }
        errorState = true;
      }
    }
  } else {  // firstReceiveCheck == true
    if (bTemperatureDataUpdate) {
      if (errorState) {
        errorState = false;
        monitorCnt = 0;
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[%s] AI disconnect Error Resolved!",
                    paramNamespace().c_str());
      }
    } else {
      monitorCnt++;
      if (monitorCnt >= params.duration_cnt || disconnect_time >= 15.0) {
        if (!errorState) {
          RCLCPP_INFO(node_ptr_->get_logger(),
                      "[%s] AI disconnect Error Occured! "
                      "Timeout [%d sec] | disconnect_time [%.2f sec]",
                      paramNamespace().c_str(), params.duration_cnt,
                      disconnect_time);
        }
        errorState = true;
      }
    }
  }

  if (errorState && monitorCnt > 0) {
    if ((monitorCnt % 30) == 0) {
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[%s] AI still disconnected... during %d sec",
                  paramNamespace().c_str(), monitorCnt);
    }
  }

  std_msgs::msg::Bool msg;
  msg.data = errorState;
  error_pub_->publish(msg);
}
