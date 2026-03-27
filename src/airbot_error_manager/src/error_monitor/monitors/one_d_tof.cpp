#include "error_monitor/monitors/one_d_tof.hpp"

void OneDTofErrorMonitor::loadParams(const YAML::Node& config) {
  if (!node_ptr_)
    return;

  // Default values
  params.monitoring_rate_ms = 50;
  params.duration_sec = 60.0;
  params.one_d_min_dist_m = 0.03;
  params.one_d_max_dist_m = 0.13;

  if (config["monitoring_rate_ms"]) {
    params.monitoring_rate_ms = config["monitoring_rate_ms"].as<int>();
  }
  if (config["occur"]) {
    if (config["occur"]["duration_sec"]) {
      params.duration_sec = config["occur"]["duration_sec"].as<double>();
    }
    if (config["occur"]["one_d_min_dist_m"]) {
      params.one_d_min_dist_m =
          config["occur"]["one_d_min_dist_m"].as<double>();
    }
    if (config["occur"]["one_d_max_dist_m"]) {
      params.one_d_max_dist_m =
          config["occur"]["one_d_max_dist_m"].as<double>();
    }
  }
}

void OneDTofErrorMonitor::printParams() const {
  if (!node_ptr_)
    return;
  RCLCPP_INFO(node_ptr_->get_logger(),
              "\n[%s] rate: %d\n"
              "duration_sec: %.1f, min_dist: %.2f, max_dist: %.2f",
              paramNamespace().c_str(), params.monitoring_rate_ms,
              params.duration_sec, params.one_d_min_dist_m,
              params.one_d_max_dist_m);
}

void OneDTofErrorMonitor::startMonitor(
    std::shared_ptr<RobotStateBlackboard> blackboard) {
  blackboard_ = blackboard;
  error_pub_ = node_ptr_->create_publisher<std_msgs::msg::Bool>(
      "error/s_code/top_tof_obstacle_error", 10);
  timer_ = node_ptr_->create_wall_timer(
      std::chrono::milliseconds(params.monitoring_rate_ms),
      [this]() { timerCallback(); });
}

/*
 * [1D ToF 에러 (S09)]
 * - 모니터링 주기:
 *    50ms
 * - 발생 조건:
 *    1D ToF 거리값이 0.03m~0.13m 범위 안에 존재하고, 상태가 60초 이상 지속될 시 발생
 * - 해제 조건:
 *    ToF 거리값이 측정 가능 범위 이탈(0.03m 미만 또는 0.13m 초과)인 값이 
 *    1회(프레임)라도 측정될 시 발생 타이머 초기화 (해제)
 */
void OneDTofErrorMonitor::timerCallback() {
  auto tof = blackboard_->getTofData();

  if (checkDataState(paramNamespace(), 10, {tof.last_update_time}) !=
      DataState::NORMAL) {
    return;
  }

  if (!tof.is_updated)
    return;

  static rclcpp::Clock clock(RCL_STEADY_TIME);

  if (tof.data.top >= params.one_d_min_dist_m &&
      tof.data.top <= params.one_d_max_dist_m) {
    if (!is_first_detect) {
      check_oned_startTime = clock.now().seconds();
      RCLCPP_INFO(node_ptr_->get_logger(), "[%s] check start dist=%.2f",
                  paramNamespace().c_str(), tof.data.top);
      is_first_detect = true;
      next_check_sec = 1;
    }

    double time_diff = clock.now().seconds() - check_oned_startTime;
    if ((time_diff >= next_check_sec) && !isError) {  // 1초 단위로 로깅
      RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Error Checking for %.2fsec",
                  paramNamespace().c_str(), time_diff);
      next_check_sec += 1;
    }

    // 에러 상태가 60초 이상 유지된 경우
    if (time_diff >= params.duration_sec) {
      if (!isError) {
        RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Error occurred",
                    paramNamespace().c_str());
      }
      next_check_sec = 1;
      isError = true;
    }
  } else {
    is_first_detect = false;
    if (isError) {
      RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Error resolved",
                  paramNamespace().c_str());
    }
    isError = false;
  }

  std_msgs::msg::Bool msg;
  msg.data = isError;
  error_pub_->publish(msg);
}
