#include "error_monitor/monitors/cliff.hpp"

void CliffErrorMonitor::loadParams(const YAML::Node& config) {
  if (!node_ptr_)
    return;

  // Default values
  params.monitoring_rate_ms = 10;
  params.accum_dist_th = 0.3;

  if (config["monitoring_rate_ms"]) {
    params.monitoring_rate_ms = config["monitoring_rate_ms"].as<int>();
  }
  if (config["occur"]) {
    if (config["occur"]["accum_dist_th_m"]) {
      params.accum_dist_th = config["occur"]["accum_dist_th_m"].as<double>();
    }
  }
}

void CliffErrorMonitor::printParams() const {
  if (!node_ptr_)
    return;
  RCLCPP_INFO(node_ptr_->get_logger(),
              "\n[%s] rate: %d\n"
              "accum_dist_th_m: %.1f",
              paramNamespace().c_str(), params.monitoring_rate_ms,
              params.accum_dist_th);
}

void CliffErrorMonitor::startMonitor(
    std::shared_ptr<RobotStateBlackboard> blackboard) {
  blackboard_ = blackboard;
  error_pub_ = node_ptr_->create_publisher<std_msgs::msg::Bool>(
      "error/s_code/cliff_detected", 10);
  timer_ = node_ptr_->create_wall_timer(
      std::chrono::milliseconds(params.monitoring_rate_ms),
      [this]() { timerCallback(); });
}

/*
 * [낭떠러지/낙하 IR 에러 (S07-2)]
 * - 모니터링 주기:
 *    10ms
 * - 발생 조건:
 *    6개 하단 IR 센서 중 하나라도 낙하를 감지한 상태를 유지하며,
 *    로봇 중심의 유클리드 누적 이동거리가 0.3m 이상 도달 시
 * - 해제 조건:
 *    1) 로봇 State가 대기(0), 스테이션위(7), 에러(9)일 경우
 *    2) 모든 IR 센서가 다시 바닥을 정상 감지(false)한 경우
 */
void CliffErrorMonitor::timerCallback() {
  static rclcpp::Clock clock(RCL_STEADY_TIME);

  double curDist, curPositionX, curPositionY, timeDiff;
  bool cliff[6] = {}, errorState = false;

  auto ir = blackboard_->getIrData();
  auto odom = blackboard_->getOdomData();
  auto rState = blackboard_->getRobotStateData();

  if (checkSensorState(paramNamespace(), 10,
                       {ir.last_update_time, odom.last_update_time}) !=
      DataState::NORMAL) {
    return;
  }

  if (!ir.is_updated || !odom.is_updated || !rState.is_updated)
    return;

  if (rState.data.state == 0 || rState.data.state == 7 ||
      rState.data.state == 9) {
    for (int i = 0; i < 6; i++) {
      isFirstCheckArray[i] = true;
    }
    std_msgs::msg::Bool msg;
    msg.data = false;
    error_pub_->publish(msg);
    return;
  }

  cliff[0] = ir.data.ff;
  cliff[1] = ir.data.fl;
  cliff[2] = ir.data.bl;
  cliff[3] = ir.data.bb;
  cliff[4] = ir.data.br;
  cliff[5] = ir.data.fr;

  for (int i = 0; i < 6; i++) {
    if (cliff[i] == false) {
      isFirstCheckArray[i] = true;
      if (preErrorState[i] == true) {  // 낙하 에러 해제시 로깅
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[%s] Cliff IR #[%d] : %s, "
                    "IR Detection Error Resolved",
                    paramNamespace().c_str(), i + 1,
                    cliff[i] ? "true" : "false");
      }
      preErrorState[i] = false;
      continue;
    } else {
      if (isFirstCheckArray[i]) {
        startErrorCheckTimeArray[i] = clock.now().seconds();
        prePositionXArray[i] = odom.data.pose.pose.position.x;
        prePositionYArray[i] = odom.data.pose.pose.position.y;
        accumDist[i] = 0.0;
        isFirstCheckArray[i] = false;
        // 낙하 에러 체크 시작 시 최초 한번 로깅
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[%s] Initial check => Cliff IR #[%d] : %s,"
                    "pre_position (X, Y): (%.3f, %.3f)",
                    paramNamespace().c_str(), i + 1,
                    cliff[i] ? "true" : "false", prePositionXArray[i],
                    prePositionYArray[i]);
      }

      timeDiff = clock.now().seconds() - startErrorCheckTimeArray[i];

      curPositionX = odom.data.pose.pose.position.x;
      curPositionY = odom.data.pose.pose.position.y;

      double dx = curPositionX - prePositionXArray[i];
      double dy = curPositionY - prePositionYArray[i];

      curDist = std::sqrt(dx * dx + dy * dy);
      accumDist[i] += curDist;

      prePositionXArray[i] = curPositionX;
      prePositionYArray[i] = curPositionY;

      if (accumDist[i] >= params.accum_dist_th) {
        if (preErrorState[i] == false) {
          RCLCPP_INFO(node_ptr_->get_logger(),
                      "[%s] Cliff IR #[%d] : %s, "
                      "IR Detection Error Occured,"
                      "timediff: %.3f sec,"
                      "Accumulated Distance: %.3f m",
                      paramNamespace().c_str(), i + 1,
                      cliff[i] ? "true" : "false", timeDiff, accumDist[i]);
        }
        errorState |= true;
        preErrorState[i] = true;
      }
    }
  }

  std_msgs::msg::Bool msg;
  msg.data = errorState;
  error_pub_->publish(msg);
}
