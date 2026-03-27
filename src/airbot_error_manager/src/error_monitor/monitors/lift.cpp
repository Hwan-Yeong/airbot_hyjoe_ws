#include "error_monitor/monitors/lift.hpp"

void LiftErrorMonitor::loadParams(const YAML::Node& config) {
  if (!node_ptr_)
    return;

  // Default values
  params.monitoring_rate_ms = 10;
  params.drop_ir_adc_th = 900;
  params.drop_ir_cnt_min = 4;
  params.imu_z_acc_low_th = 9.2;
  params.imu_z_acc_hight_th = 10.5;

  if (config["monitoring_rate_ms"]) {
    params.monitoring_rate_ms = config["monitoring_rate_ms"].as<int>();
  }
  if (config["occur"]) {
    if (config["occur"]["drop_ir_adc_th"]) {
      params.drop_ir_adc_th = config["occur"]["drop_ir_adc_th"].as<int>();
    }
    if (config["occur"]["drop_ir_cnt_min"]) {
      params.drop_ir_cnt_min = config["occur"]["drop_ir_cnt_min"].as<int>();
    }
    if (config["occur"]["imu_z_acc_low_th"]) {
      params.imu_z_acc_low_th =
          config["occur"]["imu_z_acc_low_th"].as<double>();
    }
    if (config["occur"]["imu_z_acc_hight_th"]) {
      params.imu_z_acc_hight_th =
          config["occur"]["imu_z_acc_hight_th"].as<double>();
    }
  }
}

void LiftErrorMonitor::printParams() const {
  if (!node_ptr_)
    return;
  RCLCPP_INFO(node_ptr_->get_logger(),
              "\n[%s] rate: %d\n"
              "drop_ir_adc: %d, ir_cnt_min: %d, imu_z_acc_low: %.1f, "
              "imu_z_acc_high: %.1f",
              paramNamespace().c_str(), params.monitoring_rate_ms,
              params.drop_ir_adc_th, params.drop_ir_cnt_min,
              params.imu_z_acc_low_th, params.imu_z_acc_hight_th);
}

void LiftErrorMonitor::startMonitor(
    std::shared_ptr<RobotStateBlackboard> blackboard) {
  blackboard_ = blackboard;
  error_pub_ = node_ptr_->create_publisher<std_msgs::msg::Bool>(
      "error/s_code/lifted", 10);
  timer_ = node_ptr_->create_wall_timer(
      std::chrono::milliseconds(params.monitoring_rate_ms),
      [this]() { timerCallback(); });
}

/*
 * [들림(Lift) 에러 (S07-1)]
 * - 모니터링 주기:
 *    10ms
 * - 발생 조건:
 *    아래 두 조건 동시 성립 10회(100ms 누적) 도달 후, 2초 이상 지속 시 최종 발생
 *      1) (IR) 하단 IR 센서 중 adc값 900 미만인 센서가 4개 이상 감지
 *      2) (IMU) Z축 가속도 값이 9.2 미만 또는 10.5를 초과 감지
 * - 해제 조건:
 *    1) 충전 스테이션 단자 감지 시 즉시 해제
 *    2) 모든 IR 센서가 임계값 900 이상으로 회복 시 해제 (바닥 감지)
 */
void LiftErrorMonitor::timerCallback() {
  auto ir = blackboard_->getIrData();
  auto imu = blackboard_->getImuData();
  auto st = blackboard_->getStationData();

  if (checkSensorState(paramNamespace(), 10,
                       {ir.last_update_time, imu.last_update_time,
                        st.last_update_time}) != DataState::NORMAL) {
    return;
  }

  if (!ir.is_updated || !imu.is_updated || !st.is_updated)
    return;

  static rclcpp::Clock clock(RCL_STEADY_TIME);

  int count = (ir.data.adc_ff < params.drop_ir_adc_th) +
              (ir.data.adc_fl < params.drop_ir_adc_th) +
              (ir.data.adc_fr < params.drop_ir_adc_th) +
              (ir.data.adc_bb < params.drop_ir_adc_th) +
              (ir.data.adc_bl < params.drop_ir_adc_th) +
              (ir.data.adc_br < params.drop_ir_adc_th);

  bool isChargerConnect =
      st.data.docking_status & 0x10;  // 충전 단자 인식 시 LiftFlag 해제

  // 모든 IR 센서가 false일 경우 에러 해제. 또는 충전 단자 인식 시 에러 해제
  if (count == 0 || isChargerConnect) {
    if (errorState) {
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[%s] LiftError Resolved! isChargerConnect = %d",
                  paramNamespace().c_str(), isChargerConnect);
    }
    if (errorCount > 0) {
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[%s] Error Clear. "
                  "errorCount = %d, count = %d, isChargerConnect = %d",
                  paramNamespace().c_str(), errorCount, count,
                  isChargerConnect);
    }
    errorCount = 0;
    errorState = false;
    irLiftFlag = false;
  } else if (
      count >=
      params.drop_ir_cnt_min) {  // ir 센서 true개수 4개 이상이면 ir 들림 의심
    if (!irLiftFlag) {
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[%s] Over 4 IR sensors Lift Detected! "
                  "(adc_ff : %d)  (adc_fl : %d) (adc_fr :%d) "
                  "(adc_bb : %d) (adc_bl : %d) (adc_br : %d)",
                  paramNamespace().c_str(), ir.data.adc_ff, ir.data.adc_fr,
                  ir.data.adc_fr, ir.data.adc_bb, ir.data.adc_bl,
                  ir.data.adc_br);
    }
    irLiftFlag = true;
  } else {
    irLiftFlag = false;
  }

  double acc_z = imu.data.linear_acceleration.z;

  //250521 KKS : 낙하가 감지되지 않을 경우 z축 검사하지 않음
  if (irLiftFlag && (acc_z <= params.imu_z_acc_low_th ||
                     acc_z >= params.imu_z_acc_hight_th)) {
    // [250407] hyjoe : 들림 에러 발생 의심시 imu z축 가속도값 로깅
    // (승월이나 전도시에도 해당 로그 나올 수 있음. 정확한 상태 진단을 위해 일단 로깅)
    if (!imuLiftFlag) {
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[%s] Imu z axis acceleration Lift Detected! "
                  "(acc_z: %.3f m/s^2)",
                  paramNamespace().c_str(), acc_z);
    }
    imuLiftFlag = true;
  } else {
    imuLiftFlag = false;
  }

  if (irLiftFlag && imuLiftFlag) {
    errorCount++;
  } else if (errorCount > 0) {  // 값이 잠깐 튀어도 바로 초기화 하지 않기 위해
    errorCount--;
  } /* else {
        errorCount = 0;
    }*/

  if (errorCount >= 10) {
    // [250407] hyjoe : 들림 에러 발생 시 에러 체크 카운터 로깅
    if (!liftErrorCandidate) {
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[%s] (Error Suspected!) IR & IMU both Lift Detected! "
                  "(error count: %d)",
                  paramNamespace().c_str(), static_cast<int>(errorCount));
      prevTime = clock.now().seconds();
    }
    liftErrorCandidate = true;
  }

  double timeDiff = clock.now().seconds() - prevTime;
  if (liftErrorCandidate) {
    if (!irLiftFlag) {
      liftErrorCandidate = false;
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[%s] IR & IMU both Lift Detected BUT resolve in 2 sec "
                  "(error count: %d, IR list duration: %.3f sec)",
                  paramNamespace().c_str(), static_cast<int>(errorCount),
                  timeDiff);
    } else {
      if (timeDiff >= 2) {
        if (!errorState) {
          RCLCPP_INFO(node_ptr_->get_logger(),
                      "[%s] IR & IMU both Lift Detected! "
                      "(error count: %d, IR lift duration: %.3f sec)",
                      paramNamespace().c_str(), static_cast<int>(errorCount),
                      timeDiff);
        }
        errorState = true;
      }
    }
  }

  std_msgs::msg::Bool msg;
  msg.data = errorState;
  error_pub_->publish(msg);
}
