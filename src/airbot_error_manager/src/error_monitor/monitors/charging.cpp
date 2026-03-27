#include "error_monitor/monitors/charging.hpp"

void ChargingErrorMonitor::loadParams(const YAML::Node& config) {
  if (!node_ptr_)
    return;

  // Default values
  params.monitoring_rate_ms = 1000;
  params.percentage_min_th = 1;
  params.percentage_max_th = 60;
  params.percentage_diff_th = 2;
  params.duration_sec = 1200.0;

  if (config["monitoring_rate_ms"]) {
    params.monitoring_rate_ms = config["monitoring_rate_ms"].as<int>();
  }
  if (config["occur"]) {
    if (config["occur"]["battery_percentage_min_th"]) {
      params.percentage_min_th =
          config["occur"]["battery_percentage_min_th"].as<int>();
    }
    if (config["occur"]["battery_percentage_max_th"]) {
      params.percentage_max_th =
          config["occur"]["battery_percentage_max_th"].as<int>();
    }
    if (config["occur"]["battery_percentage_diff_th"]) {
      params.percentage_diff_th =
          config["occur"]["battery_percentage_diff_th"].as<int>();
    }
    if (config["occur"]["duration_sec"]) {
      params.duration_sec = config["occur"]["duration_sec"].as<double>();
    }
  }
}

void ChargingErrorMonitor::printParams() const {
  if (!node_ptr_)
    return;
  RCLCPP_INFO(node_ptr_->get_logger(),
              "\n[%s] rate: %d\n"
              "percentage_min_th: %d, percentage_max_th: %d, "
              "percentage_diff_th: %d, duration_sec: %.1f",
              paramNamespace().c_str(), params.monitoring_rate_ms,
              params.percentage_min_th, params.percentage_max_th,
              params.percentage_diff_th, params.duration_sec);
}

void ChargingErrorMonitor::startMonitor(
    std::shared_ptr<RobotStateBlackboard> blackboard) {
  blackboard_ = blackboard;
  error_pub_ = node_ptr_->create_publisher<std_msgs::msg::Bool>(
      "error/e_code/charging", 10);
  timer_ = node_ptr_->create_wall_timer(
      std::chrono::milliseconds(params.monitoring_rate_ms),
      [this]() { timerCallback(); });
}

/*
 * [충전 에러 (E08)]
 * - 모니터링 주기:
 *    1000ms
 * - 발생 조건:
 *    충전 중 배터리 잔량이 1~60% 구간일 때 측정 시작.
 *    1200초(20분) 동안 배터리 증가량이 2% 이하일 경우
 * - 해제 조건:
 *    1) 스테이션 복귀/도킹 상태로 전환 시
 *    2) 충전량이 60% 초과 또는 1% 미만이 될 경우
 *    3) 1200초 검사 시 배터리가 2% 초과 정상 증가했을 경우
 */
void ChargingErrorMonitor::timerCallback() {
  /*
        < 충전 에러 검사 >
        해당 모니터는 10분마다 에러 발생/해제를 체크하지만,
        충전중이 아닐때 혹은 충전중 95%가 넘어가는 시점부터는 바로 해제를 반환합니다.
        체크하는 10분 사이의 간격에는 이전에 판단된 상태를 계속해서 반환합니다.

        1. 배터리 95% 이하일때만 에러 체크
        2. isCharging상태가 유지된 상태에서 10분마다 상태 체크
        3. chargeDiff = initialCharge - finalCharge 의 크기가 2% 이하이면 에러 발생.
        4. chargeDiff의 크기가 2% 이하가 아니면 에러 해제.
           (에러 발생/해제는 10분마다 판단해서 결과를 알려준다)
    */

  auto battery = blackboard_->getBatteryData();
  auto station = blackboard_->getStationData();
  auto robot_state = blackboard_->getRobotStateData();

  if (checkDataState(paramNamespace(), 10,
                     {battery.last_update_time, station.last_update_time}) !=
      DataState::NORMAL) {
    return;
  }

  if (!battery.is_updated || !station.is_updated || !robot_state.is_updated)
    return;

  static rclcpp::Clock clock(RCL_STEADY_TIME);
  double currentTime = clock.now().seconds();
  uint8_t batteryPercentage = battery.data.battery_percent;

  bool isCharging =
      station.data.docking_status & 0X30;  // charger found || start charging

  // 배터리 정보 로깅용
  if (batteryPercentage != prevBatteryPercentage) {
    RCLCPP_INFO(
        node_ptr_->get_logger(),
        "[%s] Docking status: 0x%02X\n"
        "Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / "
        "Percentage:[%d %%] / Current:[%.1f mA] / "
        "Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
        "Battery Cell Voltage:"
        "[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d\n"
        "Battery Version: 0x%02X",
        paramNamespace().c_str(), station.data.docking_status,
        battery.data.battery_manufacturer, battery.data.remaining_capacity,
        static_cast<int>(battery.data.battery_percent),
        battery.data.battery_current, battery.data.battery_voltage,
        battery.data.battery_temperature1, battery.data.battery_temperature2,
        battery.data.cell_voltage1, battery.data.cell_voltage2,
        battery.data.cell_voltage3, battery.data.cell_voltage4,
        battery.data.cell_voltage5, battery.data.battery_version);
    prevBatteryPercentage = batteryPercentage;
  }
  // 스테이션 복귀 명령시 해제 사양( 4: RETURN_CHARGER, 5: DOCKING )
  if (robot_state.data.state == 4 || robot_state.data.state == 5) {
    lastCheckTime = currentTime;
    initialCharge = batteryPercentage;
    errorState = false;
    isFirstCheck = true;
    std_msgs::msg::Bool msg;
    msg.data = false;
    error_pub_->publish(msg);
    return;
  }

  if (!errorState && isCharging) {
    if (params.percentage_min_th <= batteryPercentage &&
        batteryPercentage <= params.percentage_max_th) {  // 1 ~ 60 %
      if (isFirstCheck) {  // 측정 주기 타이머 시작
        lastCheckTime = currentTime;
        initialCharge = batteryPercentage;
        isFirstCheck = false;
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[%s] Docking status: 0x%02X\n"
                    "Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / "
                    "Percentage:[%d %%] / Current:[%.1f mA] / "
                    "Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
                    "Battery Cell Voltage:"
                    "[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d\n"
                    "Battery Version: 0x%02X",
                    paramNamespace().c_str(), station.data.docking_status,
                    battery.data.battery_manufacturer,
                    battery.data.remaining_capacity,
                    static_cast<int>(battery.data.battery_percent),
                    battery.data.battery_current, battery.data.battery_voltage,
                    battery.data.battery_temperature1,
                    battery.data.battery_temperature2,
                    battery.data.cell_voltage1, battery.data.cell_voltage2,
                    battery.data.cell_voltage3, battery.data.cell_voltage4,
                    battery.data.cell_voltage5, battery.data.battery_version);
      }
      double timediff = currentTime - lastCheckTime;
      if (timediff >= params.duration_sec) {
        int chargeDiff = static_cast<int>(batteryPercentage) -
                         static_cast<int>(initialCharge);
        if (chargeDiff <= params.percentage_diff_th) {  // 2%
          errorState = true;
          RCLCPP_INFO(node_ptr_->get_logger(),
                      "[%s] Docking status: 0x%02X\n"
                      "Manufacturer:[%d] / Remaining capacity:[%d mAh] / "
                      "Percentage:[%d %%] / Current:[%.1f mA] / "
                      "Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
                      "Battery Cell Voltage:"
                      "[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d\n"
                      "Battery Version: 0x%02X",
                      paramNamespace().c_str(), station.data.docking_status,
                      battery.data.battery_manufacturer,
                      battery.data.remaining_capacity,
                      static_cast<int>(battery.data.battery_percent),
                      battery.data.battery_current,
                      battery.data.battery_voltage,
                      battery.data.battery_temperature1,
                      battery.data.battery_temperature2,
                      battery.data.cell_voltage1, battery.data.cell_voltage2,
                      battery.data.cell_voltage3, battery.data.cell_voltage4,
                      battery.data.cell_voltage5, battery.data.battery_version);
          RCLCPP_INFO(node_ptr_->get_logger(),
                      "[%s] elapsed time since error check started: %.3f sec, "
                      "chargeDiff: %d %%",
                      paramNamespace().c_str(), timediff, chargeDiff);
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
  } else {
    lastCheckTime = currentTime;
    initialCharge = batteryPercentage;
    isFirstCheck = true;
  }
  prevBatteryPercentage = batteryPercentage;

  std_msgs::msg::Bool msg;
  msg.data = errorState;
  error_pub_->publish(msg);
}
