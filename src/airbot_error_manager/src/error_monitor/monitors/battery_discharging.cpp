#include "error_monitor/monitors/battery_discharging.hpp"

void BatteryDischargingErrorMonitor::loadParams(const YAML::Node& config) {
  if (!node_ptr_)
    return;

  // Default values
  params.monitoring_rate_ms = 1000;
  params.occure_percentage_min = 0;
  params.occure_percentage_max = 5;
  params.occure_duration_sec = 10.0;
  params.resolve_percentage_th = 10;
  params.resolve_duration_sec = 30.0;

  if (config["monitoring_rate_ms"]) {
    params.monitoring_rate_ms = config["monitoring_rate_ms"].as<int>();
  }
  if (config["occur"]) {
    if (config["occur"]["battery_percentage_min"]) {
      params.occure_percentage_min =
          config["occur"]["battery_percentage_min"].as<int>();
    }
    if (config["occur"]["battery_percentage_max"]) {
      params.occure_percentage_max =
          config["occur"]["battery_percentage_max"].as<int>();
    }
    if (config["occur"]["duration_sec"]) {
      params.occure_duration_sec = config["occur"]["duration_sec"].as<double>();
    }
  }
  if (config["resolve"]) {
    if (config["resolve"]["battery_percentage_th"]) {
      params.resolve_percentage_th =
          config["resolve"]["battery_percentage_th"].as<int>();
    }
    if (config["resolve"]["duration_sec"]) {
      params.resolve_duration_sec =
          config["resolve"]["duration_sec"].as<double>();
    }
  }
}

void BatteryDischargingErrorMonitor::printParams() const {
  if (!node_ptr_)
    return;
  RCLCPP_INFO(node_ptr_->get_logger(),
              "\n[%s] rate: %d\n"
              "occure_min: %d, occure_max: %d, occure_duration: %.1f, "
              "resolve_th: %d, resolve_duration: %.1f",
              paramNamespace().c_str(), params.monitoring_rate_ms,
              params.occure_percentage_min, params.occure_percentage_max,
              params.occure_duration_sec, params.resolve_percentage_th,
              params.resolve_duration_sec);
}

void BatteryDischargingErrorMonitor::startMonitor(
    std::shared_ptr<RobotStateBlackboard> blackboard) {
  blackboard_ = blackboard;
  error_pub_ = node_ptr_->create_publisher<std_msgs::msg::Bool>(
      "error/s_code/discharging_battery", 10);
  timer_ = node_ptr_->create_wall_timer(
      std::chrono::milliseconds(params.monitoring_rate_ms),
      [this]() { timerCallback(); });
}

/*
 * [배터리 방전 에러 (S03)]
 * - 모니터링 주기:
 *    1000ms
 * - 발생 조건:
 *    충전 중이 아닐 때, 배터리 잔량이 0% 초과 5% 이하인 상태가 10초 이상 지속 시
 * - 해제 조건:
 *    배터리 잔량이 10%를 초과하는 상태가 30초 이상 지속 시
 */
void BatteryDischargingErrorMonitor::timerCallback() {
  auto bat = blackboard_->getBatteryData();
  auto st = blackboard_->getStationData();

  if (checkDataState(paramNamespace(), 10,
                     {bat.last_update_time, st.last_update_time}) !=
      DataState::NORMAL) {
    return;
  }

  static rclcpp::Clock clock(RCL_STEADY_TIME);
  double current_time, time_diff;

  if (st.data.docking_status & 0x70) {
    if (!charge_flag) {
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[%s]CHECK AMR CHARGING ==> dockingstatus[%02x] ",
                  paramNamespace().c_str(), st.data.docking_status);
    }
    charge_flag = true;
  } else {
    if (charge_flag) {
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[%s]CHECK AMR DISCHARGING==> dockingstatus[%02x] ",
                  paramNamespace().c_str(), st.data.docking_status);
    }
    charge_flag = false;
  }

  current_time = clock.now().seconds();

  if (!error_state) {
    if (!charge_flag && bat.data.battery_percent > 0 &&
        bat.data.battery_percent <= params.occure_percentage_max) {
      // 10 % //250730 KKS : 5%로 변경  // 0%초과 5%이하
      if (!init_setting) {
        prev_time = current_time;
        init_setting = true;
      }
      time_diff = current_time - prev_time;

      if (time_diff >= params.occure_duration_sec) {  // 10 sec
        if (!prev_state) {
          RCLCPP_INFO(
              node_ptr_->get_logger(),
              "[%s] [OCCURED] battery discharging error \n"
              "elapsed time since error check started: %.3f\n"
              "Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / "
              "Percentage:[%d %%] / Current:[%.1f mA] / "
              "Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
              "Battery Cell Voltage:"
              "[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d\n"
              "Battery Version: 0x%02X",
              paramNamespace().c_str(), time_diff,
              bat.data.battery_manufacturer, bat.data.remaining_capacity,
              static_cast<int>(bat.data.battery_percent),
              bat.data.battery_current, bat.data.battery_voltage,
              bat.data.battery_temperature1, bat.data.battery_temperature2,
              bat.data.cell_voltage1, bat.data.cell_voltage2,
              bat.data.cell_voltage3, bat.data.cell_voltage4,
              bat.data.cell_voltage5, bat.data.battery_version);
        }
        error_state = true;
      } else {
        if (is_first_logging) {
          RCLCPP_INFO(
              node_ptr_->get_logger(),
              "[%s] [START] to checking battery discharging\n"
              "Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / "
              "Percentage:[%d %%] / Current:[%.1f mA] / "
              "Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
              "Battery Cell Voltage:"
              "[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d\n"
              "Battery Version: 0x%02X",
              paramNamespace().c_str(), bat.data.battery_manufacturer,
              bat.data.remaining_capacity,
              static_cast<int>(bat.data.battery_percent),
              bat.data.battery_current, bat.data.battery_voltage,
              bat.data.battery_temperature1, bat.data.battery_temperature2,
              bat.data.cell_voltage1, bat.data.cell_voltage2,
              bat.data.cell_voltage3, bat.data.cell_voltage4,
              bat.data.cell_voltage5, bat.data.battery_version);
          is_first_logging = false;
        }
        error_state = false;
      }
    } else {
      if (init_setting) {
        init_setting = false;
        prev_time = current_time;
        is_first_logging = true;
      }
    }
  } else {  // 15% 초과 & 30초 유지
    // 250730 KKS : S03에러가 5%로 변경됨에 따라 10%초과로 변경
    if (bat.data.battery_percent > params.resolve_percentage_th) {  // 15 %
      //check time
      if (!init_setting) {
        resolve_start_time = current_time;
        init_setting = true;
      }
      resolve_time_diff = current_time - resolve_start_time;

      if (resolve_time_diff >= params.resolve_duration_sec) {  // 30 sec
        if (prev_state) {
          RCLCPP_INFO(
              node_ptr_->get_logger(),
              "[%s] [RESOLVED] battery discharging error \n"
              "elapsed time since resolve check started: %.3f\n"
              "Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / "
              "Percentage:[%d %%] / Current:[%.1f mA] / "
              "Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
              "Battery Cell Voltage:"
              "[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d\n"
              "Battery Version: 0x%02X",
              paramNamespace().c_str(), resolve_time_diff,
              bat.data.battery_manufacturer, bat.data.remaining_capacity,
              static_cast<int>(bat.data.battery_percent),
              bat.data.battery_current, bat.data.battery_voltage,
              bat.data.battery_temperature1, bat.data.battery_temperature2,
              bat.data.cell_voltage1, bat.data.cell_voltage2,
              bat.data.cell_voltage3, bat.data.cell_voltage4,
              bat.data.cell_voltage5, bat.data.battery_version);
        }
        resolve_start_time = current_time;
        is_first_logging = true;
        error_state = false;
        init_setting = false;
      } else {
        if (is_first_logging) {
          RCLCPP_INFO(
              node_ptr_->get_logger(),
              "[%s] [START] to checking battery discharging resolved\n"
              "Battery Manufacturer:[%d] / Remaining capacity:[%d mAh] / "
              "Percentage:[%d %%] / Current:[%.1f mA] / "
              "Voltage:[%.1f mV] / Temp1:[%d °C] / Temp2:[%d °C]\n"
              "Battery Cell Voltage:"
              "[1]: %d, [2]: %d, [3]: %d, [4]: %d, [5]: %d\n"
              "Battery Version: 0x%02X",
              paramNamespace().c_str(), bat.data.battery_manufacturer,
              bat.data.remaining_capacity,
              static_cast<int>(bat.data.battery_percent),
              bat.data.battery_current, bat.data.battery_voltage,
              bat.data.battery_temperature1, bat.data.battery_temperature2,
              bat.data.cell_voltage1, bat.data.cell_voltage2,
              bat.data.cell_voltage3, bat.data.cell_voltage4,
              bat.data.cell_voltage5, bat.data.battery_version);
          is_first_logging = false;
        }
      }
    } else {
      resolve_start_time = current_time;
      is_first_logging = true;
      init_setting = false;
    }
  }
  prev_state = error_state;

  std_msgs::msg::Bool msg;
  msg.data = error_state;
  error_pub_->publish(msg);
}
