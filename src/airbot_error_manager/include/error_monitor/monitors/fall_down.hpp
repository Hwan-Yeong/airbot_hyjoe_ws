#pragma once

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "error_monitor/error_monitor_base.hpp"

class FallDownErrorMonitor : public ErrorMonitorBase {
 public:
  struct tParams {
    int drop_ir_adc_th;
    int drop_ir_cnt_min;
    double imu_roll_th;
    double imu_pitch_th;
    int monitoring_rate_ms;
  } params;

  const std::string paramNamespace() const override {
    return "fall_down_error";
  }

  void loadParams(const YAML::Node& config) override;
  void printParams() const override;
  void startMonitor(std::shared_ptr<RobotStateBlackboard> blackboard) override;

 private:
  void timerCallback();

  bool is_first_boot = true;
  bool prev_status = false;
  double baseline_pitch_deg = 0.0;
  double baseline_roll_deg = 0.0;
  double baseline_time = 0.0;
  void get_rpy_from_quaternion(const geometry_msgs::msg::Quaternion& quaternion,
                               double& roll, double& pitch, double& yaw);
};
