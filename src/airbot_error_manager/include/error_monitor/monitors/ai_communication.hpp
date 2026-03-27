#pragma once

#include "error_monitor/error_monitor_base.hpp"

class AICommunicationErrorMonitor : public ErrorMonitorBase {
 public:
  struct tParams {
    int duration_cnt_first;
    int duration_cnt;
    int monitoring_rate_ms;
  } params;

  const std::string paramNamespace() const override { return "ai_comm_error"; }

  void loadParams(const YAML::Node& config) override;
  void printParams() const override;
  void startMonitor(std::shared_ptr<RobotStateBlackboard> blackboard) override;

 private:
  void timerCallback();

  bool firstReceiveCheck = false;
  bool errorState = false;
  int monitorCnt = 0;

  std::chrono::steady_clock::time_point recorded_v_time;
  std::chrono::steady_clock::time_point recorded_t_time;
};
