#pragma once

#include "error_monitor/error_monitor_base.hpp"

class CliffErrorMonitor : public ErrorMonitorBase {
 public:
  struct tParams {
    double duration_sec;
    double accum_dist_th;
    int monitoring_rate_ms;
  } params;

  const std::string paramNamespace() const override { return "cliff_error"; }

  void loadParams(const YAML::Node& config) override;
  void printParams() const override;
  void startMonitor(std::shared_ptr<RobotStateBlackboard> blackboard) override;

 private:
  void timerCallback();

  double startErrorCheckTimeArray[6] = {};
  double prePositionXArray[6] = {};
  double prePositionYArray[6] = {};
  double accumDist[6] = {};
  bool isFirstCheckArray[6] = {true, true, true, true, true, true};
  bool preErrorState[6] = {};
};
