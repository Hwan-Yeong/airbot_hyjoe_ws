#pragma once

#include "error_monitor/error_monitor_base.hpp"

class BatteryDischargingErrorMonitor : public ErrorMonitorBase
{
public:
    struct tParams {
        int occure_percentage_min;
        int occure_percentage_max;
        double occure_duration_sec;
        int resolve_percentage_th;
        double resolve_duration_sec;
        int monitoring_rate_ms;
    } params;

    const std::string paramNamespace() const override { return "discharging_error"; }

    void loadParams(const YAML::Node& config) override;
    void printParams() const override;
    void startMonitor(std::shared_ptr<RobotStateBlackboard> blackboard) override;

private:
    void timerCallback();

    bool error_state = false;
    bool charge_flag = true;
    double resolve_time_diff = 0.0;
    double resolve_start_time = 0.0;
    double prev_time = 0.0;
    bool prev_state = false;
    bool init_setting = false;
    bool is_first_logging = true;
};


