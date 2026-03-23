#pragma once

#include "error_monitor/error_monitor_base.hpp"

class ChargingErrorMonitor : public ErrorMonitorBase
{
public:
    struct tParams {
        int percentage_min_th;
        int percentage_max_th;
        double duration_sec;
        int monitoring_rate_ms;
    } params;

    const std::string paramNamespace() const override { return "charging_error"; }

    void loadParams(const YAML::Node& config) override;
    void printParams() const override;
    void startMonitor(std::shared_ptr<RobotStateBlackboard> blackboard) override;

private:
    void timerCallback();

    uint8_t initialCharge = 0;
    uint8_t prevChargePercentage = 0;
    bool errorState = false;
    bool isFirstCheck = true;
    double lastCheckTime = 0;
};


