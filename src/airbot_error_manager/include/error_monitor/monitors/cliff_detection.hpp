#pragma once

#include "error_monitor/error_monitor_base.hpp"

class CliffDetectionErrorMonitor : public ErrorMonitorBase
{
public:
    struct tParams {
        double duration_sec;
        double accum_dist_th;
        int monitoring_rate_ms;
    } params;

    const std::string paramNamespace() const override { return "cliff_error"; }

    void loadParams(const std::string& ns) override;
    void printParams() const override;
    void startMonitor(std::shared_ptr<RobotStateBlackboard> blackboard) override;

private:
    void timerCallback();
};


