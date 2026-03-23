#pragma once

#include "error_monitor/error_monitor_base.hpp"

class TofErrorMonitor : public ErrorMonitorBase
{
public:
    struct tParams {
        double duration_sec;
        double one_d_min_dist_m;
        double one_d_max_dist_m;
        int monitoring_rate_ms;
    } params;

    const std::string paramNamespace() const override { return "tof_error"; }

    void loadParams(const YAML::Node& config) override;
    void printParams() const override;
    void startMonitor(std::shared_ptr<RobotStateBlackboard> blackboard) override;

private:
    void timerCallback();
};


