#pragma once

#include <unordered_map>
#include "error_monitor/error_monitor_base.hpp"

class BoardOverheatErrorMonitor : public ErrorMonitorBase
{
public:
    struct tParams {
        double temperature_th;
        double duration_sec;
        int monitoring_rate_ms;
    } params;

    const std::string paramNamespace() const override { return "board_overheat_error"; }

    void loadParams(const std::string& ns) override;
    void printParams() const override;
    void startMonitor(std::shared_ptr<RobotStateBlackboard> blackboard) override;

private:
    void timerCallback();

    bool error_state = false;
    std::unordered_map<std::string, float> overheat_occured_times_;
    std::unordered_map<std::string, float> overheat_release_start_times_;

    bool pre_board_overheat_error_ = false;
};


