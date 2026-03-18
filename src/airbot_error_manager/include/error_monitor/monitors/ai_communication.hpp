#pragma once

#include "error_monitor/error_monitor_base.hpp"

class AICommunicationErrorMonitor : public ErrorMonitorBase
{
public:
    struct tParams {
        int duration_cnt_first;
        int duration_cnt;
        int monitoring_rate_ms;
    } params;

    static std::string paramNamespace() { return "ai_error"; }

    void loadParams(const std::string& ns) override;
    void printParams() const override;
    void startMonitor(std::shared_ptr<RobotStateBlackboard> blackboard) override;

private:
    void timerCallback();

    bool firstReceiveCheck = false;
    bool errorState = false;
    int monitorCnt = 0;
};


