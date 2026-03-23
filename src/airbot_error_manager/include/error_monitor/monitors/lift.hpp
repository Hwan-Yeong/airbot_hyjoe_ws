#pragma once

#include "error_monitor/error_monitor_base.hpp"

class LiftErrorMonitor : public ErrorMonitorBase
{
public:
    struct tParams {
        int drop_ir_adc_th;
        int drop_ir_cnt_min;
        double imu_z_acc_low_th;
        double imu_z_acc_hight_th;
        int monitoring_rate_ms;
    } params;

    const std::string paramNamespace() const override { return "lift_error"; }

    void loadParams(const YAML::Node& config) override;
    void printParams() const override;
    void startMonitor(std::shared_ptr<RobotStateBlackboard> blackboard) override;

private:
    void timerCallback();

    unsigned int errorCount = 0;
    bool errorState = false;
};


