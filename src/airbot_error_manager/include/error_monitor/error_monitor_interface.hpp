#ifndef __ERROR_MONITOR_INTERFACE_HPP__
#define __ERROR_MONITOR_INTERFACE_HPP__

#include "error_monitor/error_monitor.hpp"

class ErrorMonitor;

template<typename T>
class BaseErrorMonitor
{
public:
    virtual ~BaseErrorMonitor() = default;

    virtual bool checkError(const T& input) = 0;

protected:
    // virtual bool checkErrorImpl(const T& input) = 0;

    // std::shared_ptr<ErrorMonitor> node_ptr_{};
};

class BatteryErrorMonitor : public BaseErrorMonitor<robot_custom_msgs::msg::BatteryStatus>
{
public:
    using InputType = robot_custom_msgs::msg::BatteryStatus;
    // BatteryErrorMonitor(std::shared_ptr<ErrorMonitor> node_ptr_);
    bool checkError(const InputType& input) override;
private:
    // bool checkErrorImpl(const robot_custom_msgs::msg::BatteryStatus& input) override;
};

#endif // __ERROR_MONITOR_INTERFACE_HPP__