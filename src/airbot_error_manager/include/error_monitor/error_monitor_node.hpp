#ifndef __ERROR_MONITOR_NODE_HPP__
#define __ERROR_MONITOR_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "error_monitor/error_monitor.hpp"

template<typename T>
class BaseErrorMonitor;
using namespace std::chrono_literals;

class ErrorMonitorNode : public rclcpp::Node
{
public:
    ErrorMonitorNode();
    ~ErrorMonitorNode();

    template<typename T, typename MonitorType>
    void addMonitor(std::shared_ptr<MonitorType> monitor) {
        monitors_[typeid(MonitorType)] = monitor;
    }

    template <typename T>
    bool checkAllErrors(const T& input) {
        for (auto& [key, monitor] : monitors_) {
            auto typedMonitor = std::static_pointer_cast<BaseErrorMonitor<T>>(monitor);
            if (typedMonitor && typedMonitor->checkError(input)) {
                return true;
            }
        }
        return false;
    }

private:
    void initVariables();
    void errorMonitor();
    void bottomStatusCallback(const robot_custom_msgs::msg::BottomIrData::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void batteryCallback(const robot_custom_msgs::msg::BatteryStatus::SharedPtr msg);

    bool isBottomStatusUpdate;
    bool isImuUpdate;
    bool isBatteryUpdate;
    int  count=0;

    robot_custom_msgs::msg::BatteryStatus battery_data;
    robot_custom_msgs::msg::BottomIrData bottom_status_data;
    sensor_msgs::msg::Imu imu_data;

    rclcpp::Subscription<robot_custom_msgs::msg::BottomIrData>::SharedPtr bottom_status_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::BatteryStatus>::SharedPtr battery_status_sub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr battery_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fall_down_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr board_temperature_error_pub_;

    std::map<std::type_index, std::shared_ptr<void>> monitors_;

    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // __ERROR_MONITOR_NODE_HPP__