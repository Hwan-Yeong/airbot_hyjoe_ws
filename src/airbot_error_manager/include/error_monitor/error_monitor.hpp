#ifndef ERROR_MONITOR_HPP
#define ERROR_MONITOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "error_monitor/board_temperature_monitor/board_temperature_monitor.hpp"
#include "error_monitor/fall_down_monitor/fall_down_monitor.hpp"
#include "error_monitor/battery_monitor/battery_monitor.hpp"
#include "error_monitor/error_monitor_interface.hpp"

using namespace std::chrono_literals;

class ErrorMonitor : public rclcpp::Node
{
public:
    ErrorMonitor();
    ~ErrorMonitor();

    template<typename T>
    void addMonitor(std::shared_ptr<BaseErrorMonitor<T>> monitor) {
        monitors_[typeid(T)].push_back(monitor);
    }

    template <typename T>
    bool checkAllErrors(const T& input) {
        auto it = monitors_.find(typeid(T));
        if (it != monitors_.end()) {
            for (const auto& monitor : it->second) {
                auto typedMonitor = std::static_pointer_cast<BaseErrorMonitor<T>>(monitor);
                if (typedMonitor->checkError(input)) {
                    return true;
                }
            }
        }
        return false;
    }

private:
    BoardTemperatureMonitor board_temperature_monitor_;
    FallDownMonitor fall_down_monitor_;
    // BatteryMonitor battery_monitor_;

    std::map<std::type_index, std::vector<std::shared_ptr<void>>> monitors_;

    // <std_msgs::msg::UInt8>
    rclcpp::Subscription<robot_custom_msgs::msg::BottomIrData>::SharedPtr bottom_status_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::BatteryStatus>::SharedPtr battery_status_sub_;


    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr board_temperature_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fall_down_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr battery_error_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    robot_custom_msgs::msg::BatteryStatus::SharedPtr battery_data;
    robot_custom_msgs::msg::BottomIrData bottom_status_data;
    sensor_msgs::msg::Imu imu_data;

    bool isBottomStatusUpdate;
    bool isImuUpdate;
    bool isBatteryUpdate;
    int  count=0;

    void initVariables();
    void errorMonitor();
    void bottomStatusCallback(const robot_custom_msgs::msg::BottomIrData::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void batteryCallback(const robot_custom_msgs::msg::BatteryStatus::SharedPtr msg);
};

#endif // ERROR_MONITOR_HPP