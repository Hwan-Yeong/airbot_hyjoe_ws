#ifndef ERROR_MONITOR_HPP
#define ERROR_MONITOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "error_monitor/board_temperature_monitor/board_temperature_monitor.hpp"
#include "error_monitor/fall_down_monitor/fall_down_monitor.hpp"


using namespace std::chrono_literals;

class ErrorMonitor : public rclcpp::Node
{
public:
    ErrorMonitor();
    ~ErrorMonitor();

private:
    BoardTemperatureMonitor board_temperature_monitor_;
    FallDownMonitor fall_down_monitor_;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr bottom_status_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr board_temperature_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fall_down_error_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std_msgs::msg::UInt8 bottom_status_data;
    sensor_msgs::msg::Imu imu_data;

    bool isBottomStatusUpdate;
    bool isImuUpdate;

    void initVariables();
    void errorMonitor();
    void bottomStatusCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
};

#endif // ERROR_MONITOR_HPP