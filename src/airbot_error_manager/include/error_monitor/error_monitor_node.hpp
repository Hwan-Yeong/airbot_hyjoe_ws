#ifndef __ERROR_MONITOR_NODE_HPP__
#define __ERROR_MONITOR_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "error_monitor/error_monitor.hpp"

template<typename T>
class BaseErrorMonitor;
using namespace std::chrono_literals;

/**
 * @brief 에러 판단에 필요한 모든 외부 데이터를 구독하고,
 * 각각 정해진 주기에 따라 에러를 확인하여, 에러 발생 시 bool 타입의 에러 토픽을 발행하는 노드입니다.
 */
class ErrorMonitorNode : public rclcpp::Node
{
public:
    ErrorMonitorNode();
    ~ErrorMonitorNode();

    template<typename MonitorType>
    void addMonitor(std::shared_ptr<MonitorType> monitor) {
        monitors_[typeid(MonitorType)] = monitor;
    }

    template <typename MonitorType, typename T>
    bool runMonitor(const T& input) {
        auto it = monitors_.find(typeid(MonitorType));
        if (it != monitors_.end()) {
            auto typedMonitor = std::static_pointer_cast<MonitorType>(it->second);
            if (typedMonitor && typedMonitor->checkError(input)) {
                return true;
            }
        }
        return false;
    }

private:
    void initVariables();
    void setParams();
    void errorMonitor();
    void bottomStatusCallback(const robot_custom_msgs::msg::BottomIrData::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void batteryCallback(const robot_custom_msgs::msg::BatteryStatus::SharedPtr msg);
    void stationDataCallback(const robot_custom_msgs::msg::StationData::SharedPtr msg);

    bool isBottomStatusUpdate, isImuUpdate, isBatteryUpdate, isStationDataUpdate;
    int publish_cnt_low_battery_error_, publish_cnt_fall_down_error_,
        publish_cnt_board_overheat_error_, publish_cnt_battery_discharge_error_,
        publish_cnt_charging_error_, publish_cnt_lift_error_;
    int publish_cnt_low_battery_error_rate_, publish_cnt_fall_down_error_rate_,
        publish_cnt_board_overheat_error_rate_, publish_cnt_battery_discharge_error_rate_,
        publish_cnt_charging_error_rate_, publish_cnt_lift_error_rate_;

    robot_custom_msgs::msg::BatteryStatus battery_data;
    robot_custom_msgs::msg::BottomIrData bottom_status_data;
    sensor_msgs::msg::Imu imu_data;
    robot_custom_msgs::msg::StationData station_data;

    rclcpp::Subscription<robot_custom_msgs::msg::BottomIrData>::SharedPtr bottom_status_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::BatteryStatus>::SharedPtr battery_status_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::StationData>::SharedPtr station_data_sub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        low_battery_error_pub_, fall_down_error_pub_,
        board_overheat_error_pub_, battery_discharge_error_pub_,
        charging_error_pub_, lift_error_pub_;

    std::unordered_map<std::type_index, std::shared_ptr<void>> monitors_;

    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // __ERROR_MONITOR_NODE_HPP__