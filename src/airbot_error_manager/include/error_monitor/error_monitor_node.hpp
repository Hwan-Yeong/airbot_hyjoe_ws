#ifndef __ERROR_MONITOR_NODE_HPP__
#define __ERROR_MONITOR_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
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

    void init();

    template<typename MonitorType>
    void addMonitor(std::shared_ptr<MonitorType> monitor) {
        monitor->setNode(shared_from_this());
        monitor->loadParams(MonitorType::paramNamespace());
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
    void bottomIrDataCallback(const robot_custom_msgs::msg::BottomIrData::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void batteryCallback(const robot_custom_msgs::msg::BatteryStatus::SharedPtr msg);
    void stationDataCallback(const robot_custom_msgs::msg::StationData::SharedPtr msg);
    void robotStateCallback(const robot_custom_msgs::msg::RobotState::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void tofCallback(const robot_custom_msgs::msg::TofData::SharedPtr msg);

    bool update_battery_status_low_battery, update_battery_status_battery_discharging, update_battery_status_charging,
        update_bottom_ir_data_fall_down, update_bottom_ir_data_lift, update_bottom_ir_data_cliff_detection,
        update_imu_fall_down, update_imu_lift,
        update_station_data_charging,
        update_robot_state_low_battery, update_robot_state_battery_discharging, update_robot_state_cliff_detection,
        update_odom_data_cliff_detection,
        update_tof_one_d_detection;
    int publish_cnt_low_battery_error_, publish_cnt_fall_down_error_,
        publish_cnt_board_overheat_error_, publish_cnt_battery_discharge_error_,
        publish_cnt_charging_error_, publish_cnt_lift_error_,
        publish_cnt_cliff_detection_error_,
        publish_cnt_tof_detection_error_;
    int publish_cnt_low_battery_error_rate_, publish_cnt_fall_down_error_rate_,
        publish_cnt_board_overheat_error_rate_, publish_cnt_battery_discharge_error_rate_,
        publish_cnt_charging_error_rate_, publish_cnt_lift_error_rate_,
        publish_cnt_cliff_detection_error_rate_,
        publish_cnt_tof_detection_error_rate_;

    robot_custom_msgs::msg::BatteryStatus battery_data;
    robot_custom_msgs::msg::BottomIrData bottom_ir_data;
    sensor_msgs::msg::Imu imu_data;
    robot_custom_msgs::msg::StationData station_data;
    robot_custom_msgs::msg::RobotState robot_state;
    nav_msgs::msg::Odometry odom_data;
    robot_custom_msgs::msg::TofData tof_data;

    rclcpp::Subscription<robot_custom_msgs::msg::BottomIrData>::SharedPtr bottom_ir_data_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::BatteryStatus>::SharedPtr battery_status_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::StationData>::SharedPtr station_data_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::RobotState>::SharedPtr robot_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::TofData>::SharedPtr tof_sub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        low_battery_error_pub_, fall_down_error_pub_,
        board_overheat_error_pub_, battery_discharge_error_pub_,
        charging_error_pub_, lift_error_pub_, cliff_detection_error_pub_,
        one_d_tof_detection_error_pub_;

    std::unordered_map<std::type_index, std::shared_ptr<void>> monitors_;

    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // __ERROR_MONITOR_NODE_HPP__