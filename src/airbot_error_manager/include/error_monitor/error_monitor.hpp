#ifndef __ERROR_MONITOR_HPP__
#define __ERROR_MONITOR_HPP__

#include <fstream>
// #include "tf2/LinearMath/Quaternion.hpp"
// #include "tf2/LinearMath/Matrix3x3.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "sensor_msgs/msg/imu.hpp"
#include "robot_custom_msgs/msg/bottom_ir_data.hpp"
#include "robot_custom_msgs/msg/battery_status.hpp"
#include "robot_custom_msgs/msg/station_data.hpp"
#include "error_monitor/error_monitor_node.hpp"

/**
 * @brief BaseErrorMonitor 클래스는 모든 에러모니터 클래스의 기본 인터페이스를 제공하는 추상 클래스입니다.
 * 1. 각 에러모니터는 에러 판단에 필요한 데이터를 InputType으로 정의하고 error_monitor_node에서 데이터를 받아옵니다.
 * 2. 각 에러모니터는 checkError 메소드를 구현함으로써 에러 발생을 확인하는 기능을 제공합니다.
 */
template<typename T>
class BaseErrorMonitor
{
public:
    virtual ~BaseErrorMonitor() = default;

    /**
     * @brief checkError 메소드
     * 에러를 확인하고 에러 발생 시 true를 반환합니다.
     */
    virtual bool checkError(const T& input) = 0;
};

class LowBatteryErrorMonitor : public BaseErrorMonitor<robot_custom_msgs::msg::BatteryStatus>
{
public:
    using InputType = robot_custom_msgs::msg::BatteryStatus;
    bool checkError(const InputType& input) override;
};

class BatteryDischargingErrorMonitor : public BaseErrorMonitor<robot_custom_msgs::msg::BatteryStatus>
{
public:
    using InputType = robot_custom_msgs::msg::BatteryStatus;
    bool checkError(const InputType& input) override;
};

class FallDownErrorMonitor : public BaseErrorMonitor<std::pair<robot_custom_msgs::msg::BottomIrData, sensor_msgs::msg::Imu>>
{
public:
    using InputType = std::pair<robot_custom_msgs::msg::BottomIrData, sensor_msgs::msg::Imu>;
    bool checkError(const InputType& input) override;
private:
    void get_rpy_from_quaternion(const geometry_msgs::msg::Quaternion& quaternion, double& roll, double& pitch, double& yaw);
};

class LiftErrorMonitor : public BaseErrorMonitor<std::pair<robot_custom_msgs::msg::BottomIrData, sensor_msgs::msg::Imu>>
{
public:
    using InputType = std::pair<robot_custom_msgs::msg::BottomIrData, sensor_msgs::msg::Imu>;
    bool checkError(const InputType& input) override;
private:
    unsigned int errorCount = 0;
    bool errorState = false;
};

class BoardOverheatErrorMonitor : public BaseErrorMonitor<std::nullptr_t>
{
public:
    using InputType = std::nullptr_t;
    bool checkError(const InputType& input) override;
private:
    float total_temp;
    int valid_reads ;
    double avg_temp;
    std::vector<std::string> temp_files = {
        "/sys/class/thermal/thermal_zone0/temp",
        "/sys/class/thermal/thermal_zone1/temp",
        "/sys/class/thermal/thermal_zone2/temp",
        "/sys/class/thermal/thermal_zone3/temp",
        "/sys/class/thermal/thermal_zone4/temp",
        "/sys/class/thermal/thermal_zone5/temp",
        "/sys/class/thermal/thermal_zone6/temp"
    };
};

class ChargingErrorMonitor : public BaseErrorMonitor<std::pair<robot_custom_msgs::msg::BatteryStatus, robot_custom_msgs::msg::StationData>>
{
public:
    using InputType = std::pair<robot_custom_msgs::msg::BatteryStatus, robot_custom_msgs::msg::StationData>;
    bool checkError(const InputType& input) override;
private:
    uint8_t initialCharge = 0;
    bool errorState = false;
    bool isFirstCheck = true;
    double lastCheckTime = 0;
};

#endif // __ERROR_MONITOR_HPP__