#ifndef __ERROR_MONITOR_HPP__
#define __ERROR_MONITOR_HPP__

#include <fstream>
#include <tuple>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "sensor_msgs/msg/imu.hpp"
#include "robot_custom_msgs/msg/bottom_ir_data.hpp"
#include "robot_custom_msgs/msg/battery_status.hpp"
#include "robot_custom_msgs/msg/station_data.hpp"
#include "robot_custom_msgs/msg/robot_state.hpp"
#include "robot_custom_msgs/msg/tof_data.hpp"
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

    /**
     * @brief loadParams 메소드
     * 에러 모니터에서 사용되는 에러 판단 조건 파라미터를 불러옵니다.
     */
    virtual void loadParams(const std::string& ns) = 0;

    /**
     * @brief setNode 메소드
     * 외부에서 넘겨받은 Node 객체의 포인터를 저장합니다.
     */
    void setNode(const rclcpp::Node::SharedPtr& node) {
        node_ptr_ = node;
    }
protected:
    rclcpp::Node::SharedPtr node_ptr_;
};

class LowBatteryErrorMonitor : public BaseErrorMonitor<std::pair<robot_custom_msgs::msg::BatteryStatus, robot_custom_msgs::msg::StationData>>
{
public:
    using InputType = std::pair<robot_custom_msgs::msg::BatteryStatus, robot_custom_msgs::msg::StationData>;

    struct tParams {
        int occure_percentage_min;
        int occure_percentage_max;
        int release_percentage_th;
        double release_duration_sec;
    } params;

    static std::string paramNamespace() { return "low_battery_error"; }

    bool checkError(const InputType& input) override;

    void loadParams(const std::string& ns) override {
        if (!node_ptr_) return;

        node_ptr_->declare_parameter<int>(ns + ".occure.battery_percentage_min", 10);
        node_ptr_->declare_parameter<int>(ns + ".occure.battery_percentage_max", 15);
        node_ptr_->declare_parameter<int>(ns + ".release.battery_percentage_th", 20);
        node_ptr_->declare_parameter<double>(ns + ".release.duration_sec", 30.0);

        node_ptr_->get_parameter(ns + ".occure.battery_percentage_min", params.occure_percentage_min);
        node_ptr_->get_parameter(ns + ".occure.battery_percentage_max", params.occure_percentage_max);
        node_ptr_->get_parameter(ns + ".release.battery_percentage_th", params.release_percentage_th);
        node_ptr_->get_parameter(ns + ".release.duration_sec", params.release_duration_sec);
    }

    bool station_flag = false;
    bool error_state = false;
    double current_time = 0.0;
    double release_time_diff = 0.0;
    double prev_time = 0.0;
    bool prev_state = false;
    bool init_setting = false;
    bool is_first_logging = true;
};

class BatteryDischargingErrorMonitor : public BaseErrorMonitor<std::pair<robot_custom_msgs::msg::BatteryStatus, robot_custom_msgs::msg::StationData>>
{
public:
    using InputType = std::pair<robot_custom_msgs::msg::BatteryStatus, robot_custom_msgs::msg::StationData>;

    struct tParams {
        int occure_percentage_min;
        int occure_percentage_max;
        double occure_duration_sec;
        int release_percentage_th;
        double release_duration_sec;
    } params;

    static std::string paramNamespace() { return "discharging_error"; }

    bool checkError(const InputType& input) override;

    void loadParams(const std::string& ns) override {
        if (!node_ptr_) return;

        node_ptr_->declare_parameter<int>(ns + ".occure.battery_percentage_min", 0);
        node_ptr_->declare_parameter<int>(ns + ".occure.battery_percentage_max", 10);
        node_ptr_->declare_parameter<double>(ns + ".occure.duration_sec", 10.0);
        node_ptr_->declare_parameter<int>(ns + ".release.battery_percentage_th", 15);
        node_ptr_->declare_parameter<double>(ns + ".release.duration_sec", 30.0);

        node_ptr_->get_parameter(ns + ".occure.battery_percentage_min", params.occure_percentage_min);
        node_ptr_->get_parameter(ns + ".occure.battery_percentage_max", params.occure_percentage_max);
        node_ptr_->get_parameter(ns + ".occure.duration_sec", params.occure_duration_sec);
        node_ptr_->get_parameter(ns + ".release.battery_percentage_th", params.release_percentage_th);
        node_ptr_->get_parameter(ns + ".release.duration_sec", params.release_duration_sec);
    }

    bool error_state = false;
    bool charge_flag = false;
    double release_time_diff = 0.0;
    double release_start_time = 0.0;
};

class FallDownErrorMonitor : public BaseErrorMonitor<std::pair<robot_custom_msgs::msg::BottomIrData, sensor_msgs::msg::Imu>>
{
public:
    using InputType = std::pair<robot_custom_msgs::msg::BottomIrData, sensor_msgs::msg::Imu>;

    struct tParams {
        int drop_ir_adc_th;
        int drop_ir_cnt_min;
        double imu_roll_th;
        double imu_pitch_th;
    } params;

    static std::string paramNamespace() { return "fall_down_error"; }

    bool checkError(const InputType& input) override;

    void loadParams(const std::string& ns) override {
        if (!node_ptr_) return;

        node_ptr_->declare_parameter<int>(ns + ".occure.drop_ir_adc_th", 900);
        node_ptr_->declare_parameter<int>(ns + ".occure.drop_ir_cnt_min", 3);
        node_ptr_->declare_parameter<double>(ns + ".occure.imu_roll_th_deg", 60.0);
        node_ptr_->declare_parameter<double>(ns + ".occure.imu_pitch_th_deg", 60.0);

        node_ptr_->get_parameter(ns + ".occure.drop_ir_adc_th", params.drop_ir_adc_th);
        node_ptr_->get_parameter(ns + ".occure.drop_ir_cnt_min", params.drop_ir_cnt_min);
        node_ptr_->get_parameter(ns + ".occure.imu_roll_th_deg", params.imu_roll_th);
        node_ptr_->get_parameter(ns + ".occure.imu_pitch_th_deg", params.imu_pitch_th);
    }

private:
    void get_rpy_from_quaternion(const geometry_msgs::msg::Quaternion& quaternion, double& roll, double& pitch, double& yaw);
};

class LiftErrorMonitor : public BaseErrorMonitor<std::pair<robot_custom_msgs::msg::BottomIrData, sensor_msgs::msg::Imu>>
{
public:
    using InputType = std::pair<robot_custom_msgs::msg::BottomIrData, sensor_msgs::msg::Imu>;

    struct tParams {
        int drop_ir_adc_th;
        int drop_ir_cnt_min;
        double imu_z_acc_low_th;
        double imu_z_acc_hight_th;
    } params;

    static std::string paramNamespace() { return "lift_error"; }

    bool checkError(const InputType& input) override;

    void loadParams(const std::string& ns) override {
        if (!node_ptr_) return;

        node_ptr_->declare_parameter<int>(ns + ".occure.drop_ir_adc_th", 900);
        node_ptr_->declare_parameter<int>(ns + ".occure.drop_ir_cnt_min", 4);
        node_ptr_->declare_parameter<double>(ns + ".occure.imu_z_acc_low_th", 9.2);
        node_ptr_->declare_parameter<double>(ns + ".occure.imu_z_acc_hight_th", 10.5);

        node_ptr_->get_parameter(ns + ".occure.drop_ir_adc_th", params.drop_ir_adc_th);
        node_ptr_->get_parameter(ns + ".occure.drop_ir_cnt_min", params.drop_ir_cnt_min);
        node_ptr_->get_parameter(ns + ".occure.imu_z_acc_low_th", params.imu_z_acc_low_th);
        node_ptr_->get_parameter(ns + ".occure.imu_z_acc_hight_th", params.imu_z_acc_hight_th);
    }

private:
    unsigned int errorCount = 0;
    bool errorState = false;
};

class BoardOverheatErrorMonitor : public BaseErrorMonitor<std::nullptr_t>
{
public:
    using InputType = std::nullptr_t;

    struct tParams {
        double temperature_th;
        double duration_sec;
    } params;

    static std::string paramNamespace() { return "board_overheat_error"; }

    bool checkError(const InputType& input) override;

    void loadParams(const std::string& ns) override {
        if (!node_ptr_) return;

        node_ptr_->declare_parameter<double>(ns + ".occure.temperature_th_c", 70.0);
        node_ptr_->declare_parameter<double>(ns + ".occure.duration_sec", 30.0);

        node_ptr_->get_parameter(ns + ".occure.temperature_th_c", params.temperature_th);
        node_ptr_->get_parameter(ns + ".occure.duration_sec", params.duration_sec);
    }

private:
    bool error_state = false;
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

class ChargingErrorMonitor : public BaseErrorMonitor<std::tuple<robot_custom_msgs::msg::BatteryStatus, robot_custom_msgs::msg::StationData, robot_custom_msgs::msg::RobotState>>
{
public:
    using InputType = std::tuple<robot_custom_msgs::msg::BatteryStatus, robot_custom_msgs::msg::StationData, robot_custom_msgs::msg::RobotState>;

    struct tParams {
        int percentage_th;
        double duration_sec;
    } params;

    static std::string paramNamespace() { return "charging_error"; }

    bool checkError(const InputType& input) override;

    void loadParams(const std::string& ns) override {
        if (!node_ptr_) return;

        node_ptr_->declare_parameter<int>(ns + ".occure.battery_percentage_th", 60);
        node_ptr_->declare_parameter<double>(ns + ".occure.duration_sec", 1200.0);

        node_ptr_->get_parameter(ns + ".occure.battery_percentage_th", params.percentage_th);
        node_ptr_->get_parameter(ns + ".occure.duration_sec", params.duration_sec);
    }

private:
    uint8_t initialCharge = 0;
    uint8_t prevChargePercentage = 0;
    bool errorState = false;
    bool isFirstCheck = true;
    double lastCheckTime = 0;
};

class CliffDetectionErrorMonitor : public BaseErrorMonitor<std::tuple<robot_custom_msgs::msg::BottomIrData, nav_msgs::msg::Odometry, robot_custom_msgs::msg::RobotState>>
{
public:
    using InputType = std::tuple<robot_custom_msgs::msg::BottomIrData, nav_msgs::msg::Odometry, robot_custom_msgs::msg::RobotState>;

    struct tParams {
        double duration_sec;
        double accum_dist_th;
    } params;

    static std::string paramNamespace() { return "cliff_error"; }

    bool checkError(const InputType& input) override;

    void loadParams(const std::string& ns) override {
        if (!node_ptr_) return;

        node_ptr_->declare_parameter<double>(ns + ".occure.duration_sec", 3.0);
        node_ptr_->declare_parameter<double>(ns + ".occure.accum_dist_th_m", 0.3);

        node_ptr_->get_parameter(ns + ".occure.duration_sec", params.duration_sec);
        node_ptr_->get_parameter(ns + ".occure.accum_dist_th_m", params.accum_dist_th);
    }
};

class TofErrorMonitor : public BaseErrorMonitor<robot_custom_msgs::msg::TofData>
{
public:
    using InputType = robot_custom_msgs::msg::TofData;

    struct tParams {
        double duration_sec;
        double one_d_min_dist_m;
        double one_d_max_dist_m;
    } params;

    static std::string paramNamespace() { return "tof_error"; }

    bool checkError(const InputType& input) override;

    void loadParams(const std::string& ns) override {
        if (!node_ptr_) return;

        node_ptr_->declare_parameter<double>(ns + ".occure.duration_sec", 60.0);
        node_ptr_->declare_parameter<double>(ns + ".occure.one_d_min_dist_m", 0.03);
        node_ptr_->declare_parameter<double>(ns + ".occure.one_d_max_dist_m", 0.15);

        node_ptr_->get_parameter(ns + ".occure.duration_sec", params.duration_sec);
        node_ptr_->get_parameter(ns + ".occure.one_d_min_dist_m", params.one_d_min_dist_m);
        node_ptr_->get_parameter(ns + ".occure.one_d_max_dist_m", params.one_d_max_dist_m);
    }
};

#endif // __ERROR_MONITOR_HPP__