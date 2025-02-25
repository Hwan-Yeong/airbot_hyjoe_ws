#ifndef UART_COMMUNICATION_HPP
#define UART_COMMUNICATION_HPP

#include <chrono>
#include <vector>
#include <mutex>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <atomic>
#include <thread>
#include <dlfcn.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/string.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "robot_custom_msgs/msg/battery_status.hpp"
#include "robot_custom_msgs/msg/motor_status.hpp"
#include "robot_custom_msgs/msg/tof_data.hpp"
#include "robot_custom_msgs/msg/imu_calibration.hpp"
#include "robot_custom_msgs/msg/rpm_control.hpp"
#include "robot_custom_msgs/msg/station_data.hpp"

#include "robot_bringup/uart_helpers.h"


using namespace std::chrono_literals;

// ErrorMapping 구조체 정의
struct ErrorMapping
{
    int rank;
    std::string error_string;
};

typedef struct TransmissionData
{
    double linear_velocity;  // Linear Velocity (m/s)
    double angular_velocity; // Angular Velocity (radians/s)
    int16_t left_mt_rpm;     // Left Motor RPM
    int16_t right_mt_rpm;    // Right Motor RPM
    uint8_t reset_flags;     // Reset Flags
    uint8_t motor_flags;     // Motor Enable Flags
    uint8_t docking_flags;   // DOCKING&CHARGE Flags
    uint8_t imu_flags;       // IMU Calibration Flags
    uint8_t tofnBatt_flags;       // tof Batt on/off Flags
    uint8_t index;           // Index (0~255)
} TransmissionData;

enum class ResetCommand : uint8_t
{
    // 하위 4 비트 (Reset)
    NORMAL_OPERATION = 0x0, // Normal Operation (Default)
    ODOMETRY_RESET = 0x1,   // Odometry Reset
    IMU_RESET = 0x2,        // IMU Reset
    MCU_RESET = 0xF         // MCU Reset
};

enum class MotorMode : uint8_t
{
    // 상위 4 비트 (Motor Mode)
    VW_MODE = 0x0,     // Motor VW Mode (Default)
    RPM_MODE = 0x1,    // Motor RPM Mode
    MANUAL_MODE = 0x2, // Motor Manual Mode
    BRAKE_MODE = 0x3,  // Motor Brake Mode
    DISABLE_MODE = 0xF // Motor Disable Mode
};

// DockingChargeCommand enum (상위 4 비트와 하위 4 비트에 해당하는 충전 및 도킹 명령)
enum class DockingChargeCommand : uint8_t
{
    // 상위 4 비트 (CHARGE)
    CHARGE_CONTROL_MCU = 0x0,      // MCU에서 충전 제어 (Auto) - 3.8A 고속충전 (Default)
    CHARGE_CONTROL_AP = 0x1,       // MCU에서 충전 제어 (Auto) - 1A 저속충전
    HIGH_SPEED_CHARGE_START = 0x2, // 고속 충전 시작 (3.8A 고속충전)
    LOW_SPEED_CHARGE_START = 0x3,  // 저속 충전 시작 (1A 저속충전)
    CHARGE_STOP = 0xF,             // 충전 중지

    // 하위 4 비트 (DOCKING)
    DOCKING_START = 0x1, // 도킹 시작
    DOCKING_STOP = 0x0   // 도킹 스탑
};

enum class ImuCalibrationCommand : uint8_t
{
    // 하위 4 비트 ( Calibration )
    NORMAL = 0x0, // (Default)
    CALIBRATION_START = 0x1,
    END_OF_ROTATION = 0x2
};

enum class ToFnBatt : uint8_t
{
    // 상위 4 비트
    BATT_NORMAL = 0x0,        // Batt Normal
    BATT_SLEEP = 0x1,           // Batt Sleep
    // 하위 4 비트 ( ToF On / oFF  )
    ON = 0x0, // ToF On  // (Default)
    OFF = 0x1 // ToF Off
};

class UARTCommunication : public rclcpp::Node
{
public:
    UARTCommunication();
    ~UARTCommunication();

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr odom_status_pub_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr bottom_status_pub_;

    rclcpp::Publisher<robot_custom_msgs::msg::BatteryStatus>::SharedPtr battery_status_pub_;
    rclcpp::Publisher<robot_custom_msgs::msg::TofData>::SharedPtr tof_data_pub_;
    rclcpp::Publisher<robot_custom_msgs::msg::MotorStatus>::SharedPtr motor_status_pub_;
    rclcpp::Publisher<robot_custom_msgs::msg::StationData>::SharedPtr station_data_pub_;
    // odom_packet_
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr odom_packet_test_pub_;
    // jig
    rclcpp::Publisher<robot_custom_msgs::msg::ImuCalibration>::SharedPtr jig_imu_calibration_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fw_version_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr version_request_sub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr navi_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr docking_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_stop_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr charging_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr odom_imu_reset_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tof_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr battery_sleep_sub_;
    // jig
    rclcpp::Subscription<robot_custom_msgs::msg::RpmControl>::SharedPtr jig_moter_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr jig_battery_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr jig_imu_sub;

    rclcpp::Time last_cmd_vel_time_;
    rclcpp::Duration timeout_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer2_;
    rclcpp::TimerBase::SharedPtr timer3_;

    UARTHelpers uart_;
    std::mutex serial_mutex_;
    std::mutex velocity_mutex_;
    std::atomic<bool> serial_thread_running_;
    std::thread serial_thread_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::Quaternion imu_orientation;

    nav_msgs::msg::Odometry odom_msg;
    sensor_msgs::msg::Imu imu_msg;
    std_msgs::msg::UInt8 imu_odom_status_msg;
    std_msgs::msg::UInt8 bottom_status_msg;          // cliff & lift
    robot_custom_msgs::msg::StationData station_msg; // signal & docking status
    robot_custom_msgs::msg::BatteryStatus battery_msg;
    robot_custom_msgs::msg::TofData tof_msg;
    std_msgs::msg::UInt8 imu_calib_status;
    robot_custom_msgs::msg::MotorStatus motor_msg;
    std_msgs::msg::String fwVersion_msg;
    std_msgs::msg::String cradleVersion_msg;
    geometry_msgs::msg::TransformStamped transform;
    robot_custom_msgs::msg::ImuCalibration jig_imu_calibration_msg;//jig related variables

    const uint8_t PRE_AMBLE_0 = 0xAA;
    const uint8_t PRE_AMBLE_1 = 0x55;

    TransmissionData g_transmission_data;
    double previous_roll, previous_pitch, previous_yaw;
    double amcl_pose_x_, amcl_pose_y_, amcl_pose_angle_;

private:
    // 명령어 타입을 정의하는 enum
    enum class CommandType : uint8_t
    {
        // RX_LEGACYPROTOCOL_  = 0x10,
        RX_V1PROTOCOL_ = 0x11,
        RX_TOF16PROTOCOL_ = 0x12,

        TX_V1PROTOCOL_ = 0x51,
    };

#pragma pack(push, 1) // Ensure no padding between struct members
    typedef struct ErrorCode
    {
        uint32_t ERROR_LEFT_MT_UART : 1;     // Bit 0
        uint32_t ERROR_LEFT_MT_STALL : 1;    // Bit 1
        uint32_t ERROR_LEFT_MT_SENSOR : 1;   // Bit 2
        uint32_t ERROR_LEFT_MT_OVERHEAT : 1; // Bit 3
        uint32_t ERROR_LEFT_MT_CURRENT : 1;  // Bit 4
        uint32_t ERROR_LEFT_MT_CONTROL : 1;  // Bit 5
        uint32_t RESERVED_1 : 1;             // Bit 6 (Reserved)
        uint32_t ERROR_RIGHT_MT_UART : 1;    // Bit 7

        uint32_t ERROR_RIGHT_MT_STALL : 1;    // Bit 8
        uint32_t ERROR_RIGHT_MT_SENSOR : 1;   // Bit 9
        uint32_t ERROR_RIGHT_MT_OVERHEAT : 1; // Bit 10
        uint32_t ERROR_RIGHT_MT_CURRENT : 1;  // Bit 11
        uint32_t ERROR_RIGHT_MT_CONTROL : 1;  // Bit 12
        uint32_t RESERVED_2 : 1;              // Bit 13 (Reserved)
        uint32_t ERROR_LEFT_TOF_I2C : 1;      // Bit 14
        uint32_t ERROR_RIGHT_TOF_I2C : 1;     // Bit 15

        uint32_t ERROR_1D_TOF_UART : 1;           // Bit 16
        uint32_t ERROR_IMU_UART : 1;              // Bit 17
        uint32_t ERROR_TOF_CHECK : 1;             // Bit 18
        uint32_t ERROR_BATTERY_UART : 1;          // Bit 19
        uint32_t ERROR_CHARGE_OVERCURRENT : 1;    // Bit 20
        uint32_t ERROR_DISCHARGE_OVERCURRENT : 1; // Bit 21
        uint32_t ERROR_BATTERY_TEMP : 1;          // Bit 22
        uint32_t ERROR_OVERVOLTAGE : 1;           // Bit 23

        uint32_t ERROR_SHORT_CIRCUIT : 1;      // Bit 24
        uint32_t ERROR_CHARGE : 1;             // Bit 25
        uint32_t ERROR_BATTERY_PROTECTION : 1; // Bit 26
        uint32_t RESERVED_3 : 1;               // Bit 27 (Reserved)
        uint32_t ERROR_FALLING : 1;            // Bit 28 (Warning)
        uint32_t ERROR_DOCKING : 1;            // Bit 29 (Warning)
        uint32_t RESERVED_4 : 1;               // Bit 30 (Reserved)
        uint32_t RESERVED_5 : 1;               // Bit 31 (Reserved)
    } ErrorCode;
#pragma pack(pop)     // Reset to default packing

#pragma pack(push, 1) // Ensure no padding between struct members
    typedef struct V1DataPacket
    {
        uint8_t pre_amble_0; // 0xAA
        uint8_t pre_amble_1; // 0x55
        uint8_t data_size;   // Data Size: 166
        uint8_t command;     // 0x11

        double x_position; // Odometry X (double), unit: mm (8 bytes for double) (p)
        double y_position; // Odometry Y (double), unit: mm (8 bytes for double) (p)

        uint8_t left_motor_status;  // Left Motor Status (p)
        uint8_t right_motor_status; // Right Motor Status (p)

        uint8_t motor_mode;       // Motor Mode (p)
        uint8_t left_motor_type;  // Left Motor Type: 0x01 or 0x02 (p)
        uint8_t right_motor_type; // Right Motor Type: 0x01 or 0x02 (p)

        uint8_t error_code_1; // Error Code 1 (0x00: No Error, 0x01: Error)
        uint8_t error_code_2; // Error Code 2
        uint8_t reserve;      // Reserved field

        uint8_t imu_index; // IMU Index (0-255)
        int16_t imu_yaw;   // IMU Yaw (Angle in degrees, +/- 180˚)
        int16_t imu_pitch; // IMU Pitch (Angle in degrees, +/- 90˚)
        int16_t imu_roll;  // IMU Roll (Angle in degrees, +/- 180˚)

        int16_t imu_x_acc; // IMU X-axis Acceleration (mg) (p)
        int16_t imu_y_acc; // IMU Y-axis Acceleration (mg) (p)
        int16_t imu_z_acc; // IMU Z-axis Acceleration (mg) (p)

        int16_t cell_voltage1; // Cell Voltage 1-5 (mV) (p)
        int16_t cell_voltage2; // Cell Voltage 1-5 (mV) (p)
        int16_t cell_voltage3; // Cell Voltage 1-5 (mV) (p)
        int16_t cell_voltage4; // Cell Voltage 1-5 (mV) (p)
        int16_t cell_voltage5; // Cell Voltage 1-5 (mV) (p)

        int16_t total_capacity;       // Total Capacity (mAh) (p)
        int16_t remaining_capacity;   // Remaining Capacity (mAh) (p)
        uint8_t battery_manufacturer; // Battery Manufacturer: 0x00, 0x01, or 0x02 (p)

        uint8_t charge_status; // Charge Status

        uint8_t imu_odometry_status;         // IMU and Odometry Status
        uint8_t bottom_wheel_lifting_sensor; // Bottom & Wheel Lifting Sensor
        uint8_t dock_sig_status;             // Dock Signal Status (m)
        uint8_t dock_short_ir_position;      // Dock Short IR Position (m)
        uint8_t docking_status;              // Docking Status (m)
        uint8_t dock_long_ir_position;       // Dock Long IR Position (m)
        uint8_t battery_percent;             // Battery Percentage (0-100%) (p)
        int16_t battery_voltage;             // Battery Voltage (mV) (p)
        int16_t battery_current;             // Battery Current (+charge, -discharge, 1mA) (p)
        uint8_t battery_temperature1;        // Battery Temperature (°C) (Sensor 1 and 2) (p)
        uint8_t battery_temperature2;        // Battery Temperature (°C) (Sensor 1 and 2) (p)
        int16_t design_capacity;             // Design Capacity (mAh) (p)
        int16_t number_of_cycles;            // Number of Cycles (uint16_t) (p)

        int16_t top_tof;             // Top TOF (mm) (p)
        uint8_t top_1d_tof_status;   // Top 1D TOF Status (p)
        int16_t lower_left_tof[16];  // Lower Left TOF (mm) (p)
        int16_t lower_right_tof[16]; // Lower Right TOF (mm) (p)
        uint8_t lower_tof_status;    // Left/Right TOF Status (p)

        uint8_t imu_calibration_status; // IMU Calibration Status

        int32_t left_motor_encoder;  // Left Motor Encoder (int32_t) (p)
        int32_t right_motor_encoder; // Right Motor Encoder (int32_t) (p)

        int16_t left_motor_current;  // Left Motor Current (10mA) (p)
        int16_t right_motor_current; // Right Motor Current (10mA) (p)
        int16_t left_motor_rpm;      // Left Motor RPM (p)
        int16_t right_motor_rpm;     // Right Motor RPM (p)

        uint8_t cradle_adc0; // Cradle ADC (Temperature in °C)
        uint8_t cradle_adc1; // Cradle ADC (Temperature in °C)

        uint8_t cradle_fw_ver_msb; // Cradle FW Major Version
        uint8_t cradle_fw_ver_lsb; // Cradle FW Minor Version

        uint8_t charging_mode; // Charging Mode (p)

        int8_t left_motor_temperature;  // Left Motor Temperature (°C) (p)
        int8_t right_motor_temperature; // Right Motor Temperature (°C) (p)

        uint8_t firmware_ver_msb; // Firmware Major Version
        uint8_t firmware_ver_lsb; // Firmware Minor Version
        uint8_t checksum;         // Checksum (from byte 3 to byte 167)
    } V1DataPacket;
#pragma pack(pop) // Reset to default packing

    struct ToFData16Data
    {
        uint8_t pre_amble_0;
        uint8_t pre_amble_1;
        uint8_t data_size;
        uint8_t command;
        std::vector<uint16_t> left_tof;
        std::vector<uint16_t> right_tof;
        uint8_t checksum;
    };

private:
    void initializeData();
    void setCurrentVelocity(double v, double w);
    void pubTopic();

    void batterySleepCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void tofCommandCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void dockingCommandCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void e_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void charge_cmd_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    void odom_imu_reset_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void reqVersionCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void jig_motor_callback(const robot_custom_msgs::msg::RpmControl msg);
    void jig_bettery_callback(const std_msgs::msg::UInt8 msg);
    void jig_imu_callback(const std_msgs::msg::UInt8 msg);
    void timerCallback();

    void receiveData();
    void searchAndParseData(std::vector<uint8_t> &buffer);

    uint8_t getUpperBits(uint8_t byte);
    uint8_t getLowerBits(uint8_t byte);
    uint16_t combineBytesToUint16(const std::vector<uint8_t> &data, int index);
    int16_t combineBytesToInt16(const std::vector<uint8_t> &data, int index);
    uint64_t combineBytesToUint64(const std::vector<uint8_t> &data, int startIndex);
    double combineBytesToDouble(const std::vector<uint8_t> &data, int index);

    /************************************************************************* */
    void sendProtocolV1Data(TransmissionData &data);
    
    // Default Mode
    // Reset            : Normal
    // Motor            : VW
    // charge           : 고속충전 
    // Docking          : 도킹 스탑
    // ImuCalibration   : Normal 
    // ToF              : On
    void defaultTransmissionData();

    /************************************************************************* */

    /**************************************************************************************/
    // RESET
    /**************************************************************************************/

    void setReset(ResetCommand status);
    ResetCommand getResetStatus() const;
    bool isResetStatus(ResetCommand status) const;

    /**************************************************************************************/
    // Motor Enable
    /**************************************************************************************/
    void setMotorMode(MotorMode mode);
    MotorMode getMotorMode() const;
    bool isMotorMode(MotorMode mode) const;

    /**************************************************************************************/
    // Docking & Charge Flag
    /**************************************************************************************/

    void setCharge(DockingChargeCommand set);
    void setDocking(DockingChargeCommand set);
    DockingChargeCommand getChargeMode() const;
    DockingChargeCommand getDockingMode() const;
    bool isChargeMode(DockingChargeCommand mode) const;
    bool isDockingMode(DockingChargeCommand mode) const;

    /**************************************************************************************/
    // Calibration
    /**************************************************************************************/

    void setIMUCalibration(ImuCalibrationCommand set);
    ImuCalibrationCommand getIMUCalibration() const;
    bool isIMUCalibrationMode(ImuCalibrationCommand mode) const;

    /**************************************************************************************/
    // ToF On/Off n Batt Mode Change
    /**************************************************************************************/
    void setToF(ToFnBatt status);
    void setBattMode(ToFnBatt set);
    ToFnBatt getToFStatus() const;
    ToFnBatt getBattMode() const;
    bool isToFStatus(ToFnBatt status) const;
    bool isBattMode(ToFnBatt status) const;
   
    /**************************************************************************************/
    // Motor Control
    /**************************************************************************************/

    void setLinearVelocity(double velocity);
    void setAngularVelocity(double velocity);
    void setLeftMotorRpm(int16_t rpm);
    void setRightMotorRpm(int16_t rpm);

    double getLinearVelocity() const;
    double getAngularVelocity() const;
    int16_t getLeftMotorRpm() const;
    int16_t getRightMotorRpm() const;

    /************************************************************************* */
    void parseDataFields(const std::vector<uint8_t> &data);

    void setOdomMsg(const V1DataPacket &packet);
    void setOdomStatusMsg(const V1DataPacket &packet);
    void setImuMsg(const V1DataPacket &packet);
    void setBatteryMsg(const V1DataPacket &packet);
    void setTofMsg(const V1DataPacket &packet);
    void setMotorMsg(const V1DataPacket &packet);
    void setDockingMsg(const V1DataPacket &packet);
    void setBottomMsg(const V1DataPacket &packet);
    void setFwVersionMsg(const V1DataPacket &packet);

    // void setProtocolModeMsg(CommandType protocol);
    void setJigCalibrationMsg(const V1DataPacket &packet);

    void ParseV1Data(const std::vector<uint8_t> &data, V1DataPacket &packet);
    void printV1DataPacket(const V1DataPacket &packet);
    void parseToF16Data(const std::vector<uint8_t> &data, ToFData16Data &packet);
    void printToF16Data(const ToFData16Data &packet);

    std::string formatDataPacket(const std::vector<uint8_t> &data);
    uint8_t calculateChecksum(const std::vector<uint8_t> &data, size_t start, size_t end);
    std::vector<double> computeAngularVelocity(double roll, double pitch, double yaw, double roll_dot, double pitch_dot, double yaw_dot);
    geometry_msgs::msg::Quaternion create_quaternion_from_yaw(double yaw);

    double quaternion_to_euler(const geometry_msgs::msg::Quaternion &quat);
};

#endif // UART_COMMUNICATION_HPP