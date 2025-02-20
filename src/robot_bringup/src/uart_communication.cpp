#include "robot_bringup/uart_communication.hpp"
#include <numeric> // std::accumulate
#include <cstdlib>
#include <string>
#include <chrono>
#include <iomanip>  // for std::setw, std::setfill
#include <optional> // for std::optional

// when 2024.12.24 ap image is download,USE_REMOVE_SYSLOG is false
// 양산에서는 disable 해야함. : icbaek.24.12.28
#define USE_REMOVE_SYSLOG true
#define TOF_ALWAYS_ON 0

auto previous_time = std::chrono::steady_clock::now();

// 에러 매핑 테이블 정의 (전역 변수)
const std::vector<ErrorMapping> error_map = {
    {22, "E04"}, // 0x00
    {23, "E07"}, // 0x01
    {2, "F01"},  // 0x02
    {20, "F11"}, // 0x03
    {21, "F12"}, // 0x04
    {1, "F15"},  // 0x05
    {-1, "F17"}, // 0x06
    {10, "S07"}, // 0x07
    {24, "S08"}  // 0x08
};

UARTCommunication::UARTCommunication()
    : Node("uart_communication"), uart_("/dev/ttydriver", 230400),timeout_(rclcpp::Duration::from_seconds(3))
{
    initializeData();
    uart_.openPort();

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    battery_status_pub_ = this->create_publisher<robot_custom_msgs::msg::BatteryStatus>("/battery_status", 10);

    tof_data_pub_ = this->create_publisher<robot_custom_msgs::msg::TofData>("tof_data", 10);

    motor_status_pub_ = this->create_publisher<robot_custom_msgs::msg::MotorStatus>("/motor_status", 10);

    station_data_pub_ = this->create_publisher<robot_custom_msgs::msg::StationData>("/station_data", 10);

    bottom_status_pub_ = this->create_publisher<std_msgs::msg::UInt8>("bottom_status", 10);

    imu_data_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_data", 10);

    odom_status_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/odom_status", 10);

    fw_version_pub_ = this->create_publisher<std_msgs::msg::String>("/fw_version", 10);

    jig_imu_calibration_pub_ = this->create_publisher<robot_custom_msgs::msg::ImuCalibration>("/jig_response_imu_calibration", 1);

    // 1109 change
    version_request_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/req_version",
        1,
        std::bind(&UARTCommunication::reqVersionCallback, this, std::placeholders::_1));

    navi_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10,
        std::bind(&UARTCommunication::cmdVelCallback, this, std::placeholders::_1));

    docking_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/docking_cmd",
        10,
        std::bind(&UARTCommunication::dockingCommandCallback, this, std::placeholders::_1));

    charging_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/charging_cmd",
        10,
        std::bind(&UARTCommunication::charge_cmd_callback, this, std::placeholders::_1));

    odom_imu_reset_sub = this->create_subscription<std_msgs::msg::UInt8>(
        "/odom_imu_reset_cmd",
        10,
        std::bind(&UARTCommunication::odom_imu_reset_callback, this, std::placeholders::_1));
    
    tof_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/cmd_tof",
        10,
        std::bind(&UARTCommunication::tofCommandCallback, this, std::placeholders::_1));

    battery_sleep_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/cmd_battery_sleep",
        10,
        std::bind(&UARTCommunication::batterySleepCallback, this, std::placeholders::_1));    

    e_stop_sub = this->create_subscription<std_msgs::msg::Bool>(
        "/e_stop",
        10,
        std::bind(&UARTCommunication::e_stop_callback, this, std::placeholders::_1));

    amcl_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "amcl_pose",
        10,
        std::bind(&UARTCommunication::amcl_pose_callback, this, std::placeholders::_1));

    jig_moter_sub = this->create_subscription<robot_custom_msgs::msg::RpmControl>(
        "jig_request_motor",
        10,
        std::bind(&UARTCommunication::jig_motor_callback, this, std::placeholders::_1));

    jig_battery_sub = this->create_subscription<std_msgs::msg::UInt8>(
        "jig_request_battery",
        10,
        std::bind(&UARTCommunication::jig_bettery_callback, this, std::placeholders::_1));

    jig_imu_sub = this->create_subscription<std_msgs::msg::UInt8>(
        "jig_request_imu",
        10,
        std::bind(&UARTCommunication::jig_imu_callback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(
        10ms, // Timer period: 10 milliseconds
        std::bind(&UARTCommunication::timerCallback, this));

    timer2_ = this->create_wall_timer(
        10ms, // Timer period: 10 milliseconds
        std::bind(&UARTCommunication::receiveData, this));

    timer3_ = this->create_wall_timer(
        10ms, // Timer period: 10 milliseconds
        std::bind(&UARTCommunication::pubTopic, this));

    serial_thread_running_ = true;

    last_cmd_vel_time_ = this->get_clock()->now();

#if USE_REMOVE_SYSLOG                                              // when 2024.12.24 ap image download
    std::string command = "sudo /usr/bin/rm -rf /var/log/syslog*"; // which rm => /usr/bin/rm
    int ret_code = system(command.c_str());
    if (ret_code == 0)
    {
        RCLCPP_INFO(this->get_logger(), "Successfully  %s", command.c_str());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to %s. Return code: %d", command.c_str(), ret_code);
    }
    sleep(1);       // safty delay time
    system("sync"); // safty sync
#endif
}

UARTCommunication::~UARTCommunication()
{
    serial_thread_running_ = false;
    uart_.closePort();
    if (serial_thread_.joinable()){
        serial_thread_.join();
    }
}

void UARTCommunication::initializeData()
{
    g_transmission_data.linear_velocity = 0;
    g_transmission_data.angular_velocity = 0;
    g_transmission_data.left_mt_rpm = 0;
    g_transmission_data.right_mt_rpm = 0;
    g_transmission_data.reset_flags = 0;
    g_transmission_data.motor_flags = 0;
    g_transmission_data.docking_flags = 0;
    g_transmission_data.imu_flags = 0x0;
    #if TOF_ALWAYS_ON > 0
    g_transmission_data.tof_flags = 0x0;//0x1;
    #else
    g_transmission_data.tof_flags = 0x1;
    #endif
    g_transmission_data.index = 0;

    previous_roll = 0;
    previous_pitch = 0;
    previous_yaw = 0;
    amcl_pose_x_ = 0;
    amcl_pose_y_ = 0;
    amcl_pose_angle_ = 0;
}

void UARTCommunication::setCurrentVelocity(double v, double w)
{
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    setMotorMode(MotorMode::VW_MODE);
    setLinearVelocity(v);
    setAngularVelocity(w);
}

void UARTCommunication::pubTopic()
{
    static int battery_cnt = 0;
    std::lock_guard<std::mutex> lock(serial_mutex_);
    odom_msg.header.stamp = this->get_clock()->now();
    transform.header.stamp = this->get_clock()->now();

    odom_pub_->publish(odom_msg);
    odom_status_pub_->publish(imu_odom_status_msg);
    tf_broadcaster_->sendTransform(transform);
    imu_data_pub_->publish(imu_msg);
    bottom_status_pub_->publish(bottom_status_msg);
    tof_data_pub_->publish(tof_msg);
    motor_status_pub_->publish(motor_msg);
    station_data_pub_->publish(station_msg);
    jig_imu_calibration_pub_->publish(jig_imu_calibration_msg);

    if (++battery_cnt % 100 == 0){
        battery_status_pub_->publish(battery_msg);
        battery_cnt = 0;
    }
}

void UARTCommunication::reqVersionCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    if (msg->data & 0x01){
        fw_version_pub_->publish(fwVersion_msg);
    }
}

void UARTCommunication::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    setCurrentVelocity(msg->linear.x, msg->angular.z);
    last_cmd_vel_time_ = this->get_clock()->now(); // Reset the watchdog timer
}

void UARTCommunication::jig_motor_callback(const robot_custom_msgs::msg::RpmControl msg)
{
    RCLCPP_INFO(this->get_logger(), "jig_motor_callback ");
    uint8_t command = msg.motor_enable;
    if (command == 0x01){
        int16_t left_motor_rpm = (int16_t)((msg.left_motor_rpm_msb << 8) | msg.left_motor_rpm_lsb);
        int16_t right_motor_rpm = (int16_t)((msg.right_motor_rpm_msb << 8) | msg.right_motor_rpm_lsb);
        defaultTransmissionData();
        setMotorMode(MotorMode::RPM_MODE);
        setLeftMotorRpm(left_motor_rpm);
        setRightMotorRpm(right_motor_rpm);
    }else{
        RCLCPP_INFO(this->get_logger(), "motor_enable error : %u ", command);
    }
}
void UARTCommunication::jig_bettery_callback(const std_msgs::msg::UInt8 msg)
{
    RCLCPP_INFO(this->get_logger(), "jig_bettery_callback ");
    uint8_t remote_mode;
    uint8_t command = msg.data;
    if (command == 0x01){
        defaultTransmissionData();
        setCharge(DockingChargeCommand::HIGH_SPEED_CHARGE_START);
    }else if (command == 0x02)
    {
        defaultTransmissionData();
        setCharge(DockingChargeCommand::LOW_SPEED_CHARGE_START);
    }else{
        RCLCPP_INFO(this->get_logger(), "command error : %u ", command);
    }
}


void UARTCommunication::jig_imu_callback(const std_msgs::msg::UInt8 msg)
{
    RCLCPP_INFO(this->get_logger(), "jig_imu_callback ");
    uint8_t command = msg.data;
    if (command == 0x01){
        setIMUCalibration(ImuCalibrationCommand::CALIBRATION_START);
    }else if(command == 0x02)
    {
        setIMUCalibration(ImuCalibrationCommand::END_OF_ROTATION);
    }else{
        RCLCPP_INFO(this->get_logger(), "command error : %u ", command);
    }
}
void UARTCommunication::timerCallback()
{
    auto current_time = this->get_clock()->now();
    sendProtocolV1Data(g_transmission_data);
}

uint8_t UARTCommunication::calculateChecksum(const std::vector<uint8_t> &data, size_t start, size_t end)
{
    uint8_t checksum = 0;
    for (size_t i = start; i <= end; ++i)
    {
        checksum += data[i];
    }
    return checksum;
}


/**************************************************************************************/
// RESET
/**************************************************************************************/

// Reset 상태를 설정하는 함수 (하위 4 비트에 Reset 명령 설정)
void UARTCommunication::setReset(ResetCommand status)
{
    if(status == ResetCommand::NORMAL_OPERATION){
        RCLCPP_INFO(this->get_logger(), "NORMAL_OPERATION ");
    }else if(status == ResetCommand::ODOMETRY_RESET){
        RCLCPP_INFO(this->get_logger(), "ODOMETRY_RESET ");
    }else if(status == ResetCommand::IMU_RESET){
        RCLCPP_INFO(this->get_logger(), "IMU_RESET ");
    }else if(status == ResetCommand::MCU_RESET){
        RCLCPP_INFO(this->get_logger(), "MCU_RESET ");
    }else{
        RCLCPP_INFO(this->get_logger(), "setReset error");
    }
    // 기존 Reset 명령을 지우고 새 명령으로 설정
    g_transmission_data.reset_flags &= 0xF0;  // 상위 4 비트는 그대로 두고, 하위 4 비트 초기화
    g_transmission_data.reset_flags |= static_cast<uint8_t>(status); // 하위 4 비트에 새로운 Reset 명령 설정
}

// Reset 상태 값을 읽어오는 함수 (하위 4 비트)
ResetCommand UARTCommunication::getResetStatus() const
{
    return static_cast<ResetCommand>(g_transmission_data.reset_flags & 0x0F); // 하위 4 비트 추출
}

// 설정된 Reset 상태가 특정 값과 일치하는지 확인하는 함수 (하위 4 비트)
bool UARTCommunication::isResetStatus(ResetCommand status) const
{
    return getResetStatus() == status;
}

/**************************************************************************************/
// Motor Enable
/**************************************************************************************/

// Motor Mode 설정 함수 (상위 4 비트에 Motor Mode 명령 설정)
void UARTCommunication::setMotorMode(MotorMode mode)
{
    MotorMode current_mode = static_cast<MotorMode>(getUpperBits(g_transmission_data.motor_flags));
    if(current_mode != mode){
        if(mode == MotorMode::VW_MODE){
            RCLCPP_INFO(this->get_logger(), "VW_MODE");
        }else if(mode == MotorMode::RPM_MODE){
            RCLCPP_INFO(this->get_logger(), "RPM_MODE");
        }else if(mode == MotorMode::MANUAL_MODE){
            RCLCPP_INFO(this->get_logger(), "MANUAL_MODE");
        }else if(mode == MotorMode::BRAKE_MODE){
            RCLCPP_INFO(this->get_logger(), "BRAKE_MODE");
        }else if(mode == MotorMode::DISABLE_MODE){
            RCLCPP_INFO(this->get_logger(), "DISABLE_MODE");
        }else{
            RCLCPP_INFO(this->get_logger(), "setMotorMode error");
        }
    }
    // 기존 모터 모드를 지우고 새 모드로 설정
    g_transmission_data.motor_flags &= 0x0F;  // 상위 4 비트 초기화 (하위 4 비트만 남기기)
    g_transmission_data.motor_flags |= (static_cast<uint8_t>(mode) << 4); // 새 모드를 상위 4 비트에 설정
}

// Motor Mode 값을 읽어오는 함수 (상위 4 비트)
MotorMode UARTCommunication::getMotorMode() const
{
    return static_cast<MotorMode>(g_transmission_data.motor_flags >> 4); // 상위 4 비트 추출
}

// Motor Mode가 설정되었는지 확인하는 함수
bool UARTCommunication::isMotorMode(MotorMode mode) const
{
    return getMotorMode() == mode;
}


/**************************************************************************************/
// Docking & Charge Flag
/**************************************************************************************/

// 충전 설정 함수 (상위 4 비트에 충전 명령 설정)
void UARTCommunication::setCharge(DockingChargeCommand set)
{
    uint8_t chargingStatus = getUpperBits(g_transmission_data.docking_flags);
    if(set != static_cast<DockingChargeCommand>(chargingStatus)){
        if (set == DockingChargeCommand::CHARGE_CONTROL_MCU){
            RCLCPP_INFO(this->get_logger(), "CHARGE_CONTROL_MCU");
        }else if(set == DockingChargeCommand::CHARGE_CONTROL_AP){
            RCLCPP_INFO(this->get_logger(), "CHARGE_CONTROL_AP");
        }else if(set == DockingChargeCommand::HIGH_SPEED_CHARGE_START){
            RCLCPP_INFO(this->get_logger(), "HIGH_SPEED_CHARGE_START");
        }else if(set == DockingChargeCommand::LOW_SPEED_CHARGE_START){
            RCLCPP_INFO(this->get_logger(), "LOW_SPEED_CHARGE_START");
        }else if(set == DockingChargeCommand::CHARGE_STOP){
            RCLCPP_INFO(this->get_logger(), "CHARGE_STOP");
        }else{
            RCLCPP_INFO(this->get_logger(), "setCharge error");
        }
    }
    
    // 기존 충전 명령을 지우고 새 명령으로 설정
    g_transmission_data.docking_flags &= 0x0F;  // 하위 4 비트는 그대로 두고, 상위 4 비트 초기화
    g_transmission_data.docking_flags |= (static_cast<uint8_t>(set) << 4); // 상위 4 비트에 새로운 충전 명령 설정
}

// 도킹 관련 설정 함수 (하위 4 비트)
void UARTCommunication::setDocking(DockingChargeCommand set)
{
    uint8_t status = getUpperBits(g_transmission_data.docking_flags);
    if(set != static_cast<DockingChargeCommand>(status)){
        if (set == DockingChargeCommand::DOCKING_START){
            RCLCPP_INFO(this->get_logger(), "DOCKING_START");
        }else if(set == DockingChargeCommand::DOCKING_STOP){
            RCLCPP_INFO(this->get_logger(), "DOCKING_STOP");
        }else{
            RCLCPP_INFO(this->get_logger(), "setDocking error");
        }
    }
    // 기존 도킹 명령을 지우고 새 명령으로 설정
    g_transmission_data.docking_flags &= 0xF0;  // 상위 4 비트는 그대로 두고, 하위 4 비트 초기화
    g_transmission_data.docking_flags |= (static_cast<uint8_t>(set) & 0x0F); // 하위 4 비트에 새로운 도킹 명령 설정
}

// 충전 관련 값을 읽어오는 함수 (상위 4 비트)
DockingChargeCommand UARTCommunication::getChargeMode() const
{
    return static_cast<DockingChargeCommand>(g_transmission_data.docking_flags >> 4); // 상위 4 비트 추출
}

// 도킹 관련 값을 읽어오는 함수 (하위 4 비트)
DockingChargeCommand UARTCommunication::getDockingMode() const
{
    return static_cast<DockingChargeCommand>(g_transmission_data.docking_flags & 0x0F); // 하위 4 비트 추출
}

// 충전 모드가 설정되었는지 확인하는 함수 (상위 4 비트)
bool UARTCommunication::isChargeMode(DockingChargeCommand mode) const
{
    return getChargeMode() == mode;
}

// 도킹 모드가 설정되었는지 확인하는 함수 (하위 4 비트)
bool UARTCommunication::isDockingMode(DockingChargeCommand mode) const
{
    return getDockingMode() == mode;
}

/**************************************************************************************/
// Calibration
/**************************************************************************************/

// IMU Calibration 명령을 설정하는 함수 (하위 4 비트)
void UARTCommunication::setIMUCalibration(ImuCalibrationCommand set)
{
    if (set == ImuCalibrationCommand::NORMAL){
        RCLCPP_INFO(this->get_logger(), "NORMAL");
    }else if(set == ImuCalibrationCommand::CALIBRATION_START){
        RCLCPP_INFO(this->get_logger(), "CALIBRATION_START");
    }else if(set == ImuCalibrationCommand::END_OF_ROTATION){
        RCLCPP_INFO(this->get_logger(), "END_OF_ROTATION");
    }else{
        RCLCPP_INFO(this->get_logger(), "setIMUCalibration error");
    }
    // 기존 IMU Calibration 명령을 지우고 새 명령으로 설정
    g_transmission_data.imu_flags &= 0xF0;  // 상위 4 비트는 그대로 두고, 하위 4 비트 초기화
    g_transmission_data.imu_flags |= (static_cast<uint8_t>(set) & 0x0F); // 하위 4 비트에 새로운 IMU Calibration 명령 설정
}

// IMU Calibration 값을 읽어오는 함수 (하위 4 비트)
ImuCalibrationCommand UARTCommunication::getIMUCalibration() const
{
    return static_cast<ImuCalibrationCommand>(g_transmission_data.imu_flags & 0x0F); // 하위 4 비트 추출
}

// IMU Calibration 모드가 설정되었는지 확인하는 함수 (하위 4 비트)
bool UARTCommunication::isIMUCalibrationMode(ImuCalibrationCommand mode) const
{
    return getIMUCalibration() == mode;
}

/**************************************************************************************/
// ToF On/Off
/**************************************************************************************/

// ToF 상태를 설정하는 함수 (하위 4 비트에 ToF On/Off 설정)
void UARTCommunication::setToF(ToFStatus status)
{
    if (status == ToFStatus::ON){
        RCLCPP_INFO(this->get_logger(), "TOF-ON");
    }else if(status == ToFStatus::OFF){
        RCLCPP_INFO(this->get_logger(), "TOF-OFF");
    }else{
        RCLCPP_INFO(this->get_logger(), "setToF Error"); 
    }
    // 하위 4 비트 설정 (ToF On/Off)
    g_transmission_data.tof_flags &= 0xF0;  // 기존 하위 4 비트 초기화 (상위 4 비트만 남기기)
    g_transmission_data.tof_flags |= static_cast<uint8_t>(status); // 하위 4 비트에 새로운 ToF 상태 설정
}

// ToF 상태 값을 읽어오는 함수 (하위 4 비트)
ToFStatus UARTCommunication::getToFStatus() const
{
    return static_cast<ToFStatus>(g_transmission_data.tof_flags & 0x0F); // 하위 4 비트 추출
}

// ToF 상태가 설정되었는지 확인하는 함수 (하위 4 비트)
bool UARTCommunication::isToFStatus(ToFStatus status) const
{
    return getToFStatus() == status;
}

/**************************************************************************************/
// Motor Control
/**************************************************************************************/

void UARTCommunication::setLinearVelocity(double velocity) {
    g_transmission_data.linear_velocity = velocity;
}

void UARTCommunication::setAngularVelocity(double velocity) {
    g_transmission_data.angular_velocity = velocity;
}

void UARTCommunication::setLeftMotorRpm(int16_t rpm) {
    g_transmission_data.left_mt_rpm = rpm;
}

void UARTCommunication::setRightMotorRpm(int16_t rpm) {
    g_transmission_data.right_mt_rpm = rpm;
}

double UARTCommunication::getLinearVelocity() const {
    return g_transmission_data.linear_velocity;
}

double UARTCommunication::getAngularVelocity() const {
    return g_transmission_data.angular_velocity;
}

int16_t UARTCommunication::getLeftMotorRpm() const {
    return g_transmission_data.left_mt_rpm;
}

int16_t UARTCommunication::getRightMotorRpm() const {
    return g_transmission_data.right_mt_rpm;
}
/**************************************************************************************/

void UARTCommunication::sendProtocolV1Data(TransmissionData& data)
{
    // 데이터 크기 계산: 헤더 4바이트 + 데이터 27바이트 = 31바이트
    std::vector<uint8_t> message(31, 0);

    message[0] = 0xAA; // PRE_AMBLE_0
    message[1] = 0x55; // PRE_AMBLE_1
    message[2] = 28;   // Data_Size (28바이트, 헤더 및 데이터 크기를 포함)
    message[3] = 0x51; // Header (Command Type)

    // Linear Velocity (mm/s로 변환 후 double -> IEEE 754 표현으로 저장)
    double linear_vel_mm = data.linear_velocity * 1000.0; // m/s를 mm/s로 변환
    uint64_t linear_velocity_bits = *reinterpret_cast<uint64_t*>(&linear_vel_mm); // IEEE 754로 변환
    for (int i = 0; i < 8; ++i) {
        message[4 + i] = (linear_velocity_bits >> (56 - 8 * i)) & 0xFF;
    }

    // Angular Velocity (radian/s, double -> IEEE 754 표현으로 저장)
    uint64_t angular_velocity_bits = *reinterpret_cast<uint64_t*>(&data.angular_velocity);
    for (int i = 0; i < 8; ++i) {
        message[12 + i] = (angular_velocity_bits >> (56 - 8 * i)) & 0xFF;
    }

    // Left Motor RPM (int16_t, Big-Endian 저장)
    message[20] = (data.left_mt_rpm >> 8) & 0xFF; // MSB
    message[21] = data.left_mt_rpm & 0xFF;        // LSB

    // Right Motor RPM (int16_t, Big-Endian 저장)
    message[22] = (data.right_mt_rpm >> 8) & 0xFF; // MSB
    message[23] = data.right_mt_rpm & 0xFF;        // LSB

    // Reset Flags
    message[24] = data.reset_flags;

    // Motor Enable Flags
    message[25] = data.motor_flags;

    // DOCKING&CHARGE Flags
    message[26] = data.docking_flags;

    // IMU Calibration Flags
    message[27] = data.imu_flags;

    // tof Flags
    message[28] = data.tof_flags;

    // Index
    message[29] = data.index; // Index (0~255)

    // 체크섬 계산 (3번째 바이트부터 29번째 바이트까지)
    message[30] = calculateChecksum(message, 3, 29); // 체크섬 값은 예시로 설정, 실제로는 데이터 기반으로 계산해야 합니다.

    uart_.sendData(message);
    // UART로 데이터 전송 (디버그용으로 출력하는 예시)
    // for (auto byte : message) {
    //     std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    // }
    // std::cout << std::endl;
}

void UARTCommunication::defaultTransmissionData(void) {
    memset(&g_transmission_data, 0, sizeof(TransmissionData));
}


uint8_t UARTCommunication::getUpperBits(uint8_t byte)
{
    return (byte >> 4);
}
uint8_t UARTCommunication::getLowerBits(uint8_t byte)
{
    return (byte & 0x0F);
}

uint16_t UARTCommunication::combineBytesToUint16(const std::vector<uint8_t> &data, int index)
{
    return (static_cast<uint16_t>(data[index]) << 8) | data[index + 1];
}

int16_t UARTCommunication::combineBytesToInt16(const std::vector<uint8_t> &data, int index)
{
    return (static_cast<int16_t>(data[index]) << 8) | data[index + 1];
}

uint32_t combineBytesToUint32(const std::vector<uint8_t> &data, size_t offset)
{
    return (data[offset] << 24) | (data[offset + 1] << 16) | (data[offset + 2] << 8) | data[offset + 3];
}

uint64_t UARTCommunication::combineBytesToUint64(const std::vector<uint8_t> &data, int startIndex)
{
    uint64_t value = 0;
    for (int i = 0; i < 8; i++)
    {
        value = (value << 8) | data[startIndex + i];
    }
    return value;
}

double UARTCommunication::combineBytesToDouble(const std::vector<uint8_t> &data, int index)
{
    double result;

    uint8_t byteArray[] = {
        data[index + 7],
        data[index + 6],
        data[index + 5],
        data[index + 4],
        data[index + 3],
        data[index + 2],
        data[index + 1],
        data[index + 0]};

    // std::memcpy(&result, &data[index], sizeof(double));
    std::memcpy(&result, byteArray, sizeof(double));

    return result;
}

/*V1DataPacket*/

void UARTCommunication::setOdomMsg(const V1DataPacket &packet)
{
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = (packet.x_position) / 1000.0;
    odom_msg.pose.pose.position.y = (packet.y_position) / 1000.0;
    odom_msg.pose.pose.position.z = 0.0;

    // ***USE IMU Orientation
    odom_msg.pose.pose.orientation = imu_orientation;

    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";

    transform.transform.translation.x = odom_msg.pose.pose.position.x;
    transform.transform.translation.y = odom_msg.pose.pose.position.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = odom_msg.pose.pose.orientation;
}

void UARTCommunication::setImuMsg(const V1DataPacket &packet)
{
    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";

    double yaw = -packet.imu_yaw * 0.01 * (M_PI / 180.0);
    double pitch = packet.imu_pitch * 0.01 * (M_PI / 180.0);
    double roll = packet.imu_roll * 0.01 * (M_PI / 180.0);

    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = current_time - previous_time;
    double dt = elapsed_seconds.count();

    double roll_dot = (roll - previous_roll) / dt;
    double pitch_dot = (pitch - previous_pitch) / dt;
    double yaw_dot = (yaw - previous_yaw) / dt;

    previous_roll = roll;
    previous_pitch = pitch;
    previous_yaw = yaw;
    previous_time = current_time;

    std::vector<double> angular_velocity = computeAngularVelocity(roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot);
    imu_msg.angular_velocity.x = angular_velocity[0];
    imu_msg.angular_velocity.y = angular_velocity[1];
    imu_msg.angular_velocity.z = angular_velocity[2];

    tf2::Quaternion imu_q;
    imu_q.setRPY(roll, pitch, yaw);
    imu_msg.orientation = tf2::toMsg(imu_q);
    imu_orientation = imu_msg.orientation;

    int16_t linear_acceleration_x = packet.imu_x_acc;
    int16_t linear_acceleration_y = packet.imu_y_acc;
    int16_t linear_acceleration_z = packet.imu_z_acc;
    imu_msg.linear_acceleration.x = linear_acceleration_x * 9.81 / 1000.0;
    imu_msg.linear_acceleration.y = linear_acceleration_y * 9.81 / 1000.0;
    imu_msg.linear_acceleration.z = linear_acceleration_z * 9.81 / 1000.0;

    geometry_msgs::msg::TransformStamped imu_transform;
    imu_transform.header.stamp = this->get_clock()->now();
    imu_transform.header.frame_id = "base_link";
    imu_transform.child_frame_id = "imu_link";
    imu_transform.transform.translation.x = 0.0;
    imu_transform.transform.translation.y = 0.0;
    imu_transform.transform.translation.z = 0.0;
    imu_transform.transform.rotation = imu_msg.orientation;

    tf_broadcaster_->sendTransform(imu_transform);
}

void UARTCommunication::setBatteryMsg(const V1DataPacket &packet)
{
    battery_msg.battery_voltage = static_cast<double>(packet.battery_voltage) / 100.0f;
    battery_msg.battery_current = static_cast<double>(packet.battery_current) / 10.0f;

    battery_msg.battery_percent = packet.battery_percent;
    battery_msg.cell_voltage1 = packet.cell_voltage1;
    battery_msg.cell_voltage2 = packet.cell_voltage2;
    battery_msg.cell_voltage3 = packet.cell_voltage3;
    battery_msg.cell_voltage4 = packet.cell_voltage4;
    battery_msg.cell_voltage5 = packet.cell_voltage5;
    battery_msg.total_capacity = packet.total_capacity;
    battery_msg.remaining_capacity = packet.remaining_capacity;
    battery_msg.battery_manufacturer = packet.battery_manufacturer;
    battery_msg.battery_temperature1 = packet.battery_temperature1;
    battery_msg.battery_temperature2 = packet.battery_temperature2;
    battery_msg.design_capacity = packet.design_capacity;
    battery_msg.number_of_cycles = packet.number_of_cycles;

    battery_msg.charge_status = static_cast<int>(packet.charging_mode);
    battery_msg.charging_mode = packet.charging_mode;
}

float calculateAverage(const uint16_t *array, size_t size)
{
    uint32_t sum = 0;

    for (size_t i = 0; i < size; i++)
    {
        sum += array[i];
    }

    return static_cast<float>(sum) / size;
}

void UARTCommunication::setTofMsg(const V1DataPacket &packet)
{
    tof_msg.top = static_cast<double>(packet.top_tof) / 1000.0f;

    // Left ToF Remapping
    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            int sensor_index = row * 4 + (3 - col);
            int topic_index = row * 4 + col;
            tof_msg.bot_left[topic_index] = static_cast<double>(packet.lower_left_tof[sensor_index]) / 1000.0f;
        }
    }

    // Right ToF Remapping
    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            int sensor_index = (3 - row) * 4 + col;
            int topic_index = row * 4 + col;
            tof_msg.bot_right[topic_index] = static_cast<double>(packet.lower_right_tof[sensor_index]) / 1000.0f;
        }
    }

    // status
    tof_msg.top_status = packet.top_1d_tof_status;
    tof_msg.bot_status = packet.lower_tof_status;

    // amcl pose
    tof_msg.robot_x = amcl_pose_x_;
    tof_msg.robot_y = amcl_pose_y_;
    tof_msg.robot_angle = amcl_pose_angle_;
}

void UARTCommunication::setMotorMsg(const V1DataPacket &packet)
{
    motor_msg.left_motor_encoder = packet.left_motor_encoder;
    motor_msg.right_motor_encoder = packet.right_motor_encoder;

    motor_msg.left_motor_current = packet.left_motor_current;
    motor_msg.right_motor_current = packet.right_motor_current;
    
    motor_msg.left_motor_rpm = packet.left_motor_rpm;
    motor_msg.right_motor_rpm = packet.right_motor_rpm;

    motor_msg.left_motor_temperature = packet.left_motor_temperature;
    motor_msg.right_motor_temperature = packet.right_motor_temperature;
 
    motor_msg.left_motor_status = packet.left_motor_status;
    motor_msg.right_motor_status = packet.right_motor_status;

    motor_msg.left_motor_type = packet.left_motor_type;
    motor_msg.right_motor_type = packet.right_motor_type;
 
    motor_msg.motor_mode = packet.motor_mode;
    
    // motor_msg.motor_type = getLowerBits(packet.motor_mode);
    // motor_msg.left_status = packet.mo
    // motor_msg.right_status
}

void UARTCommunication::setDockingMsg(const V1DataPacket &packet)
{
    station_msg.sig_short = packet.dock_short_ir_position;
    station_msg.sig_long = packet.dock_long_ir_position;
    station_msg.receiver_status = getUpperBits(packet.dock_sig_status);
    station_msg.docking_status = packet.docking_status;

    // if (station_msg.docking_status & 0x10)
    // {
    //     uint8_t dockingCmd = 0x00;
    //     setDocking(static_cast<DockingChargeCommand>(dockingCmd));
    // }
    // else
    // {
    //     uint8_t chargingCmd = 0x00;
    //     setCharge(static_cast<DockingChargeCommand>(chargingCmd));
    // }
}

void UARTCommunication::setBottomMsg(const V1DataPacket &packet)
{
    bottom_status_msg.data = packet.bottom_wheel_lifting_sensor;
}

void UARTCommunication::setOdomStatusMsg(const V1DataPacket &packet)
{
    imu_odom_status_msg.data = packet.imu_odometry_status;
}

void UARTCommunication::setFwVersionMsg(const V1DataPacket &packet)
{
    std::string tmp = std::to_string(packet.firmware_ver_msb) + '.' + std::to_string(packet.firmware_ver_lsb);
    if (fwVersion_msg.data != tmp)
    {
        RCLCPP_INFO(this->get_logger(), "packet.firmware Major version : %u", packet.firmware_ver_msb);
        RCLCPP_INFO(this->get_logger(), "packet.firmware Minor version : %u", packet.firmware_ver_lsb);
        fwVersion_msg.data = std::to_string(packet.firmware_ver_msb) + '.' + std::to_string(packet.firmware_ver_lsb);
        RCLCPP_INFO(this->get_logger(), "fwVersion_msg.data : %s", fwVersion_msg.data.c_str());
        //    fw_version_pub_->publish(fwVersion_msg);
    }
}

void UARTCommunication::ParseV1Data(const std::vector<uint8_t> &data, V1DataPacket &packet)
{
    // Header fields
    packet.pre_amble_0 = data[0];  // 0xAA
    packet.pre_amble_1 = data[1];  // 0x55
    packet.data_size = data[2];    // Data Size: 166
    packet.command = data[3];      // Command: 0x11

    // Odometry data
    packet.x_position = combineBytesToDouble(data, 4);  // X Position (double)
    packet.y_position = combineBytesToDouble(data, 12); // Y Position (double)

    // Motor status and types
    packet.left_motor_status = data[20];   // Left Motor Status
    packet.right_motor_status = data[21];  // Right Motor Status
    packet.motor_mode = data[22];          // Motor Mode
    packet.left_motor_type = data[23];     // Left Motor Type (0x01 or 0x02)
    packet.right_motor_type = data[24];    // Right Motor Type (0x01 or 0x02)

    // Error codes
    packet.error_code_1 = data[25];        // Error Code 1 (0x00: No Error, 0x01: Error)
    packet.error_code_2 = data[26];        // Error Code 2
    packet.reserve = data[27];             // Reserved field

    // IMU data (Index, Yaw, Pitch, Roll, Acceleration)
    packet.imu_index = data[28];           // IMU Index (0-255)
    packet.imu_yaw = combineBytesToInt16(data, 29);   // IMU Yaw (16-bit)
    packet.imu_pitch = combineBytesToInt16(data, 31); // IMU Pitch (16-bit)
    packet.imu_roll = combineBytesToInt16(data, 33);  // IMU Roll (16-bit)

    packet.imu_x_acc = combineBytesToInt16(data, 35); // IMU X-axis Acceleration
    packet.imu_y_acc = combineBytesToInt16(data, 37); // IMU Y-axis Acceleration
    packet.imu_z_acc = combineBytesToInt16(data, 39); // IMU Z-axis Acceleration

    // Battery voltage and capacity data
    packet.cell_voltage1 = combineBytesToInt16(data, 41);  // Cell Voltage 1 (mV)
    packet.cell_voltage2 = combineBytesToInt16(data, 43);  // Cell Voltage 2 (mV)
    packet.cell_voltage3 = combineBytesToInt16(data, 45);  // Cell Voltage 3 (mV)
    packet.cell_voltage4 = combineBytesToInt16(data, 47);  // Cell Voltage 4 (mV)
    packet.cell_voltage5 = combineBytesToInt16(data, 49);  // Cell Voltage 5 (mV)

    packet.total_capacity = combineBytesToInt16(data, 51);       // Total Capacity (mAh)
    packet.remaining_capacity = combineBytesToInt16(data, 53);   // Remaining Capacity (mAh)
    packet.battery_manufacturer = data[55];  // Battery Manufacturer (0x00, 0x01, or 0x02)

    // Charging status and other sensor data
    packet.charge_status = data[56];          // Charge Status
    packet.imu_odometry_status = data[57];    // IMU and Odometry Status
    packet.bottom_wheel_lifting_sensor = data[58]; // Bottom Wheel Lifting Sensor
    packet.dock_sig_status = data[59];        // Dock Signal Status
    packet.dock_short_ir_position = data[60]; // Dock Short IR Position
    packet.docking_status = data[61];         // Docking Status
    packet.dock_long_ir_position = data[62];  // Dock Long IR Position
    packet.battery_percent = data[63];        // Battery Percentage (0-100%)
    packet.battery_voltage = combineBytesToInt16(data, 64); // Battery Voltage (mV)
    packet.battery_current = combineBytesToInt16(data, 66); // Battery Current (mA)
    packet.battery_temperature1 = data[68];   // Battery Temperature Sensor 1
    packet.battery_temperature2 = data[69];   // Battery Temperature Sensor 2
    packet.design_capacity = combineBytesToInt16(data, 70); // Design Capacity (mAh)
    packet.number_of_cycles = combineBytesToInt16(data, 72); // Number of Cycles (uint16_t)

    // TOF and IMU calibration data
    packet.top_tof = combineBytesToInt16(data, 74);       // Top TOF (mm)
    packet.top_1d_tof_status = data[76];   // Top 1D TOF Status

    // Lower Left TOF (mm) - Populate the array using a loop
    for (int i = 0; i < 16; ++i)
    {
        packet.lower_left_tof[i] = combineBytesToInt16(data, 77 + (i * 2)); // Lower Left TOF (mm)
    }

    // Lower Right TOF (mm) - Populate the array using a loop
    for (int i = 0; i < 16; ++i)
    {
        packet.lower_right_tof[i] = combineBytesToInt16(data, 109 + (i * 2)); // Lower Right TOF (mm)
    }

    packet.lower_tof_status = data[141];    // Left/Right TOF Status

    packet.imu_calibration_status = data[142];  // IMU Calibration Status

    // Motor encoder and current data
    packet.left_motor_encoder = combineBytesToUint32(data, 143);  // Left Motor Encoder (int32_t)
    packet.right_motor_encoder = combineBytesToUint32(data, 147); // Right Motor Encoder (int32_t)

    packet.left_motor_current = combineBytesToInt16(data, 151);  // Left Motor Current (10mA)
    packet.right_motor_current = combineBytesToInt16(data, 153); // Right Motor Current (10mA)
    packet.left_motor_rpm = combineBytesToInt16(data, 155);     // Left Motor RPM
    packet.right_motor_rpm = combineBytesToInt16(data, 157);    // Right Motor RPM

    // Cradle data (Temperature and Firmware version)
    packet.cradle_adc0 = data[159];  // Cradle ADC (Temperature in °C)
    packet.cradle_adc1 = data[160];  // Cradle ADC (Temperature in °C)
    packet.cradle_fw_ver_msb = data[161]; // Cradle FW Major Version
    packet.cradle_fw_ver_lsb = data[162]; // Cradle FW Minor Version

    packet.charging_mode = data[163]; // Charging Mode

    // Motor temperatures and firmware version
    packet.left_motor_temperature = data[164];  // Left Motor Temperature (°C)
    packet.right_motor_temperature = data[165]; // Right Motor Temperature (°C)

    packet.firmware_ver_msb = data[166]; // Firmware Major Version
    packet.firmware_ver_lsb = data[167]; // Firmware Minor Version
    packet.checksum = data[168];         // Checksum (from byte 3 to byte 167)
}

std::tuple<uint8_t, int, std::string> getRankAndErrorCode(uint8_t occurred_error, uint8_t error_code)
{
    // 0x00 (평상 시)일 경우 아무 출력도 하지 않음
    if (occurred_error == 0x00)
    {
        return {occurred_error, -1, ""}; // Rank와 에러코드 문자열을 비우고 반환
    }

    int rank = -1;
    std::string error_string = "UNKNOWN";

    // error_code 범위 확인
    if (error_code < sizeof(error_map) / sizeof(error_map[0]))
    {
        rank = error_map[error_code].rank;
        error_string = error_map[error_code].error_string;
    }

    return {occurred_error, rank, error_string}; // 결과 반환
}

void UARTCommunication::printV1DataPacket(const V1DataPacket &packet)
{
 // Print basic information
    RCLCPP_INFO(this->get_logger(), "Pre_amble_0: 0x%02x", static_cast<int>(packet.pre_amble_0));
    RCLCPP_INFO(this->get_logger(), "Pre_amble_1: 0x%02x", static_cast<int>(packet.pre_amble_1));
    RCLCPP_INFO(this->get_logger(), "Data_Size: %d", static_cast<int>(packet.data_size));
    RCLCPP_INFO(this->get_logger(), "Command: 0x%02x", static_cast<int>(packet.command));

    // Print odometry data
    RCLCPP_INFO(this->get_logger(), "X Position: %f mm", packet.x_position);
    RCLCPP_INFO(this->get_logger(), "Y Position: %f mm", packet.y_position);

    // Print motor status and types
    RCLCPP_INFO(this->get_logger(), "Left Motor Status: 0x%02x", static_cast<int>(packet.left_motor_status));
    RCLCPP_INFO(this->get_logger(), "Right Motor Status: 0x%02x", static_cast<int>(packet.right_motor_status));

    RCLCPP_INFO(this->get_logger(), "Motor Mode: 0x%02x", static_cast<int>(packet.motor_mode));
    RCLCPP_INFO(this->get_logger(), "Left Motor Type: 0x%02x", static_cast<int>(packet.left_motor_type));
    RCLCPP_INFO(this->get_logger(), "Right Motor Type: 0x%02x", static_cast<int>(packet.right_motor_type));

    // Print error codes
    RCLCPP_INFO(this->get_logger(), "Error Code 1: 0x%02x", static_cast<int>(packet.error_code_1));
    RCLCPP_INFO(this->get_logger(), "Error Code 2: 0x%02x", static_cast<int>(packet.error_code_2));
    RCLCPP_INFO(this->get_logger(), "reserve: 0x%02x", static_cast<int>(packet.reserve));

    // Print IMU data
    RCLCPP_INFO(this->get_logger(), "IMU Index: %d", static_cast<int>(packet.imu_index));
    RCLCPP_INFO(this->get_logger(), "IMU Yaw: %d degrees", packet.imu_yaw);
    RCLCPP_INFO(this->get_logger(), "IMU Pitch: %d degrees", packet.imu_pitch);
    RCLCPP_INFO(this->get_logger(), "IMU Roll: %d degrees", packet.imu_roll);
    RCLCPP_INFO(this->get_logger(), "IMU X Acceleration: %d mg", packet.imu_x_acc);
    RCLCPP_INFO(this->get_logger(), "IMU Y Acceleration: %d mg", packet.imu_y_acc);
    RCLCPP_INFO(this->get_logger(), "IMU Z Acceleration: %d mg", packet.imu_z_acc);

    // Print cell voltages
    RCLCPP_INFO(this->get_logger(), "Cell Voltage 1: %d mV", packet.cell_voltage1);
    RCLCPP_INFO(this->get_logger(), "Cell Voltage 2: %d mV", packet.cell_voltage2);
    RCLCPP_INFO(this->get_logger(), "Cell Voltage 3: %d mV", packet.cell_voltage3);
    RCLCPP_INFO(this->get_logger(), "Cell Voltage 4: %d mV", packet.cell_voltage4);
    RCLCPP_INFO(this->get_logger(), "Cell Voltage 5: %d mV", packet.cell_voltage5);

    // Print battery info
    RCLCPP_INFO(this->get_logger(), "Total Capacity: %d mAh", packet.total_capacity);
    RCLCPP_INFO(this->get_logger(), "Remaining Capacity: %d mAh", packet.remaining_capacity);
    RCLCPP_INFO(this->get_logger(), "Battery Manufacturer: 0x%02x", static_cast<int>(packet.battery_manufacturer));

    // Print charge status
    RCLCPP_INFO(this->get_logger(), "Charge Status: 0x%02x", static_cast<int>(packet.charge_status));

    RCLCPP_INFO(this->get_logger(), "IMU and Odometry Status: 0x%02x", static_cast<int>(packet.imu_odometry_status));
    RCLCPP_INFO(this->get_logger(), "Bottom Wheel Lifting Sensor: 0x%02x", static_cast<int>(packet.bottom_wheel_lifting_sensor));
    RCLCPP_INFO(this->get_logger(), "Dock Signal Status: 0x%02x", static_cast<int>(packet.dock_sig_status));
    RCLCPP_INFO(this->get_logger(), "Dock Short IR Position: 0x%02x", static_cast<int>(packet.dock_short_ir_position));
    RCLCPP_INFO(this->get_logger(), "Docking Status: 0x%02x", static_cast<int>(packet.docking_status));
    RCLCPP_INFO(this->get_logger(), "Dock Long IR Position: 0x%02x", static_cast<int>(packet.dock_long_ir_position));
    RCLCPP_INFO(this->get_logger(), "Battery Percentage: %d%%", packet.battery_percent);
    RCLCPP_INFO(this->get_logger(), "Battery Voltage: %d mV", packet.battery_voltage);
    RCLCPP_INFO(this->get_logger(), "Battery Current: %d mA", packet.battery_current);
    RCLCPP_INFO(this->get_logger(), "Battery Temperature 1: %d°C", packet.battery_temperature1);
    RCLCPP_INFO(this->get_logger(), "Battery Temperature 2: %d°C", packet.battery_temperature2);
    RCLCPP_INFO(this->get_logger(), "Design Capacity: %d mAh", packet.design_capacity);
    RCLCPP_INFO(this->get_logger(), "Number of Cycles: %d", packet.number_of_cycles);

    RCLCPP_INFO(this->get_logger(), "Top TOF: %d mm", packet.top_tof);  // Top TOF (mm)
    RCLCPP_INFO(this->get_logger(), "Top 1D TOF Status: 0x%02x", static_cast<int>(packet.top_1d_tof_status));  // Top 1D TOF Status

    // Print TOF values (arrays)
    for (int i = 0; i < 16; ++i)
    {
        RCLCPP_INFO(this->get_logger(), "Lower Left TOF[%d]: %d mm", i, packet.lower_left_tof[i]);
    }

    for (int i = 0; i < 16; ++i)
    {
        RCLCPP_INFO(this->get_logger(), "Lower Right TOF[%d]: %d mm", i, packet.lower_right_tof[i]);
    }
    RCLCPP_INFO(this->get_logger(), "Lower TOF Status: 0x%02x", static_cast<int>(packet.lower_tof_status));


    // Print additional fields as needed
    RCLCPP_INFO(this->get_logger(), "IMU Calibration Status: 0x%02x", static_cast<int>(packet.imu_calibration_status));
    RCLCPP_INFO(this->get_logger(), "Left Motor Encoder: %d", packet.left_motor_encoder);
    RCLCPP_INFO(this->get_logger(), "Right Motor Encoder: %d", packet.right_motor_encoder);

    // Motor current and RPM
    RCLCPP_INFO(this->get_logger(), "Left Motor Current: %d (10mA)", packet.left_motor_current);
    RCLCPP_INFO(this->get_logger(), "Right Motor Current: %d (10mA)", packet.right_motor_current);
    RCLCPP_INFO(this->get_logger(), "Left Motor RPM: %d", packet.left_motor_rpm);
    RCLCPP_INFO(this->get_logger(), "Right Motor RPM: %d", packet.right_motor_rpm);

    // Print cradle and firmware versions
    RCLCPP_INFO(this->get_logger(), "Cradle ADC0: 0x%02x", static_cast<int>(packet.cradle_adc0));
    RCLCPP_INFO(this->get_logger(), "Cradle ADC1: 0x%02x", static_cast<int>(packet.cradle_adc1));

    RCLCPP_INFO(this->get_logger(), "Cradle FW Major Version: 0x%02x", static_cast<int>(packet.cradle_fw_ver_msb));
    RCLCPP_INFO(this->get_logger(), "Cradle FW Minor Version: 0x%02x", static_cast<int>(packet.cradle_fw_ver_lsb));
    
    RCLCPP_INFO(this->get_logger(), "Charging Mode: 0x%02x", static_cast<int>(packet.charging_mode));
    
    // Print motor temperatures
    RCLCPP_INFO(this->get_logger(), "Left Motor Temperature: %d°C", packet.left_motor_temperature);
    RCLCPP_INFO(this->get_logger(), "Right Motor Temperature: %d°C", packet.right_motor_temperature);

    RCLCPP_INFO(this->get_logger(), "Firmware Major Version: 0x%02x", static_cast<int>(packet.firmware_ver_msb));
    RCLCPP_INFO(this->get_logger(), "Firmware Minor Version: 0x%02x", static_cast<int>(packet.firmware_ver_lsb));
 
    // Print checksum
    RCLCPP_INFO(this->get_logger(), "Checksum: 0x%02x", static_cast<int>(packet.checksum));
}

void UARTCommunication::parseToF16Data(const std::vector<uint8_t> &data, ToFData16Data &packet)
{
    size_t index = 0;

    // PRE_AMBLE
    packet.pre_amble_0 = data[index++];
    packet.pre_amble_1 = data[index++];

    // 데이터 크기 및 명령
    packet.data_size = data[index++];
    packet.command = data[index++];

    // LEFT TOF 데이터 파싱 (64개)
    for (int i = 0; i < 64; ++i)
    {
        uint16_t value = static_cast<uint16_t>(data[index] << 8 | data[index + 1]);
        packet.left_tof.push_back(value);
        index += 2;
    }

    // RIGHT TOF 데이터 파싱 (64개)
    for (int i = 0; i < 64; ++i)
    {
        uint16_t value = static_cast<uint16_t>(data[index] << 8 | data[index + 1]);
        packet.right_tof.push_back(value);
        index += 2;
    }

    // CHECKSUM
    packet.checksum = data[index];
}

void UARTCommunication::printToF16Data(const ToFData16Data &packet)
{
    RCLCPP_INFO(this->get_logger(), "=======================================================");
    RCLCPP_INFO(this->get_logger(), "PRE_AMBLE_0: 0x%02X", packet.pre_amble_0);
    RCLCPP_INFO(this->get_logger(), "PRE_AMBLE_1: 0x%02X", packet.pre_amble_1);
    RCLCPP_INFO(this->get_logger(), "Data Size: %d", packet.data_size);
    RCLCPP_INFO(this->get_logger(), "Command: 0x%02X", packet.command);

    // LEFT TOF 출력
    for (size_t i = 0; i < packet.left_tof.size(); ++i)
    {
        RCLCPP_INFO(this->get_logger(), "LEFT TOF %zu: %d mm", i, packet.left_tof[i]);
    }

    // RIGHT TOF 출력
    for (size_t i = 0; i < packet.right_tof.size(); ++i)
    {
        RCLCPP_INFO(this->get_logger(), "RIGHT TOF %zu: %d mm", i, packet.right_tof[i]);
    }

    RCLCPP_INFO(this->get_logger(), "Checksum: 0x%02X", packet.checksum);
    RCLCPP_INFO(this->get_logger(), "=======================================================");
}

void UARTCommunication::setJigCalibrationMsg(const V1DataPacket &packet)
{
    jig_imu_calibration_msg.calibration_status = packet.imu_calibration_status;
    jig_imu_calibration_msg.yaw = packet.imu_yaw;
}

void UARTCommunication::parseDataFields(const std::vector<uint8_t> &data)
{
    if (data.size() < 6)
    {
        RCLCPP_ERROR(this->get_logger(), "Data size too small for parsing.");
        return;
    }

    CommandType command = static_cast<CommandType>(data[3]);

    switch (command)
    {
    case CommandType::RX_V1PROTOCOL_:
    {
        V1DataPacket packet;

        ParseV1Data(data, packet);

        // setProtocolModeMsg(CommandType::RX_V1PROTOCOL_);
        setOdomMsg(packet);
        setImuMsg(packet);
        setBatteryMsg(packet);
        setTofMsg(packet);
        setMotorMsg(packet);
        setDockingMsg(packet);
        setBottomMsg(packet);
        setOdomStatusMsg(packet);
        setFwVersionMsg(packet);
        setJigCalibrationMsg(packet);
        // TODO Error 
        // 차주 FW 협의 필요
    }
    break;

    case CommandType::RX_TOF16PROTOCOL_:
    {
        ToFData16Data packet;

        // 데이터 파싱
        // setProtocolModeMsg(CommandType::RX_TOF16PROTOCOL_);
        parseToF16Data(data, packet);
    }
    break;
    
    default:
        RCLCPP_WARN(this->get_logger(), "Unknown command: 0x%02x", data[3]);
        break;
    }
}

void UARTCommunication::searchAndParseData(std::vector<uint8_t> &buffer)
{
    const std::array<uint8_t, 2> preamble = {0xAA, 0x55}; // 패킷의 프리엠블

    while (buffer.size() >= 5)
    {
        auto preamble_position = std::search(buffer.begin(), buffer.end(), preamble.begin(), preamble.end());

        if (preamble_position == buffer.end())
        {
            buffer.erase(buffer.begin());
            continue;
        }

        size_t preamble_index = std::distance(buffer.begin(), preamble_position);

        if (buffer.size() - preamble_index < 5)
        {
            RCLCPP_WARN(this->get_logger(), "Data size too small to contain all fields, waiting for more data.");
            return;
        }

        uint8_t data_size = buffer[preamble_index + 2];

        if (buffer.size() - preamble_index < data_size + 3)
        {
            return;
        }

        uint8_t checksum = 0;
        for (size_t i = preamble_index + 3; i < preamble_index + 3 + data_size - 1; ++i)
        {
            checksum += buffer[i];
        }

        // std::cerr << "checksum : " << (int)checksum << std::endl;
        // std::cerr << "calculate Checksum : " << (int)buffer[preamble_index + 3 + data_size - 1] << std::endl;

        if (checksum == buffer[preamble_index + 3 + data_size - 1])
        {
            // std::cerr << "Checksum success" << std::endl;

            // 수신한 버퍼 내용을 출력
            // std::cerr << "Received Buffer (Valid Packet): " << preamble_index + 3 + data_size << " : " ;
            // for (size_t i = preamble_index; i < preamble_index + 3 + data_size; ++i)
            // {
            //     std::cerr << std::dec << std::uppercase << static_cast<int>(buffer[i]) << " ";
            // }
            // std::cerr << std::dec << std::endl; // 16진수 출력을 마치고 다시 10진수로 설정

            std::vector<uint8_t> packet_data(buffer.begin() + preamble_index, buffer.begin() + preamble_index + 3 + data_size);
            parseDataFields(packet_data);

            buffer.erase(buffer.begin(), buffer.begin() + preamble_index + 3 + data_size);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Checksum mismatch, discarding packet.");
            buffer.erase(buffer.begin(), buffer.begin() + preamble_index + 3 + data_size);
        }
    }
}

void UARTCommunication::receiveData()
{
    static std::vector<uint8_t> buffer;
    const int num_bytes_to_read = 1024;
    std::vector<uint8_t> new_data(num_bytes_to_read);

    size_t available_bytes = uart_.available();

    if (available_bytes > 0)
    {

        // RCLCPP_INFO(this->get_logger(), "receive Data : ");

        size_t bytes_to_read = std::min(available_bytes, static_cast<size_t>(num_bytes_to_read));
        int bytes_read = uart_.read(new_data.data(), bytes_to_read);

        if (bytes_read > 0)
        {

            new_data.resize(bytes_read);
            buffer.insert(buffer.end(), new_data.begin(), new_data.end());

            searchAndParseData(buffer);
        }
    }
    else
    {
        std::cerr << "No data received or read error." << std::endl;
    }
}

std::string UARTCommunication::formatDataPacket(const std::vector<uint8_t> &data)
{
    std::ostringstream oss;
    for (const auto &byte : data)
    {
        oss << "0x" << std::hex << static_cast<int>(byte) << " ";
    }
    return oss.str();
}

std::vector<double> UARTCommunication::computeAngularVelocity(double roll, double pitch, double yaw, double roll_dot, double pitch_dot, double yaw_dot)
{
    std::vector<double> angular_velocity(3);
    angular_velocity[0] = roll_dot + yaw_dot * std::sin(pitch);
    angular_velocity[1] = pitch_dot * std::cos(roll) - yaw_dot * std::sin(roll) * std::cos(pitch);
    angular_velocity[2] = pitch_dot * std::sin(roll) + yaw_dot * std::cos(roll) * std::cos(pitch);
    return angular_velocity;
}

geometry_msgs::msg::Quaternion UARTCommunication::create_quaternion_from_yaw(double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.w = cos(yaw * 0.5);
    q.x = 0.0;
    q.y = 0.0;
    q.z = sin(yaw * 0.5);
    return q;
}

void UARTCommunication::batterySleepCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    bool bBatterySleep = msg->data;
    //setBatterySleep(bBatterySleep);
    RCLCPP_ERROR(this->get_logger(), "BATTERY-SLEEP"); 
}

void UARTCommunication::tofCommandCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    #if TOF_ALWAYS_ON == 0
    ToFStatus status;
    if(msg->data){
        status = ToFStatus::ON;
        RCLCPP_INFO(this->get_logger(), "COMMAND-CALLBACK TOF-ON");
    }else{
        status = ToFStatus::OFF;
        RCLCPP_INFO(this->get_logger(), "COMMAND-CALLBACK TOF-OFF");
    }
    setToF(status);
    #endif 
}

void UARTCommunication::dockingCommandCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    setDocking(static_cast<DockingChargeCommand>(msg->data)); 
}

void UARTCommunication::e_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    setMotorMode(MotorMode::BRAKE_MODE);
}

void UARTCommunication::charge_cmd_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    setCharge(static_cast<DockingChargeCommand>(msg->data)); 

}

void UARTCommunication::odom_imu_reset_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    ResetCommand reset_cmd = static_cast<ResetCommand>(msg->data);
    setReset(reset_cmd);
}

void UARTCommunication::amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    const auto &pose = msg->pose.pose;
    amcl_pose_x_ = pose.position.x;
    amcl_pose_y_ = pose.position.y;
    amcl_pose_angle_ = quaternion_to_euler(pose.orientation);
}

double UARTCommunication::quaternion_to_euler(const geometry_msgs::msg::Quaternion &quat)
{
    tf2::Quaternion q;
    tf2::fromMsg(quat, q);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw; // Return yaw as theta
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UARTCommunication>());
    rclcpp::shutdown();

    return 0;
}
