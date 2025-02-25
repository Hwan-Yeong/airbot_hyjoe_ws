#ifndef UDP_COMMUNICATION_HPP
#define UDP_COMMUNICATION_HPP

#include "udp_interface/libNetwork.h" //#include "libNetwork.h"

#include <fstream>
#include <filesystem>
#include <chrono>
#include <vector>
#include <algorithm>
#include <mutex>
#include <sstream>
#include <iomanip>
#include <dlfcn.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <std_msgs/msg/string.hpp>

#include "robot_custom_msgs/msg/block_area.hpp"
#include "robot_custom_msgs/msg/block_area_list.hpp"
#include "robot_custom_msgs/msg/position.hpp"
#include "robot_custom_msgs/msg/error_list.hpp"
#include "robot_custom_msgs/msg/error_list_array.hpp"
#include "robot_custom_msgs/msg/battery_status.hpp"
#include "robot_custom_msgs/msg/camera_data.hpp"
#include "robot_custom_msgs/msg/camera_data_array.hpp"
#include "robot_custom_msgs/msg/line_laser_data.hpp"
#include "robot_custom_msgs/msg/line_laser_data_array.hpp"
#include "robot_custom_msgs/msg/tof_data.hpp"
#include "robot_custom_msgs/msg/motor_status.hpp"
#include "robot_custom_msgs/msg/station_data.hpp"
//testcode
#include "robot_custom_msgs/msg/test_position.hpp"

#include "robot_custom_msgs/msg/rpm_control.hpp"
#include "robot_custom_msgs/msg/imu_calibration.hpp"

#include "robot_custom_msgs/msg/robot_state.hpp"
#include "robot_custom_msgs/msg/navi_state.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#define USE_NEW_PROTOCOL 1
#define PROTOCOL_V17 0
#define SOFTWARE_VERSION "2.5.8A"

enum class REQUEST_STATUS
{
    VOID,
    START,
    STOP,
    RUN,
    COMPLETE,
    FAIL,
};

enum class REQUEST_SOC_CMD {
  // SOC TRIGGER CMD
  VOID,
  START_AUTO_MAPPING,   // SOC 명령
  START_MANUAL_MAPPING, // SOC 명령
  START_NAVIGATION,     // SOC 명령
  START_RETURN_CHARGER, // SOC 명령
  START_DOCKING,        // SOC 명령
  START_CHARGING,       // SOC 명령
  STOP_CHARGING,        // SOC 명령
  STOP_AUTO_MAPPING,    // SOC 명령
  STOP_MANUAL_MAPPING,  // SOC 명령
  PAUSE_NAVIGATION,     // SOC 명령
  RESUME_NAVIGATION,    // SOC 명령
  STOP_NAVIGATION,      // SOC 명령
  STOP_RETURN_CHARGER,  // SOC 명령
  STOP_DOCKING,         // SOC 명령
  START_FACTORY_NAVIGATION, //reserve SOC명령
  STOP_FACTORY_NAVIGATION, //reserve SOC명령

  // reserve code
  // ROTATION
  // MANUAL_MOVING,
  // LIDAR_STOP,
  // LIDAR_START,
  // EMERGENCY_STOP,
};

enum class REQUEST_DATA_WHAT
{
    ROBOT_POSITION,
    MAP,
    SW_VERSION,
    MAP_STATUS,
    TARGET_POSITION,
    BATTERY_STATUS,
    DOCKING_STATUS,
    ROBOT_INFO,
    MODIFIED_MAP,
    MOVING_INFO,
    TARGET_POSITION_CALCUL,
    ROBOT_VELOCITY,
    MOTOR_STATUS,
    CAMERA_STATUS,
    LINE_LASER_STATUS,
    TOF_STATUS,
    ERROR_LIST,
    LIDAR_DATA,
    CLIFF_LIFT_DATA,
    DOCK_RECEIVER,
    ALL_STATUS,
};

enum class ROBOT_STATE:int{
  IDLE = 0,
  AUTO_MAPPING,
  MANUAL_MAPPING,
  NAVIGATION,
  RETURN_CHARGER,
  DOCKING,
  UNDOCKING,
  ONSTATION,
  FACTORY_NAVIGATION,
  ERROR,
};

enum class ROBOT_STATUS : int{
  VOID = 0,
  READY,
  START,
  PAUSE,
  COMPLETE,
  FAIL,
};

enum class NODE_STATUS
{
    IDLE,
    AUTO_MAPPING,
    MANUAL_MAPPING,
    NAVI,
};

enum class NAVI_STATE
{
    IDLE,
    MOVE_GOAL,
    ARRIVED_GOAL,
    PAUSE,
    FAIL,
    ROTAION,
    COMPLETE_ROTATION,
    READY,
};

enum class NAVI_FAIL_REASON
{
    VOID,
    NODE_OFF,
    SERVER_NO_ACTION,
    GOAL_ABORT,
    UNKWON,
};

enum class UDP_COMMUNICATION
{
    NORMAL,
    INSPECTION,
    AP_JIG_MODE,
    AMR_JIG_MODE,
};

enum JIG_HEADER
{
    AMR_MODE = 0x00,
    WHEEL_MOTOR = 0x01,
    BATTERY = 0x02,
    CLIFF_LIFT = 0x03,
    DOCK_RECEIVER = 0x04,
    TOF = 0x05,
    FRONT_LIDAR = 0x06,
    REAR_LIDAR = 0x07,
    IMU_CALIBRATION = 0x08,
};

enum JIG_AP_HEADER
{
    AP_JIG = 0xF0,
    AP_JIG_RAM_MEMORY = 0xF1,
    AP_JIG_DISK_MEMORY = 0xF2,
    AP_JIG_FRONT_LIDAR = 0xF3,
    AP_JIG_BACK_LIDAR = 0xF4,
};

enum class JIG_DATA_KEY
{
    MODE,
    WHEEL_MOTOR,
    BATTERY,
    CLIFF_LIFT,
    DOCK_RECEIVER,
    TOF,
    FRONT_LIDAR,
    REAR_LIDAR,
    IMU_CALIBRATION,
};

enum ROTATION_COMMAND
{
    ROTATE_RELATIVE,
    ROTATE_ABSOLUTE,
    ROTATE_360_CCW,
    ROTATE_360_CW,
};

struct pose
{
    bool valid;
    double x;
    double y;
    double theta;
    double timestamp;

    pose() : valid(false),x(0.0), y(0.0),theta(0.0), timestamp(0.0) {}
};

struct MapInfo
{
    bool bReceived;
    double resolution; 
    int width;
    int height;
    double origin_x;
    double origin_y;
    std::vector<uint8_t> map_data;
};

struct LidarSensorInfo
{
    int front_distance;
    int rear_distance;
};

struct motorData
{
    uint8_t type; // 0x01 : type-C , 0x02 : type-A
    uint8_t status;
    int32_t encoder;
    int16_t rpm;
    int16_t current; //mA
    int8_t tempterature;
};

struct MotorInfo
{
    uint8_t mode;
    motorData left;
    motorData right;
};

struct cameraData
{
    uint8_t class_id;
    uint8_t confidence;
    pose position;
    double width;
    double height;
    double distance;
};

struct CarmeraSensorInfo
{
    uint8_t num;
    #if USE_NEW_PROTOCOL > 0
    std::vector<ObjectDataV2> data;
    #else
    std::vector<cameraData> data;
    #endif
};

struct lineLaserData
{
    pose position;
    uint8_t direction;
    double height;
    double distance;
};

struct LineLaserInfo
{
    uint8_t num;
    #if USE_NEW_PROTOCOL > 0
    std::vector<LLDataV2> data;
    #else
    std::vector<lineLaserData> data;
    #endif
    
};

struct TofData
{
    uint8_t status;
    double distance;
};


struct multiTofData //4x4 array
{
    uint8_t status;
    std::vector<int> data;
    multiTofData() : status(0), data(16) {}
};

struct TofSensorInfo
{
    TofData top;
    multiTofData left_bottom;
    multiTofData right_bottom;
};


union CliffLiftInfo {
    struct {
        uint8_t cliff_front_center : 1;
        uint8_t cliff_front_left : 1;
        uint8_t cliff_back_left : 1;
        uint8_t cliff_back_center : 1;
        uint8_t cliff_back_right : 1;
        uint8_t cliff_front_right : 1;
        uint8_t wheel_lift_left : 1;
        uint8_t wheel_lift_right : 1;
    };
    uint8_t value;
};

union SigReceiver {
    struct {
        uint8_t front_left : 1;
        uint8_t front_right : 1;
        uint8_t side_left : 1;
        uint8_t side_right : 1;
        uint8_t reserved : 4;
    };
    uint8_t value;
};

struct DockingInfo
{
    uint8_t status;
    SigReceiver receiver;
};

struct BatteryInfo
{
    uint8_t status;
    int16_t cell_voltage1; // Cell Voltage 1-5 (mV) (p)
    int16_t cell_voltage2; // Cell Voltage 1-5 (mV) (p)
    int16_t cell_voltage3; // Cell Voltage 1-5 (mV) (p)
    int16_t cell_voltage4; // Cell Voltage 1-5 (mV) (p)
    int16_t cell_voltage5; // Cell Voltage 1-5 (mV) (p)

    int16_t total_capacity;     // Total Capacity (mAh) (p)
    int16_t remaining_capacity; // Remaining Capacity (mAh) (p)
    uint8_t manufacturer;       // Battery Manufacturer: 0x00, 0x01, or 0x02 (p)

    uint8_t percent;          // Battery Percentage (0-100%) (p)
    double voltage;          // Battery Voltage (mV) (p)
    double current;          // Battery Current (+charge, -discharge, 1mA) (p)
    uint8_t temperature1;     // Battery Temperature (°C) (Sensor 1 and 2) (p)
    uint8_t temperature2;     // Battery Temperature (°C) (Sensor 1 and 2) (p)
    int16_t design_capacity;  // Design Capacity (mAh) (p)
    int16_t number_of_cycles; // Number of Cycles (uint16_t) (p)

    uint8_t charge_status; // Charge Status (p)
    int16_t charging_mode; // Charging Mode (p)
};

struct SoftWareVersion
{
    bool bSet;
    double timestamp;
    std::string total_ver;
    std::string mcu_ver;
    std::string ai_ver;
    std::string sw_ver;
};


struct RobotSpeed
{
    double v;
    double w;
};

struct ErrorInfo
{
    uint8_t rank;
    std::string code;
};

struct SocData
{
    pose robotPosition;
    LidarSensorInfo lidarInfo;
    MotorInfo motorInfo;
    CarmeraSensorInfo cameraInfo;
    LineLaserInfo lineLaserInfo;
    TofSensorInfo tofInfo;
    CliffLiftInfo cliffLiftInfo;
    DockingInfo dockingInfo;
    BatteryInfo battInfo;
    SoftWareVersion version;
    pose targetPosition;
    MapInfo mapInfo;
    RobotSpeed velocity;
    uint8_t robotStatus;
    uint8_t actionStatus;
    std::string notification;
    uint8_t lineLaserCalib;
    uint8_t movingStatus;
    ErrorInfo errorInfo;
};

struct rotationData
{
    bool progress;
    int type;
    double target;
    double accAngle;
    double preTheta;
};



using namespace std::chrono_literals;

class UdpCommunication : public rclcpp::Node
{
private :
    std::mutex map_mutex_;
    std::mutex scan_front_mutex_;
    std::mutex scan_back_mutex_;

    std::mutex battery_mutex_;
    std::mutex wheel_motor_mutex_;
    std::mutex tof_mutex_;
    std::mutex imu_cal_mutex_;
    std::mutex bottom_status_mutex_;
    std::mutex charger_data_mutex_;

    std::mutex camera_mutex_;
    std::mutex line_laser_mutex_;
    
    //subscriber
    //mcu_sub
    rclcpp::Subscription<robot_custom_msgs::msg::TofData>::SharedPtr tof_status_sub;
    rclcpp::Subscription<robot_custom_msgs::msg::BatteryStatus>::SharedPtr battery_status_sub;
    rclcpp::Subscription<robot_custom_msgs::msg::MotorStatus>::SharedPtr motor_status_sub;
    rclcpp::Subscription<robot_custom_msgs::msg::MotorStatus>::SharedPtr motor_data_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr bottom_status_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::StationData>::SharedPtr station_data_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::ImuCalibration>::SharedPtr imu_sub_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fw_version_sub;
    //ai_sub
    rclcpp::Subscription<robot_custom_msgs::msg::LineLaserDataArray>::SharedPtr line_laser_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::CameraDataArray>::SharedPtr camera_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ai_version_sub;
    //ap_sub
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_front_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_back_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_pose_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::ErrorListArray>::SharedPtr error_list_sub_;
    //state_sub
    rclcpp::Subscription<robot_custom_msgs::msg::RobotState>::SharedPtr req_state_sub;
    rclcpp::Subscription<robot_custom_msgs::msg::NaviState>::SharedPtr req_navi_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr node_status_sub_;

    //publisher
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr e_stop_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr manual_move_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr charge_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lidar_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tof_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr linelaser_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr batterySleep_cmd_pub_;

    rclcpp::Publisher<robot_custom_msgs::msg::Position>::SharedPtr move_target_pub_;
    rclcpp::Publisher<robot_custom_msgs::msg::TestPosition>::SharedPtr test_move_target_pub_;
    rclcpp::Publisher<robot_custom_msgs::msg::BlockAreaList>::SharedPtr block_area_pub_;
    rclcpp::Publisher<robot_custom_msgs::msg::BlockAreaList>::SharedPtr block_wall_pub_;     
	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr req_version_pub_;
    rclcpp::Publisher<robot_custom_msgs::msg::RpmControl>::SharedPtr jig_request_motor_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr jig_request_battery_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr jig_request_imu_pub_;
    rclcpp::Publisher<robot_custom_msgs::msg::RobotState>::SharedPtr req_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr station_pose_pub_;
    
    //rclcpp::Publisher<robot_custom_msgs::msg::MotorRpm>::SharedPtr cmd_rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr req_soc_cmd_pub_;

    //timer
    rclcpp::TimerBase::SharedPtr udpTimer_;
    rclcpp::TimerBase::SharedPtr socAutoSendTimer_;
    rclcpp::TimerBase::SharedPtr versionTimer_;
    rclcpp::TimerBase::SharedPtr map_Timer_;
    rclcpp::TimerBase::SharedPtr linear_target_timer_;
    rclcpp::TimerBase::SharedPtr rotation_target_timer_;

    //udp protocol data
    Mapping_t ReqMapping;
    Mapping_t& reqMapping = ReqMapping;
    Driving_t ReqNavigation;
    Driving_t& reqNavigation = ReqNavigation;
    TargetPosition_t ReqTargetPosition;
    TargetPosition_t& reqTargetPosition = ReqTargetPosition;
    DockingStatus_t ReqDocking;
    DockingStatus_t& reqDocking = ReqDocking;
    MotorManual_VW_t ReqManualVWMove;
    MotorManual_VW_t& reqManualVWMove = ReqManualVWMove;
    MotorManual_RPM_t ReqManualRpmMove;
    MotorManual_RPM_t& reqManualRpmMove = ReqManualRpmMove;
    ModifiedMapDataB_t modifiedMap;
    ExcelSteps_t ReqExcelator;
    ExcelSteps_t &reqExcelator = ReqExcelator;
    TargetPosition_t ReqCalculateTargetPose;
    Rotation_t ReqRotation;
    Rotation_t& reqRotation = ReqRotation;

    #if PROTOCOL_V17 > 0
    FactoryMode_t ReqFactoryMode;
    FactoryMode_t& reqFactoryMode = ReqFactoryMode;
    BatterySleepMode_t ReqBatterySleep;
    BatterySleepMode_t& reqBatterySleep = ReqBatterySleep;
    InspectionMode_t ReqInspectionMode;
    InspectionMode_t &reqInspectionMode = ReqInspectionMode;
    StationRepositioning_t ReqStationPose;
    StationRepositioning_t& reqStationPose = ReqStationPose;
    #endif

    std::vector<ErrorList_t> error_list;

    ROBOT_STATE robotState;
    ROBOT_STATUS robotStatus;
    NAVI_STATE movingState;
    NAVI_FAIL_REASON movefail_reason;
    NODE_STATUS nodeState;

    SocData socData; //soc
    std::unordered_map<JIG_DATA_KEY,std::vector<uint8_t>> jigData; //jig
    sensor_msgs::msg::LaserScan::SharedPtr apJigFrontLaserData;
    sensor_msgs::msg::LaserScan::SharedPtr apJigBackLaserData;

    UDP_COMMUNICATION communicationMode;
    double initNodeTime;
    rotationData rotation;
    pose base_odom;
    pose odom;
    bool end_linear_target;

public:
    UdpCommunication();
    ~ UdpCommunication();

    void initializeData();
    void setCommnicationMode(UDP_COMMUNICATION set);
    UDP_COMMUNICATION getCommunicationMode();
    void debugLogCommnicationMode(UDP_COMMUNICATION set);
    void procSocCommunication();
    void procAmrJigCommunication();
    void procApJigCommunication();

    void autoSocDataSender();
    void reqSocActionChecker();
    void reqSocOptionChecker();
    void reqSocDataChecker();
    void generateSocCommand();
    void directRequestCommand();

    void generateVersion();

    void setRobotState(ROBOT_STATE state, ROBOT_STATUS status);

    void setNodeState(NODE_STATUS set);
    void setNaviState(NAVI_STATE set);
    void setNaviFailReason(NAVI_FAIL_REASON set);

    void debugLogRobotState(ROBOT_STATE set);
    void debugLogRobotStatus(ROBOT_STATUS set);

    void debugLogNodeState(NODE_STATUS set);
    void debugLogNaviState(NAVI_STATE set);
    void debugLogNaviFailReason(NAVI_FAIL_REASON set);

    void setSocRobotPoseData(bool valid, double x, double y, double theta);
    void setSocFrontLidarData(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void setSocRearLidarData(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void setSocWheelMotorData(const robot_custom_msgs::msg::MotorStatus::SharedPtr msg);
    void setSocCameraData(const robot_custom_msgs::msg::CameraDataArray::SharedPtr msg);
    void setSocLineLaserData(const robot_custom_msgs::msg::LineLaserDataArray::SharedPtr msg);
    void setTofStatus(uint8_t top, uint8_t bottom_left, uint8_t bottom_right);
    void setSocTofData(const robot_custom_msgs::msg::TofData::SharedPtr msg);
    void setSocCliffLiftData(const std_msgs::msg::UInt8::SharedPtr msg);
    void setSocDockReceiverData(const robot_custom_msgs::msg::StationData::SharedPtr msg);
    void setSocBatteryData(const robot_custom_msgs::msg::BatteryStatus::SharedPtr msg);
    void setSocTargetPosition(double x, double y, double theta);
    void setSocMapData(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void setSocRobotVelocity();
    void setSocRobotState();
    void setSocMovingState();
    void setSocError(const robot_custom_msgs::msg::ErrorListArray::SharedPtr msg);

    void setJigData(JIG_DATA_KEY key, const std::vector<uint8_t>& data);
    std::vector<uint8_t> getJigData(JIG_DATA_KEY key);

    bool isValidTargetPose();
    pose getTargetPose();
    bool isValidRobotPose();
    pose getRobotPose();
    MapInfo getMapInfo();

    //callback
    void udp_callback();
    void lidarFrontCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void lidarBackCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void lineLaserCallback(const robot_custom_msgs::msg::LineLaserDataArray::SharedPtr msg);
    void cameraCallback(const robot_custom_msgs::msg::CameraDataArray::SharedPtr msg);
    void tofCallback(const robot_custom_msgs::msg::TofData::SharedPtr msg);
    void errorListCallback(const robot_custom_msgs::msg::ErrorListArray::SharedPtr msg);
    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void slamPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
	void fw_version_callback(const std_msgs::msg::String::SharedPtr msg);
    void ai_version_callback(const std_msgs::msg::String::SharedPtr msg);
    void bottomStatusCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void stationDataCallback(const robot_custom_msgs::msg::StationData::SharedPtr msg);
    void batteryCallback(const robot_custom_msgs::msg::BatteryStatus::SharedPtr msg);
    void motorCallback(const robot_custom_msgs::msg::MotorStatus::SharedPtr msg);
    void imuCalibrationCallback(const robot_custom_msgs::msg::ImuCalibration::SharedPtr msg);
    void robotStateCallback(const robot_custom_msgs::msg::RobotState::SharedPtr msg);
    void movingStateCallback(const robot_custom_msgs::msg::NaviState::SharedPtr msg);
    void nodeStateCallback(const std_msgs::msg::UInt8::SharedPtr msg);

    //response - data
    void resSocData(REQUEST_DATA_WHAT what);
    void resSocMap();
    void resSocRobotPose();
    void resSocSoftWareVision();
    void resSocNodeStatus(); //삭제 예정
    #if USE_NEW_PROTOCOL > 0
    void resSocRobotState();
    void resSocRobotStatus();
    #endif
    void resSocTargetPosition();
    void resSocBattData();
    void resSocDockingStatus();
    void resSocRobotInfo();
    void resSocModifiedMap();
    void resSocMovingInfo();
    void resSocCalculateTarget();
    void resSocRobotVelocity();
    void resSocWheelMotorData();
    void resSocCameraData();
    void resSocLineLaserData();
    void resSocTofData();
    void resSocErrorList();
    void resSocLidarData();
    #if USE_NEW_PROTOCOL > 0
    void resSocCliffLiftData();
    void resSocDockReceiverData();
    #endif
    void resSocAllSensor();

    void resPonseJigCommand(int command, const std::vector<uint8_t>& packet);
    void resPonseJigData(JIG_DATA_KEY key);
    void logCommandAndData(int command, const std::vector<uint8_t>& data, int option); 
    void printLidar(int command, const std::vector<uint8_t>& data);
    void printTof(const std::vector<uint8_t>& data);

    UDP_COMMUNICATION checkUdpCommunicationMode();

    std::vector<uint8_t> makeResponseJigCommandPacket(JIG_HEADER header, std::vector<uint8_t> packet);
    void jigCheckWheelMotor(const std::vector<uint8_t>& packet);
    void jigCheckBattery(const std::vector<uint8_t>& packet);
    void jigCheckCliffLift();
    void jigCheckDockReceiver();
    void jigCheckTofSensor(const std::vector<uint8_t>& packet);
    void jigCheckFrontLidar();
    void jigCheckRearLidar();
    void jigCheckImuCalibration(const std::vector<uint8_t>& packet);
    void jigProcessor(int header, const std::vector<uint8_t>& packet);

    void sendRamData(const std::vector<uint8_t>& ram_info);
    void apJigCheckRam();
    void sendEmmcData(const std::vector<uint8_t>& emmc_info);
    void apJigCheckEmmc();
    void sendLidarData(int header, const std::vector<uint8_t>& lidar_info);
    void apJigCheckFrontLiDAR();
    void apJigCheckBackLiDAR();
    void apJigProcessor(int header, const std::vector<uint8_t>& packet);


    //data publisher
    void publishEmergencyCommand();
    void publishVelocityCommand(double v, double w);
    void publishChargingCommand(bool start);
    void publishBlockArea(const ByPassOne_t& parsedData);
    void publishBlockWall(const ByPassOne_t& parsedData);
    void publishClearVirtualWall();
    void publishVersionRequest();
    void publishTargetPosition(double x,double y,double theta);
    void publishLidarOnOff(bool on_off);
    void publishSensorOnOff(bool set);
    void publishBatterySleep();
    void publishStationPose(double x, double y, double theta);

    void clearErrorList();

    void systemRebootCommand();

    void resetVersionTimer();

    //mapping function
    void savePGMFile(const ModifiedMapDataB_t &settings, const std::string &filename);
    bool readPGM(const std::string& pgm_filename, int& width_map, int& height_map, std::vector<uint8_t>& mapData, double& origin_x, double& origin_y);

    double quaternion_to_euler(const geometry_msgs::msg::Quaternion& quat);
    std::vector<uint8_t> mapDataTypeConvert(std::vector<int8_t> src, int height, int width, double origin_x, double origin_y);
    std::vector<uint8_t> jigDataConvertWheelMotor(const robot_custom_msgs::msg::MotorStatus::SharedPtr msg);
    std::vector<uint8_t> jigDataConvertBattery(const robot_custom_msgs::msg::BatteryStatus::SharedPtr msg);
    std::vector<uint8_t> jigDataConvertTof(const robot_custom_msgs::msg::TofData::SharedPtr msg);
    std::vector<uint8_t> jigDataConvertLidar(int left, int right);
    std::vector<uint8_t> jigDataConvertImuCalibration(const robot_custom_msgs::msg::ImuCalibration::SharedPtr msg);
    uint8_t getLowerBits(uint8_t byte);
    uint8_t getUpperBits(uint8_t byte);
    
    void generateFrontBackScan(const sensor_msgs::msg::LaserScan::SharedPtr msg, std::vector<int>& vecDist);
    void splitLidarScan(const sensor_msgs::msg::LaserScan::SharedPtr msg, std::vector<int>& left, std::vector<int>& right);

    int checkRotationDirection(double diff);
    bool checkRotationTarget(double diff);
    void progressRotationTarget(double target, double current);
    void progressRotation360(int direction, double current);
	void reset_Rotationtimer();
	void startRotateMonitor();
    void progressRotation();
    void stopMonitorRotate();
    bool startRotation(int type, double targetAngle);
    int getMinDistanceFromLidarSensor(const std::vector<int>& vecDistance);
    double normalize_angle(double angle);

    void enableLinearTargetMoving();
    void disableLinearTargetMovoing();
    double getDistance(pose base, pose current);
    void processLinearMoving();

    //test_code
    void publishTestTargetPosition(double x,double y,double theta, int type);
};

#endif // UDP_COMMUNICATION_HPP
