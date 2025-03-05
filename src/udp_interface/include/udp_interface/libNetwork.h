#ifndef LIBNETWOKR_H
#define LIBNETWOKR_H

#include <vector>
#include <string>

#define BASE_VERSION "0.35"

#ifdef LOG_SETTING
    #ifdef DEBUG_MODE
        #define VERSION BASE_VERSION ".2.1"
    #else
        #define VERSION BASE_VERSION ".2.0"
    #endif
#elif defined(OTA_SETTING)
    #ifdef DEBUG_MODE
        #define VERSION BASE_VERSION ".1.1"
    #else
        #define VERSION BASE_VERSION ".1.0"
    #endif
#else // NORMAL 모드
    #ifdef DEBUG_MODE
        #define VERSION BASE_VERSION ".0.1"
    #else
        #define VERSION BASE_VERSION ".0.0"
    #endif
#endif

#define getPositionIdx 1 
#define getLidarSensorStatusIdx 2
#define getMotorStatusIdx 3
#define getCameraStatusIdx 4
#define getLineLaserStatusIdx 5
#define getTofStatusIdx 6
#define getIRStatusIdx 7
#define getSonicStatusIdx 8
#define getBatteryStatusIdx 9
#define getSoftwareVersionIdx 10
#define getRobotInfoIdx 11
#define getErrorListIdx 12
#define getTargetPositionIdx 13
#define getMapStatusIdx 14
#define getMapDataIdx 15
#define getLidarDataIdx 16
#define getModifiedMapDataIdx 17 
#define getXXXStatusIdx 18
#define getRobotSpeedIdx 19
#define getTargetPositionCalculateIdx 20

#define getRobotStatusIdx 21
#define getActionStatusIdx 22
#define getNotificationIdx 23

#define getAICalibrationIdx 25
#define getLogDataIdx 26

#define getDockingStatusIdx 30 

#define getMotorStatusV2Idx 31
#define getRecvIRStatusIdx 32
#define getCliffIRStatusIdx 33
#define getCameraStatusV2Idx 34
#define getLineLaserStatusV2Idx 35
#define getTofStatusV2Idx 36
#define getBatteryStatusV2Idx 37

#if false
#define getIntegrationMapDataIdx 31
#define getIntegrationModifiedMapDataIdx 32 
#endif

#define getAllStatusIdx 40
#define getAllMovingInfoIdx 41
#define getAllStatusV2Idx 42
#define getAllMovingInfoV2Idx 43

#define setReturnToChargingStationIdx 50
#define setEmergencyStopIdx 51 
#define setStartLidarIdx 52 
#define setStopLidarIdx 53
#define setStartChargingIdx 54
#define setStopChargingIdx 55
#define setSoftwareResetIdx 56
#define setMaxDrivingSpeedIdx 57
#define setMotorManual_RPMIdx 58
#define setMotorManual_VWIdx 59
#define setTargetPositionIdx 60
#define setDrivingIdx 61
#define setMappingIdx 62
#define setModifiedMapDataIdx 63
#define setHeartbeatIdx 64
#define setExcelStepsIdx 65
#define setRotationIdx 66
#define setTimeDataIdx 67

#define setIntegrationModifiedMapDataIdx 70
#define setByPassOneDataIdx 71
#define setByPassTwoDataIdx 72
#define setPGMMapDataIdx 73

#define setFactoryModeIdx 74
#define setBatterySleepModeIdx 75
#define setSensorInspectionModeIdx 76
#define setStationRepositioningIdx 77

#define setOTAIdx 80
#define setOTADataIdx 81

#define setFactoryResetIdx 83
#define setRecoveryIdx 84

#define setDockingStatusIdx 90 

// Everybot First
#define emergencyMSGERRORIdx 100
#define emergencyMSGOTAIdx 101

//
#define setSettingCommandIdx 251
#define setConfigCommandIdx 252

#define objectReq   0x01
#define objectRes   0x02
#define objectEmergency   0x0F
#define objectJig  0xC8
#define objectOTAData  0x0E
/******************************************** */

// #1
struct Positions_t {
    double x;
    double y;
    double theta;
};

// #2
struct LidarSensorStatus_t {
    int front;
    int rear;
};

// #3
struct MotorStatus_t {
    double leftSpeed;
    double  rightSpeed;
    double leftPower;
    double rightPower;
    int leftStatus;
    int rightStatus;
};

struct MotorStatusV2_t {
    double leftRPM;
    double  rightRPM;
    double leftCurrent;
    double rightCurrent;
    int leftType;
    int rightType;
};

// #4
struct CameraStatus_t {
    int status;
};

// #5
struct LineLaserStatus_t {
    int left;
    int right;
};

// #6
struct TofStatus_t {
    int leftStatus;
    double leftDistance;
    int rightStatus; 
    double rightDistance;
    int topStatus;
    double topDistance;
};

struct TofStatusV2_t {
    int leftStatus;
    int rightStatus; 
    int topStatus;
};

// #7
struct IRStatus_t {
    bool sensor1;
    bool sensor2;
    bool sensor3;
    bool sensor4;
    bool sensor5;
    bool sensor6;
    bool stationIR1;
    bool stationIR2;
    bool stationIR3;
    bool stationIR4;
};

struct RecvIRStatus_t {
    bool RecvIR1;
    bool RecvIR2;
    bool RecvIR3;
    bool RecvIR4;
};

struct CliffIRStatus_t {
    bool CliffIR1;
    bool CliffIR2;
    bool CliffIR3;
    bool CliffIR4;
    bool CliffIR5;
    bool CliffIR6;
};

// #8
struct SonicStatus_t {
    int leftStatus;
    bool LeftSenseing;
    int rightStatus;
    bool rightSenseing;
};

// #9
struct BatteryStatus_t {
    double  voltage;
    double  current;
    double  temperature;
    double  chargeState;
    double  capacity;
    double  designCapacity;
    double  percentage;
    double  supplyStatus;
    double  supplyHealth;
    double  supplyTechnology;
    double  present;
    double  cellVoltage;
    double  cellTemperature;
    double  location;
    double  serialNumber;
};

struct BatteryStatusV2_t {
    int cell_voltage1;
    int cell_voltage2;
    int cell_voltage3;
    int cell_voltage4;
    int cell_voltage5;
    int total_capacity;
    int remaining_capacity;
    int battery_manufacturer;
    int battery_percent;
    double battery_voltage;
    double battery_current;
    int battery_temperature1;
    int battery_temperature2;
    int design_capacity;
    int number_of_cycles;
};

// #10
struct SoftwareVersion_t {
    std::string version;
};

// #11
struct RobotInfo_t {
    std::string serialNumber;
    int status;
};

// #12
struct ErrorList_t {
    int resolved;
    int rank;
    std::string errorCode;
};

// #13
struct TargetPosition_t {
    double x;
    double y;
    double theta;
};

struct StationRepositioning_t
{
    double x;
    double y;
    double theta;
};

struct AICalibration_t {
    int result;
};

// #14 -
struct MapStatus_t {
    double Temp;
};

// #15
struct MapData_t {
    double width;
    double height;
    double resolution;
    double posX;
    double posY;
    std::string data;
};

struct IntegrationMapData_t {
    std::string data;
};
// #16
struct LidarData_t {
    double front;
    double rear;
};

// #17
struct ModifiedMapData_t {
    double width;
    double height;
    std::string	data;
};

struct ModifiedMapDataB_t {
    double width;
    double height;
    std::vector<unsigned char> data;
};

struct PGMMapData_t {
    std::string	 name;
    std::string	 sha256;
    std::string  data;
};

struct PGMMapDataB_t {
    std::string	 name;
    bool	      sha256;
    std::vector<unsigned char> data;
};


struct RobotSpeed_t {
    double mS;
    double radS;
};

struct RobotStatus_t {
    int status;
};

struct ActionStatus_t {
    int status;
};

struct Notification_t {
    int code;
    std::string	desccription;
};

struct IntegrationModifiedMapData_t {
    std::string	data;
};

struct LogData_t {
    std::string	data;
};

struct ByPassData_t {
    std::string	data;
};

struct Position {
    double x;
    double y;
};

struct Room {
    std::string id;
    std::string name;
    std::string color;
    std::string desc;
    std::vector<Position> robot_path;
    std::vector<Position> image_path;
};

struct BlockArea {
    std::string id;
    std::vector<Position> robot_path;
    std::vector<Position> image_path;
};

struct ChargingStation {
    std::string id;
    Position robot_position;
    Position image_position;
};

struct InitPosition {
    std::string id;
    Position robot_position;
    Position image_position;
};

struct ByPassOne_t {
    std::string uid;
    std::string version;
    std::string modified;
    std::vector<Room> room_list;
    std::vector<BlockArea> block_area;
    std::vector<BlockArea> block_wall;
    std::vector<ChargingStation> charging_station;
    std::vector<InitPosition> init_position;
};

struct IntegrationModifiedMapDataB_t {
    std::vector<unsigned char>	data;
};

struct AllStatus_t {
    std::string	data;
};

struct AllStatusV2_t {
    std::string	data;
};

struct AllMovingInfo_t {
    std::string	data;
};

struct AllMovingInfoV2_t {
    std::string	data;
};

// #Debug Docking
struct DockingStatus_t {
    int dock;
};


/******************************************** */


// #50
struct ReturnToChargingStation_t {
    bool Return;
};

// #51
struct EmergencyStop_t {
    bool Return;
};

// #52
struct StartLidar_t {
    bool Return;

};

// #53
struct StopLidar_t {
    bool Return;

};

// #54
struct StartCharging_t {
    bool Return;
};

// #55  
struct StopCharging_t {
    bool Return;

};

// #56
struct SoftwareReset_t {
    bool Return;

};

struct FactoryMode_t {
    bool start;
};

struct BatterySleepMode_t {
    bool start;
};

struct InspectionMode_t {
    bool start;
};

// #57
struct MaxDrivingSpeed_t {
    double mS;

};

// #58
struct MotorManual_RPM_t {
    int leftRPM;
    int rightRPM;
};

// # 59
struct MotorManual_VW_t {
    double mS;
    double radS;
};

// # 61
struct Driving_t {
    int set;

};

// # 62
struct Mapping_t {
    int set;
};

#if 0
struct ModifiedMapData_t {
    double x;
    double y;
    double theta;
};
#endif

// # 63
struct heartbeat_t {
    bool Return;
};

struct ExcelSteps_t {
    int set;
};

struct Rotation_t {
    int type;
    double radian;
};

struct TimeData_t {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
};

struct Status_t {
    int status;
};

struct ota_t {
    std::string	checksum;
    int size;
};

struct otadata_t {
    std::string	checksum;
    std::string	data;
};

struct otadataB_t {
    std::string	checksum;
    std::vector<unsigned char>	data;
};

struct SettingCommand_t {
    int set;
};

struct ConfigCommand_t {
    int set;
};

struct version_t {
    std::string	version;
};

struct ObjectDataV2
{
    uint8_t class_id;      // 객체의 클래스 ID
    uint8_t confidence;    // 신뢰도 (0~100)
    int16_t x;             // X 좌표
    int16_t y;             // Y 좌표
    int16_t theta;         // 각도 (Theta)
    int16_t width;         // 너비 (Width)
    int16_t height;        // 높이 (Height)
    uint16_t distance;     // 거리 (Distance)
};

struct LLDataV2
{
    int16_t x;
    int16_t y;
    int16_t theta;
    // int16_t width;
    int8_t direction; // Added field for Direction
    // int8_t reserved;  // Added field for Reserved
    int16_t height;
    uint16_t distance;
};


enum OTASDatastep {
    DStep0 = 0, // 초기 단계
    DStep1 = 1, // 단계 1
    DStep2 = 2, // 단계 2
    DStepP = 0xA, // 단계 Pass
    DStepF = 0xF  // 단계 Fail
};

#ifdef __cplusplus
extern "C" {
#endif

void APIGetVersion(std::string &data);
std::string APIGetTotalVersion();
void APISetTotalVersion(std::string data);
void APISetEnc(bool enable);

void start_server();
void stop_server();
/*********************************************************************/
bool reqGetPosition(void);
bool reqGetLidarSensorStatus(void);
bool reqGetMotorStatus(void);
bool reqGetMotorStatusV2(void);
bool reqGetCameraStatus(void);
bool reqGetCameraStatusV2(void);
bool reqGetLineLaserStatus(void);
bool reqGetLineLaserStatusV2(void);
bool reqGetTofStatus(void);
bool reqGetTofStatusV2(void);
bool reqGetIRStatus(void);
bool reqGetRecvIRStatus(void);
bool reqGetCliffIRStatus(void);

bool reqGetSonicStatus(void);
bool reqGetBatteryStatus(void);
bool reqGetBatteryStatusV2(void);
bool reqGetSoftwareVersion(void);
bool reqGetRobotInfo(void);
bool reqGetErrorList(void);
bool reqGetTargetPosition(void);
bool reqGetMapStatus(void);
bool reqGetMapData(void);
bool reqGetLidarData(void);
bool reqGetModifiedMapData(void);
bool reqGetAllStatus(void);
bool reqGetAllMovingInfo(void);
bool reqGetAllStatusV2(void);
bool reqGetAllMovingInfoV2(void);
bool reqGetDockingStatus(void);
bool reqGetRobotSpeed(void);
bool reqGetRobotStatus(void);
bool reqGetActionStatus(void);
bool reqGetNotification(void);

bool reqGetTargetPositionCalculate(TargetPosition_t &settings);
bool reqGetAICalibration(AICalibration_t &settings);

bool reqGetLogData(void);

// void resGetPosition(double x, double y, double theta);
void resGetjson2(const std::string& mode, const std::string& waterLevel, const std::string& soundVolume, bool tumbleDryerFlag, int dryLevel, const std::string& dryPower, const std::string& language, const std::string& country, const std::string& utcOffset);
void resGetjson_with_base64(const std::string& base64Data);

void resGetPosition(double x, double y, double theta);
void resGetLidarSensorStatus(int front, int rear);
void resGetMotorStatus(double MotorStatusleftSpeed, double MotorStatusrightSpeed,double MotorStatusleftPower, double MotorStatusrightPower,int MotorStatusleftStatus, int MotorStatusrightStatus);
void resGetMotorStatusV2(double MotorStatusleftRPM, double MotorStatusrightRPM,double MotorStatusleftCurrent, double MotorStatusrightCurrent,int MotorStatusleftType, int MotorStatusrightType);

void resGetCameraStatus(int status);
void resGetCameraStatusV2(int status, const std::vector<ObjectDataV2> &objects);
void resGetLineLaserStatus(int left, int right);
void resGetLineLaserStatusV2(int status, const  std::vector<LLDataV2>& llDataList);
void resGetTofStatus(int leftStatus, double leftDistance,int rightStatus, double rightDistance,int topStatus, double topDistance);
void resGetTofStatusV2(int leftStatus, const std::vector<int> &leftDistances,
                     int rightStatus, const std::vector<int> &rightDistances,
                     int topStatus, int topDistance);
void resGetIRStatus(bool sensor1, bool sensor2, bool sensor3, bool sensor4, bool sensor5, bool sensor6, bool stationIR1, bool stationIR2, bool stationIR3, bool stationIR4);

void resGetRecvIRStatus(bool RecvIR1, bool RecvIR2, bool RecvIR3, bool RecvIR4);
void resGetCliffIRStatus(bool CliffIR1, bool CliffIR2, bool CliffIR3, bool CliffIR4, bool CliffIR5, bool CliffIR6);

void resGetSonicStatus(int leftStatus, bool LeftSenseing, int rightStatus, bool rightSenseing);
void resGetBatteryStatus(double voltage,double current,double temperature,double chargeState,double capacity,double designCapacity,double percentage,double supplyStatus,double supplyHealth,double supplyTechnology,double present,double cellVoltagem,double cellTemperature,double location,const std::string& serialNumber);
void resGetBatteryStatusV2(int cell_voltage1, int cell_voltage2, int cell_voltage3,
                         int cell_voltage4, int cell_voltage5, int total_capacity,
                         int remaining_capacity, int battery_manufacturer, int battery_percent,
                         double battery_voltage, double battery_current, int battery_temperature1,
                         int battery_temperature2, int design_capacity, int number_of_cycles);
void resGetSoftwareVersion(const std::string &version);
void resGetRobotInfo(const std::string &serialNumber, int status);
void resGetErrorList(const std::vector<ErrorList_t> &ErrorList);
void resGetTargetPosition(double x, double y, double theta);
void resGetMapStatus(bool mapping, const std::string&  data);


void resGetMapDataS(double width, double height, double resolution, double posX, double posY, const std::string &data);
void resGetMapDataB(double width, double height, double resolution, double posX, double posY,  const unsigned char* data, size_t length);
void resGetLidarData(double front, double rear);
void resGetNotification(int code, std::string Notification);

void resGetModifiedMapData(const std::string&  data);
void resGetModifiedMapDataB(double width, double height, const unsigned char* data, size_t length);
void resGetRobotSpeed(double mS, double radS);
void resGetTargetPositionCalculate(double Time, double Distance, double x, double y, double theta);
void resGetRobotStatus(int status);
void resGetActionStatus(int status);

void resGetAICalibration(int result);

bool resGetLogData(const std::string &folderPath);

void resGetAllStatus(
    int LidarSensorleft, int LidarSensorright, 
    double MotorStatusleftSpeed, double MotorStatusrightSpeed,double MotorStatusleftPower, double MotorStatusrightPower,int MotorStatusleftStatus, int MotorStatusrightStatus,
    int Camerastatus, 
    int LineLaserleft, int LineLaserright, 
    int TofStatusleftStatus, double TofStatusleftDistance,int TofStatusrightStatus, double TofStatusrightDistance,int TofStatustopStatus, double TofStatustopDistance,
    bool IRsensor1, bool IRsensor2, bool IRsensor3, bool IRsensor4, bool IRsensor5, bool IRsensor6,bool stationIR1, bool stationIR2, bool stationIR3, bool stationIR4,
    int SonicleftStatus, int SonicleftSenseing, int SonicrightStatus, int SonicrightSenseing,
    double Batteryvoltage, double Batterycurrent, double Batterytemperature, double BatterychargeState, double Batterycapacity,
    double BatterydesignCapacity, double Batterypercentage, double BatterysupplyStatus, double BatterysupplyHealth,
    double BatterysupplyTechnology, double Batterypresent, double BatterycellVoltage, double BatterycellTemperature,
    double Batterylocation, std::string BatteryserialNumber);

    void resGetAllStatusV2(
        int LidarSensorfront, int LidarSensorrear,
        double MotorStatusleftRPM, double MotorStatusrightRPM, double MotorStatusleftCurrent, double MotorStatusrightCurrent, int MotorStatusleftType, int MotorStatusrightType,
        int Camerastatus, const std::vector<ObjectDataV2> &objects,
        int LineLaserStatus, const std::vector<LLDataV2> &llDataList,
        int TofStatusleftStatus, int TofStatusrightStatus, int TofStatustopStatus,
        const std::vector<int> &leftDistances, const std::vector<int> &rightDistances, int TofStatustopDistance,
        bool CliffIR1, bool CliffIR2, bool CliffIR3, bool CliffIR4, bool CliffIR5, bool CliffIR6,
        bool RecvIR1, bool RecvIR2, bool RecvIR3, bool RecvIR4,
        double cellVoltage1, double cellVoltage2, double cellVoltage3, double cellVoltage4, double cellVoltage5,
        double totalCapacity, double remainingCapacity, int batteryManufacturer, double batteryPercent,
        double batteryVoltage, double batteryCurrent, double batteryTemperature1, double batteryTemperature2,
        double designCapacity, int numberOfCycles);

void resGetAllMovingInfo(
    int movingState,
    double x, double y,double theta,
    double Targetx, double Targety,double Targettheta);
    
void resGetAllMovingInfoV2(
        int movingState,
        bool validPosition,
        double x, double y, double theta,
        bool validTargetPosition,
        double Targetx, double Targety, double Targettheta);

void resGetIntegrationMapDataB(const unsigned char *data, size_t length);
void resGetIntegrationModifiedMapDataB(const unsigned char *data, size_t length);

void resGetDockingStatus(int State);

/*********************************************************************/

bool reqSetReturnToChargingStation(void);
bool reqSetEmergencyStop(void);
bool reqSetStartLidar(void);
bool reqSetStopLidar(void);
bool reqSetStartCharging(void);
bool reqSetStopCharging(void);
bool reqSetSoftwareReset(void);
bool reqSetMaxDrivingSpeed(MaxDrivingSpeed_t& settings);
bool reqSetMotorManual_RPM(MotorManual_RPM_t& settings);
bool reqSetMotorManual_VW(MotorManual_VW_t& settings);
bool reqSetTargetPosition(TargetPosition_t& settings);
bool reqSetStationRepositioning(StationRepositioning_t &settings);
bool reqSetDriving(Driving_t& settings);
bool reqSetMapping(Mapping_t& settings);
bool reqSetModifiedMapDataS(std::string&  data);
//bool reqSetModifiedMapDataB(std::vector<unsigned char> &data) ;
bool reqSetModifiedMapDataB(ModifiedMapDataB_t &data);
bool reqSetByPassOneData(ByPassOne_t &data);
bool reqSetPGMMapData(PGMMapDataB_t &data);

bool reqSetIntegrationModifiedMapDataB(IntegrationModifiedMapDataB_t &data);

bool reqSetheartbeat(void);
bool reqSetExcelSteps(ExcelSteps_t &data);
bool reqSetRotation(Rotation_t &data);
bool reqSetTimeData(TimeData_t &data);

bool reqSetOTA(ota_t &data);
bool reqSetOTAdata(otadataB_t &data);
bool reqSetFactoryReset(void);
bool reqSetRecovery(void);

bool reqSetFactoryMode(FactoryMode_t &data);
bool reqSetBatterySleepMode(BatterySleepMode_t &data);
bool reqSetSensorInspectionMode(InspectionMode_t &data);

bool reqSetDockingState(DockingStatus_t &data);

void resSetReturnToChargingStation(bool Result);
void resSetEmergencyStop(bool Result);
void resSetStartLidar(bool Result);
void resSetStopLidar(bool Result);
void resSetStartCharging(bool Result);
void resSetStopCharging(bool Result);
void resSetSoftwareReset(bool Result);
void resSetMaxDrivingSpeed(bool Result);
void resSetMotorManual_RPM(bool Result);
void resSetMotorManual_VW(bool Result);
void resSetTargetPosition(bool Result);
void resSetStationRepositioningIdx(bool Result);
void resSetDriving(bool Result);
void resSetMapping(bool Result);
void resSetModifiedMapData(bool Result);
void resSetPGMMapData(bool Result);

void resSetIntegrationModifiedMapData(bool Result);
void resSetByPassOneData(bool Result);
void resSetByPassTwoData(bool Result);

void resSetheartbeat(bool Result);
void resSetExcelSteps(bool Result);
void resSetRotation(bool Result);
void resSetTimeData(bool Result);

void resSetOTA(bool Result);
// void resSetOTAdata(bool Result);
void resSetOTAdata(int progress, int state);
void resSetFactoryReset(bool Result);
void resSetRecovery(bool Result);

void resSetFactoryMode(bool Result);
void resSetBatterySleepMode(bool Result);
void resSetSensorInspectionMode(bool Result);

void resSetDockingState(bool Result);

bool RecvErrorRes(void);

/*********************************************************************/
void EmergencyMSG_OTA(int state,int progress,std::string version);
void EmergencyMSG_ERROR(const std::vector<ErrorList_t> &ErrorList);

bool reqSetSettingCommand(SettingCommand_t &data);
void resSetSettingCommand(bool Result);
bool reqSetConfigCommand(ConfigCommand_t &data);
void resSetConfigCommand(bool Result);
/*********************************************************************/
bool API_RecvJigData(int command, std::vector<uint8_t>& data);
void API_SendJigData(int command, const std::vector<uint8_t> &data);


/*********************************************************************/
std::string API_FileRead(const std::string &filename, unsigned char *key);
void API_FileSave(const std::string& plaintext, const std::string& filename, const unsigned char* key, const unsigned char* iv);
bool API_FileExist(const std::string& filename);
void API_FileDelete(const std::string& filename);
// start_server 없이 사용을 위해서는 API_GenerateKey 필수
void API_GenerateKey();
bool API_FileDataRead(ByPassOne_t &data);
bool API_FileDataApply();

#ifdef __cplusplus
}
#endif

#endif // LIBNETWOKR_H
