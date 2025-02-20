#include "udp_communication.hpp"

#define FW_VERSION_REQUEST 0x01
#define AI_VERSION_REQUEST 0x02

#define AUTO_CHARGE_HIGH          0x0         // MCU에서 충전 제어 (Auto) - 3.8A 고속충전
#define AUTO_CHARGE_LOW           0x1         // MCU에서 충전 제어 (Auto) - 1A 저속충전
#define CHARGE_STOP               0xF         // 충전 중지

#define FRONT_REAR_MIN_ANGLE (178.0f * M_PI / 180.0f)
#define FRONT_REAR_MAX_ANGLE (182.0f * M_PI / 180.0f)

#define LEFT_MIN_ANGLE  (222.0f * M_PI / 180.0f)
#define LEFT_MAX_ANGLE  (226.0f * M_PI / 180.0f)

#define RIGHT_MIN_ANGLE (134.0f * M_PI / 180.0f)
#define RIGHT_MAX_ANGLE (138.0f * M_PI / 180.0f)

#define AP_JIG_CHECK_ON_AMR 1 // AMR에서 단품지그 테스트할 때 : 1

#define MOTOR_DEFAULT_MODE 0x00
#define MOTOR_RPM_MODE     0x01
#define MOTOR_BREAK_MODE   0x03
#define MOTOR_DISABLE_MODE 0x0F
#define MOTOR_MODE_ERROR   0xFF

#define BATTERY_DEFAULT_MODE 0x00
#define BATTERY_HIGH_SPEED_MODE 0x01
#define BATTERY_LOW_SPEED_MODE 0x02
#define BATTERY_MODE_ERROR 0xFF

#define BOOT_TIME_OUT 60

const std::string MapEditFile = "/home/airbot/MapEdit.bin";

UdpCommunication::UdpCommunication() : rclcpp::Node("udp_communication")
{
    initializeData();

    amcl_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose",10,std::bind(&UdpCommunication::amclCallback, this, std::placeholders::_1));
    map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 1, std::bind(&UdpCommunication::mapCallback, this, std::placeholders::_1));
    battery_status_sub = this->create_subscription<robot_custom_msgs::msg::BatteryStatus>( "/battery_status", 10, std::bind(&UdpCommunication::batteryCallback, this, std::placeholders::_1));
    motor_status_sub = this->create_subscription<robot_custom_msgs::msg::MotorStatus>("/motor_status", 10, std::bind(&UdpCommunication::motorCallback, this, std::placeholders::_1));
    tof_status_sub = this->create_subscription<robot_custom_msgs::msg::TofData>("/tof_data", 10, std::bind(&UdpCommunication::tofCallback, this, std::placeholders::_1));
    station_data_sub_ = this->create_subscription<robot_custom_msgs::msg::StationData>("/station_data", 10, std::bind(&UdpCommunication::stationDataCallback, this, std::placeholders::_1));
    bottom_status_sub_ = this->create_subscription<std_msgs::msg::UInt8>("bottom_status", 10, std::bind(&UdpCommunication::bottomStatusCallback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<robot_custom_msgs::msg::ImuCalibration>("/jig_response_imu_calibration", 100, std::bind(&UdpCommunication::imuCalibrationCallback, this, std::placeholders::_1));
    fw_version_sub = this->create_subscription<std_msgs::msg::String>("/fw_version", 10, std::bind(&UdpCommunication::fw_version_callback, this, std::placeholders::_1));
    ai_version_sub = this->create_subscription<std_msgs::msg::String>("ai_version", 10, std::bind(&UdpCommunication::ai_version_callback, this, std::placeholders::_1));
    error_list_sub_ = this->create_subscription<robot_custom_msgs::msg::ErrorListArray>("/error_list", 10, std::bind(&UdpCommunication::errorListCallback, this, std::placeholders::_1));
    lidar_front_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan_front", 10, std::bind(&UdpCommunication::lidarFrontCallback, this, std::placeholders::_1));
    lidar_back_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan_back", 10, std::bind(&UdpCommunication::lidarBackCallback, this, std::placeholders::_1));
    line_laser_sub_ = this->create_subscription<robot_custom_msgs::msg::LineLaserDataArray>("/line_laser_data", 10, std::bind(&UdpCommunication::lineLaserCallback, this, std::placeholders::_1));
    camera_sub_ = this->create_subscription<robot_custom_msgs::msg::CameraDataArray>("/camera_data", 10, std::bind(&UdpCommunication::cameraCallback, this, std::placeholders::_1));

    req_state_sub = this->create_subscription<robot_custom_msgs::msg::RobotState>("/state_datas", 10, std::bind(&UdpCommunication::robotStateCallback, this, std::placeholders::_1)); 
    req_navi_sub = this->create_subscription<robot_custom_msgs::msg::NaviState>("/navi_datas", 10, std::bind(&UdpCommunication::movingStateCallback, this, std::placeholders::_1)); 
    node_status_sub_ = this->create_subscription<std_msgs::msg::UInt8>("/node_status", 10, std::bind(&UdpCommunication::nodeStateCallback, this, std::placeholders::_1)); 
    
    req_soc_cmd_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/soc_cmd", 10);
    e_stop_pub = this->create_publisher<std_msgs::msg::Bool>("/e_stop", 10);
    move_target_pub_ = this->create_publisher<robot_custom_msgs::msg::Position>("/move_target", 10);
    manual_move_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    charge_pub = this->create_publisher<std_msgs::msg::UInt8>("/charging_cmd", 10);
    block_area_pub_ = this->create_publisher<robot_custom_msgs::msg::BlockAreaList>("/block_areas", 10);
    block_wall_pub_ = this->create_publisher<robot_custom_msgs::msg::BlockAreaList>("/block_walls", 10);
    req_version_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/req_version", 1);
    
    jig_request_motor_pub_ = this->create_publisher<robot_custom_msgs::msg::RpmControl>("jig_request_motor", 10);
    jig_request_battery_pub_ = this->create_publisher<std_msgs::msg::UInt8>("jig_request_battery", 10);
    jig_request_imu_pub_ = this->create_publisher<std_msgs::msg::UInt8>("jig_request_imu", 10);

    lidar_cmd_pub_ = this->create_publisher<std_msgs::msg::Bool>("/cmd_lidar", 10);
    tof_cmd_pub_ = this->create_publisher<std_msgs::msg::Bool>("/cmd_tof", 10);
    linelaser_cmd_pub_ = this->create_publisher<std_msgs::msg::Bool>("/cmd_linelaser", 10);
    batterySleep_cmd_pub_ = this->create_publisher<std_msgs::msg::Bool>("/cmd_battery_sleep", 10);
    // req_state_pub_ = this->create_publisher<robot_custom_msgs::msg::RobotState>("/request_state", 10);
    //cmd_rpm_pub_ = this->create_publisher<robot_custom_msgs::msg::MotorRpm>("/cmd_rpm", 10);
    
    versionTimer_ = this->create_wall_timer( 1000ms, std::bind(&UdpCommunication::publishVersionRequest, this));
    udpTimer_ = this->create_wall_timer(500ms, std::bind(&UdpCommunication::udp_callback, this));
    socAutoSendTimer_ = this->create_wall_timer(1000ms, std::bind(&UdpCommunication::autoSocDataSender, this));
}

UdpCommunication::~UdpCommunication()
{

}

void UdpCommunication::initializeData()
{
    initNodeTime = this->now().seconds();
    //init State
    robotState = ROBOT_STATE::IDLE;
    robotStatus = ROBOT_STATUS::VOID;
    movingState = NAVI_STATE::IDLE;
    movefail_reason = NAVI_FAIL_REASON::VOID;
    nodeState = NODE_STATUS::IDLE;
    //init Version
    socData.version.bSet = false;
    socData.version.sw_ver = std::string(SOFTWARE_VERSION);
    socData.version.mcu_ver = "0.0";
    socData.version.ai_ver = "0.0";
    //initRobotPosition
    socData.robotPosition.valid = false;
    socData.robotPosition.x = 0.0;
    socData.robotPosition.y = 0.0;
    socData.robotPosition.theta = 0.0; 

    udpMode = UDP_COMMUNICATION::NORMAL;
    bSensorOn = false;
}


void UdpCommunication::setSocRobotPoseData(bool valid, double x, double y, double theta)
{
    socData.robotPosition.valid = valid;
    socData.robotPosition.x = x;
    socData.robotPosition.y = y;
    socData.robotPosition.theta = theta; 
}

void UdpCommunication::setSocFrontLidarData(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if(robotState != ROBOT_STATE::ONSTATION && msg->ranges.size() <= 0){
        RCLCPP_INFO(this->get_logger(), "Front ScanData is Empty!");
    }else{
        std::vector<int> dist_vec;
        generateFrontBackScan(msg,dist_vec);
        socData.lidarInfo.front_distance = getMinDistanceFromLidarSensor(dist_vec);
    }
}

void UdpCommunication::setSocRearLidarData(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if(robotState != ROBOT_STATE::ONSTATION && msg->ranges.size() <= 0){
        RCLCPP_INFO(this->get_logger(), "Rear ScanData is Empty!");
    }else{
        std::vector<int> dist_vec;
        generateFrontBackScan(msg,dist_vec);
        socData.lidarInfo.rear_distance = getMinDistanceFromLidarSensor(dist_vec);
    }
}

void UdpCommunication::setSocWheelMotorData(const robot_custom_msgs::msg::MotorStatus::SharedPtr msg)
{
    socData.motorInfo.mode = msg->motor_mode;
    socData.motorInfo.left.current = msg->left_motor_current;
    socData.motorInfo.left.encoder = msg->left_motor_encoder;
    socData.motorInfo.left.rpm = msg->left_motor_rpm;
    socData.motorInfo.left.status = msg->left_motor_status;
    socData.motorInfo.left.type = msg->left_motor_type;
    socData.motorInfo.left.tempterature = msg->left_motor_temperature;
    socData.motorInfo.right.current = msg->right_motor_current;
    socData.motorInfo.right.encoder = msg->right_motor_encoder;
    socData.motorInfo.right.rpm = msg->right_motor_rpm;
    socData.motorInfo.right.status = msg->right_motor_status;
    socData.motorInfo.right.type = msg->right_motor_type;
    socData.motorInfo.right.tempterature = msg->right_motor_temperature;
}

void UdpCommunication::setSocCameraData(const robot_custom_msgs::msg::CameraDataArray::SharedPtr msg)
{
    uint8_t num = msg->num;
    if(num > 0){
        #if USE_NEW_PROTOCOL > 0
        ObjectDataV2 temp;
        #else
        cameraData temp;
        #endif
        socData.cameraInfo.num = num;
        socData.cameraInfo.data.clear();
        for(int i = 0; i < socData.cameraInfo.num; i++){
            temp.class_id = msg->data_array[i].id;
            temp.confidence = msg->data_array[i].score;
            temp.height = msg->data_array[i].height;
            temp.width = msg->data_array[i].width;
            #if USE_NEW_PROTOCOL > 0
            temp.x = msg->data_array[i].x;
            temp.y = msg->data_array[i].y;
            temp.theta = msg->data_array[i].theta;
            #else
            temp.position.x = msg->data_array[i].x;
            temp.position.y = msg->data_array[i].y;
            temp.position.y = msg->data_array[i].theta;
            #endif
            temp.distance = msg->data_array[i].distance;
            socData.cameraInfo.data.push_back(temp);
        }

    }
}

void UdpCommunication::setSocLineLaserData(const robot_custom_msgs::msg::LineLaserDataArray::SharedPtr msg)
{
    uint8_t num = msg->num;
    
    if(num  > 0){
        #if USE_NEW_PROTOCOL > 0
        LLDataV2 temp;
        #else
        lineLaserData temp;
        #endif

        socData.lineLaserInfo.num = num;
        socData.lineLaserInfo.data.clear();
        
        for(int i = 0; i < socData.lineLaserInfo.num; i++){
            temp.direction = msg->data_array[i].direction;
            temp.distance = msg->data_array[i].distance;
            temp.height = msg->data_array[i].height;
            #if USE_NEW_PROTOCOL > 0
            temp.x = msg->data_array[i].x;
            temp.y = msg->data_array[i].y;
            temp.theta = msg->data_array[i].theta;
            #else
            temp.position.x = msg->data_array[i].x;
            temp.position.y = msg->data_array[i].y;
            temp.position.y = msg->data_array[i].theta;
            #endif
            socData.lineLaserInfo.data.push_back(temp);
        }
    }
}

void UdpCommunication::setTofStatus(uint8_t top, uint8_t bottom_left, uint8_t bottom_right)
{
    if(socData.tofInfo.top.status != top){
        if(top == 0x0){
            RCLCPP_INFO(this->get_logger(), "TOP-TOF ON");
        }else if(top == 0xF){
            RCLCPP_INFO(this->get_logger(), "TOP-TOF ERROR");
        }
    }
    if(socData.tofInfo.left_bottom.status != bottom_left){
        if(bottom_left == 0x0){
            RCLCPP_INFO(this->get_logger(), "BOTTOMLEFT-TOF ON");
        }else if(bottom_left == 0x1){
            RCLCPP_INFO(this->get_logger(), "BOTTOMLEFT-TOF OFF");
        }else if(bottom_left == 0xF){
            RCLCPP_INFO(this->get_logger(), "BOTTOMLEFT-TOF ERROR");
        }
    }
    if(socData.tofInfo.right_bottom.status != bottom_right){
        if(bottom_right == 0x0){
            RCLCPP_INFO(this->get_logger(), "BOTTOMRIGHT-TOF ON");
        }else if(bottom_right == 0x1){
            RCLCPP_INFO(this->get_logger(), "BOTTOMRIGHT-TOF OFF");
        }else if(bottom_right == 0xF){
            RCLCPP_INFO(this->get_logger(), "BOTTOMRIGHT-TOF ERROR");
        }
    }
    socData.tofInfo.top.status = top;
    socData.tofInfo.left_bottom.status = bottom_left;
    socData.tofInfo.right_bottom.status = bottom_right;
    
    // if(bSensorOn){
    //     if(socData.tofInfo.left_bottom.status == 0x1 ||  socData.tofInfo.right_bottom.status == 0x1){
    //         publishTofOnOff(true);
    //     }
    // }else{
    //      if(socData.tofInfo.left_bottom.status == 0x0 ||  socData.tofInfo.right_bottom.status == 0x0){
    //         publishTofOnOff(false);
    //     }
    // }
}
void UdpCommunication::setSocTofData(const robot_custom_msgs::msg::TofData::SharedPtr msg)
{
    uint8_t bottom_left = getLowerBits(msg->bot_status);
    uint8_t bottom_right = getUpperBits(msg->bot_status);
    setTofStatus(msg->top_status,bottom_left,bottom_right);
    socData.tofInfo.top.distance = msg->top;
    socData.tofInfo.right_bottom.status = getUpperBits(msg->bot_status);

    for(int i = 0; i < 16; i++){
        socData.tofInfo.left_bottom.data[i] = msg->bot_left[i];
        socData.tofInfo.right_bottom.data[i] = msg->bot_right[i];
    }
}

void UdpCommunication::setSocCliffLiftData(const std_msgs::msg::UInt8::SharedPtr msg)
{
    socData.cliffLiftInfo.value = msg->data;
}

void UdpCommunication::setSocDockReceiverData(const robot_custom_msgs::msg::StationData::SharedPtr msg)
{
    socData.dockingInfo.status = msg->docking_status;
    socData.dockingInfo.receiver.value = msg->receiver_status;
}

void UdpCommunication::setSocBatteryData(const robot_custom_msgs::msg::BatteryStatus::SharedPtr msg)
{
    socData.battInfo.cell_voltage1 = msg->cell_voltage1;
    socData.battInfo.cell_voltage1 = msg->cell_voltage2;
    socData.battInfo.cell_voltage1 = msg->cell_voltage3;
    socData.battInfo.cell_voltage1 = msg->cell_voltage4;
    socData.battInfo.cell_voltage1 = msg->cell_voltage5;

    socData.battInfo.current = (static_cast<double>(msg->battery_current) * 10) / 10.0;
    socData.battInfo.voltage = msg->battery_voltage;
    socData.battInfo.percent = msg->battery_percent;
    socData.battInfo.number_of_cycles = msg->number_of_cycles;
    socData.battInfo.temperature1 = msg->battery_temperature1;
    socData.battInfo.temperature2 = msg->battery_temperature2;
    socData.battInfo.manufacturer = msg->battery_manufacturer;
    socData.battInfo.remaining_capacity = msg->remaining_capacity;
    socData.battInfo.total_capacity = msg->total_capacity;
    socData.battInfo.design_capacity = msg->design_capacity;

    socData.battInfo.charge_status = msg->charge_status;
    socData.battInfo.charging_mode = msg->charging_mode;
}

void UdpCommunication::generateVersion()
{
    socData.version.bSet = true;
    socData.version.timestamp = this->now().seconds();
    socData.version.total_ver = (socData.version.sw_ver + ":" + socData.version.mcu_ver + " : " + socData.version.ai_ver);
    RCLCPP_INFO(this->get_logger(), "generateVersion SW VER : %s", socData.version.total_ver.c_str());
}

void UdpCommunication::setSocTargetPosition(double x, double y, double theta)
{
    socData.targetPosition.x = x;
    socData.targetPosition.y = y;
    socData.targetPosition.theta = theta;
    RCLCPP_INFO(this->get_logger(), "setSocTargetPosition X : %f, Y : %f, Theta : %f",x,y,theta);
}

void UdpCommunication::setSocMapData(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    socData.mapInfo.bReceived = true;
    socData.mapInfo.height = msg->info.height;
    socData.mapInfo.width = msg->info.width;
    socData.mapInfo.origin_x = msg->info.origin.position.x;
    socData.mapInfo.origin_y = msg->info.origin.position.y;
    socData.mapInfo.resolution = msg->info.resolution;
    socData.mapInfo.map_data = mapDataTypeConvert(msg->data,socData.mapInfo.height,socData.mapInfo.width,socData.mapInfo.origin_x,socData.mapInfo.origin_y);
}

void UdpCommunication::setSocRobotVelocity()
{

}

void UdpCommunication::setSocRobotState()
{

}

void UdpCommunication::setSocMovingState()
{
    
}

void UdpCommunication::setSocError(const robot_custom_msgs::msg::ErrorListArray::SharedPtr msg)
{

}

void UdpCommunication::setRobotState(ROBOT_STATE state, ROBOT_STATUS status)
{
    if(robotState != state){
        // if(state == ROBOT_STATE::ONSTATION){
        //     publishSensorOnOff(false);
        // }
        debugLogRobotState(state);
    }
    robotState = state;

    if(robotStatus != status){
        debugLogRobotStatus(status);
    }
    robotStatus = status;
}

void UdpCommunication::setNodeState(NODE_STATUS set)
{
    if(nodeState != set){
        debugLogNodeState(set);
    }
    nodeState = set;
}

void UdpCommunication::setNaviState(NAVI_STATE set)
{
    if(movingState != set){
        // if(set == NAVI_STATE::ARRIVED_GOAL){
        //     publishSensorOnOff(false);
        // }
        debugLogNaviState(set);
    }
    movingState = set;
}

void UdpCommunication::setNaviFailReason(NAVI_FAIL_REASON set)
{
    if(movefail_reason != set){
        debugLogNaviFailReason(set);
    }
    movefail_reason = set;
}

void UdpCommunication::debugLogRobotState(ROBOT_STATE set)
{
    switch (set)
    {
    case ROBOT_STATE::IDLE :
        RCLCPP_INFO(this->get_logger(), "ROBOT_STATE::IDLE");
        break;
    case ROBOT_STATE::AUTO_MAPPING :
        RCLCPP_INFO(this->get_logger(), "ROBOT_STATE::AUTO_MAPPING");
        break;
    case ROBOT_STATE::MANUAL_MAPPING :
        RCLCPP_INFO(this->get_logger(), "ROBOT_STATE::MANUAL_MAPPING");
        break;
    case ROBOT_STATE::NAVIGATION :
        RCLCPP_INFO(this->get_logger(), "ROBOT_STATE::NAVIGATION");
        break;
    case ROBOT_STATE::RETURN_CHARGER :
        RCLCPP_INFO(this->get_logger(), "ROBOT_STATE::RETURN_CHARGER");
        break;
    case ROBOT_STATE::DOCKING :
        RCLCPP_INFO(this->get_logger(), "ROBOT_STATE::DOCKING");
        break;
    case ROBOT_STATE::UNDOCKING :
        RCLCPP_INFO(this->get_logger(), "ROBOT_STATE::UNDOCKING");
        break;
    case ROBOT_STATE::ONSTATION :
        RCLCPP_INFO(this->get_logger(), "ROBOT_STATE::ONSTATION");
        break;
    case ROBOT_STATE::FACTORY_NAVIGATION :
        RCLCPP_INFO(this->get_logger(), "ROBOT_STATE::FACTORY_NAVIGATION");
        break;
    case ROBOT_STATE::ERROR :
        RCLCPP_INFO(this->get_logger(), "ROBOT_STATE::ERROR");
        break;                                
    default:
        break;
    }
}
void UdpCommunication::debugLogRobotStatus(ROBOT_STATUS set)
{
    switch (set)
    {
    case ROBOT_STATUS::VOID :
        RCLCPP_INFO(this->get_logger(), "ROBOT_STATUS::VOID");
        break;
    case ROBOT_STATUS::START :
        RCLCPP_INFO(this->get_logger(), "ROBOT_STATUS::START");
        break;
    case ROBOT_STATUS::PAUSE :
        RCLCPP_INFO(this->get_logger(), "ROBOT_STATUS::PAUSE");
        break;
    case ROBOT_STATUS::COMPLETE :
        RCLCPP_INFO(this->get_logger(), "ROBOT_STATUS::COMPLETE");
        break;
    case ROBOT_STATUS::FAIL :
        RCLCPP_INFO(this->get_logger(), "ROBOT_STATUS::FAIL");
        break;                              
    default:
        break;
    }
}

void UdpCommunication::debugLogNodeState(NODE_STATUS set)
{
    switch (set)
    {
    case NODE_STATUS::IDLE :
        RCLCPP_INFO(this->get_logger(), "NODE_STATUS::IDLE");
        break;
    case NODE_STATUS::AUTO_MAPPING :
        RCLCPP_INFO(this->get_logger(), "NODE_STATUS::AUTO_MAPPING");  
        break;
    case NODE_STATUS::MANUAL_MAPPING :
        RCLCPP_INFO(this->get_logger(), "ODE_STATUS::MANUAL_MAPPING");
        break;
    case NODE_STATUS::NAVI :
        RCLCPP_INFO(this->get_logger(), "NODE_STATUS::NAVI");
        break;                     
    default:
        break;
    }
}

void UdpCommunication::debugLogNaviState(NAVI_STATE set)
{
    switch (set)
    {
    case NAVI_STATE::IDLE :
        RCLCPP_INFO(this->get_logger(), "NAVI_STATE::IDLE");
        break;
    case NAVI_STATE::MOVE_GOAL :
        RCLCPP_INFO(this->get_logger(), "NAVI_STATE::MOVE_GOAL");
        break;
    case NAVI_STATE::ARRIVED_GOAL :
        RCLCPP_INFO(this->get_logger(), "NAVI_STATE::ARRIVED_GOAL");
        break;
    case NAVI_STATE::PAUSE :
        RCLCPP_INFO(this->get_logger(), "NAVI_STATE::PAUSE");
        break;
    case NAVI_STATE::FAIL :
        RCLCPP_INFO(this->get_logger(), "NAVI_STATE::FAIL");
        break;
    case NAVI_STATE::ROTAION :
        RCLCPP_INFO(this->get_logger(), "NAVI_STATE::ROTAION");
        break;
    case NAVI_STATE::COMPLETE_ROTATION :
        RCLCPP_INFO(this->get_logger(), "NAVI_STATE::COMPLETE_ROTATION");
        break;                       
    default:
        break;
    }
}

void UdpCommunication::debugLogNaviFailReason(NAVI_FAIL_REASON set)
{
    switch (set)
    {
    case NAVI_FAIL_REASON::NODE_OFF :
        RCLCPP_INFO(this->get_logger(), "NAVI_FAIL_REASON::NODE_OFF");
        break;
    case NAVI_FAIL_REASON::SERVER_NO_ACTION :
        RCLCPP_INFO(this->get_logger(), "NAVI_FAIL_REASON::SERVER_NO_ACTION");
        break;
    case NAVI_FAIL_REASON::GOAL_ABORT :
        RCLCPP_INFO(this->get_logger(), "NAVI_FAIL_REASON::GOAL_ABORT");
        break;
    case NAVI_FAIL_REASON::UNKWON :
        RCLCPP_INFO(this->get_logger(), "NAVI_FAIL_REASON::UNKWON ");
        break;                       
    default:
        break;
    }
}

void UdpCommunication::setJigData(JIG_DATA_KEY key, const std::vector<uint8_t>& data) {
    jigData[key] = data; // key에 해당하는 데이터를 저장
}

// 데이터를 조회하는 함수
std::vector<uint8_t> UdpCommunication::getJigData(JIG_DATA_KEY key)
{
    auto it = jigData.find(key);
    if (it != jigData.end()) {
        return it->second; // key가 존재하면 해당 데이터를 반환
    }

    return {}; // key가 존재하지 않으면 빈 벡터 반환
}

void UdpCommunication::nodeStateCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    setNodeState(static_cast<NODE_STATUS>(msg->data));
}

void UdpCommunication::robotStateCallback(const robot_custom_msgs::msg::RobotState::SharedPtr msg)
{
    setRobotState(static_cast<ROBOT_STATE>(msg->state),static_cast<ROBOT_STATUS>(msg->status));
}

void UdpCommunication::movingStateCallback(const robot_custom_msgs::msg::NaviState::SharedPtr msg)
{
    setNaviState(static_cast<NAVI_STATE>(msg->state));
    setNaviFailReason(static_cast<NAVI_FAIL_REASON>(msg->fail_reason));
}

void UdpCommunication::batteryCallback(const robot_custom_msgs::msg::BatteryStatus::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(battery_mutex_);

    if(udpMode == UDP_COMMUNICATION::AMR_JIG_MODE){
        std::vector<uint8_t> data = jigDataConvertBattery(msg);
        setJigData(JIG_DATA_KEY::BATTERY,data);
    } else if(udpMode == UDP_COMMUNICATION::AP_JIG_MODE){
        return ;
    } else {
        setSocBatteryData(msg);
    }
}

void UdpCommunication::motorCallback(const robot_custom_msgs::msg::MotorStatus::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(wheel_motor_mutex_);

    if(udpMode == UDP_COMMUNICATION::AMR_JIG_MODE){
        std::vector<uint8_t> data = jigDataConvertWheelMotor(msg);
        setJigData(JIG_DATA_KEY::WHEEL_MOTOR,data);
    } else if(udpMode == UDP_COMMUNICATION::AP_JIG_MODE){
        return ;
    } else {
        setSocWheelMotorData(msg);
    }
}

void UdpCommunication::amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    const auto& pose = msg->pose.pose;
    bool valid = true;
    double x = static_cast<int>(pose.position.x * 1000000 + 0.5) / 1000000.0;
    double y = static_cast<int>(pose.position.y * 1000000 + 0.5) / 1000000.0;
    double theta = quaternion_to_euler(pose.orientation);
    setSocRobotPoseData(valid,x,y,theta);
}

void UdpCommunication::fw_version_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // if(socData.version.mcu_ver != msg.data){

    // }
    socData.version.mcu_ver = msg->data;
}

void UdpCommunication::ai_version_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // if(socData.version.ai_ver != msg->data){

    // }
    socData.version.ai_ver = msg->data;
}
		
void UdpCommunication::publishVersionRequest()
{
    double now_sec = this->now().seconds();
    double runTime = now_sec-initNodeTime;

    std::string mcu_version = socData.version.mcu_ver;
    std::string ai_version = socData.version.ai_ver;

    if(runTime >= BOOT_TIME_OUT){
        generateVersion();
        resetVersionTimer();
        return;
    }else{
        std_msgs::msg::UInt8 msgs;
        msgs.data = 0;
        if(mcu_version != "0.0" ){ //&& ai_version != "0.0"
            generateVersion();
            resetVersionTimer();
            return;
        }

        if (mcu_version == "0.0"){
            msgs.data |= FW_VERSION_REQUEST;
        }
        if (ai_version == "0.0"){
            msgs.data |= AI_VERSION_REQUEST;
        } 
            
        if (msgs.data != 0){
            req_version_pub_->publish(msgs);
        }
    }
    RCLCPP_INFO(this->get_logger(), "publishVersionRequest : %f",runTime);
}

void UdpCommunication::generateFrontBackScan(const sensor_msgs::msg::LaserScan::SharedPtr msg, std::vector<int>& vecDist)
{
    for (int i = 0; i < msg->ranges.size(); i++) {
        float angle = msg->angle_min + i * msg->angle_increment;
        if (angle >= FRONT_REAR_MIN_ANGLE && angle <= FRONT_REAR_MAX_ANGLE) {
            int dist = static_cast<int>(msg->ranges[i] * 1000);
            if (abs(dist) > 1) {
                vecDist.push_back(dist);
            }
        }
    }
}

void UdpCommunication::lidarFrontCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(scan_front_mutex_);

    if(udpMode == UDP_COMMUNICATION::AMR_JIG_MODE){
        std::vector<int> vec_left, vec_right;
        splitLidarScan(msg,vec_left,vec_right);
        std::vector<uint8_t> data = jigDataConvertLidar(getMinDistanceFromLidarSensor(vec_left),getMinDistanceFromLidarSensor(vec_right));
        setJigData(JIG_DATA_KEY::FRONT_LIDAR,data);
    } else if(udpMode == UDP_COMMUNICATION::AP_JIG_MODE){
        apJigFrontLaserData = msg;
        // RCLCPP_INFO(this->get_logger(), "ranges[0], ranges[100] : '%f', '%f'", msg->ranges[0],
        //             msg->ranges[100]);
    }else{
        setSocFrontLidarData(msg);
    }
}

void UdpCommunication::lidarBackCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(scan_back_mutex_);

    if(udpMode == UDP_COMMUNICATION::AMR_JIG_MODE){
        std::vector<int> vec_left, vec_right;
        splitLidarScan(msg,vec_left,vec_right);
        std::vector<uint8_t> data = jigDataConvertLidar(getMinDistanceFromLidarSensor(vec_left),getMinDistanceFromLidarSensor(vec_right));
        setJigData(JIG_DATA_KEY::REAR_LIDAR,data);
    } else if(udpMode == UDP_COMMUNICATION::AP_JIG_MODE){
        apJigBackLaserData = msg;
        // RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", msg->ranges[0],
        //         msg->ranges[100]);
    }else{
        setSocRearLidarData(msg);
    }
}

void UdpCommunication::lineLaserCallback(const robot_custom_msgs::msg::LineLaserDataArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(line_laser_mutex_);
    setSocLineLaserData(msg);
}

void UdpCommunication::cameraCallback(const robot_custom_msgs::msg::CameraDataArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(camera_mutex_);
    setSocCameraData(msg);
}

void UdpCommunication::tofCallback(const robot_custom_msgs::msg::TofData::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(tof_mutex_);

    if(udpMode == UDP_COMMUNICATION::AMR_JIG_MODE){
        std::vector<uint8_t> data = jigDataConvertTof(msg);
        setJigData(JIG_DATA_KEY::TOF,data);
    } else if(udpMode == UDP_COMMUNICATION::AP_JIG_MODE){
        return ;
    }else{
        setSocTofData(msg);
    }
}

void UdpCommunication::errorListCallback(const robot_custom_msgs::msg::ErrorListArray::SharedPtr msg)
{
    size_t array_size = msg->data_array.size();
    for (int i=0; i<array_size; ++i) {
        const auto& error = msg->data_array[i];

        bool isSameRank = false;
        for (const auto& existing_error : error_list) {
            if (existing_error.rank == error.rank) {
                isSameRank = true;
                break;
            }
        }

        if (!isSameRank) {
            RCLCPP_INFO(this->get_logger(), "Received error: rank = %d, error_code = %s",
                        error.rank, error.error_code.c_str());
            ErrorList_t error_entry;
            error_entry.rank = error.rank;
            error_entry.errorCode = error.error_code;  // error_code 필드를 적절히 대응
            error_list.push_back(error_entry);  
        }
    }
    EmergencyMSG_ERROR(error_list);
}

void UdpCommunication::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    setSocMapData(msg);
}

void UdpCommunication::bottomStatusCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(bottom_status_mutex_);

    if(udpMode == UDP_COMMUNICATION::AMR_JIG_MODE){
        std::vector<uint8_t> data(1);
        data[0] = static_cast<uint8_t>(msg->data & 0xFF);
        setJigData(JIG_DATA_KEY::CLIFF_LIFT,data);
    } else if(udpMode == UDP_COMMUNICATION::AP_JIG_MODE){
        return ;
    }else{
        setSocCliffLiftData(msg);
    }
}

void UdpCommunication::stationDataCallback(const robot_custom_msgs::msg::StationData::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(charger_data_mutex_);

    if(udpMode == UDP_COMMUNICATION::AMR_JIG_MODE){
        std::vector<uint8_t> data(1);
        data[0] = static_cast<uint8_t>(msg->receiver_status & 0xFF);
        setJigData(JIG_DATA_KEY::DOCK_RECEIVER,data);
    } else if(udpMode == UDP_COMMUNICATION::AP_JIG_MODE){
        return ;
    }else{
        setSocDockReceiverData(msg);
    }
}

void UdpCommunication::imuCalibrationCallback(const robot_custom_msgs::msg::ImuCalibration::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(imu_cal_mutex_);

    std::vector<uint8_t> data = jigDataConvertImuCalibration(msg);
    setJigData(JIG_DATA_KEY::IMU_CALIBRATION,data);
}

void UdpCommunication::resSocData(REQUEST_DATA_WHAT what)
{
    switch (what)
    {
    case REQUEST_DATA_WHAT::ROBOT_POSITION:
        resSocRobotPose();
        break;
    case REQUEST_DATA_WHAT::MAP:
        resSocMap();
        break;
    case REQUEST_DATA_WHAT::SW_VERSION :
        resSocSoftWareVision();
        break;
    case REQUEST_DATA_WHAT::MAP_STATUS :
        resSocNodeStatus();
        break;
    case REQUEST_DATA_WHAT::TARGET_POSITION :
        resSocTargetPosition();
        break;
    case REQUEST_DATA_WHAT::BATTERY_STATUS :
        resSocBattData();
        break;
    case REQUEST_DATA_WHAT::DOCKING_STATUS :
        resSocDockingStatus();
        break;
    case REQUEST_DATA_WHAT::ROBOT_INFO :
        resSocRobotInfo();
        break;
    case REQUEST_DATA_WHAT::MODIFIED_MAP :
        resSocModifiedMap();
        break;
    case REQUEST_DATA_WHAT::MOVING_INFO :
        resSocMovingInfo();
        break;
    case REQUEST_DATA_WHAT::ROBOT_VELOCITY :
        resSocRobotVelocity();
        break;
    case REQUEST_DATA_WHAT::MOTOR_STATUS :
        resSocWheelMotorData();
        break;                                
    case REQUEST_DATA_WHAT::CAMERA_STATUS :
        resSocCameraData();
        break;
    case REQUEST_DATA_WHAT::LINE_LASER_STATUS :
        resSocLineLaserData();
        break;
    case REQUEST_DATA_WHAT::TOF_STATUS :
        resSocTofData();
        break;
    case REQUEST_DATA_WHAT::ERROR_LIST :
        resSocErrorList();
        break;
    case REQUEST_DATA_WHAT::LIDAR_DATA :
        resSocLidarData();
        break;
    #if USE_NEW_PROTOCOL > 0    
    case REQUEST_DATA_WHAT::CLIFF_LIFT_DATA :
        resSocCliffLiftData();
        break;
    case REQUEST_DATA_WHAT::DOCK_RECEIVER :
        resSocDockReceiverData();
        break;
    #endif        
    case REQUEST_DATA_WHAT::ALL_STATUS :
        resSocAllSensor();
        break;                                
    default:
        break;
    }
}

void UdpCommunication::resSocRobotPose()
{
    pose pose = getRobotPose();
    resGetPosition(pose.x, pose.y, pose.theta);
}

void UdpCommunication::resSocMap()
{
    MapInfo mapInfo = getMapInfo();
    if((nodeState == NODE_STATUS::AUTO_MAPPING || nodeState == NODE_STATUS::MANUAL_MAPPING)){
        resGetMapDataB(mapInfo.width,mapInfo.height,mapInfo.resolution,mapInfo.origin_x,mapInfo.origin_y, mapInfo.map_data.data() ,mapInfo.map_data.size());
    }else{
        bool bReadMap;
        int height,width,size;
        double origin_x,origin_y;
        std::vector<uint8_t> saved_map;
        if(robotState == ROBOT_STATE::FACTORY_NAVIGATION){
            bReadMap = readPGM("/home/airbot/airbot_ws/install/airbot_navigation/share/airbot_navigation/maps/factory_map.pgm", width, height, saved_map, origin_x, origin_y);
            size = width * height;
            if(bReadMap){
                if(saved_map.empty() || size == 0){
                    RCLCPP_INFO(this->get_logger(), "factory map is empty");    
                }else{
                    resGetMapDataB(width,height,mapInfo.resolution,origin_x,origin_y, saved_map.data(),saved_map.size());      
                }
            }else{
                RCLCPP_INFO(this->get_logger(), "factory map read fail error");
            }
        }
        else{
            bReadMap = readPGM("/home/airbot/test1_map.pgm", width, height, saved_map, origin_x, origin_y);
            size = width * height;
            if(bReadMap){
                if(saved_map.empty() || size == 0){
                    RCLCPP_INFO(this->get_logger(), "map is empty");    
                }else{
                    resGetMapDataB(width,height,mapInfo.resolution,origin_x,origin_y, saved_map.data(),saved_map.size());      
                }
            }else{
                RCLCPP_INFO(this->get_logger(), "map read fail error");
            }
        }
    }
}

void UdpCommunication::resSocSoftWareVision()
{
	resGetSoftwareVersion(socData.version.total_ver);
}

void UdpCommunication::resSocNodeStatus()
{
    switch (nodeState)
    {
    case NODE_STATUS::AUTO_MAPPING :
    case NODE_STATUS::MANUAL_MAPPING :
        resGetMapStatus(true, "mapping");
        break;
    case NODE_STATUS::NAVI :
        resGetMapStatus(false, "navigation");
        break;
    default:
        resGetMapStatus(false, "IDLE");
        break;
    }
}

#if USE_NEW_PROTOCOL > 0
void UdpCommunication::resSocRobotState()
{
    resGetRobotStatus(static_cast<int>(robotState));
}

void UdpCommunication::resSocRobotStatus()
{
    resGetActionStatus(static_cast<int>(robotStatus));
}
#endif

void UdpCommunication::resSocTargetPosition()
{
    pose pose = getTargetPose();
    resGetTargetPosition(pose.x, pose.y, pose.theta);
}

void UdpCommunication::resSocBattData()
{
    #if USE_NEW_PROTOCOL > 0
    resGetBatteryStatusV2(socData.battInfo.cell_voltage1,socData.battInfo.cell_voltage2,socData.battInfo.cell_voltage3,socData.battInfo.cell_voltage4,socData.battInfo.cell_voltage5,
    socData.battInfo.total_capacity,socData.battInfo.remaining_capacity,socData.battInfo.manufacturer,
    socData.battInfo.percent,socData.battInfo.voltage,socData.battInfo.current,socData.battInfo.temperature1,socData.battInfo.temperature2,
    socData.battInfo.design_capacity,socData.battInfo.number_of_cycles);
    #else
    double state = static_cast<double>(socData.battInfo.charge_status);
    resGetBatteryStatus(socData.battInfo.voltage,socData.battInfo.current,socData.battInfo.temperature1,state,socData.battInfo.remaining_capacity,socData.battInfo.design_capacity,socData.battInfo.percent,
    0.0,0.0,0,0,0.0,0.0,0.0,"0.0");
    #endif
}

void UdpCommunication::resSocDockingStatus()
{
    resGetDockingStatus(socData.dockingInfo.status);
}

void UdpCommunication::resSocRobotInfo()
{
    resGetRobotInfo("EVERYBOT", 1);
}

void UdpCommunication::resSocModifiedMap()
{
    int height,width;
    double origin_x,origin_y;
    std::vector<uint8_t> modified_map;

    std::cout << "reqGetModifiedMapData <<<<<" << '\n';

    if (!readPGM("/home/airbot/test1_map.pgm", width, height, modified_map, origin_x, origin_y)) {
        std::cerr << "Error reading PGM file." << std::endl;
        return;
    } 
    resGetModifiedMapDataB(width,height,modified_map.data(),modified_map.size()); 
}

void UdpCommunication::resSocMovingInfo()
{
    int status = static_cast<int>(movingState);
    #if USE_NEW_PROTOCOL > 0
    resGetAllMovingInfoV2(status,
    socData.robotPosition.valid,socData.robotPosition.x,socData.robotPosition.y,socData.robotPosition.theta,
    socData.targetPosition.valid,socData.targetPosition.x,socData.targetPosition.y,socData.targetPosition.theta);
    #else
    resGetAllMovingInfo(status,socData.robotPosition.x,socData.robotPosition.y,socData.robotPosition.theta,socData.targetPosition.x,socData.targetPosition.y,socData.targetPosition.theta);
    #endif
}

void UdpCommunication::resSocCalculateTarget()
{
    //robotPostion to Target - distance & time
    // double Time = 0, double Distance = 0;
    // resGetTargetPositionCalculate( Time,Distance, Settings11.x,Settings11.y,Settings11.theta);
}

void UdpCommunication::resSocRobotVelocity()
{
    resGetRobotSpeed(socData.velocity.v,socData.velocity.w);
}

void UdpCommunication::resSocLidarData()
{
    resGetLidarData(socData.lidarInfo.front_distance, socData.lidarInfo.rear_distance);
}

void UdpCommunication::resSocWheelMotorData()
{
    #if USE_NEW_PROTOCOL > 0
    resGetMotorStatusV2(socData.motorInfo.left.rpm,socData.motorInfo.right.rpm,socData.motorInfo.left.current,socData.motorInfo.right.current,socData.motorInfo.left.status,socData.motorInfo.right.status);
    #else
    resGetMotorStatus(socData.motorInfo.left.rpm,socData.motorInfo.right.rpm,socData.motorInfo.left.current,socData.motorInfo.right.current,socData.motorInfo.left.status,socData.motorInfo.right.status);
    #endif
}

void UdpCommunication::resSocCameraData()
{
    #if USE_NEW_PROTOCOL > 0
    resGetCameraStatusV2(0,socData.cameraInfo.data);
    #else
    resGetCameraStatus(0);
    #endif
}

void UdpCommunication::resSocLineLaserData()
{
    #if USE_NEW_PROTOCOL > 0
    resGetLineLaserStatusV2(0,socData.lineLaserInfo.data);
    #else
    resGetLineLaserStatus(0, 0);
    #endif
}

#if USE_NEW_PROTOCOL > 0    
void UdpCommunication::resSocCliffLiftData()
{
    resGetCliffIRStatus(socData.cliffLiftInfo.cliff_front_center,socData.cliffLiftInfo.cliff_front_left,socData.cliffLiftInfo.cliff_back_left,
    socData.cliffLiftInfo.cliff_back_center,socData.cliffLiftInfo.cliff_back_right,socData.cliffLiftInfo.cliff_front_right);
}
void UdpCommunication::resSocDockReceiverData()
{
    resGetRecvIRStatus((bool)socData.dockingInfo.receiver.front_left,(bool)socData.dockingInfo.receiver.front_right,
    (bool)socData.dockingInfo.receiver.side_left,(bool)socData.dockingInfo.receiver.side_right);
}
#endif

void UdpCommunication::resSocTofData()
{
    #if USE_NEW_PROTOCOL > 0
    resGetTofStatusV2(socData.tofInfo.left_bottom.status,socData.tofInfo.left_bottom.data,socData.tofInfo.right_bottom.status,socData.tofInfo.right_bottom.data,socData.tofInfo.top.status,socData.tofInfo.top.distance);
    #else
    resGetTofStatus(0, 0.0,0,0.0,0,0.0);
    #endif
    
}

void UdpCommunication::resSocErrorList()
{
    if (!error_list.empty())
    {
        for(const auto& error : error_list)
        {
            RCLCPP_INFO(this->get_logger(), "Received error: rank = %d, error_code = %s", error.rank, error.errorCode.c_str());
        }
                    
        resGetErrorList(error_list);
        clearErrorList();
    } 
}

void UdpCommunication::resSocAllSensor()
{
    #if USE_NEW_PROTOCOL > 0
    int camera_status = 0, lineLaser_status = 0;
    resGetAllStatusV2(socData.lidarInfo.front_distance,socData.lidarInfo.rear_distance,
    socData.motorInfo.left.rpm,socData.motorInfo.right.rpm,socData.motorInfo.left.current,socData.motorInfo.right.current,socData.motorInfo.left.type,socData.motorInfo.right.type,
    camera_status,socData.cameraInfo.data,lineLaser_status,socData.lineLaserInfo.data,
    socData.tofInfo.left_bottom.status,socData.tofInfo.right_bottom.status,socData.tofInfo.top.status,socData.tofInfo.left_bottom.data,socData.tofInfo.right_bottom.data,socData.tofInfo.top.distance,
    socData.cliffLiftInfo.cliff_front_center,socData.cliffLiftInfo.cliff_front_left,socData.cliffLiftInfo.cliff_back_left,socData.cliffLiftInfo.cliff_back_center,socData.cliffLiftInfo.cliff_back_right,socData.cliffLiftInfo.cliff_front_right,
    (bool)socData.dockingInfo.receiver.front_left,(bool)socData.dockingInfo.receiver.front_right,(bool)socData.dockingInfo.receiver.side_left,(bool)socData.dockingInfo.receiver.side_right,
    socData.battInfo.cell_voltage1,socData.battInfo.cell_voltage2,socData.battInfo.cell_voltage3,socData.battInfo.cell_voltage4,socData.battInfo.cell_voltage5,
    socData.battInfo.total_capacity,socData.battInfo.remaining_capacity,socData.battInfo.manufacturer,
    socData.battInfo.percent,socData.battInfo.voltage,socData.battInfo.current,socData.battInfo.temperature1,socData.battInfo.temperature2,
    socData.battInfo.design_capacity,socData.battInfo.number_of_cycles);
    #else
    resGetAllStatus(socData.lidarInfo.front_distance,socData.lidarInfo.rear_distance,
    0.0,0.0,0.0,0.0,0,0,0,0,0,0,0.0,0,0.0,0,0.0,
    (bool)socData.cliffLiftInfo.cliff_front_center,(bool)socData.cliffLiftInfo.cliff_front_left,(bool)socData.cliffLiftInfo.cliff_back_left,(bool)socData.cliffLiftInfo.cliff_back_center,(bool)socData.cliffLiftInfo.cliff_back_right,
    (bool)socData.cliffLiftInfo.cliff_front_right,(bool)socData.dockingInfo.receiver.front_left,(bool)socData.dockingInfo.receiver.front_right,(bool)socData.dockingInfo.receiver.side_left,(bool)socData.dockingInfo.receiver.side_right,
    0,0,0,0,
    socData.battInfo.voltage,socData.battInfo.current,0.0,0.0,socData.battInfo.remaining_capacity,socData.battInfo.design_capacity,socData.battInfo.percent,0.0,0.0,0.0,0.0,0.0,0.0,0.0,"0.0");
    #endif
}

void UdpCommunication::autoSocDataSender()
{
    #if USE_NEW_PROTOCOL > 0
    double nowSec = this->now().seconds();
    if(socData.version.bSet){
        if(nowSec-socData.version.timestamp <= 10){
            resSocSoftWareVision();
        }else{
            socData.version.bSet = false;
        }
    }
    resSocMovingInfo();
    //resSocAllSensor();
    resSocRobotState();
    resSocRobotStatus();
    #endif
}

void UdpCommunication::reqSocDataChecker()
{
    if (reqGetPosition()){
        resSocData(REQUEST_DATA_WHAT::ROBOT_POSITION);
    }if (reqGetMapData()){   
        resSocData(REQUEST_DATA_WHAT::MAP);
    }if (reqGetSoftwareVersion()){
        resSocData(REQUEST_DATA_WHAT::SW_VERSION);
    }if (reqGetMapStatus()){
        resSocData(REQUEST_DATA_WHAT::MAP_STATUS);
    }if (reqGetTargetPosition()){
        resSocData(REQUEST_DATA_WHAT::TARGET_POSITION);
    }if (reqGetBatteryStatus()){
        resSocData(REQUEST_DATA_WHAT::BATTERY_STATUS);
    }if (reqGetDockingStatus()){
        resSocData(REQUEST_DATA_WHAT::DOCKING_STATUS);
    }if (reqGetRobotInfo()){
        resSocData(REQUEST_DATA_WHAT::ROBOT_INFO);
    }if (reqGetModifiedMapData()){
        resSocData(REQUEST_DATA_WHAT::MODIFIED_MAP);   
    }if (reqGetAllMovingInfo()){
        resSocData(REQUEST_DATA_WHAT::MOVING_INFO);
    }if(reqGetRobotSpeed()){
        resSocData(REQUEST_DATA_WHAT::ROBOT_VELOCITY);
    }if (reqGetMotorStatus()){
        resSocData(REQUEST_DATA_WHAT::MOTOR_STATUS);
    }if (reqGetCameraStatus()){
        resSocData(REQUEST_DATA_WHAT::CAMERA_STATUS);
    }if (reqGetLineLaserStatus()){
        resSocData(REQUEST_DATA_WHAT::LINE_LASER_STATUS);
    }if (reqGetTofStatus()){
        resSocData(REQUEST_DATA_WHAT::TOF_STATUS);
    }if (reqGetErrorList()){
        resSocData(REQUEST_DATA_WHAT::ERROR_LIST);
    }if (reqGetLidarData()){
        resSocData(REQUEST_DATA_WHAT::LIDAR_DATA);
    }if (reqGetAllStatus()){
        resSocData(REQUEST_DATA_WHAT::ALL_STATUS);
    }
    // if(reqGetTargetPositionCalculate(reqCalculTarget)){
    //     responseData(REQUEST_DATA_WHAT::TARGET_POSITION_CALCUL);
    // }
}

void UdpCommunication::reqSocOptionChecker()
{
    if (reqSetModifiedMapDataB(modifiedMap)){
        RCLCPP_INFO(this->get_logger(), "Request map-modify");
        std::string filename = "/home/airbot/test1_map.pgm";
        savePGMFile(modifiedMap, filename);
        std::cout << "PGM file saved as " << filename << std::endl;
        resSetModifiedMapData(true);
    }
    ByPassOne_t parsedData;
    if (reqSetByPassOneData(parsedData)){
        //RCLCPP_INFO(this->get_logger(), "Request by Pass Data UID : %s , Version : %s, Modified : %s ",parsedData.uid,parsedData.version,parsedData.modified ); 
        publishBlockWall(parsedData);
        publishBlockArea(parsedData);
        resSetByPassOneData(true);
    }

    if (reqSetExcelSteps(settings9)){
        RCLCPP_INFO(this->get_logger(), "Request bExcelSteps"); 
        resSetExcelSteps(true);
    }

    // if(reqGetTargetPositionCalculate(Settings11)){
    //     double Time = 0, Distance = 0;
    //     resGetTargetPositionCalculate( Time,Distance, Settings11.x,Settings11.y,Settings11.theta);
    // }
}


void UdpCommunication::generateSocCommand()
{  
    auto command_msg = std_msgs::msg::UInt8();
    if (reqSetMapping(reqMapping)){
        resSetMapping(true);
        if(reqMapping.set == 1){
            command_msg.data = int(REQUEST_SOC_CMD::START_MANUAL_MAPPING);
            req_soc_cmd_pub_->publish(command_msg);
            RCLCPP_INFO(this->get_logger(), "Request Start-Manual-Mapping");   
        }else if(reqMapping.set == 2){
            command_msg.data = int(REQUEST_SOC_CMD::START_AUTO_MAPPING);
            req_soc_cmd_pub_->publish(command_msg);
            RCLCPP_INFO(this->get_logger(), "Request Start-Auto-Mapping");   
        }else if(reqMapping.set == 4){
            if( robotState == ROBOT_STATE::MANUAL_MAPPING ){
                command_msg.data = int(REQUEST_SOC_CMD::STOP_MANUAL_MAPPING);
            } else if( robotState == ROBOT_STATE::AUTO_MAPPING){
                command_msg.data = int(REQUEST_SOC_CMD::STOP_AUTO_MAPPING);
            }
            req_soc_cmd_pub_->publish(command_msg);
            RCLCPP_INFO(this->get_logger(), "Request Mapping-Stop");
        }
    }else if (reqSetDriving(reqNavigation)){
        resSetDriving(true);
        if (reqNavigation.set == 1){
            RCLCPP_INFO(this->get_logger(), "Request Start-Navigation");
            command_msg.data = int(REQUEST_SOC_CMD::START_NAVIGATION);      
            req_soc_cmd_pub_->publish(command_msg);
        }else if (reqNavigation.set == 4){
            RCLCPP_INFO(this->get_logger(), "Request Stop-Navigation");
            command_msg.data = int(REQUEST_SOC_CMD::STOP_NAVIGATION);
            req_soc_cmd_pub_->publish(command_msg);
        }else if (reqNavigation.set == 2){
            if(robotState == ROBOT_STATE::NAVIGATION){
                command_msg.data = int(REQUEST_SOC_CMD::PAUSE_NAVIGATION);
                req_soc_cmd_pub_->publish(command_msg);
            }else{
                RCLCPP_INFO(this->get_logger(), "Robot is not Running Navigation, can`t Pause Navigation");
            }
        }else if (reqNavigation.set == 3){
            if(robotState == ROBOT_STATE::NAVIGATION){
                command_msg.data = int(REQUEST_SOC_CMD::RESUME_NAVIGATION);
                req_soc_cmd_pub_->publish(command_msg);
            }else{
                RCLCPP_INFO(this->get_logger(), "Robot is not Running Navigation, can`t Pause Navigation");
            }
        }
    }else if (reqSetTargetPosition(reqTargetPosition)){
            if(robotState != ROBOT_STATE::NAVIGATION){
                command_msg.data = int(REQUEST_SOC_CMD::START_NAVIGATION);
                req_soc_cmd_pub_->publish(command_msg);
            }
            resSetTargetPosition(true);
            setSocTargetPosition(reqTargetPosition.x,reqTargetPosition.y,reqTargetPosition.theta);
            //publishSensorOnOff(true);
            publishTargetPosition(reqTargetPosition.x,reqTargetPosition.y,reqTargetPosition.theta);
    }
    else if(reqSetReturnToChargingStation()){
        resSetReturnToChargingStation(true);
        //publishSensorOnOff(true);
        command_msg.data = int(REQUEST_SOC_CMD::START_RETURN_CHARGER);
        req_soc_cmd_pub_->publish(command_msg); 
        RCLCPP_INFO(this->get_logger(), "Request ReturnToCharger");
    }else if (reqSetDockingState(reqDocking)){
        resSetDockingState(true);
        if(reqDocking.dock==true){
            command_msg.data = int(REQUEST_SOC_CMD::START_DOCKING);
            req_soc_cmd_pub_->publish(command_msg);
            RCLCPP_INFO(this->get_logger(), "Request start-docking");
        }else{
            if(robotState == ROBOT_STATE::DOCKING ){
                command_msg.data = int(REQUEST_SOC_CMD::STOP_DOCKING);
                req_soc_cmd_pub_->publish(command_msg);
                RCLCPP_INFO(this->get_logger(), "Request stop-docking");
            }else{
                RCLCPP_INFO(this->get_logger(), "robot is not docking can`t stop");
            }
        }
    }
    #if 0
    else if(reqFactoryNavigation()){
        resFactoryNavigation(true);
        RCLCPP_INFO(this->get_logger(), "Request Start-FactoryNavigation");
        command_msg.data = int(REQUEST_SOC_CMD::START_FACTORY_NAVIGATION);
        req_soc_cmd_pub_->publish(command_msg);
    }
    #endif
}

void UdpCommunication::directRequestCommand()
{
    if (reqSetSoftwareReset()){
        RCLCPP_INFO(this->get_logger(), "Request Reboot");
        resSetSoftwareReset(true); // 재부팅 시에는 응답 먼저 날리고 재부팅 실행 - 실행 후 보고 하면 보고를 할 수 없음
        systemRebootCommand();
    }

    if (reqSetEmergencyStop()){
        RCLCPP_INFO(this->get_logger(), "Request EmergencyStop");
        publishEmergencyCommand();
        resSetEmergencyStop(true);
    }

    if (reqSetStartCharging()){
        RCLCPP_INFO(this->get_logger(), "Request StartCharging");
        publishChargingCommand(true);
        //publishSensorOnOff(true);
        resSetStartCharging(true);
    }else if (reqSetStopCharging()){
        RCLCPP_INFO(this->get_logger(), "Request StopCharging");
        //publishSensorOnOff(false);
        publishChargingCommand(false);
        resSetStopCharging(true);
    }

    if(reqSetMotorManual_VW(reqManualVWMove)){
        if(rotation.progress){
            stopMonitorRotate();
        }
        publishVelocityCommand(reqManualVWMove.mS,reqManualVWMove.radS);
        resSetMotorManual_VW(true);
    }else if (reqSetRotation(reqRotation)){
        resSetRotation(true);
        RCLCPP_INFO(this->get_logger(), "Request Rotation type : %d, targetAngle : %f",reqRotation.type,reqRotation.radian);
        if(!startRotation(reqRotation.type,reqRotation.radian)){
        }
        RCLCPP_INFO(this->get_logger(), "Request Rotation type : %d, targetAngle : %f",reqRotation.type,reqRotation.radian);   
    }

    if(reqSetStartLidar()){
        RCLCPP_INFO(this->get_logger(), "Request start-Lidar");
        publishLidarOnOff(true);
    }else if(reqSetStopLidar()){
        RCLCPP_INFO(this->get_logger(), "Request stop-Lidar");
        publishLidarOnOff(false);
    }
#if 0
    bool set;
    if(reqSetSensorInspectionMode(set)){
        resSetSensorInspectionMode(true);
        publishSensorOnOff(set);
    }

    if(reqSetBatterySleepMode()){
        resSetBatterySleepMode(true);
        publishBatterySleep();
    }
#endif
}


void UdpCommunication::reqSocActionChecker()
{  
    generateSocCommand();
    directRequestCommand();
}

void UdpCommunication::publishTargetPosition(double x,double y,double theta)
{
    robot_custom_msgs::msg::Position msg;
    msg.x = x;
    msg.y = y;
    msg.theta = theta;
    move_target_pub_->publish(msg);
}

void UdpCommunication::publishLidarOnOff(bool on_off)
{
    std_msgs::msg::Bool lidar_cmd;
    if(on_off){
        lidar_cmd.data = true;
        lidar_cmd_pub_->publish(lidar_cmd);
        resSetStartLidar(true);
    }else{
        lidar_cmd.data = false;
        lidar_cmd_pub_->publish(lidar_cmd);
    }
    resSetStopLidar(true);
}

void UdpCommunication::publishTofOnOff(bool set)
{
    std_msgs::msg::Bool tof_cmd;
    if(set){
        tof_cmd.data = true;
    }else{
        tof_cmd.data = false;
    }
    tof_cmd_pub_->publish(tof_cmd);
}

void UdpCommunication::publishSensorOnOff(bool set)
{
    std_msgs::msg::Bool lidar_cmd;
    std_msgs::msg::Bool tof_cmd;
    std_msgs::msg::Bool linelaser_cmd;
    if(set){
        lidar_cmd.data = true;
        tof_cmd.data = true;
        linelaser_cmd.data = true;
    }else{
        lidar_cmd.data = false;
        tof_cmd.data = false;
        linelaser_cmd.data = false;
    }
    lidar_cmd_pub_->publish(lidar_cmd);
    tof_cmd_pub_->publish(tof_cmd);
    linelaser_cmd_pub_->publish(linelaser_cmd);
    bSensorOn = set;
}

void UdpCommunication::publishBatterySleep()
{
    std_msgs::msg::Bool battery_sleep_cmd;
    battery_sleep_cmd.data = true;
    batterySleep_cmd_pub_->publish(battery_sleep_cmd);

}

void UdpCommunication::logCommandAndData(int command, const std::vector<uint8_t>& data, int option) 
{
    if(option == 1)
        RCLCPP_INFO(this->get_logger(), "=============== API_RecvJigData Start ================ ");
    else
        RCLCPP_INFO(this->get_logger(), "=============== API_SendJigData Start ================ ");
    
    RCLCPP_INFO(this->get_logger(), "send jig Command: %d", command);

    if(data.size() > 0){
        if(command == 5) {
            printTof(data);
        }
        else if(command == 6 || command == 7){
            printLidar(command,data);
        } 
        else 
        {
            int i = 0;
            for (const auto& byte : data) {
                RCLCPP_INFO(this->get_logger(), "Data[%d]: 0x%02X", i, byte);
                i++;
            }
        }
    }
    
    if(option == 1)
        RCLCPP_INFO(this->get_logger(), "=============== API_RecvJigData END ================ ");
    else 
        RCLCPP_INFO(this->get_logger(), "=============== API_SendJigData END ================ ");
}

void UdpCommunication::printLidar(int command, const std::vector<uint8_t>& data)
{
    if (data.size() < 4) {
        RCLCPP_ERROR(this->get_logger(), "Error: Insufficient data size for lidar distances.");
        return;
    }

    // Extract left obstacle distance (mm)
    uint16_t lidar_left_obstacle_distance = (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);

    // Extract right obstacle distance (mm)
    uint16_t lidar_right_obstacle_distance = (static_cast<uint16_t>(data[2]) << 8) | static_cast<uint16_t>(data[3]);

    // Log the distances
    if(command == 6)
        RCLCPP_INFO(this->get_logger(), "front Lidar Distances (mm):");
    else if (command == 7)
        RCLCPP_INFO(this->get_logger(), "rear Lidar Distances (mm):");
    
    RCLCPP_INFO(this->get_logger(), "  Left Obstacle Distance: %d mm", lidar_left_obstacle_distance);
    RCLCPP_INFO(this->get_logger(), "  Right Obstacle Distance: %d mm", lidar_right_obstacle_distance);
}

void UdpCommunication::printTof(const std::vector<uint8_t>& data)
{
    if( !data.empty() && data.size() % 2 != 0)
    {
        RCLCPP_WARN(this->get_logger(), "Data size is odd, last byte will be ignored.");
        return ;
    }
    
    for (size_t i = 0; i < data.size(); i++) {
        RCLCPP_INFO(this->get_logger(), "Raw Data[%zu]: 0x%02X (%d)", i, data[i], data[i]);
    }

    for (size_t i = 0; i < data.size(); i += 2) {
        uint16_t tof_distance = (static_cast<uint16_t>(data[i]) << 8) | static_cast<uint16_t>(data[i+1]);
        RCLCPP_INFO(this->get_logger(), " tof Data[%zu]: %d", i/2 , tof_distance);
    }
}

void UdpCommunication::procSocCommunication()
{
    reqSocDataChecker();
    reqSocOptionChecker();
    reqSocActionChecker();
}

void UdpCommunication::procAmrJigCommunication()
{
    std::vector<uint8_t> packet;
    
    //AMR JIG
    for(int i = 1; i < 9; i++)
    {
        if(API_RecvJigData(i, packet)){
            logCommandAndData(i, packet, 1);
            jigProcessor(i,packet);
            break;
        }
    }
}

void UdpCommunication::procApJigCommunication()
{
    std::vector<uint8_t> packet;

    //AP JIG
    for(int i = 241; i < 245; i++ )
    {
        if(API_RecvJigData(i, packet)){
            // logCommandAndData(i, packet, 1); 로그 자리
            apJigProcessor(i,packet);
            break;
        }
    }
}

UDP_COMMUNICATION UdpCommunication::checkUdpCommunicationMode()
{
    std::vector<uint8_t> packet;
    
    if(API_RecvJigData(static_cast<int>(JIG_HEADER::AMR_MODE), packet)){
        logCommandAndData(static_cast<int>(JIG_HEADER::AMR_MODE), packet, 1);
        if(!packet.empty() && packet[0] == 1){
            resPonseJigCommand(JIG_HEADER::AMR_MODE,packet);
            resetSocAutoSendTimer();
            udpMode = UDP_COMMUNICATION::AMR_JIG_MODE;
        }
        else if(!packet.empty() && packet[0] == 0){
            udpMode = UDP_COMMUNICATION::NORMAL;
        }
    }
    else if (API_RecvJigData(JIG_AP_HEADER::AP_JIG, packet)){
        // logCommandAndData(static_cast<int>(JIG_AP_HEADER::AP_JIG), packet, 1);
        if(!packet.empty() && packet[0] == 1){
            resPonseJigCommand(JIG_AP_HEADER::AP_JIG,packet);
            resetSocAutoSendTimer();
            udpMode = UDP_COMMUNICATION::AP_JIG_MODE;
        }
        else if(!packet.empty() && packet[0] == 0){
            udpMode = UDP_COMMUNICATION::NORMAL;
        }
    }

    return udpMode;  
}

void UdpCommunication::udp_callback()
{
    UDP_COMMUNICATION udpCommucation = checkUdpCommunicationMode();

    switch (udpCommucation)
    {
    case UDP_COMMUNICATION::AMR_JIG_MODE: procAmrJigCommunication();        break;
    case UDP_COMMUNICATION::AP_JIG_MODE: procApJigCommunication();          break;
    case UDP_COMMUNICATION::NORMAL: procSocCommunication();                 break;
    default:
        RCLCPP_INFO(this->get_logger(), "UDP_COMMUNICATION value error");
        break;
    }
}

bool UdpCommunication::isValidRobotPose()
{
    return true;
}
//**********************************************************************************///
// END OF UDP Callback
pose UdpCommunication::getRobotPose(){
    return socData.robotPosition;
}

bool UdpCommunication::isValidTargetPose()
{
    return true;
}

pose UdpCommunication::getTargetPose(){
    return socData.targetPosition;
}

MapInfo UdpCommunication::getMapInfo(){
    return socData.mapInfo;
}

void UdpCommunication::clearErrorList(){
    error_list.clear();
}

void UdpCommunication::systemRebootCommand()
{
    RCLCPP_INFO(this->get_logger(), "start-Reboot");
    int reboot_cmd = std::system("sudo /home/airbot/reboot_script.sh");          
}

void UdpCommunication::publishBlockArea(const ByPassOne_t& parsedData)
{
    robot_custom_msgs::msg::BlockAreaList msgs;
    for (const auto& block_area : parsedData.block_area)
    {
        robot_custom_msgs::msg::BlockArea message;  // Define the message here inside the function

        message.id = block_area.id;  // Set the id for the block area

        // Populate robot_path
        for (const auto& position : block_area.robot_path)
        {
            robot_custom_msgs::msg::Position pos_msg;
            pos_msg.x = position.x;
            pos_msg.y = position.y;
            message.robot_path.push_back(pos_msg);
        }

        // Populate image_path
        for (const auto& position : block_area.image_path)
        {
            robot_custom_msgs::msg::Position pos_msg;
            pos_msg.x = position.x;
            pos_msg.y = position.y;
            message.image_path.push_back(pos_msg);
        }
        msgs.block_area_list.push_back(message);
    }
    block_area_pub_->publish(msgs);  // Publish the BlockArea message
    if(parsedData.block_area.empty()){
        RCLCPP_INFO(this->get_logger(), "publishBlockArea empty");
    }
    // else{
    //     RCLCPP_INFO(this->get_logger(), "publishBlockArea size : %u" , parsedData.block_area.size());
    // }
}

void UdpCommunication::publishBlockWall(const ByPassOne_t& parsedData)
{
    robot_custom_msgs::msg::BlockAreaList msgs;
    for (const auto& block_wall : parsedData.block_wall)
    {
        // Assuming block_wall is a BlockArea-like structure or a list of positions
        robot_custom_msgs::msg::BlockArea wall_list;  // Create BlockArea message for walls

        wall_list.id = block_wall.id;
        // Populate robot_path with block_wall positions (if they're positions)
        for (const auto& wall_position : block_wall.robot_path)
        {
            robot_custom_msgs::msg::Position wall_data;
            wall_data.x = wall_position.x;
            wall_data.y = wall_position.y;
            wall_list.robot_path.push_back(wall_data);
        }

        // Populate image_path
        for (const auto& position : block_wall.image_path)
        {
            robot_custom_msgs::msg::Position wall_data;
            wall_data.x = position.x;
            wall_data.y = position.y;
            wall_list.image_path.push_back(wall_data);
        }

        msgs.block_area_list.push_back(wall_list);
    }
    block_wall_pub_->publish(msgs);  // Publish the BlockArea message
    
    if(parsedData.block_wall.empty()){
        RCLCPP_INFO(this->get_logger(), "publishBlockWall empty");
    }
    // else{
    //     RCLCPP_INFO(this->get_logger(), "publishBlockWall size : %u" , parsedData.block_wall.size());
    // }
}

void UdpCommunication::publishEmergencyCommand()
{
    std_msgs::msg::Bool e_stop_cmd;
    if(socData.motorInfo.mode != MOTOR_BREAK_MODE){
        e_stop_cmd.data = true; //e stop is called
        e_stop_pub->publish(e_stop_cmd);
    }else{
        RCLCPP_INFO(this->get_logger(), "Motor is Aready Break!");
    }
}

void UdpCommunication::publishChargingCommand(bool start)
{
    std_msgs::msg::UInt8 charge_cmd;
    if(start){
        charge_cmd.data = AUTO_CHARGE_HIGH;
        charge_pub->publish(charge_cmd);
        RCLCPP_INFO(this->get_logger(), "Request-Startcharging");
    }else{
        charge_cmd.data = CHARGE_STOP;
        charge_pub->publish(charge_cmd);
        RCLCPP_INFO(this->get_logger(), "Request- Stopcharging");
    }
}

void UdpCommunication::publishVelocityCommand(double v, double w)
{
    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_msg.linear.x = v;
    cmd_msg.angular.z = w;
    manual_move_pub_->publish(cmd_msg);
    RCLCPP_INFO(this->get_logger(), "publishVelocityCommand V : %f, W : %f ", v,w);
}

void UdpCommunication::savePGMFile(const ModifiedMapDataB_t &settings, const std::string &filename)
{
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open())
    {
        throw std::runtime_error("Unable to open file for writing");
    }
    std::cout << "settings : " << settings.width << "/" << settings.height << std::endl;
    file << "P5\n"
        << settings.width << " " << settings.height << "\n255\n";

    file.write(reinterpret_cast<const char *>(settings.data.data()), settings.data.size());

    file.close();
}

bool UdpCommunication::readPGM(const std::string& pgm_filename, int& width_map, int& height_map, 
            std::vector<uint8_t>& mapData, double& origin_x, double& origin_y) {

    // Reading the PGM file
    std::ifstream file(pgm_filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << pgm_filename << std::endl;
        return false;
    }

    std::string line;
    std::getline(file, line);
    if (line != "P5") {
        std::cerr << "Error: Unsupported file format (only P5 is supported)" << std::endl;
        return false;
    }

    // Skip comments
    while (std::getline(file, line) && line[0] == '#') {
        // Skip comment lines
    }

    std::istringstream sizeStream(line);
    sizeStream >> width_map >> height_map;

    std::getline(file, line);
    std::istringstream maxValueStream(line);
    int maxValue;
    maxValueStream >> maxValue;

    // Check for valid maxValue
    if (maxValue != 255) {
        std::cerr << "Error: Only 8-bit PGM files are supported" << std::endl;
        return false;
    }

    // Read pixel data
    mapData.resize(width_map * height_map);
    file.read(reinterpret_cast<char*>(mapData.data()), mapData.size());

    // for (int y = 0; y < height_map / 2; ++y) {
    //     for (int x = 0; x < width_map; ++x) {
    //         std::swap(data_value[y * width_map + x], data_value[(height_map - 1 - y) * width_map + x]);
    //     }
    // }

    if (!file) {
        std::cerr << "Error: Reading pixel data failed" << std::endl;
        return false;
    }

    file.close(); // Close the PGM file after reading

    // Derive the YAML file name from the PGM file name
    std::string yaml_filename = pgm_filename.substr(0, pgm_filename.find_last_of('.')) + ".yaml";
    // std::string yaml_filename = "test1_map.yaml";
    std::ifstream file1(yaml_filename);
    if (!file1.is_open()) {
        std::cerr << "Error: Could not open file " << yaml_filename << std::endl;
        return false;
    }
    // Reading the YAML file
    std::string line1;
    while (std::getline(file1, line1)) {
        if (line1.find("origin:") != std::string::npos) {
            std::istringstream ss(line1.substr(line1.find('[') + 1, line1.find(']') - line1.find('[') - 1));
            char comma;
            ss >> origin_x >> comma >> origin_y;
            if (ss.fail() || comma != ',') {
                std::cerr << "Error: Failed to parse 'origin' in YAML file" << std::endl;
                return false;
            }
            return true;
        }
    }

    return true;
}

std::vector<uint8_t> UdpCommunication::mapDataTypeConvert(std::vector<int8_t> src, int height, int width, double origin_x, double origin_y)
{
    // 동적 메모리 할당
    //unsigned char* mapData = new unsigned char[height * width];
    std::vector<uint8_t> mapData(height * width);

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int index = y*width + x;//width
            int value = src[index];  // uint8_t를 int로 캐스팅
            int transformed_x = x;  //
            int transformed_y = (height-1) - y;  //
            int transformed_index = transformed_y * width + transformed_x;

            unsigned char pixel_value;
            if (value == -1)  // OccupancyGrid에서 -1은 보통 'unknown'을 의미
            {
                pixel_value = 127; // Gray for unknown
            }
            else if (value >= 0 && value <= 100)
            {
                pixel_value = static_cast<unsigned char>(255 - (value * 255) / 100); // Scale 0-100 to 0-255                
            }
            else
            {
                pixel_value = 128; // Default gray for unexpected values
            }

            mapData[transformed_index] = pixel_value;                           
        }
    }
    
    return mapData;  // 동적으로 할당된 메모리를 반환
}

double UdpCommunication::quaternion_to_euler(const geometry_msgs::msg::Quaternion& quat)
{
    tf2::Quaternion q;
    tf2::fromMsg(quat, q);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw; // Return yaw as theta
}

void UdpCommunication::resetSocAutoSendTimer()
{
    RCLCPP_INFO(this->get_logger(), "resetSocAutoSendTimer");
    socAutoSendTimer_.reset();
}

void UdpCommunication::resetVersionTimer()
{
    std::lock_guard<std::mutex> lock(versionTimer_mutex_);
    RCLCPP_INFO(this->get_logger(), "resetVersionTimer");
    versionTimer_.reset();
}

void UdpCommunication::reset_Rotationtimer()
{
    rotation.progress = false;
    if(rotation_target_timer_){
        RCLCPP_INFO(this->get_logger(), "reset_Rotationtimer");
        rotation_target_timer_.reset();
        RCLCPP_INFO(this->get_logger(), "reset_Rotationtimer - end ");
    }else{
        RCLCPP_INFO(this->get_logger(), "odom_target_timeris allready reset ");
    }
}

uint8_t UdpCommunication::getLowerBits(uint8_t byte) {
    return (byte & 0x0F);
}

// 상위 4비트를 반환하는 함수
uint8_t UdpCommunication::getUpperBits(uint8_t byte) {
    return ((byte >> 4) & 0x0F);
}

//응답
void UdpCommunication::resPonseJigCommand(int command, const std::vector<uint8_t>& packet)
{
    logCommandAndData(command, packet, 2);
    API_SendJigData(command, packet);
}

//데이터 파싱
void UdpCommunication::resPonseJigData(JIG_DATA_KEY key)
{
    logCommandAndData(static_cast<int>(key),getJigData(key), 2);
    API_SendJigData(static_cast<int>(key),getJigData(key));
}
/**
 * @brief 지그 응답 패킷에 0값을 넣어주는 함수
 * 
 *       지그 패킷 갯수가 항상 일정해야하기 때문에 0값을 생성하여 맞춰줌
 *  
 * @param header 
 * @param packet 
 * @return std::vector<uint8_t> 
 */
std::vector<uint8_t> UdpCommunication::makeResponseJigCommandPacket(JIG_HEADER header, std::vector<uint8_t> packet)
{
    std::vector<uint8_t> ret;

    if(header == JIG_HEADER::WHEEL_MOTOR)
        ret.resize(11);
    else if(header == JIG_HEADER::BATTERY)
        ret.resize(10);
    else if(header == JIG_HEADER::IMU_CALIBRATION)
        ret.resize(3);
    else {
        RCLCPP_INFO(this->get_logger(), "jig Command Response Protocol Error!!");
        ret.resize(1);
    }

    if (packet.size() > ret.size()) {
        RCLCPP_WARN(this->get_logger(), "Packet size (%zu) exceeds response size (%zu), truncating.", packet.size(), ret.size());
    }

    if (!packet.empty()) {
        for (size_t i = 0; i < ret.size(); i++) {
            if (i < packet.size()) {
                ret[i] = packet[i];
            } else {
                ret[i] = 0;
            }
        }
    }

    return ret;
}

void UdpCommunication::jigCheckWheelMotor(const std::vector<uint8_t>& packet)
{
    if(packet[0] == 0x00){
        resPonseJigData(JIG_DATA_KEY::WHEEL_MOTOR);
    }
    else if (packet[0] == 0x01){
        std::vector<uint8_t> newPacket = makeResponseJigCommandPacket(JIG_HEADER::WHEEL_MOTOR,packet);
        resPonseJigCommand(JIG_HEADER::WHEEL_MOTOR,newPacket);
        robot_custom_msgs::msg::RpmControl rpmCtrlMsg;
        rpmCtrlMsg.motor_enable = packet[0];
        rpmCtrlMsg.left_motor_rpm_msb = packet[1];
        rpmCtrlMsg.left_motor_rpm_lsb = packet[2];
        rpmCtrlMsg.right_motor_rpm_msb = packet[3];
        rpmCtrlMsg.right_motor_rpm_lsb = packet[4];
        jig_request_motor_pub_->publish(rpmCtrlMsg);
    }else{
        RCLCPP_INFO(this->get_logger(), "jigCheckWheelMotor Protocol Error!!");
    }
}

void UdpCommunication::jigCheckBattery(const std::vector<uint8_t>& packet)
{
    if(packet[0] == 0x00){
        resPonseJigData(JIG_DATA_KEY::BATTERY); 
    }else if(packet[0] & 0x03){
        std::vector<uint8_t> newPacket = makeResponseJigCommandPacket(JIG_HEADER::BATTERY,packet);
        resPonseJigCommand(JIG_HEADER::BATTERY,newPacket);
        std_msgs::msg::UInt8 chargingModeMsg;
        chargingModeMsg.data = packet[0];
        jig_request_battery_pub_->publish(chargingModeMsg);
        //robot_custom_msgs::msg::RpmControl rpmCtrlMsg;
    }else{
        RCLCPP_INFO(this->get_logger(), "jigCheckBattery Protocol Error!!");
    }  
}

void UdpCommunication::jigCheckCliffLift()
{
    resPonseJigData(JIG_DATA_KEY::CLIFF_LIFT);
}

void UdpCommunication::jigCheckDockReceiver()
{
    resPonseJigData(JIG_DATA_KEY::DOCK_RECEIVER);
}

void UdpCommunication::jigCheckTofSensor(const std::vector<uint8_t>& packet)
{
    resPonseJigData(JIG_DATA_KEY::TOF);
} 

void UdpCommunication::jigCheckFrontLidar()
{
    resPonseJigData(JIG_DATA_KEY::FRONT_LIDAR);
}

void UdpCommunication::jigCheckRearLidar()
{
    resPonseJigData(JIG_DATA_KEY::REAR_LIDAR);
}

void UdpCommunication::jigCheckImuCalibration(const std::vector<uint8_t>& packet)
{
    if(packet[0] == 0x00){
        resPonseJigData(JIG_DATA_KEY::IMU_CALIBRATION);
    }else if(packet[0] & 0x03){
        std::vector<uint8_t> newPacket = makeResponseJigCommandPacket(JIG_HEADER::IMU_CALIBRATION,packet);
        resPonseJigCommand(JIG_HEADER::IMU_CALIBRATION,newPacket);
        std_msgs::msg::UInt8 imu_cal_msg;
        imu_cal_msg.data = packet[0];
        jig_request_imu_pub_->publish(imu_cal_msg);
    }else{
        RCLCPP_INFO(this->get_logger(), "jigCheckImuCalibration Protocol Error!!");
    }
}

void UdpCommunication::jigProcessor(int header, const std::vector<uint8_t>& packet)
{
    switch (header)
    { 
    case JIG_HEADER::WHEEL_MOTOR:
        jigCheckWheelMotor(packet);
        break;
    case JIG_HEADER::BATTERY:
        jigCheckBattery(packet);
        break;
    case JIG_HEADER::CLIFF_LIFT:
        jigCheckCliffLift();
        break;
    case JIG_HEADER::DOCK_RECEIVER:
        jigCheckDockReceiver();
        break;
    case JIG_HEADER::TOF:
        jigCheckTofSensor(packet);
        break;
    case JIG_HEADER::FRONT_LIDAR:
        jigCheckFrontLidar();
        break;
    case JIG_HEADER::REAR_LIDAR:
        jigCheckRearLidar();
        break;
    case JIG_HEADER::IMU_CALIBRATION:
        jigCheckImuCalibration(packet);
        break;                              
    default:
        break;
    }
}

std::vector<uint8_t> UdpCommunication::jigDataConvertWheelMotor(const robot_custom_msgs::msg::MotorStatus::SharedPtr msg)
{
    std::vector<uint8_t> ret(11);
    
    switch(msg->motor_mode){
        case 0x00: ret[0] = MOTOR_DEFAULT_MODE; break;
        case 0x01: ret[0] = MOTOR_RPM_MODE; break;
        default: 
            ret[0] = MOTOR_MODE_ERROR; 
            RCLCPP_WARN(this->get_logger(), "Invalid motor mode received: 0x%02X", msg->motor_mode);
            break;
    } 

    ret[1] = static_cast<uint8_t>((static_cast<int>(msg->left_motor_rpm) >> 8) & 0xFF);
    ret[2] = static_cast<uint8_t>(static_cast<int>(msg->left_motor_rpm) & 0xFF);
    ret[3] = static_cast<uint8_t>((static_cast<int>(msg->right_motor_rpm) >> 8) & 0xFF);
    ret[4] = static_cast<uint8_t>(static_cast<int>(msg->right_motor_rpm) & 0xFF);
    ret[5] = static_cast<uint8_t>((static_cast<int>(msg->left_motor_current) >> 8) & 0xFF);
    ret[6] = static_cast<uint8_t>(static_cast<int>(msg->left_motor_current) & 0xFF);
    ret[7] = static_cast<uint8_t>((static_cast<int>(msg->right_motor_current) >> 8) & 0xFF);
    ret[8] = static_cast<uint8_t>(static_cast<int>(msg->right_motor_current) & 0xFF);
    ret[9] = static_cast<uint8_t>(msg->left_motor_temperature);
    ret[10] = static_cast<uint8_t>(msg->right_motor_temperature);
    return ret;
}

std::vector<uint8_t> UdpCommunication::jigDataConvertBattery(const robot_custom_msgs::msg::BatteryStatus::SharedPtr msg)
{
    std::vector<uint8_t> ret(10);

    switch (msg->charge_status) {
        case 0x00: ret[0] = BATTERY_DEFAULT_MODE; break;
        case 0x02: ret[0] = BATTERY_HIGH_SPEED_MODE; break;
        case 0x03: ret[0] = BATTERY_LOW_SPEED_MODE; break;
        default:   
            ret[0] = BATTERY_MODE_ERROR; 
            RCLCPP_WARN(this->get_logger(), "Invalid battery mode received: 0x%02X", msg->charge_status);
            break;
    }
    
    ret[1] = static_cast<uint8_t>((msg->number_of_cycles >> 8) & 0xFF);
    ret[2] = static_cast<uint8_t>(msg->number_of_cycles & 0xFF);
    ret[3] = static_cast<uint8_t>((static_cast<int>(msg->battery_voltage*100) >> 8) & 0xFF);
    ret[4] = static_cast<uint8_t>(static_cast<int>(msg->battery_voltage*100) & 0xFF);
    ret[5] = static_cast<uint8_t>(msg->battery_temperature1 & 0xFF);
    ret[6] = static_cast<uint8_t>(msg->battery_temperature2 & 0xFF);
    ret[7] = static_cast<uint8_t>(msg->battery_percent & 0xFF);
    ret[8] = static_cast<uint8_t>((static_cast<int>(msg->battery_current*10) >> 8) & 0xFF);
    ret[9] = static_cast<uint8_t>(static_cast<int>(msg->battery_current*10) & 0xFF);

    return ret;
}

std::vector<uint8_t> UdpCommunication::jigDataConvertTof(const robot_custom_msgs::msg::TofData::SharedPtr msg)
{
    std::vector<uint8_t> ret(64);
     for(int i = 0; i < 32; i += 2){
        int idx_right = i;
        int idx_left = i+32;
        uint16_t temp_right = static_cast<uint16_t>(msg->bot_right[i/2]*1000);
        uint16_t temp_left = static_cast<uint16_t>(msg->bot_left[i/2]*1000);
        ret[idx_right]  = static_cast<uint8_t>((temp_right >> 8) & 0xFF);
        ret[idx_right+1] = static_cast<uint8_t>(temp_right & 0xFF);
        ret[idx_left] = static_cast<uint8_t>((temp_left >> 8) & 0xFF);  
        ret[idx_left+1] = static_cast<uint8_t>(temp_left & 0xFF);
    }
    return ret;
}

std::vector<uint8_t> UdpCommunication::jigDataConvertLidar(int left, int right)
{
    std::vector<uint8_t> ret;
    if(left != std::numeric_limits<int>::max() && right != std::numeric_limits<int>::max()){
        ret.push_back(static_cast<uint8_t>((left >> 8) & 0xFF));
        ret.push_back(static_cast<uint8_t>(left & 0xFF));
        ret.push_back(static_cast<uint8_t>((right >> 8) & 0xFF));
        ret.push_back(static_cast<uint8_t>(right & 0xFF));
    }
    
    return ret;
}

std::vector<uint8_t> UdpCommunication::jigDataConvertImuCalibration(const robot_custom_msgs::msg::ImuCalibration::SharedPtr msg)
{
    std::vector<uint8_t> ret;
    ret.push_back(msg->calibration_status);
    ret.push_back(static_cast<uint8_t>((msg->yaw >> 8) & 0xFF)); // high byte
    ret.push_back(static_cast<uint8_t>(msg->yaw & 0xFF)); // low byte

    return ret;
}

void UdpCommunication::splitLidarScan(const sensor_msgs::msg::LaserScan::SharedPtr msg, std::vector<int>& left, std::vector<int>& right)
{
    float current_angle = msg->angle_min;
    for (int i = 0; i < msg->ranges.size(); i++, current_angle += msg->angle_increment) {
        float range = msg->ranges[i];

        if (!std::isfinite(range) || range <= 0.001) {
            continue;
        }

        int dist = static_cast<int>(range * 1000);
        if (abs(dist) <= 1) {
            continue;
        }

        // 좌측 범위 확인
        if (current_angle >= LEFT_MIN_ANGLE && current_angle <= LEFT_MAX_ANGLE) {
            left.push_back(dist);
        }
        // 우측 범위 확인 (else if 사용하여 불필요한 비교 줄임)
        else if (current_angle >= RIGHT_MIN_ANGLE && current_angle <= RIGHT_MAX_ANGLE) {
            right.push_back(dist);
        }
    }
}

int UdpCommunication::getMinDistanceFromLidarSensor(const std::vector<int>& vecDistance)
{
    int ret = std::numeric_limits<int>::max();
    if(!vecDistance.empty()){
        ret = *std::min_element(vecDistance.begin(), vecDistance.end());
    }
    return ret;
}

void UdpCommunication::apJigProcessor(int header, const std::vector<uint8_t>& packet)
{
    switch (header)
    {
    case JIG_AP_HEADER::AP_JIG_RAM_MEMORY:
        apJigCheckRam();
        break;
    case JIG_AP_HEADER::AP_JIG_DISK_MEMORY:
        apJigCheckEmmc();
        break;
    case JIG_AP_HEADER::AP_JIG_FRONT_LIDAR:
        apJigCheckFrontLiDAR();
        break;
    case JIG_AP_HEADER::AP_JIG_BACK_LIDAR:
        apJigCheckBackLiDAR();
        break;
    
    default:
        RCLCPP_INFO(this->get_logger(), "Unknown command");
        break;
    }
}
//------------------------------------------------------
//everybot@everybot-edu:~/ros2_ws$ cat /proc/meminfo 
//MemTotal:       15992188 kB
//MemFree:         6279864 kB
//MemAvailable:   11137800 kB
// 구조체 데이터를 전송하는 함수
void UdpCommunication::sendRamData(const std::vector<uint8_t>& ram_info)
{
    try {
        // UDP 값전송
        resPonseJigCommand(JIG_AP_HEADER::AP_JIG_RAM_MEMORY,ram_info);
        // serial_.write(reinterpret_cast<const uint8_t*>(&ram_info.size[0]), ram_info.size.size());
        
        // 전송된 데이터 값을 정수 및 헥사로 출력
        for (size_t i = 0; i < ram_info.size(); i += 4) {
            uint32_t value = *reinterpret_cast<const uint32_t*>(&ram_info[i]);
            value = __builtin_bswap32(value);
            RCLCPP_INFO(this->get_logger(), "RAM value (int): %u, Hex: 0x%08X", value, value);
        }

        RCLCPP_INFO(this->get_logger(), "RAM data sent successfully.");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send RAM data: %s", e.what());
    }
}

void UdpCommunication::apJigCheckRam()
{
    RCLCPP_INFO(this->get_logger(), "Starting RAM check...");

    std::ifstream meminfo("/proc/meminfo");
    if (!meminfo.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Could not open /proc/meminfo");
        return;
    }

    std::string line;
    std::vector<uint8_t> ram_info(12);  // RAM 정보를 담을 벡터 초기화

    // /proc/meminfo 파일에서 필요한 정보를 파싱
    while (std::getline(meminfo, line)) {
        if (line.find("MemTotal:") == 0) {
            uint32_t mem_total;
            sscanf(line.c_str(), "MemTotal: %u kB", &mem_total);  // MemTotal 값을 추출
            mem_total = __builtin_bswap32(mem_total); // 바이트 순서 변경
            memcpy(ram_info.data(), &mem_total, sizeof(mem_total)); // MemTotal 값을 구조체에 저장
        } else if (line.find("MemFree:") == 0) {
            uint32_t mem_free;
            sscanf(line.c_str(), "MemFree: %u kB", &mem_free);    // MemFree 값을 추출
            mem_free = __builtin_bswap32(mem_free);
            memcpy(ram_info.data() + 4, &mem_free, sizeof(mem_free)); // MemFree 값을 구조체에 저장
        } else if (line.find("MemAvailable:") == 0) {
            uint32_t mem_available;
            sscanf(line.c_str(), "MemAvailable: %u kB", &mem_available);  // MemAvailable 값을 추출
            mem_available = __builtin_bswap32(mem_available);
            memcpy(ram_info.data() + 8, &mem_available, sizeof(mem_available)); // MemAvailable 값을 구조체에 저장
        }
    }

    meminfo.close();

    sendRamData(ram_info);
}

//구조체 데이터를 전송하는 함수
void UdpCommunication::sendEmmcData(const std::vector<uint8_t>& emmc_info)
{
    try {
        // UDP 값전송
        resPonseJigCommand(JIG_AP_HEADER::AP_JIG_DISK_MEMORY, emmc_info);
        // Transmit eMMC data (4 bytes)
        // serial_.write(reinterpret_cast<const uint8_t*>(emmc_info), emmc_info.size());
        
        // Log sent eMMC data
        for (size_t i = 0; i < emmc_info.size(); ++i) {
            uint32_t value = static_cast<uint32_t>(emmc_info[i]);
            RCLCPP_INFO(this->get_logger(), "eMMC value (int): %u, Hex: 0x%02X", value, value);
        }

        RCLCPP_INFO(this->get_logger(), "eMMC data sent successfully.");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send eMMC data: %s", e.what());
    }
}
//------------------------------------------------------
//EMMC Information:
//mmcblk0    14.9G disk
//구조체 데이터를 전송하는 함수
void UdpCommunication::apJigCheckEmmc()
{
    RCLCPP_INFO(rclcpp::get_logger("EMMC"), "Starting EMMC check...");

    std::array<char, 128> buffer;
    std::string result;

    // EMMC 정보 확인을 위한 'lsblk' 명령어 실행 (mmcblk로 시작하는 디바이스 필터링)
    FILE* pipe = popen("lsblk -o NAME,SIZE --noheadings | grep -i '^mmcblk0'", "r");
    if (!pipe) {
        RCLCPP_ERROR(rclcpp::get_logger("EMMC"), "Failed to run EMMC check command");
        return;
    }

    // 명령어 출력 결과를 읽어서 result에 저장
    if (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
        result = buffer.data();
    }

    // 명령어 종료
    int returnCode = pclose(pipe);
    if (returnCode != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("EMMC"), "Command exited with code %d", returnCode);
        return;
    }

    // 디바이스가 발견되지 않은 경우 예외 처리
    if (result.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("EMMC"), "No EMMC device found.");
        return;
    }

    // 결과 로그 출력
    RCLCPP_INFO(rclcpp::get_logger("EMMC"), "EMMC check completed: \n%s", result.c_str());

    std::vector<uint8_t> emmc_info(4);  // EMMC 정보를 담을 벡터 초기화

    // 결과에서 사이즈 파싱
    std::istringstream iss(result);
    std::string device_name, size_str;
    if (iss >> device_name >> size_str) {
        // 사이즈 문자열에서 단위 문자("G", "M" 등) 제거
        char unit = size_str.back();
        size_str.pop_back();
        uint32_t size_in_kb = std::stoi(size_str);

        // 단위에 따라 KB로 변환
        if (unit == 'G') {
            size_in_kb *= 1024 * 1024;  // GB to KB
        } else if (unit == 'M') {
            size_in_kb *= 1024;  // MB to KB
        }

        // EMMC 크기를 구조체에 저장
        emmc_info[0] = static_cast<uint8_t>(size_in_kb >> 24);  // 가장 상위 바이트
        emmc_info[1] = static_cast<uint8_t>(size_in_kb >> 16);  // 두 번째 바이트
        emmc_info[2] = static_cast<uint8_t>(size_in_kb >> 8);   // 세 번째 바이트
        emmc_info[3] = static_cast<uint8_t>(size_in_kb);        // 가장 하위 바이트

        // EMMC 크기를 로그로 출력
        RCLCPP_INFO(rclcpp::get_logger("EMMC"), "EMMC Size: %u KB (size[0]: 0x%02X, size[1]: 0x%02X, size[2]: 0x%02X, size[3]: 0x%02X)",
                size_in_kb, emmc_info[0], emmc_info[1], emmc_info[2], emmc_info[3]);
    }

    // 구조체 데이터를 바이너리로 전송
    sendEmmcData(emmc_info);
}
//----------------------------------------------
// LiDAR 데이터 전송 함수
void UdpCommunication::sendLidarData(int header, const std::vector<uint8_t>& lidar_info)
{
    try {
        // LiDAR 데이터 전송 (2바이트)
        // UDP 값전송
        resPonseJigCommand(header, lidar_info);
        
        // 전송된 데이터 값을 정수 및 헥사로 출력
        for (size_t i = 0; i < lidar_info.size(); ++i) { // 벡터의 크기 사용
            int distance_value = static_cast<int>(lidar_info[i]);
            RCLCPP_INFO(this->get_logger(), "Distance value (int): %d, Hex: 0x%02X", distance_value, distance_value);
        }

        RCLCPP_INFO(this->get_logger(), "LiDAR data sent successfully.");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send LiDAR data: %s", e.what());
    }
}

void UdpCommunication::apJigCheckFrontLiDAR()
{
    RCLCPP_INFO(this->get_logger(), "Starting back check...");

    if (apJigFrontLaserData && !apJigFrontLaserData->ranges.empty()) {
#if AP_JIG_CHECK_ON_AMR
        float first_range = apJigFrontLaserData->ranges[100];
#else
        float first_range = apJigFrontLaserData->ranges[0];
#endif        
        uint16_t distance_cm = static_cast<uint16_t>(std::round(first_range * 100.0f));
        
        std::vector<uint8_t> lidar_info(2);
        lidar_info[0] = static_cast<uint8_t>((distance_cm >> 8) & 0xFF); // 상위 바이트
        lidar_info[1] = static_cast<uint8_t>(distance_cm & 0xFF); // 하위 바이트
        
        RCLCPP_INFO(this->get_logger(), "Front LiDAR: %.2f meters", first_range);
        sendLidarData(JIG_AP_HEADER::AP_JIG_FRONT_LIDAR, lidar_info);
    } else {
        RCLCPP_WARN(this->get_logger(), "No front LiDAR data available.");
    }
}

void UdpCommunication::apJigCheckBackLiDAR()
{
    RCLCPP_INFO(this->get_logger(), "Starting back check...");

    if (apJigBackLaserData && !apJigBackLaserData->ranges.empty()) {
#if AP_JIG_CHECK_ON_AMR
        float first_range = apJigBackLaserData->ranges[100];
#else
        float first_range = apJigBackLaserData->ranges[0];
#endif        
        uint16_t distance_cm = static_cast<uint16_t>(std::round(first_range * 100.0f));

        // LiDARInfo 구조체의 인스턴스를 생성합니다.
        std::vector<uint8_t> lidar_info(2);
        // 거리 값을 2바이트로 분할하여 저장합니다.
        lidar_info[0] = static_cast<uint8_t>((distance_cm >> 8) & 0xFF); // 상위 바이트
        lidar_info[1] = static_cast<uint8_t>(distance_cm & 0xFF);       // 하위 바이트

        // 거리 정보를 로그로 출력합니다.
        RCLCPP_INFO(this->get_logger(), "Back LiDAR: %.2f meters", first_range);
        // LiDAR 데이터 전송
        sendLidarData(JIG_AP_HEADER::AP_JIG_BACK_LIDAR, lidar_info);
    } else {
        RCLCPP_WARN(this->get_logger(), "No back LiDAR data available.");
    }
}

bool UdpCommunication::startRotation(int type, double targetAngle)
{
    bool ret = false;
    pose robotPose = getRobotPose();
    //movingState = NAVI_STATE::IDLE;
    rotation.type = type;
    if(type == 0){
        rotation.target = robotPose.theta+targetAngle;
        ret = true;
    }else if(type==1){
        rotation.target = targetAngle;
        ret = true;
    }else if(type==2 || type==3){
        rotation.accAngle = 0.0;
        rotation.preTheta = normalize_angle(robotPose.theta);
        ret = true;
    }else{
        RCLCPP_INFO(this->get_logger(), "Request Rotate type error : %d", type);
    }

    if(ret){
        startRotateMonitor();
    }

    return ret;
}

//Timer to checck Odom value to do One point rotation
void UdpCommunication::startRotateMonitor()
{
    if(!rotation.progress){
        rotation_target_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&UdpCommunication::progressRotation, this));
        rotation.progress = true;
    }else{
        RCLCPP_INFO(this->get_logger(), "already Rotation progressing");
    }
    
}

void UdpCommunication::stopMonitorRotate()
{
    reset_Rotationtimer();
}

void UdpCommunication::progressRotation()
{
    bool completeRotation = false;
    // If navigation is enabled, use amcl_pose
    pose current = getRobotPose();
    if(rotation.type == 0 || rotation.type == 1){
        progressRotationTarget(rotation.target,current.theta);
    }else{
        int direction = (rotation.type == 3) ? 1 : -1;
        progressRotation360(direction,current.theta);
    }
}

int UdpCommunication::checkRotationDirection(double diff)
{
    return (diff > 0) ? 1 : -1;
}

bool UdpCommunication::checkRotationTarget(double diff)
{
    bool ret = false;
    if (std::fabs(diff) < 0.175){
        RCLCPP_INFO(this->get_logger(), "Target angle reached"); 
        ret = true;
    }
    return ret;
}

void UdpCommunication::progressRotationTarget(double target, double current)
{
    double v = 0, w = 0;
    double nomalize_target = normalize_angle(target);
    double nomalize_current = normalize_angle(current);
    double angle_diff = normalize_angle(nomalize_target-nomalize_current);

    if(checkRotationTarget(angle_diff)){
        publishVelocityCommand(v,w);
        //movingState = NAVI_STATE::COMPLETE_ROTATION;//this->robot_status = 6;
        stopMonitorRotate();
        return;
    }
    
    int direction = checkRotationDirection(angle_diff);
    w = direction * 0.3; // Adjust speed if needed
    publishVelocityCommand(v,w);
}

void UdpCommunication::progressRotation360(int direction, double current)
{
    double v = 0, w = 0;
    double nomalizeTheta = normalize_angle(current);
    double delta_theta = normalize_angle(nomalizeTheta - rotation.preTheta);

    rotation.accAngle += std::fabs(delta_theta);
    rotation.preTheta = nomalizeTheta;

    if (rotation.accAngle >= 2*M_PI){
        RCLCPP_INFO(this->get_logger(), "Completed 360-degree rotation");
        publishVelocityCommand(v,w);
        // movingState = NAVI_STATE::COMPLETE_ROTATION;//this->robot_status = 6;
        stopMonitorRotate();
        return;
    }

    w =  direction * 0.3;
    publishVelocityCommand(v,w);
}

double UdpCommunication::normalize_angle(double angle) {
    // Normalize angle to the range [-π, π]
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}