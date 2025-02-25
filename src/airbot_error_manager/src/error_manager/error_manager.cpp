#include "error_manager/error_manager.h"

using namespace std::chrono_literals;

ErrorManager::ErrorManager()
    : Node("airbot_error_manager")
{
    // E-error
    e_left_motor_stuck_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/todo1",
        100,std::bind(&ErrorManager::leftMotorStuckErrorCallback, this, std::placeholders::_1)
    );
    e_right_motor_stuck_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/todo2",
        100,std::bind(&ErrorManager::rightMotorStuckErrorCallback, this, std::placeholders::_1)
    );
    e_scan_dirty_front_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/scan_dirty_front",
        100,std::bind(&ErrorManager::scanDirtyFrontErrorCallback, this, std::placeholders::_1)
    );
    e_scan_dirty_back_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/scan_dirty_back",
        100,std::bind(&ErrorManager::scanDirtyBackErrorCallback, this, std::placeholders::_1)
    );
    e_docking_station_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/todo3",
        100,std::bind(&ErrorManager::dockingStationErrorCallback, this, std::placeholders::_1)
    );
    e_charging_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/todo4",
        100,std::bind(&ErrorManager::chargingErrorCallback, this, std::placeholders::_1)
    );
    
    // F-error
    f_battery_charging_overcurrent_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/todo5",
        100,std::bind(&ErrorManager::batChargingOverCurrentErrorCallback, this, std::placeholders::_1)
    );
    f_battery_discharging_overcurrent_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/todo6",
        100,std::bind(&ErrorManager::batDischargingOverCurrentErrorCallback, this, std::placeholders::_1)
    );
    f_top_tof_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/todo7",
        100,std::bind(&ErrorManager::topTofErrorCallback, this, std::placeholders::_1)
    );
    f_camera_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/todo8",
        100,std::bind(&ErrorManager::cameraErrorCallback, this, std::placeholders::_1)
    );
    f_right_motor_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/todo9",
        100,std::bind(&ErrorManager::rightMotorErrorCallback, this, std::placeholders::_1)
    );
    f_left_motor_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/todo10",
        100,std::bind(&ErrorManager::leftMotorErrorCallback, this, std::placeholders::_1)
    );
    f_scan_front_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/scan_error_front",
        100,std::bind(&ErrorManager::scanFrontErrorCallback, this, std::placeholders::_1)
    );
    f_scan_back_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/scan_error_back",
        100,std::bind(&ErrorManager::scanBackErrorCallback, this, std::placeholders::_1)
    );
    f_battery_overheat_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/todo11",
        100,std::bind(&ErrorManager::batOverHeatErrorCallback, this, std::placeholders::_1)
    );
    f_bot_tof_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/todo12",
        100,std::bind(&ErrorManager::botTofErrorCallback, this, std::placeholders::_1)
    );

    // S-error
    s_unreachable_goal_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/todo13",
        100,std::bind(&ErrorManager::unReachableGoalErrorCallback, this, std::placeholders::_1)
    );
    s_change_temporary_goal_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/todo14",
        100,std::bind(&ErrorManager::changeTempGoalErrorCallback, this, std::placeholders::_1)
    );
    s_fall_down_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/todo15",
        100,std::bind(&ErrorManager::fallDownErrorCallback, this, std::placeholders::_1)
    );
    s_unable_to_docking_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/todo16",
        100,std::bind(&ErrorManager::unableToDockingErrorCallback, this, std::placeholders::_1)
    );
    s_board_overheat_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/todo17",
        100,std::bind(&ErrorManager::boardOverHeatErrorCallback, this, std::placeholders::_1)
    );
    s_station_overheat_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/todo18",
        100,std::bind(&ErrorManager::stationOverHeatErrorCallback, this, std::placeholders::_1)
    );
    
    error_list_pub_ = this->create_publisher<robot_custom_msgs::msg::ErrorListArray>("/error_list", 10);

    pub_timer_ = this->create_wall_timer(
        200ms,
        std::bind(&ErrorManager::publishErrorList, this));
}

ErrorManager::~ErrorManager()
{
}

void ErrorManager::publishErrorList()
{
    robot_custom_msgs::msg::ErrorListArray error_msg_array;

    for (auto it = error_list_.begin(); it != error_list_.end();) {
        auto& error = **it;
        error.count++;
        
        robot_custom_msgs::msg::ErrorList error_msg;
        error_msg.count = error.count;
        error_msg.rank = error.rank;
        error_msg.error_code = error.error_code;

        if (error.count > CLEAR_CNT) {
            it = error_list_.erase(it);
        } else {
            error_msg_array.data_array.push_back(error_msg);
            ++it;
        }
    }

    if (!error_list_.empty()) {
        error_list_pub_->publish(error_msg_array);
        printErrorList();
    }
}

/**
 * @brief Left Motor 구속 및 제어 불가 시 에러
 * @note E04
 * @param msg
 */
void ErrorManager::leftMotorStuckErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 0;
    std::string errorCode = "E04";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

/**
 * @brief Right Motor 구속 및 제어 불가 시 에러
 * @note E04-1
 * @param msg
 */
void ErrorManager::rightMotorStuckErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 0;
    std::string errorCode = "E04-1";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

/**
 * @brief Front Lidar 이물질로 인해 기능을 상실한 경우 에러
 * @note E05
 * @param msg
 */
void ErrorManager::scanDirtyFrontErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 14;
    std::string errorCode = "E05";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

/**
 * @brief Back Lidar 이물질로 인해 기능을 상실한 경우 에러
 * @note E06
 * @param msg
 */
void ErrorManager::scanDirtyBackErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 15;
    std::string errorCode = "E06";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

/**
 * @brief Docking Station을 확인하지 못하는 경우 에러
 * @note E07
 * @param msg
 */
void ErrorManager::dockingStationErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 0;
    std::string errorCode = "E07";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

/**
 * @brief Docking Station에서 충전이 불가능한 경우 에러
 * @note E08
 * @param msg
 */
void ErrorManager::chargingErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 0;
    std::string errorCode = "E08";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

/**
 * @brief (충전시) 배터리 과전류에 의한 화재 및 파손 방지 에러
 * @note F01
 * @param msg
 */
void ErrorManager::batChargingOverCurrentErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 0;
    std::string errorCode = "F01";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

/**
 * @brief (방전시) 배터리 과전류에 의한 화재 및 파손 방지 에러
 * @note F01-1
 * @param msg
 */
void ErrorManager::batDischargingOverCurrentErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 0;
    std::string errorCode = "F01-1";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

/**
 * @brief 1D ToF 센서와 통신이 안될 경우 에러
 * @note F07
 * @param msg
 */
void ErrorManager::topTofErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 0;
    std::string errorCode = "F07";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

/**
 * @brief Camera 센서와 통신이 안될 경우 에러
 * @note F09-2
 * @param msg
 */
void ErrorManager::cameraErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 0;
    std::string errorCode = "F09-2";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

/**
 * @brief Right 구동모터 통신 불가 에러
 * @note F11
 * @param msg
 */
void ErrorManager::rightMotorErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 0;
    std::string errorCode = "F11";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

/**
 * @brief Left 구동모터 통신 불가 에러
 * @note F12
 * @param msg
 */
void ErrorManager::leftMotorErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 0;
    std::string errorCode = "F12";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

/**
 * @brief Front Lidar 센서와 통신이 안될 경우 에러
 * @note F13
 * @param msg
 */
void ErrorManager::scanFrontErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 12;
    std::string errorCode = "F13";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }

}

/**
 * @brief Back Lidar 센서와 통신이 안될 경우 에러
 * @note F14
 * @param msg
 */
void ErrorManager::scanBackErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 13;
    std::string errorCode = "F14";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

/**
 * @brief 배터리 과열에 의한 화재 및 파손 방지 에러
 * @note F15
 * @param msg
 */
void ErrorManager::batOverHeatErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 0;
    std::string errorCode = "F15";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

/**
 * @brief Multi TOF 센서와 통신이 안될 경우 에러
 * @note F17
 * @param msg
 */
void ErrorManager::botTofErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 0;
    std::string errorCode = "F17";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

/**
 * @brief 현재 구역에서 다른 구역으로 이동이 불가한 경우 목적지 도달 불가 에러
 * @note S05
 * @param msg
 */
void ErrorManager::unReachableGoalErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 0;
    std::string errorCode = "S05";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

/**
 * @brief 사용자가 설정한 목적지가 아닌 대체 목적지에 도달하였을 경우 목적지 임시 변경 에러
 * @note S05-2
 * @param msg
 */
void ErrorManager::changeTempGoalErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 0;
    std::string errorCode = "S05-2";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

/**
 * @brief 제품의 전도 발생 시 기기 넘어짐 에러
 * @note S07
 * @param msg
 */
void ErrorManager::fallDownErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 0;
    std::string errorCode = "S07";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

/**
 * @brief 도킹을 못하는 경우 에러
 * @note S08
 * @param msg
 */
void ErrorManager::unableToDockingErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 0;
    std::string errorCode = "S08";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

/**
 * @brief SoC 보드의 과열에 의한 화재 및 파손 방지 시스템 보드 과열 에러
 * @note S10-2
 * @param msg
 */
void ErrorManager::boardOverHeatErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 0;
    std::string errorCode = "S10-2";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

/**
 * @brief 충전부 과열에 의한 화재 및 파손 방지 에러
 * @note S11
 * @param msg
 */
void ErrorManager::stationOverHeatErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 0;
    std::string errorCode = "S11";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

void ErrorManager::updateErrorLists(int rank, std::string code)
{
    auto it = std::find_if(error_list_.begin(), error_list_.end(),
        [rank](const std::shared_ptr<robot_custom_msgs::msg::ErrorList>& error) {
        return error->rank == rank;
    });

    if (it == error_list_.end()) {
        addError(rank, code);
    }
}

void ErrorManager::addError(int rank, const std::string &error_code)
{
    auto it = std::find_if(error_list_.begin(), error_list_.end(),
        [rank](const std::shared_ptr<robot_custom_msgs::msg::ErrorList>& error) {
            return error->rank == rank;
        });

    if (it != error_list_.end()) {
        return;
    }

    if (error_list_.size() >= ERROR_LIST_SIZE) {
        error_list_.erase(error_list_.begin());
    }

    auto new_error = std::make_shared<robot_custom_msgs::msg::ErrorList>();
    new_error->count = 0;
    new_error->rank = rank;
    new_error->error_code = error_code;
    error_list_.push_back(new_error);
}

void ErrorManager::printErrorList(){
    RCLCPP_INFO(this->get_logger(),
        "================= Num of error: %d =================",
        static_cast<int>(error_list_.size()));
    for (const auto& error : error_list_) {
        RCLCPP_INFO(this->get_logger(), 
            "Error - Rank: %d, Code: %s, Count: %d", 
            error->rank, error->error_code.c_str(), error->count);
    }
}