#include "ament_index_cpp/get_package_share_directory.hpp"
#include "error_manager/error_manager_node.hpp"

ErrorManagerNode::ErrorManagerNode()
    : Node("airbot_error_manager"), error_list_{}
{
    std::string error_list{};
    this->declare_parameter("error_list", "error_list.yaml");
    this->get_parameter("error_list", error_list);

    int log_level{};
    this->declare_parameter("log_level", 10);
    this->get_parameter("log_level", log_level);
    rcutils_ret_t set_logger = rcutils_logging_set_logger_level(this->get_logger().get_name(), static_cast<RCUTILS_LOG_SEVERITY>(log_level));
    if (set_logger != RCUTILS_RET_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set logger level (error code: %d)", set_logger);
    }

    int publish_rate{};
    this->declare_parameter("publish_rate_ms", 1000);
    this->get_parameter("publish_rate_ms", publish_rate);
    auto publish_rate_ms = std::chrono::milliseconds(publish_rate);

    // [250407] hyjoe : 에러 퍼블리싱 횟수 1회
    this->declare_parameter("error_publish_cnt", 1);
    this->get_parameter("error_publish_cnt", pub_cnt);

    this->declare_parameter("error_list_size", 1000);
    this->get_parameter("error_list_size", error_list_size);

    try {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("airbot_error_manager");
        // RCLCPP_INFO(this->get_logger(), "package_share_directory: %s, file: %s", package_share_directory.c_str(), error_list.c_str());
        std::string full_path = package_share_directory + "/" + "config" + "/" + error_list;
        this->config = YAML::LoadFile(full_path)["airbot_error_manager"]["error_list"];
        // RCLCPP_INFO(this->get_logger(), "YAML Config: \n%s", YAML::Dump(this->config).c_str());
    } catch (const std::exception& e) { //ament_index_cpp::get_package_share_directory()가 제대로 작동하지 않을 경우
        RCLCPP_ERROR(this->get_logger(), "Failed to load config file: %s", e.what());
        std::string fallback_path = "/home/airbot/airbot_ws/install/airbot_error_manager/share/airbot_error_manager/config/" + error_list;
        this->config = YAML::LoadFile(fallback_path)["airbot_error_manager"]["error_list"];
    }
    rclcpp::QoS qos_state_profile = rclcpp::QoS(5)
                            .reliable()
                            .durability_volatile();

    // 에러 발생 후 에러 리스트에서 관리하지 않을 에러 등록(SOC에 에러해제 보내지 않음.)
    erase_after_pub_error_codes_.insert("S05"); // 이동불가 에러
    erase_after_pub_error_codes_.insert("S08"); // 도킹불가 에러 (from state_manager)
    erase_after_pub_error_codes_.insert("E04"); // 좌측구동모터 구속 에러 (from mcu)
    erase_after_pub_error_codes_.insert("E04-1"); // 우측구동모터 구속 에러 (from mcu)
    erase_after_pub_error_codes_.insert("E05"); // 전면 라이다 센서 오염 (from lidar)
    erase_after_pub_error_codes_.insert("E06"); // 후면 라이다 센서 오염 (from mcu)
    erase_after_pub_error_codes_.insert("E07"); // 스테이션 위치 확인 불가 (from state_manager)
    erase_after_pub_error_codes_.insert("E08"); // 충전불가 (from mcu)

    error_list_pub_ = this->create_publisher<robot_custom_msgs::msg::ErrorListArray>("/error_list", 10);
    robot_state_sub_ = this->create_subscription<robot_custom_msgs::msg::RobotState>(
        "/state_datas", qos_state_profile, std::bind(&ErrorManagerNode::robotStateCallback, this, std::placeholders::_1)
    );

    pub_timer_ = this->create_wall_timer(
        publish_rate_ms,
        std::bind(&ErrorManagerNode::publishErrorList, this));
    RCLCPP_INFO(this->get_logger(), "node initialized");
}

ErrorManagerNode::~ErrorManagerNode()
{
    RCLCPP_INFO(this->get_logger(), "node terminated");
}

void ErrorManagerNode::init()
{
    this->initSubscribers(this->config);
    RCLCPP_INFO(this->get_logger(), "airbot_error_manager node init finished!");
}

void ErrorManagerNode::initSubscribers(const YAML::Node& config)
{
    auto pnode = std::static_pointer_cast<ErrorManagerNode>(this->shared_from_this());
    if (!pnode) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get shared pointer from this");
        return;
    }

    if (!config) {
        RCLCPP_ERROR(this->get_logger(), "YAML does not contain 'error_list' key");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "================== ERROR MANAGER SUBSCRIBERS ==================");
    for (const auto& error_category : config) {
        std::string category_key = error_category.first.as<std::string>();
        YAML::Node category_value = error_category.second;
        for (const auto& error : category_value) {
            std::string sub_topic = error.second["sub_topic"].as<std::string>();
            std::string error_code = error.second["error_code"].as<std::string>();

            error_code_descriptions_[error_code] = error.second["description"].as<std::string>();

            RCLCPP_INFO(this->get_logger(), "Sub topic: %s (error_code: %s)",
                        sub_topic.c_str(), error_code.c_str());

            auto callback = [this, pnode, error_code](std_msgs::msg::Bool::SharedPtr msg)
            { errorCallback(error_code, msg); };
            auto subs = this->create_subscription<std_msgs::msg::Bool>(sub_topic, 10, callback);
            this->subscribers_.push_back(subs);
        }
    }
    RCLCPP_INFO(this->get_logger(), "===============================================================");
}

void ErrorManagerNode::errorCallback(const std::string& error_code, std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        updateErrorLists(error_code);
    } else {
        releaseErrorLists(error_code);
    }
}

void ErrorManagerNode::robotStateCallback(const robot_custom_msgs::msg::RobotState::SharedPtr msg)
{
    robot_state_ = msg->state;
}

void ErrorManagerNode::publishErrorList()
{
    robot_custom_msgs::msg::ErrorListArray error_msg_array;
    rclcpp::Time now_time = rclcpp::Clock(RCL_STEADY_TIME).now();
    builtin_interfaces::msg::Time msg_time;
    msg_time.sec = static_cast<int32_t>(now_time.seconds());
    msg_time.nanosec = now_time.nanoseconds() % 1000000000;
    error_msg_array.timestamp = msg_time;

    static bool isReleasedAllError = false;

    cur_robot_state = robot_state_;

    if (!isReleasedAllError && prev_robot_state == 9 && cur_robot_state != 9) { //ROBOT_STATE::ERROR
        RCLCPP_INFO(this->get_logger(), "Realesed all error in error_lists_ [Num of errors: %zu]", error_list_.size());
        allErrorReleased();
        isReleasedAllError = true;
        return;
    }

    LidarErrorRelease();

    if (cur_robot_state == 9) {
        isReleasedAllError = false;
    }

    for (auto it = error_list_.begin(); it != error_list_.end();) {
        auto& error = *it;

        if (!error.should_publish) { // 퍼블리시 필요 없고, 해제 상태인데 카운트도 넘었으면 삭제
            if (!error.error.error_occurred && error.error.count > pub_cnt) {
                it = error_list_.erase(it);
            } else {
                ++it;
            }
            continue;
        }

        // 상태 변화 체크: 발생 → 해제로 바뀐 경우, 새로운 에러가 추가된 경우
        if ((!error.error.error_occurred && error.error.count == 1)
            || (error.error.count == 1 && error.error.error_occurred)) {
            printErrorList();
        }

        robot_custom_msgs::msg::ErrorList error_msg = error.error;
        error.error.count++;
        error.has_occurred_before = true;
        error_msg_array.data_array.push_back(error_msg);

        if (error.error.count > pub_cnt) { // pub_cnt 만큼 퍼블리싱 했으면 발행 정지.
            if (erase_after_pub_error_codes_.count(error.error.error_code)) { // 에러 발생 후 에러 리스트에서 관리하지 않을 에러 제거
                it = error_list_.erase(it);
                continue;
            } else {
                error.should_publish = false;
            }
        }
        ++it;
    }

    if (!error_msg_array.data_array.empty()) {
        error_list_pub_->publish(error_msg_array);
    }

    prev_robot_state = robot_state_;
}

void ErrorManagerNode::updateErrorLists(std::string code)
{
    auto it = std::find_if(error_list_.begin(), error_list_.end(),
        [&code](const tErrorList& error) {
            return error.error.error_code == code;
    });

    if (it == error_list_.end()) {
        addError(code);
    } else {
        if (!it->error.error_occurred) { // 이전에 해제된 에러가 다시 발생
            it->error.error_occurred = true;
            it->error.count = 1;
            it->should_publish = true;
        }
    }
}

void ErrorManagerNode::addError(const std::string &error_code)
{
    auto it = std::find_if(error_list_.begin(), error_list_.end(),
        [&error_code](const tErrorList& error) {
            return error.error.error_code == error_code;
    });

    if (it != error_list_.end()) { // error_list_에 이미 동일한 error가 존재하면 무시.
        return;
    }

    if (error_list_.size() > static_cast<size_t>(error_list_size)) { // error_list_ 최대 크기 제한
        error_list_.erase(error_list_.begin());
    }

    tErrorList new_error;
    new_error.error.count = 1;
    new_error.error.error_code = error_code;

    new_error.error.error_occurred = true;
    new_error.has_occurred_before = false; // 최초 발생
    new_error.should_publish = true;

    error_list_.push_back(new_error);
}

void ErrorManagerNode::releaseErrorLists(std::string code)
{
    if (error_list_.empty()) { // error_list_ 비어있으면 실행 x
        return;
    }

    auto it = std::find_if(error_list_.begin(), error_list_.end(),
    [&code](const tErrorList& error) {
        return error.error.error_code == code;
    });

    if (it != error_list_.end()) { // 해제는 발생했던 에러에만 적용
        if (it->has_occurred_before && it->error.error_occurred) { // 과거에 발생한 적이 있을 때만 해제 메시지 퍼블리싱 활성화
            it->error.error_occurred = false;
            it->should_publish = true;
            it->error.count = 1; // 카운트 초기화 (다시 pub_cnt 만큼 보내기 위해)
        }
    }
}

void ErrorManagerNode::allErrorReleased()
{
    for (auto& error : error_list_) {
        error.error.error_occurred = false;
        error.should_publish = true;
        error.error.count = 1;
    }
}

void ErrorManagerNode::LidarErrorRelease(){

    static rclcpp::Clock clock(RCL_STEADY_TIME);
    static std::map<std::string, double> lidar_error_occured_times;

    if (error_list_.empty()) {
        return;
    }

    for (auto& error : error_list_)
    {
        if (error.error.error_code.find("F13") != std::string::npos
         || error.error.error_code.find("F14") != std::string::npos) {
            if (error.error.error_occurred) {
                // 에러가 활성 상태인 경우
                if (lidar_error_occured_times.find(error.error.error_code) == lidar_error_occured_times.end()) {
                    // 에러코드와 타임 추가 (타이머 시작)
                    lidar_error_occured_times[error.error.error_code] = clock.now().seconds();
                    RCLCPP_INFO(this->get_logger(), "Error %s detected. Starting 5s auto release timer.", error.error.error_code.c_str());
                } else {
                    // 맵에 있다면 시간 확인
                    double trigger_time = lidar_error_occured_times[error.error.error_code];
                    if ((clock.now().seconds() - trigger_time) >= 5.0) {
                        RCLCPP_INFO(this->get_logger(), "auto release error %s after 5 seconds.", error.error.error_code.c_str());
                        error.error.error_occurred = false;
                        error.should_publish = true;
                        error.error.count = 1; // 해제 상태를 퍼블리시하기 위해 카운트 리셋
                        lidar_error_occured_times.erase(error.error.error_code);
                    }
                }
            } else {
                if (lidar_error_occured_times.count(error.error.error_code)) {
                    lidar_error_occured_times.erase(error.error.error_code);
                }
            }
        }
    }
}


void ErrorManagerNode::printErrorList(){
    if (error_list_.empty()) {
        return;
    }

    std::stringstream ss;
    ss << "Error: ";
    for (size_t i = 0; i < error_list_.size(); ++i) {
        const auto& err = error_list_[i];
        std::string code = err.error.error_code;
        std::string occurred = err.error.error_occurred ? "OCCURED" : "RELEASED";
        std::string description = error_code_descriptions_[code];

        ss << "[" << code << " (" << occurred << "): " << description << "] ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
}
