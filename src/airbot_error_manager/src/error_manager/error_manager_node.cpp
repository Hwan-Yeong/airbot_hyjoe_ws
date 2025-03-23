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

    error_list_pub_ = this->create_publisher<robot_custom_msgs::msg::ErrorListArray>("/error_list", 10);

    pub_timer_ = this->create_wall_timer(
        publish_rate_ms,
        std::bind(&ErrorManagerNode::publishErrorList, this));
}

ErrorManagerNode::~ErrorManagerNode()
{
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
            int rank = error.second["rank"].as<int>();

            RCLCPP_INFO(this->get_logger(), "Sub topic: %s (error_code: %s, rank: %d)",
                        sub_topic.c_str(), error_code.c_str(), rank);

            auto callback = [this, pnode, error_code, rank](std_msgs::msg::Bool::SharedPtr msg)
            { errorCallback(error_code, rank, msg); };
            auto subs = this->create_subscription<std_msgs::msg::Bool>(sub_topic, 10, callback);
            this->subscribers_.push_back(subs);
        }
    }
    RCLCPP_INFO(this->get_logger(), "===============================================================");
}

void ErrorManagerNode::errorCallback(const std::string& error_code, int rank, std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        updateErrorLists(rank, error_code);
    } else {
        releaseErrorLists(rank, error_code);
    }
}

void ErrorManagerNode::publishErrorList()
{
    robot_custom_msgs::msg::ErrorListArray error_msg_array;

    for (auto it = error_list_.begin(); it != error_list_.end();) {
        auto& error = *it;

        if (!error.should_publish) {
            ++it;
            continue;
        }

        error.error.count++;

        robot_custom_msgs::msg::ErrorList error_msg;
        error_msg = error.error;
        error_msg_array.data_array.push_back(error_msg);

        if (error.error.count >= CLEAR_CNT) {
            error.should_publish = false;
            if (!error.error.error_occurred) { //5번 다 보냈는데, 에러해제 상태면 삭제하세요.
                it = error_list_.erase(it);
                continue;
            } else { //5번 다 보냈는데, 여전히 에러발생 상태면 카운트 초기화합니다.
                error.error.count = 0;
            }
        }
        ++it;
    }

    if (!error_msg_array.data_array.empty()) {
        error_list_pub_->publish(error_msg_array);
        printErrorList();
    }
}

void ErrorManagerNode::updateErrorLists(int rank, std::string code)
{
    auto it = std::find_if(error_list_.begin(), error_list_.end(),
        [&code](const tErrorList& error) {
            return error.error.error_code == code;
    });

    if (it == error_list_.end()) {
        addError(rank, code, ErrorType::OCCURRED);
    }
}

void ErrorManagerNode::addError(int rank, const std::string &error_code, ErrorType type)
{
    auto it = std::find_if(error_list_.begin(), error_list_.end(),
        [&error_code](const tErrorList& error) {
            return error.error.error_code == error_code;
    });

    if (it != error_list_.end()) { // error_list_에 이미 동일한 error가 존재하면 무시.
        return;
    }

    if (error_list_.size() > ERROR_LIST_SIZE) { // error_list_에 에러가 너무 많으면(10개까지만 관리) 가장 오래된 에러는 삭제. (필요한가..?)
        error_list_.erase(error_list_.begin());
    }

    tErrorList new_error;
    new_error.error.count = 0;
    new_error.error.rank = rank;
    new_error.error.error_code = error_code;
    // if (!error_list_.empty() && it->has_occurred_before) { // 이전에 에러 발생한 적 있었을 때만 처리 (최초 실행 시 에러 해제 올라가는 것 방지)
    //     new_error.should_publish = true;
    // }

    if (type == ErrorType::OCCURRED) {
        new_error.error.error_occurred = true;
        new_error.has_occurred_before = true; // 최초 발생 처리
        new_error.should_publish = true;
    } else {
        new_error.error.error_occurred = false;
        // new_error.has_occurred_before = false; // 해제 상태로 초기화
        if (!error_list_.empty() && it->has_occurred_before) { // 이전에 에러 발생한 적 있었을 때만 처리 (최초 실행 시 에러 해제 올라가는 것 방지)
            new_error.should_publish = true;
        }
    }
    error_list_.push_back(new_error);

    ///////////////////////////////

    RCLCPP_INFO(this->get_logger(),
        "New error added: Code: %s, Rank: %d, Occurred: %s, Should Publish: %s",
        new_error.error.error_code.c_str(), new_error.error.rank,
        new_error.error.error_occurred ? "true" : "false",
        new_error.should_publish ? "true" : "false");

    RCLCPP_INFO(this->get_logger(),
        "============= Current Error List (%ld) =============", error_list_.size());

    for (const auto& error : error_list_) {
        RCLCPP_INFO(this->get_logger(),
            "Error - Code: %s, Rank: %d, Occurred: %s, Count: %d, Should Publish: %s",
            error.error.error_code.c_str(), error.error.rank,
            error.error.error_occurred ? "true" : "false",
            error.error.count,
            error.should_publish ? "true" : "false");
    }
    RCLCPP_INFO(this->get_logger(), "=====================================================");
}

void ErrorManagerNode::releaseErrorLists(int rank, std::string code)
{
    if (error_list_.empty()) { // error_list_ 비어있으면 실행 x
        return;
    }

    auto it = std::find_if(error_list_.begin(), error_list_.end(),
    [&code](const tErrorList& error) {
        return error.error.error_code == code;
    });

    if (it != error_list_.end()) { // 기존 에러가 존재하면 상태 업데이트
        if (it->has_occurred_before) { // 과거에 발생한 적이 있을 때만 해제 메시지 퍼블리싱 활성화
            it->error.error_occurred = false;
            it->should_publish = true;
            it->error.count = 0; // 카운트 초기화 (다시 CLEAR_CNT 만큼 보내기 위해)
        }
    } else { // 기존 에러가 없다면 새로운 항목 추가 (에러 해제 상태로)
        addError(rank, code, ErrorType::CLEARED);
    }
}

void ErrorManagerNode::printErrorList(){
    if (error_list_.empty()) {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "================= Publishing error =================");
    for (const auto& error : error_list_) {
        if (error.should_publish) {
            RCLCPP_INFO(this->get_logger(),
                "Error - Occured: %s, Rank: %d, Code: %s, Count: %d, Should Publish: %s",
                error.error.error_occurred ? "true" : "false",
                error.error.rank,
                error.error.error_code.c_str(),
                error.error.count/*,
                error.should_publish ? "true" : "false"*/);
        } else continue;
    }
}