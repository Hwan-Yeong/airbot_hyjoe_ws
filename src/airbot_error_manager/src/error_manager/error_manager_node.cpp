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

    this->declare_parameter("error_publish_cnt", 3);
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

void ErrorManagerNode::publishErrorList()
{
    static int print_cnt = 0;
    bool print_now = false;

    robot_custom_msgs::msg::ErrorListArray error_msg_array;
    rclcpp::Time now_time = rclcpp::Clock(RCL_STEADY_TIME).now();
    builtin_interfaces::msg::Time msg_time;
    msg_time.sec = static_cast<int32_t>(now_time.seconds());
    msg_time.nanosec = now_time.nanoseconds() % 1000000000;
    error_msg_array.timestamp = msg_time;

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
            print_now = true;
        }

        robot_custom_msgs::msg::ErrorList error_msg = error.error;
        error.error.count++;
        error.has_occurred_before = true;
        error_msg_array.data_array.push_back(error_msg);

        if (error.error.count > pub_cnt) { // pub_cnt 만큼 퍼블리싱 했으면 발행 정지.
            error.should_publish = false;
        }
        ++it;
    }

    if (!error_msg_array.data_array.empty()) {
        error_list_pub_->publish(error_msg_array);
    }

    // error_list 출력
    if (print_now || ++print_cnt % 10 == 0) {
        printErrorList();
    }
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

void ErrorManagerNode::printErrorList(){
    if (error_list_.empty()) {
        return;
    }

    std::stringstream ss;
    ss << "[ Error List: ";
    for (size_t i = 0; i < error_list_.size(); ++i) {
        const auto& err = error_list_[i];
        ss << err.error.error_code << ": " << (err.error.error_occurred ? "true" : "false");
        if (i != error_list_.size() - 1) {
            ss << " & ";
        }
    }
    ss << " ]";
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
}
