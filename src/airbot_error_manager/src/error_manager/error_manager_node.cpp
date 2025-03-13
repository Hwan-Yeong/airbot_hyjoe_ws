#include "ament_index_cpp/get_package_share_directory.hpp"
#include "error_manager/error_manager_node.hpp"

ErrorManagerNode::ErrorManagerNode()
    : Node("airbot_error_manager")
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
        removeFromErrorLists(error_code);
    }
}

void ErrorManagerNode::publishErrorList()
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

void ErrorManagerNode::updateErrorLists(int rank, std::string code)
{
    auto it = std::find_if(error_list_.begin(), error_list_.end(),
        [&code](const std::shared_ptr<robot_custom_msgs::msg::ErrorList>& error) {
            return error->error_code == code;
    });

    if (it == error_list_.end()) {
        addError(rank, code);
    }
}

void ErrorManagerNode::addError(int rank, const std::string &error_code)
{
    auto it = std::find_if(error_list_.begin(), error_list_.end(),
        [&error_code](const std::shared_ptr<robot_custom_msgs::msg::ErrorList>& error) {
            return error->error_code == error_code;
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

void ErrorManagerNode::removeFromErrorLists(std::string code)
{
    auto it = std::find_if(error_list_.begin(), error_list_.end(),
        [&code](const std::shared_ptr<robot_custom_msgs::msg::ErrorList>& error) {
            return error->error_code == code;
    });
    if (it != error_list_.end()) {
        error_list_.erase(it);
    }
}

void ErrorManagerNode::printErrorList(){
    RCLCPP_INFO(this->get_logger(),
        "================= Num of error: %d =================",
        static_cast<int>(error_list_.size()));
    for (const auto& error : error_list_) {
        RCLCPP_INFO(this->get_logger(),
            "Error - Rank: %d, Code: %s, Count: %d",
            error->rank, error->error_code.c_str(), error->count);
    }
}