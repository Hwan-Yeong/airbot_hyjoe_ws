#include "error_manager/error_manager.h"

#define CLEAR_CNT 5

using namespace std::chrono_literals;

ErrorManager::ErrorManager()
    : Node("airbot_error_manager")
{
    scan_front_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/scan_error_front",
        100,
        std::bind(&ErrorManager::scanFrontErrorCallback, this, std::placeholders::_1));
    
    scan_back_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/scan_error_back",
        100,
        std::bind(&ErrorManager::scanBackErrorCallback, this, std::placeholders::_1));
    
    scan_dirty_front_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/scan_dirty_front",
        100,
        std::bind(&ErrorManager::scanDirtyFrontErrorCallback, this, std::placeholders::_1));
    
    scan_dirty_back_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/scan_dirty_back",
        100,
        std::bind(&ErrorManager::scanDirtyBackErrorCallback, this, std::placeholders::_1));
    
    error_list_pub_ = this->create_publisher<robot_custom_msgs::msg::ErrorListArray>("/error_list", 10);

    pub_timer_ = this->create_wall_timer(
        200ms,
        std::bind(&ErrorManager::publishErrorList, this));
}

ErrorManager::~ErrorManager()
{
}

void ErrorManager::scanFrontErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 12;
    std::string errorCode = "F13";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }

}

void ErrorManager::scanBackErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 13;
    std::string errorCode = "F14";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

void ErrorManager::scanDirtyFrontErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 14;
    std::string errorCode = "E05";
    if (msg->data) {
        updateErrorLists(rank, errorCode);
    }
}

void ErrorManager::scanDirtyBackErrorCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int rank = 15;
    std::string errorCode = "E06";
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
    const size_t max_size = 10;

    auto it = std::find_if(error_list_.begin(), error_list_.end(),
        [rank](const std::shared_ptr<robot_custom_msgs::msg::ErrorList>& error) {
            return error->rank == rank;
        });

    if (it != error_list_.end()) {
        return;
    }

    if (error_list_.size() >= max_size) {
        error_list_.erase(error_list_.begin());
    }

    auto new_error = std::make_shared<robot_custom_msgs::msg::ErrorList>();
    new_error->count = 0;
    new_error->rank = rank;
    new_error->error_code = error_code;
    error_list_.push_back(new_error);
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
    }
}
