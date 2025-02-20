#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <chrono>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

class RogManagerNode : public rclcpp::Node
{
public:
    RogManagerNode() : Node("log_manager")
    {
        RCLCPP_INFO(this->get_logger(), "RogManagerNode has started.");
#if 0 //after of the next
        allMovingInfo_subscriber_ = this->create_subscription<std_msgs::msg::Bool>("allMovingInfo", 10, std::bind(&AIInterface::allMovingInfoCallback, this, std::placeholders::_1));
#endif
        // 타이머 설정: 1시간(3600초) 간격
        timer_ = this->create_wall_timer(
            3600s,  // 1시간 주기 -> 3600s
            std::bind(&RogManagerNode::checkAndDeleteSyslog, this)
        );
    }

    ~RogManagerNode()
    {
 
    }

private:
#if 0 //after of the next
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr allMovingInfo_subscriber_;

    void allMovingInfoCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {   
        bool allMovingInfo = msg->data;
        RCLCPP_INFO(this->get_logger(), "allMovingInfo is %s ", allMovingInfo ? "true" : "false");
    }
#endif

    void checkAndDeleteSyslog() {
        std::string target_path = "/var/log/";
        std::string target_ros_path = "/home/airbot/.ros/log/";
        
        // 경로 존재 여부 확인 : 
        if (fs::exists(target_path)) {
            RCLCPP_INFO(this->get_logger(), "Path exists: %s", target_path.c_str());

            // 삭제 명령 실행
            std::string command = "sudo /bin/rm -rf /var/log/syslog*";
            int ret_code = system(command.c_str());

            if (ret_code == 0) {
                RCLCPP_INFO(this->get_logger(), "Successfully deleted %s", target_path.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to delete %s. Return code: %d", target_path.c_str(), ret_code);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Path does not exist: %s", target_path.c_str());
        }
#if 0 //
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // 경로 존재 여부 확인
        if (fs::exists(target_ros_path)) {
            RCLCPP_INFO(this->get_logger(), "Path exists: %s", target_ros_path.c_str());

            // 삭제 명령 실행
            std::string command = "/bin/rm -rf /home/airbot/.ros/log/*";
            int ret_code = system(command.c_str());

            if (ret_code == 0) {
                RCLCPP_INFO(this->get_logger(), "Successfully deleted %s", target_ros_path.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to delete %s. Return code: %d", target_ros_path.c_str(), ret_code);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Path does not exist: %s", target_ros_path.c_str());
        }
 #endif
    }

    rclcpp::TimerBase::SharedPtr timer_;  // 타이머 객체

};

void signal_handler(int signal)
{
    switch(signal)
    {
        case SIGINT:
        case SIGABRT:
        case SIGSEGV:
        case SIGTERM:
        case SIGKILL:
        default:
            break;
    }

    rclcpp::shutdown(); // This ensures that the node shuts down properly on SIGINT
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);

    auto node = std::make_shared<RogManagerNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
