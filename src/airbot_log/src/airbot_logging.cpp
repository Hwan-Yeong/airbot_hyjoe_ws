
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/fmt/ostr.h>
#include <queue>
#include <mutex>
#include <filesystem>
#include <string>
#include <dlfcn.h>
#include "airbot_log/libNetwork.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

const std::string folderPath = "/home/airbot/app_rw/log";

// 로그 엔트리를 표현하는 구조체
struct LogEntry {
    builtin_interfaces::msg::Time stamp;
    std::string message;

    // 타임스탬프 기반 비교 연산자 (우선순위 큐에서 사용)
    bool operator<(const LogEntry& other) const {
        return (stamp.sec < other.stamp.sec) || 
               (stamp.sec == other.stamp.sec && stamp.nanosec < other.stamp.nanosec);
    }
};



class UnifiedLogger : public rclcpp::Node
{
public:
    UnifiedLogger() : Node("airbot_logging")
    {
        load_config();
        std::filesystem::create_directories(folderPath);  // 폴더 생성
        // spdlog 로거 설정
        
        auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            "/home/airbot/app_rw/log/airbot_log.txt", log_file_size, log_backup_count); 
        logger_ = std::make_shared<spdlog::logger>("ros2_logger", rotating_sink);
        spdlog::set_default_logger(logger_);
        spdlog::set_level(spdlog::level::debug);        
        logger_->set_pattern("%v");  // 메시지만 출력

        // /rosout 토픽 구독
        subscription_ = this->create_subscription<rcl_interfaces::msg::Log>(
            "/rosout", 100, std::bind(&UnifiedLogger::log_callback, this, std::placeholders::_1));

        // 주기적 로그 처리를 위한 타이머 설정
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&UnifiedLogger::process_log_queue, this));
    }

private:

    size_t log_file_size = 10 * 1024 * 1024; // 기본값 10MB
    size_t log_backup_count = 9; // 기본값 9개
    std::vector<std::string> filtered_nodes;
     std::string config_path = "/home/airbot/app_rw/config/airbotconfig.ini";

    void load_config() {
        // 설정 파일이 없으면 기본값으로 생성
        if (!std::filesystem::exists(config_path)) {
            std::filesystem::create_directories("/home/airbot/app_rw/config");
            std::ofstream config_file(config_path);
            config_file << "[Logging]\n";
            config_file << "FileSize=10485760\n";  // 10MB
            config_file << "BackupCount=9\n";
            config_file << "FilteredNodes=amcl\n";
            config_file.close();
        }

        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini(config_path, pt);
        
        log_file_size = pt.get<size_t>("Logging.FileSize", 10 * 1024 * 1024);
        log_backup_count = pt.get<size_t>("Logging.BackupCount", 9);
        
        std::string nodes = pt.get<std::string>("Logging.FilteredNodes", "");
        std::istringstream ss(nodes);
        std::string node;
        while (std::getline(ss, node, ',')) {
            filtered_nodes.push_back(node);
            RCLCPP_INFO(this->get_logger(), "filtered_nodes : %s", node.c_str());
        }
    }

    // 로그 메시지 수신 콜백        
    void log_callback(const rcl_interfaces::msg::Log::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (std::find(filtered_nodes.begin(), filtered_nodes.end(), msg->name) != filtered_nodes.end()) {
            return;
        }
        log_queue_.push({msg->stamp, format_log_message(msg)});
    }

    // 로그 큐 처리 및 정렬된 로그 기록
    void process_log_queue()
    {
        std::vector<LogEntry> entries;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            while (!log_queue_.empty()) {
                entries.push_back(log_queue_.top());
                log_queue_.pop();
            }
        }

        // 타임스탬프 기준으로 로그 엔트리 정렬
        std::sort(entries.begin(), entries.end());

        // 정렬된 로그 메시지 기록
        for (const auto& entry : entries) {
            logger_->info(entry.message);
        }

        // 로그 전송 체크
        if(reqGetLogData()){            
            if (resGetLogData(folderPath)){
                RCLCPP_INFO(this->get_logger(), "folder processed successfully");
            }
            else {
                RCLCPP_INFO(this->get_logger(), "failed to process folder");
            }
        }
    }

    // 로그 메시지 포맷팅
    std::string format_log_message(const rcl_interfaces::msg::Log::SharedPtr msg)
    {
        return fmt::format("[{}] [{}] [{}] {}",
                           msg->name,
                           severity_to_string(msg->level),
                           format_time(msg->stamp),
                           msg->msg);
    }

    // 로그 레벨을 문자열로 변환
    std::string severity_to_string(uint8_t severity)
    {
        switch (severity)
        {
            case rcl_interfaces::msg::Log::DEBUG: return "DEBUG";
            case rcl_interfaces::msg::Log::INFO: return "INFO";
            case rcl_interfaces::msg::Log::WARN: return "WARN";
            case rcl_interfaces::msg::Log::ERROR: return "ERROR";
            case rcl_interfaces::msg::Log::FATAL: return "FATAL";
            default: return "UNKNOWN";
        }
    }

    // 타임스탬프를 문자열로 포맷팅
    std::string format_time(const builtin_interfaces::msg::Time& time)
    {
        // std::chrono::system_clock::time_point tp{std::chrono::seconds(time.sec) + std::chrono::nanoseconds(time.nanosec)};
        // return spdlog::format_log_time(tp);
        
        std::time_t t = time.sec;
        std::tm tm = *std::localtime(&t);

        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << "." << (time.nanosec / 1000000); // 밀리초까지 표시
        return oss.str();
    }

    std::shared_ptr<spdlog::logger> logger_;
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    // 타임스탬프 순으로 정렬된 로그 메시지를 저장하는 우선순위 큐
    // std::priority_queue<LogEntry, std::vector<LogEntry>, std::greater<LogEntry>> log_queue_;
    std::priority_queue<LogEntry, std::vector<LogEntry>, std::less<LogEntry>> log_queue_;

    std::mutex queue_mutex_;  // 큐 접근 동기화를 위한 뮤텍스
};
void *g_handle = nullptr;
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    g_handle = dlopen("/home/airbot/airbot_ws/install/airbot_log/lib/libLOGNetwork.so", RTLD_LAZY);
    if (!g_handle) {
        std::cerr << "airbot_logging - Cannot open library: " << dlerror() << '\n';
        return 1;
    }

    APISetEnc(false);
    start_server();

    rclcpp::spin(std::make_shared<UnifiedLogger>());
    rclcpp::shutdown();
    return 0;
}
