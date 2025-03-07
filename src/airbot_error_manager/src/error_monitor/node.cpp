#include "error_monitor/error_monitor.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ErrorMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}