#include "error_monitor/error_monitor_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ErrorMonitorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}