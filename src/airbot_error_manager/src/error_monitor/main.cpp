#include "error_monitor/error_monitor_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ErrorMonitorNode>();
    node->init();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);    
    executor.spin();

    //rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}