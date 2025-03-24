#include "node_manager/node_manager.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<NodeManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}