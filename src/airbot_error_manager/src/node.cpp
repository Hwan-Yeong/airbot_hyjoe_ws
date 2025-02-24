#include "error_manager/error_manager.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ErrorManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}