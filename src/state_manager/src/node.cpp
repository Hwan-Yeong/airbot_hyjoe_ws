#include "state_manager/state_manager.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<airbot_state::StateManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}