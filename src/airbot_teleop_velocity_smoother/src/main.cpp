#include "teleop_velocity_smoother_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<airbot::TeleopVelocitySmoother>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}