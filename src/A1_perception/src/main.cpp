#include "perception_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<A1::perception::PerceptionNode>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}