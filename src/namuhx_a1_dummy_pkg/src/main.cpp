#include "namuhx_a1_dummy_node.hpp"


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NamuhxA1DummyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
