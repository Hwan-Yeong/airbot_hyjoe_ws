#include "A1_keepout/A1_keepout.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto a1_keepout_node = std::make_shared<NavKeepoutService>();
    a1_keepout_node->run();
    rclcpp::shutdown();
    return 0;
}