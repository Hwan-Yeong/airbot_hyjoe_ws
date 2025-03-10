#include <filesystem>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include "maneuver.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<A1::maneuver::ManeuverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}