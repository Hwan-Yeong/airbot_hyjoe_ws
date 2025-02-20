// 2024-12-26 clabil v1.0
#include <filesystem>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "perception_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<A1::perception::PerceptionNode>();
    node->init();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}