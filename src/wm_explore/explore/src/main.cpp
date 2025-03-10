//
// Created by wavem on 25. 2. 24.
//

#include <explore/logger.hpp>

#include "rclcpp/rclcpp.hpp"
#include "explore/explore.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    std::shared_ptr<explore::Explore> node = std::make_shared<explore::Explore>();
    node->Init();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
