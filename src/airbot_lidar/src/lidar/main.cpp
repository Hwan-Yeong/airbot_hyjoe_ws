#include <rclcpp/rclcpp.hpp>
#include "lidar/node_lidar_ros.h"
#include "node_lidar.h"
#include <thread>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("airbot_lidar");

    LidarManager lidar_manager(node);

    std::thread lidar_thread([&]() {
        lidar_manager.runLoop();
    });

    rclcpp::spin(node);

    if (lidar_thread.joinable())
        lidar_thread.join();

    rclcpp::shutdown();
    return 0;
}
