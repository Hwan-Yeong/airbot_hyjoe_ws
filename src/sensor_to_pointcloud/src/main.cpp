#include "sensor_to_pointcloud_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sensor_to_pointcloud::SensorToPointcloudNode>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}