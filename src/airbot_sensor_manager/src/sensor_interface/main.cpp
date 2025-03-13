#include "sensor_interface/sensor_interface_node.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}