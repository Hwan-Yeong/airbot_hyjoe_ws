#include "airbot_sensor_to_pointcloud/sensor_to_pointcloud_node.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorToPointcloud>());
  rclcpp::shutdown();
  return 0;
}