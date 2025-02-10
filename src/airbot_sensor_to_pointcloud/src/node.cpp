#include "airbot_sensor_to_pointcloud/sensor_to_pointcloud.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensoeToPointcloud>());
  rclcpp::shutdown();
  return 0;
}