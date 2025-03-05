#include "explore/explore.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<explore::Explore>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
