#include "explore/explore.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<explore::Explore>());
  rclcpp::shutdown();
  return 0;
}
