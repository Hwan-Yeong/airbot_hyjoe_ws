#include "localization_service/localization_service.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto localization_service_node = std::make_shared<LocalizationService>();
    localization_service_node->run();
    rclcpp::shutdown();
    return 0;
}