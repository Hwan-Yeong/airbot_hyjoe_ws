//
// Created by changun on 25. 2. 13.
//
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "presentation/warmup_server.hpp"

int main(int argc, char ** argv){
    rclcpp::init(argc,argv);

    rclcpp::spin(std::make_shared<explore::WarmupServer>());
    rclcpp::shutdown();
    return 0;
}
