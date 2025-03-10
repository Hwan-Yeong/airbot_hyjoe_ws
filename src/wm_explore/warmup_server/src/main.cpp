//
// Created by changun on 25. 2. 13.
//
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "presentation/warmup_server.hpp"

int main(int argc, char ** argv){
    rclcpp::init(argc,argv);
    //rclcpp::executors::MultiThreadedExecutor executor;
    //executor.add_node(std::make_shared<explore::WarmupServer>());
    rclcpp::spin(std::make_shared<explore::WarmupServer>());

    //executor.spin();
    rclcpp::shutdown();
    return 0;
}
