#ifndef __BOARD_TEMPERATURE_MONITOR_HPP__
#define __BOARD_TEMPERATURE_MONITOR_HPP__
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <string>
#include <vector>
#include <chrono>
#include <ctime>
#include <fstream>

class BoardTemperatureMonitor
{
public:
    BoardTemperatureMonitor();
    ~BoardTemperatureMonitor();

    bool errorMonitor();

private:
    float total_temp;
    int valid_reads ;
    double avg_temp;

    std::vector<std::string> temp_files = {
    "/sys/class/thermal/thermal_zone0/temp",
    "/sys/class/thermal/thermal_zone1/temp",
    "/sys/class/thermal/thermal_zone2/temp",
    "/sys/class/thermal/thermal_zone3/temp",
    "/sys/class/thermal/thermal_zone4/temp",
    "/sys/class/thermal/thermal_zone5/temp",
    "/sys/class/thermal/thermal_zone6/temp"
    };
};

#endif //__BOARD_TEMPERATURE_MONITOR_HPP__