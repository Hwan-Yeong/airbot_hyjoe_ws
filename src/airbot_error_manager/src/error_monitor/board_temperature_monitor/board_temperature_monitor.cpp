#include "error_monitor/board_temperature_monitor/board_temperature_monitor.hpp"

BoardTemperatureMonitor::BoardTemperatureMonitor()
{
}

BoardTemperatureMonitor::~BoardTemperatureMonitor()
{
}

bool BoardTemperatureMonitor::errorMonitor()
{
    // AP 보드 온도 에러 판단 (Board temperature error check)
    for (const auto& file_path : temp_files) {
        std::ifstream file(file_path);
        if (!file) {
            RCLCPP_ERROR(rclcpp::get_logger("temp_monitor"),
                        "Failed to read temperature file: %s", file_path.c_str());
            continue;
        }

        std::string line;
        try {
            std::getline(file, line);
            file.close();

            float temp_value = std::stof(line) / 1000.0; // Convert from millidegrees to degrees

            // Check for high temperature warning
            if (temp_value > 70.0) {
                RCLCPP_WARN(rclcpp::get_logger("temp_monitor"),
                            "Warning: High temperature detected!,File: %s, Temperature: %.2f°C",
                            file_path.c_str(), temp_value);
                return true; // Return immediately if any temperature exceeds 70°C
            }
        }
        catch (const std::invalid_argument& e) {
            RCLCPP_ERROR(rclcpp::get_logger("temp_monitor"),
                        "Invalid temperature data in file %s: %s",
                        file_path.c_str(), e.what());
        }
    }

    return false; // No high temperature detected
}
