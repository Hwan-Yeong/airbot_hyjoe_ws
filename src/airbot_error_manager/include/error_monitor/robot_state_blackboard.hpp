#pragma once

#include <mutex>
#include <memory>
#include "sensor_msgs/msg/imu.hpp"
#include "robot_custom_msgs/msg/bottom_ir_data.hpp"
#include "robot_custom_msgs/msg/battery_status.hpp"
#include "robot_custom_msgs/msg/station_data.hpp"
#include "robot_custom_msgs/msg/robot_state.hpp"
#include "robot_custom_msgs/msg/tof_data.hpp"
#include "robot_custom_msgs/msg/camera_data_array.hpp"
#include "robot_custom_msgs/msg/camera_data.hpp"
#include "robot_custom_msgs/msg/ai_temperature.hpp"
#include "robot_custom_msgs/msg/ap_temperature.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

class RobotStateBlackboard {
public:
    struct BatteryData {
        robot_custom_msgs::msg::BatteryStatus data;
        bool is_updated = false;
        std::chrono::steady_clock::time_point last_update_time;
    };

    struct IrData {
        robot_custom_msgs::msg::BottomIrData data;
        bool is_updated = false;
        std::chrono::steady_clock::time_point last_update_time;
    };

    struct ImuData {
        sensor_msgs::msg::Imu data;
        bool is_updated = false;
        std::chrono::steady_clock::time_point last_update_time;
    };

    struct StationData {
        robot_custom_msgs::msg::StationData data;
        bool is_updated = false;
        std::chrono::steady_clock::time_point last_update_time;
    };

    struct RobotStateData {
        robot_custom_msgs::msg::RobotState data;
        bool is_updated = false;
        std::chrono::steady_clock::time_point last_update_time;
    };

    struct OdomData {
        nav_msgs::msg::Odometry data;
        bool is_updated = false;
        std::chrono::steady_clock::time_point last_update_time;
    };

    struct TofData {
        robot_custom_msgs::msg::TofData data;
        bool is_updated = false;
        std::chrono::steady_clock::time_point last_update_time;
    };

    struct AiTemperatureData {
        robot_custom_msgs::msg::AiTemperature data;
        bool is_updated = false;
        std::chrono::steady_clock::time_point last_update_time;
    };

    struct ApTemperatureData {
        robot_custom_msgs::msg::ApTemperature data;
        bool is_updated = false;
        std::chrono::steady_clock::time_point last_update_time;
    };

    struct AiVersionData {
        std_msgs::msg::String data;
        bool is_updated = false;
        std::chrono::steady_clock::time_point last_update_time;
    };

    // --- Setters ---
    void setBatteryData(const robot_custom_msgs::msg::BatteryStatus& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        battery_.data = msg;
        battery_.is_updated = true;
        battery_.last_update_time = std::chrono::steady_clock::now();
    }

    void setIrData(const robot_custom_msgs::msg::BottomIrData& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        ir_.data = msg;
        ir_.is_updated = true;
        ir_.last_update_time = std::chrono::steady_clock::now();
    }

    void setImuData(const sensor_msgs::msg::Imu& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        imu_.data = msg;
        imu_.is_updated = true;
        imu_.last_update_time = std::chrono::steady_clock::now();
    }

    void setStationData(const robot_custom_msgs::msg::StationData& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        station_.data = msg;
        station_.is_updated = true;
        station_.last_update_time = std::chrono::steady_clock::now();
    }

    void setRobotStateData(const robot_custom_msgs::msg::RobotState& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        robot_state_.data = msg;
        robot_state_.is_updated = true;
        robot_state_.last_update_time = std::chrono::steady_clock::now();
    }

    void setOdomData(const nav_msgs::msg::Odometry& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        odom_.data = msg;
        odom_.is_updated = true;
        odom_.last_update_time = std::chrono::steady_clock::now();
    }

    void setTofData(const robot_custom_msgs::msg::TofData& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        tof_.data = msg;
        tof_.is_updated = true;
        tof_.last_update_time = std::chrono::steady_clock::now();
    }

    void setAiTemperatureData(const robot_custom_msgs::msg::AiTemperature& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        ai_temp_.data = msg;
        ai_temp_.is_updated = true;
        ai_temp_.last_update_time = std::chrono::steady_clock::now();
    }

    void setApTemperatureData(const robot_custom_msgs::msg::ApTemperature& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        ap_temp_.data = msg;
        ap_temp_.is_updated = true;
        ap_temp_.last_update_time = std::chrono::steady_clock::now();
    }

    void setAiVersionData(const std_msgs::msg::String& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        ai_version_.data = msg;
        ai_version_.is_updated = true;
        ai_version_.last_update_time = std::chrono::steady_clock::now();
    }

    // --- Getters ---
    BatteryData getBatteryData() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return battery_;
    }

    IrData getIrData() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return ir_;
    }

    ImuData getImuData() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return imu_;
    }

    StationData getStationData() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return station_;
    }

    RobotStateData getRobotStateData() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return robot_state_;
    }

    OdomData getOdomData() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return odom_;
    }

    TofData getTofData() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return tof_;
    }

    AiTemperatureData getAiTemperatureData() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return ai_temp_;
    }

    ApTemperatureData getApTemperatureData() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return ap_temp_;
    }

    AiVersionData getAiVersionData() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return ai_version_;
    }

private:
    mutable std::mutex mtx_;

    BatteryData battery_;
    IrData ir_;
    ImuData imu_;
    StationData station_;
    RobotStateData robot_state_;
    OdomData odom_;
    TofData tof_;
    AiTemperatureData ai_temp_;
    ApTemperatureData ap_temp_;
    AiVersionData ai_version_;
};


