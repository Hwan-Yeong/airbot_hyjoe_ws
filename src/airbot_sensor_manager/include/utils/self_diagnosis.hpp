#ifndef __SELF_DIAGNOSIS_HPP__
#define __SELF_DIAGNOSIS_HPP__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <yaml-cpp/yaml.h>

#include "utils/common_struct.hpp"
#include "cloud_converter/cloud_converter.hpp"

namespace sensor_manager {

class SelfDiagnosis {
public:
    SelfDiagnosis(
        rclcpp::Node* node,
        const std::unordered_map<std::string, CloudConverterPtr>& converters
    );
    ~SelfDiagnosis() = default;

    /**
     * @brief Measures time difference between data reception and publication.
     *        Logs a warning if it exceeds 5 times the 'publish_rate_ms'.
     * 
     * @param sensor_type Sensor Type Enum
     * @param receive_time Time when data was received (rclcpp::Time)
     * @param publish_rate_ms Configured publish rate in ms
     */
    void check_latency(SensorType sensor_type, const rclcpp::Time& receive_time, unsigned int publish_rate_ms);

    /**
     * @brief Runs a startup self-diagnosis by feeding dummy data to converters
     *        and checking if they produce output.
     * 
     * @param config The full sensor configuration YAML node
     */
    void run_startup_diagnosis(const YAML::Node& config);

private:
    rclcpp::Node* node_;
    const std::unordered_map<std::string, CloudConverterPtr>& converters_;

    // Helper functions for dummy data generation
    void check_single_sensor(const std::string& sensor_name, const YAML::Node& sensor_config);
    
    // Virtual Sensor Data Generators
    std::shared_ptr<void> create_dummy_tof_data();
    std::shared_ptr<void> create_dummy_camera_data();
    std::shared_ptr<void> create_dummy_bottom_ir_data();
    std::shared_ptr<void> create_dummy_collision_data();
};

} // namespace sensor_manager

#endif // __SELF_DIAGNOSIS_HPP__
