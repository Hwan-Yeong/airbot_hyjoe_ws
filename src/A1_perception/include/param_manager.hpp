#ifndef __PARAM_MANAGER_HPP__
#define __PARAM_MANAGER_HPP__

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace A1::perception
{
class PerceptionNode;
class ParamManager
{
   public:
    ParamManager(std::shared_ptr<PerceptionNode> node_ptr_, const std::string& full_path);
    // void update_parameters(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void update_parameters(const std::vector<float>& data);
    bool save_config_file(const std::string& output_path);

   private:
    void set_config_value(YAML::Node& node, const std::string& key, const YAML::Node& value);
    YAML::Node yaml_;
    YAML::Node node_config_;
    std::vector<std::string> layer_names = {
        "multi_tof_left_drop_off_diff_idx_57",
        "multi_tof_left_drop_off_diff_idx_60",
        "multi_tof_left_drop_off_diff_idx_63",
        "multi_tof_right_drop_off_diff_idx_56",
        "multi_tof_right_drop_off_diff_idx_59",
        "multi_tof_right_drop_off_diff_idx_62"};
    std::shared_ptr<PerceptionNode> node_ptr{};
};
}  // namespace A1::perception

#endif  // __PARAM_MANAGER_HPP__