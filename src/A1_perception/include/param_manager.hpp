#ifndef __PARAM_MANAGER_HPP__
#define __PARAM_MANAGER_HPP__

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <unistd.h>

namespace A1::perception
{
class PerceptionNode;
class ParamManager
{
   public:
    ParamManager(
        std::shared_ptr<PerceptionNode> node_ptr_,
        const std::string& full_path,
        const std::string& target_str);

    void update_parameters(const std::vector<float>& data);
    bool save_config_file(const std::string& output_path);

   private:
    void set_config_value(YAML::Node& node, const std::string& key, const YAML::Node& value);
    YAML::Node yaml_;
    YAML::Node node_config_;
    std::map<std::string, std::vector<std::string>> target_map_;
    std::shared_ptr<PerceptionNode> node_ptr{};
};
}  // namespace A1::perception

#endif  // __PARAM_MANAGER_HPP__