#include "param_manager.hpp"

#include "perception_node.hpp"

namespace A1::perception
{
ParamManager::ParamManager(std::shared_ptr<PerceptionNode> node_ptr_, const std::string& full_path)
{
    this->node_ptr = node_ptr_;
    this->yaml_ = YAML::LoadFile(full_path);
    this->node_config_ = yaml_["A1_perception"]["node"];
}

void ParamManager::set_config_value(YAML::Node& node, const std::string& key, const YAML::Node& value)
{
    size_t dot_pos = key.find('.');
    if (dot_pos == std::string::npos)
    {
        node[key] = value;
    }
    else
    {
        std::string first = key.substr(0, dot_pos);
        std::string rest = key.substr(dot_pos + 1);
        YAML::Node target_node = node[first];
        set_config_value(target_node, rest, value);
    }
}

bool ParamManager::save_config_file(const std::string& output_path)
{
    try
    {
        std::filesystem::path dir = std::filesystem::path(output_path).parent_path();
        if (!dir.empty() && !std::filesystem::exists(dir))
        {
            std::filesystem::create_directories(dir);
        }
        std::ofstream fout(output_path);
        fout << this->yaml_;
        return true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->node_ptr->get_logger(), "Failed to save calibration config file: %s", e.what());
        return false;
    }
}

void ParamManager::update_parameters(const std::vector<float>& data)
{
    size_t n = data.size();
    std::ostringstream oss;
    for (size_t i = 0; i < n; ++i)
    {
        float rounded_value = std::round(data[i] * 10000.0f) / 10000.0f;
        oss << rounded_value;
        if (i != data.size() - 1)
        {
            oss << ", ";
        }
        std::string layer_name = layer_names[i];
        std::string key = "layers." + layer_name + ".filter.compose.filters.drop_off_diff.standard_dist";
        YAML::Node node_value(rounded_value);
        this->set_config_value(this->node_config_, key, node_value);
    }
    std::string result = "[" + oss.str() + "]";
    RCLCPP_INFO(this->node_ptr->get_logger(), "[Calibration] Updated parameters values: %s", result.c_str());

    this->yaml_["A1_perception"]["node"] = this->node_config_;
}
}  // namespace A1::perception