#include "param_manager.hpp"

#include "perception_node.hpp"

namespace A1::perception
{
ParamManager::ParamManager(
    std::shared_ptr<PerceptionNode> node_ptr_,
    const std::string& full_path,
    const std::string& target_str)
{
    this->node_ptr = node_ptr_;
    this->yaml_ = YAML::LoadFile(full_path);
    this->node_config_ = yaml_["A1_perception"]["node"];
    auto targets = YAML::Load(target_str);

    std::map<std::string, std::vector<std::string>> result;
    for (const auto& target_item : targets)
    {
        std::string direction = target_item.first.as<std::string>();
        auto array = target_item.second.as<std::vector<std::string>>();
        result[direction] = array;
    }
    this->target_map_ = result;
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

        // // 기존 코드
        // std::ofstream fout(output_path);
        // if (!fout.is_open())
        // {
        //     RCLCPP_ERROR(this->node_ptr->get_logger(), "Failed to open file for writing: %s", output_path.c_str());
        //     return false;
        // }
        // fout << this->yaml_;
        // fout.flush();

        // // fsync 추가 코드
        YAML::Emitter out;
        out << yaml_;
        if (!out.good())
        {
            RCLCPP_ERROR(this->node_ptr->get_logger(), "Failed to serialize YAML node");
            return false;
        }

        FILE* fp = std::fopen(output_path.c_str(), "w");
        if (!fp)
        {
            RCLCPP_ERROR(this->node_ptr->get_logger(), "Failed to open file for writing: %s", output_path.c_str());
            return false;
        }

        if (std::fputs(out.c_str(), fp) == EOF)
        {
            RCLCPP_ERROR(this->node_ptr->get_logger(), "Failed to write YAML content to file");
            std::fclose(fp);
            return false;
        }

        std::fflush(fp);

        int fd = fileno(fp);
        if (fd != -1)
            fsync(fd);

        std::fclose(fp);
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

        std::string key = "index_" + std::to_string(i);
        const auto& target = this->target_map_[key];
        for (const auto& param : target)
        {
            YAML::Node node_value(rounded_value);
            this->set_config_value(this->node_config_, param, node_value);

            RCLCPP_INFO(
                this->node_ptr->get_logger(), "[Calibration] param: %s, value: %f", param.c_str(), rounded_value);
        }
    }
    std::string result = "[" + oss.str() + "]";
    RCLCPP_INFO(this->node_ptr->get_logger(), "[Calibration] Updated parameters values: %s", result.c_str());

    this->yaml_["A1_perception"]["node"] = this->node_config_;
}
}  // namespace A1::perception