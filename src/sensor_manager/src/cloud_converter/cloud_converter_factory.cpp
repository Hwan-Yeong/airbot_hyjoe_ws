#include "cloud_converter/cloud_converter_factory.hpp"
#include "cloud_converter/cloud_converter.hpp"

namespace sensor_manager {

CloudConverterPtr CloudConverterFactory::create(std::shared_ptr<SensorManagerNode> node_ptr, const std::string& type, const YAML::Node& config)
{
    if (type == "tof_mono")
    {
        return std::make_shared<TofMonoCloudConverter>(node_ptr, config);
    }
    else if (type == "tof_multi")
    {
        return nullptr;
    }
    else if (type == "tof_multi_left")
    {
        return nullptr;
    }
    else if (type == "tof_multi_right")
    {
        return nullptr;
    }
    else if (type == "camera")
    {
        return std::make_shared<CameraCloudConverter>(node_ptr, config);
    }
    else if (type == "bottom_ir")
    {
        return std::make_shared<BottomIrCloudConverter>(node_ptr, config);
    }
    else if (type == "collision_front" || type == "collision_rear")
    {
        return std::make_shared<CollisionCloudConverter>(node_ptr, config);
    }
    else if (type == "empty")
    {
        return std::make_shared<EmptyCloudConverter>(node_ptr, config);
    }
    else
    {
        throw std::runtime_error("Unknown Sensor type: " + type);
    }
}

CloudConverterPtr CloudConverterFactory::create(std::shared_ptr<SensorManagerNode> node_ptr, const YAML::Node& config)
{
    if (!config.IsMap() || config.size() != 1)
    {
        auto s = YAML::Dump(config);
        throw std::runtime_error("Invalid sensor config format.");
    }

    auto it = config.begin();
    std::string type = it->first.as<std::string>();
    YAML::Node filter_config = it->second;

    return create(node_ptr, type, filter_config);
}

}
