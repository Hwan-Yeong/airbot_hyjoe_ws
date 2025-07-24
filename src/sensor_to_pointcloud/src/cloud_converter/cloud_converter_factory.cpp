#include "cloud_converter/cloud_converter_factory.hpp"
#include "cloud_converter/cloud_converter.hpp"

namespace sensor_to_pointcloud {

CloudConverterPtr CloudConverterFactory::create(std::shared_ptr<SensorToPointcloudNode> node_ptr, const std::string& type, const YAML::Node& config)
{
    if (type == "camera")
    {
        return std::make_shared<CameraCloudConverter>(node_ptr, config);
    }
    else
    {
        throw std::runtime_error("Unknown Sensor type: " + type);
    }
}

CloudConverterPtr CloudConverterFactory::create(std::shared_ptr<SensorToPointcloudNode> node_ptr, const YAML::Node& config)
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
