#include "filter/filter_factory.hpp"

#include "filter/filter.hpp"
#include "perception_node.hpp"

namespace A1::perception
{

BaseFilterPtr
FilterFactory::create(std::shared_ptr<PerceptionNode> node_ptr, const std::string& type, const YAML::Node& config)
{
    if (type == "compose")
    {
        return std::make_shared<ComposeFilter>(node_ptr, config);
    }
    else if (type == "timeout")
    {
        return std::make_shared<TimeoutFilter>(node_ptr, config);
    }
    else if (type == "density")
    {
        return std::make_shared<DensityFilter>(node_ptr, config);
    }
    else if (type == "polygon_roi")
    {
        return std::make_shared<PolygonRoIFilter>(node_ptr, config);
    }
    else if (type == "sector_roi")
    {
        return std::make_shared<SectorRoIFilter>(node_ptr, config);
    }
    else if (type == "drop_off")
    {
        return std::make_shared<DropOffFilter>(node_ptr, config);
    }
    else if (type == "one_d_tof_stop")
    {
        return std::make_shared<OneDTofFilter>(node_ptr, config);
    }
    else
    {
        throw std::runtime_error("Unknown filter type: " + type);
    }
}
BaseFilterPtr FilterFactory::create(std::shared_ptr<PerceptionNode> node_ptr, const YAML::Node& config)
{
    // config는 단일 mapping(예: compose: { filters: { ... } })이어야 함
    if (!config.IsMap() || config.size() != 1)
    {
        auto s = YAML::Dump(config);
        throw std::runtime_error("Invalid filter config format.");
    }

    auto it = config.begin();
    std::string type = it->first.as<std::string>();
    YAML::Node filter_config = it->second;

    return create(node_ptr, type, filter_config);
}

}  // namespace A1::perception