#ifndef __FILTER_FACTORY_HPP__
#define __FILTER_FACTORY_HPP__

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "filter/filter.hpp"

namespace A1::perception
{
// Forward declaration
class PerceptionNode;

/**
 * @brief 필터 객체를 생성하는 팩토리 클래스.
 *
 * 이 클래스는 PerceptionNode와 YAML 형식의 설정 정보를 기반으로
 * 적절한 필터 객체(BaseFilterPtr)를 생성하는 역할을 수행합니다.
 *
 * 제공하는 정적 메서드:
 * - create(std::shared_ptr<PerceptionNode> node_ptr, const std::string& type, const YAML::Node& config)
 *   : 필터 타입을 명시하여 필터 객체를 생성합니다.
 * - create(std::shared_ptr<PerceptionNode> node_ptr, const YAML::Node& config)
 *   : 별도의 타입 정보 없이 YAML 설정 정보를 사용하여 필터 객체를 생성합니다.
 *
 * 사용 예:
 *   auto filter = FilterFactory::create(perception_node_ptr, "필터타입", config);
 *   또는
 *   auto filter = FilterFactory::create(perception_node_ptr, config);
 *
 * @note 이 클래스는 필터 생성을 위한 팩토리 패턴의 구현체입니다.
 */
class FilterFactory
{
   public:
    FilterFactory() = default;
    ~FilterFactory() = default;

    static BaseFilterPtr
    create(std::shared_ptr<PerceptionNode> node_ptr, const std::string& type, const YAML::Node& config);
    static BaseFilterPtr create(std::shared_ptr<PerceptionNode> node_ptr, const YAML::Node& config);
};

}  // namespace A1::perception
#endif  // __FILTER_FACTORY_HPP__