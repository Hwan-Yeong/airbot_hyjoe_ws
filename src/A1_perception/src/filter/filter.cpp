#include "filter/filter.hpp"

#include <memory>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <pcl/segmentation/impl/extract_polygonal_prism_data.hpp>
#include "motor_status.hpp"

#include "filter/filter_factory.hpp"
#include "perception_node.hpp"

namespace A1::perception
{

BaseFilter::BaseFilter(std::shared_ptr<PerceptionNode> node_ptr_) : node_ptr(node_ptr_)
{
}

std::shared_ptr<PerceptionNode> BaseFilter::getNodePtr() const
{
    return node_ptr;
}

LayerVector BaseFilter::update(LayerVector layer_vector)
{
    auto filtered_layer = this->updateImpl(layer_vector);

    for (auto it = filtered_layer.begin(); it != filtered_layer.end();)
    {
        if (it->cloud.empty())
        {
            it = filtered_layer.erase(it);
        }
        else
        {
            ++it;
        }
    }

    return filtered_layer;
}

ComposeFilter::ComposeFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config)
    : BaseFilter(node_ptr_)
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
    for (const auto& filter_config : it->second)
    {
        auto filter = FilterFactory::create(node_ptr, filter_config.first.as<std::string>(), filter_config.second);
        this->filters.push_back(filter);
    }
}

LayerVector ComposeFilter::updateImpl(LayerVector layer_vector)
{
    LayerVector filtered_layer = layer_vector;
    for (const auto& filter : filters)
    {
        filtered_layer = filter->update(filtered_layer);
    }
    return filtered_layer;
}

TimeoutFilter::TimeoutFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config)
    : BaseFilter(node_ptr_)
{
    auto s = YAML::Dump(config);
    this->timeout = rclcpp::Duration(std::chrono::milliseconds(config["timeout_milliseconds"].as<uint32_t>()));
}

LayerVector TimeoutFilter::updateImpl(LayerVector layer_vector)
{
    auto now = this->getNodePtr()->now();
    for (auto& layer : layer_vector)
    {
        if (now - layer.timestamp > timeout)
        {
            layer.cloud.clear();
        }
    }
    return layer_vector;
}

DensityFilter::DensityFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config)
    : BaseFilter(node_ptr_)
{
    this->max_count = config["max_count"].as<int32_t>();
    this->radius = config["radius"].as<float>();
}

LayerVector DensityFilter::updateImpl(LayerVector layer_vector)
{
    // 전체 레이어의 점들을 unified_cloud에 모으고, 각 점이 어느 레이어에 속하는지 저장합니다.
    pcl::PointCloud<pcl::PointXYZ> unified_cloud;
    // 각 점의 레이어 인덱스를 저장합니다.
    std::vector<std::size_t> point_layer_indices;
    for (std::size_t li = 0; li < layer_vector.size(); ++li)
    {
        auto& layer = layer_vector[li];
        unified_cloud.insert(unified_cloud.end(), layer.cloud.begin(), layer.cloud.end());
        for (std::size_t i = 0; i < layer.cloud.size(); ++i)
        {
            point_layer_indices.push_back(li);
        }
    }
    if (unified_cloud.empty())
    {
        RCLCPP_DEBUG(
            this->node_ptr->get_logger(), "%s():%d:  DensityFilter: unified_cloud is empty.", __FUNCTION__, __LINE__);
        return layer_vector;
    }

    // kd-tree 생성 및 전체 클라우드 설정
    pcl::search::KdTree<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(unified_cloud.makeShared());

    // 기존 neighbor_count, neighbor_indices 계산은 그대로 유지합니다.
    std::size_t num_points = unified_cloud.size();
    std::vector<int> neighbor_count(num_points, 0);
    std::vector<std::vector<int>> neighbor_indices(num_points);

    for (std::size_t i = 0; i < num_points; ++i)
    {
        std::vector<int> indices;
        std::vector<float> sqr_dists;
        int found = kdtree.radiusSearch(unified_cloud[i], this->radius, indices, sqr_dists);
        // 자기 자신 포함이므로 실제 이웃 수는 found - 1
        neighbor_count[i] = found - 1;

        for (int j = 0; j < found; ++j)
        {
            if (indices[j] != static_cast<int>(i))
            {
                neighbor_indices[i].push_back(indices[j]);
            }
        }
    }

    // effective_count를 neighbor_count로 초기화합니다.
    std::vector<int> effective_count = neighbor_count;

    // 밀도가 높은 순서대로 정렬하기 위한 인덱스 배열 생성
    std::vector<std::size_t> sorted_indices(num_points);
    for (std::size_t i = 0; i < num_points; ++i)
    {
        sorted_indices[i] = i;
    }
    std::sort(
        sorted_indices.begin(),
        sorted_indices.end(),
        [&](std::size_t a, std::size_t b) { return effective_count[a] > effective_count[b]; });

    // 제거 대신 effective_count 감소: 후보 포인트의 effective_count가 max_count보다 크면
    // 해당 후보를 제거 처리하고, 그 이웃들의 effective_count 값을 1씩 감소시킵니다.
    std::vector<bool> removed(num_points, false);
    for (std::size_t idx : sorted_indices)
    {
        if (removed[idx])
            continue;
        if (effective_count[idx] > static_cast<int>(this->max_count))
        {
            // 후보 점을 제거 처리
            removed[idx] = true;
            // 후보의 이웃에 대해서 effective_count를 감소시킵니다.
            for (int nb_idx : neighbor_indices[idx])
            {
                if (!removed[nb_idx])
                {
                    effective_count[nb_idx] = std::max(0, effective_count[nb_idx] - 1);
                }
            }
        }
    }

    RCLCPP_DEBUG(
        this->node_ptr->get_logger(),
        "%s():%d: DensityFilter: %ld points are removed.",
        __FUNCTION__,
        __LINE__,
        std::count(removed.begin(), removed.end(), true));

    // 각 레이어별 새로운 클라우드를 생성합니다.
    std::vector<pcl::PointCloud<pcl::PointXYZ>> new_clouds(layer_vector.size());
    for (std::size_t i = 0; i < unified_cloud.size(); ++i)
    {
        if (!removed[i])
        {
            std::size_t layer_idx = point_layer_indices[i];
            new_clouds[layer_idx].push_back(unified_cloud[i]);
        }
    }

    // 각 레이어의 클라우드를 갱신합니다.
    for (std::size_t li = 0; li < layer_vector.size(); ++li)
    {
        layer_vector[li].cloud = new_clouds[li];
    }

    return layer_vector;
}

RoIFilter::RoIFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config) : BaseFilter(node_ptr_)
{
    this->use_inside = config["use_inside"].as<bool>();
}

LayerVector RoIFilter::updateImpl(LayerVector layer_vector)
{
    // coordinate transform
    auto position = this->node_ptr->getPosition();
    geometry_msgs::msg::Transform transform_msg;
    tf2::toMsg(position.getTransform().inverse(), transform_msg);
    Eigen::Affine3f inverse_transform = tf2::transformToEigen(transform_msg).cast<float>();

    // roi filter
    for (auto& layer : layer_vector)
    {
        auto layer_base_link = layer;
        pcl::transformPointCloud(layer.cloud, layer_base_link.cloud, inverse_transform);

        auto it = layer.cloud.begin();
        auto jt = layer_base_link.cloud.begin();
        for (; it != layer.cloud.end();)
        {
            pcl::PointXY point{jt->x, jt->y};
            if (this->isInside(point) != this->use_inside)
            {
                it = layer.cloud.erase(it);
                jt = layer_base_link.cloud.erase(jt);
            }
            else
            {
                ++it;
                ++jt;
            }
        }
    }
    return layer_vector;
}

PolygonRoIFilter::PolygonRoIFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config)
    : RoIFilter(node_ptr_, config)
{
    auto polygon = config["polygon"].as<std::vector<std::pair<float, float>>>();
    for (const auto& point : polygon)
    {
        this->polygon.emplace_back(point.first, point.second);
    }
    auto first = *this->polygon.begin();
    auto last = *this->polygon.rbegin();

    if (first.x != last.x || first.y != last.y)
    {
        this->polygon.push_back(*this->polygon.begin());
    }
}

bool PolygonRoIFilter::isInside(pcl::PointXY point)
{
    return pcl::isXYPointIn2DXYPolygon<pcl::PointXY>(point, this->polygon);
}

SectorRoIFilter::SectorRoIFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config)
    : RoIFilter(node_ptr_, config)
{
    this->min_range = config["min_range"].as<float>();
    this->max_range = config["max_range"].as<float>();
    this->min_angle = config["min_angle"].as<float>() * M_PI / 180.0;
    this->max_angle = config["max_angle"].as<float>() * M_PI / 180.0;
}

bool SectorRoIFilter::isInside(pcl::PointXY point)
{
    float range = std::sqrt(point.x * point.x + point.y * point.y);
    float angle = std::atan2(point.y, point.x);
    return range >= this->min_range && range <= this->max_range && angle >= this->min_angle && angle <= this->max_angle;
}

DropOffFilter::DropOffFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config)
    : BaseFilter(node_ptr_)
{
    this->min_range = config["min_range"].as<float>();
    this->max_range = config["max_range"].as<float>();
    this->line_length = config["line_length"].as<float>();
    this->resolution = config["resolution"].as<float>();
}

LayerVector DropOffFilter::updateImpl(LayerVector layer_vector)
{
    // coordinate transform
    auto node = this->getNodePtr();
    auto position = node->getPosition();
    geometry_msgs::msg::Transform transform_msg;
    tf2::toMsg(position.getTransform().inverse(), transform_msg);
    Eigen::Affine3f inverse_transform = tf2::transformToEigen(transform_msg).cast<float>();

    auto now = node->now();

    double heading_x = cos(position.yaw);
    double heading_y = sin(position.yaw);

    double orthogonal_x = -heading_y;
    double orthogonal_y = heading_x;
    double length = this->line_length;
    double resolution = this->resolution;

    MotorStatus motor_status = node->getMotorStatus();

    for (auto& layer : layer_vector)
    {
        auto layer_base_link = layer;
        pcl::transformPointCloud(layer.cloud, layer_base_link.cloud, inverse_transform);

        auto it = layer.cloud.begin();
        auto jt = layer_base_link.cloud.begin();

        for (; it != layer.cloud.end();)
        {
            if (motor_status.isMoveToFoward())
            {
                pcl::PointXY point{jt->x, jt->y};
                pcl::PointXY point_global{it->x, it->y};
                float distance = std::sqrt(point.x * point.x + point.y * point.y);
                if (distance >= this->min_range && distance <= this->max_range)
                {
                    std::string key =
                        std::to_string(point_global.x).substr(0, std::to_string(point_global.x).find(".") + 3) + "," +
                        std::to_string(point_global.y).substr(0, std::to_string(point_global.y).find(".") + 3);
                    if (node->getDropOffLayerMap().find(key) == node->getDropOffLayerMap().end())
                    {
                        double start_x = point_global.x - orthogonal_x * length;
                        double start_y = point_global.y - orthogonal_y * length;
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                        for (double t = 0; t <= length * 2; t += resolution)
                        {
                            pcl::PointXYZ new_point;
                            new_point.x = start_x + orthogonal_x * t;
                            new_point.y = start_y + orthogonal_y * t;
                            new_point.z = 0.0;
                            cloud->points.push_back(new_point);
                        }
                        auto line_layer = Layer();
                        line_layer.sensor_timestamp = layer.sensor_timestamp;
                        line_layer.timestamp = now;
                        line_layer.cloud = *cloud;
                        node->getDropOffLayerMap()[key] = line_layer;

                        // 1 -> DROP_OFF STOP
                        node->sendActionStop(1);
                    }
                }
            }
            it = layer.cloud.erase(it);
            jt = layer_base_link.cloud.erase(jt);
        }
    }

    return layer_vector;
}

OneDTofFilter::OneDTofFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config)
    : BaseFilter(node_ptr_)
{
    this->layer_vector_size = 0;
}

LayerVector OneDTofFilter::updateImpl(LayerVector layer_vector)
{
    // coordinate transform
    auto node = this->getNodePtr();
    int size_sum = 0;
    for (auto& layer : layer_vector)
    {
        size_sum += layer.cloud.size();
    }
    if (this->layer_vector_size < size_sum)
    {
        MotorStatus motor_status = node->getMotorStatus();
        if (motor_status.isMoveToFoward())
        {
            // 2 -> One_D_TOF STOP
            node->sendActionStop(2);
        }
    }
    this->layer_vector_size = size_sum;
    return layer_vector;
}

}  // namespace A1::perception
