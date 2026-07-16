#include "filter/filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <pcl/segmentation/impl/extract_polygonal_prism_data.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "motor_status.hpp"

#include "filter/filter_factory.hpp"
#include "perception_node.hpp"

namespace A1::perception
{

BaseFilter::BaseFilter(std::shared_ptr<PerceptionNode> node_ptr_) : node_ptr(node_ptr_)
{
    this->last_log_time = node_ptr->now();
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

rclcpp::Time BaseFilter::getLastLogTime() const
{
    return last_log_time;
}

void BaseFilter::setLastLogTime(rclcpp::Time time)
{
    last_log_time = time;
}

bool BaseFilter::logIntervalPassed()
{
    rclcpp::Time now = node_ptr->now();
    bool isPassed = false;
    if ((now - last_log_time).seconds() >= 1.0)
    {
        isPassed = true;
        setLastLogTime(now);
    }
    return isPassed;
}

ComposeFilter::ComposeFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config)
    : BaseFilter(node_ptr_)
{
    // config는 단일 mapping(예: compose: { filters: { ... } })이어야 함
    if (!config.IsMap() || config.size() != 1)
    {
        auto s = YAML::Dump(config);
        RCLCPP_ERROR(node_ptr->get_logger(), "Invalid filter config format.");
        // throw std::runtime_error("Invalid filter config format.");
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
    auto timeout_ms = getYamlValue<uint32_t>(__FUNCTION__, config, "timeout_milliseconds", 100);
    this->timeout = rclcpp::Duration(std::chrono::milliseconds(timeout_ms));
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
    this->max_count = getYamlValue<int>(__FUNCTION__, config, "max_count", 3);
    this->radius = getYamlValue<float>(__FUNCTION__, config, "radius", 0.3);
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
        RCLCPP_DEBUG(this->node_ptr->get_logger(), "DensityFilter: unified_cloud is empty.");
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
        "DensityFilter: %ld points are removed.",
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
    this->use_inside = getYamlValue<bool>(__FUNCTION__, config, "use_inside", true);
    this->use_when_climb = getYamlValue<bool>(__FUNCTION__, config, "use_when_climb", true);
    this->delete_out_of_range = getYamlValue<bool>(__FUNCTION__, config, "delete_out_of_range", true);
    this->delete_all_when_climb = getYamlValue<bool>(__FUNCTION__, config, "delete_all_when_climb", false);
}

LayerVector RoIFilter::updateImpl(LayerVector layer_vector)
{
    auto node = this->node_ptr;
    // 2025.06.02, 강성준, 승월시 장애물 검출 여부 파라미터 추가
    if (node->isClimb() && this->use_when_climb == false)
    {
        for (auto& layer : layer_vector)
        {
            if (layer.isDeletable || this->delete_all_when_climb)
                layer.cloud.clear();
        }
        return layer_vector;
    }

    // coordinate transform
    auto position = node->getPosition();
    geometry_msgs::msg::Transform transform_msg;
    tf2::toMsg(position.getTransform().inverse(), transform_msg);
    Eigen::Affine3f inverse_transform = tf2::transformToEigen(transform_msg).cast<float>();

    // roi filter
    for (auto& layer : layer_vector)
    {
        if (layer.isDeletable == false)
        {
            continue;
        }
        auto layer_base_link = layer;
        pcl::transformPointCloud(layer.cloud, layer_base_link.cloud, inverse_transform);

        auto it = layer.cloud.begin();
        auto jt = layer_base_link.cloud.begin();
        bool isDeletable = true;
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
                isDeletable = false;
            }
        }
        if (this->delete_out_of_range == false)
        {
            layer.isDeletable = isDeletable;
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
    this->min_range = getYamlValue<float>(__FUNCTION__, config, "min_range", 0.3);
    this->max_range = getYamlValue<float>(__FUNCTION__, config, "max_range", 1.0);
    this->min_angle = getYamlValue<float>(__FUNCTION__, config, "min_angle", -50.3) * M_PI / 180.0;
    this->max_angle = getYamlValue<float>(__FUNCTION__, config, "max_angle", 50.3) * M_PI / 180.0;
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
    this->min_range = getYamlValue<float>(__FUNCTION__, config, "min_range", 0.45);
    this->line_length = getYamlValue<float>(__FUNCTION__, config, "line_length", 0.05);
    this->resolution = getYamlValue<float>(__FUNCTION__, config, "resolution", 0.05);
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
        if (layer.isDeletable == false)
        {
            continue;
        }
        auto layer_base_link = layer;
        pcl::transformPointCloud(layer.cloud, layer_base_link.cloud, inverse_transform);

        auto it = layer.cloud.begin();
        auto jt = layer_base_link.cloud.begin();

        for (; it != layer.cloud.end();)
        {
            if (!(motor_status.isMoveToFoward() && node->isClimb() == false))
            {
                it = layer.cloud.erase(it);
                jt = layer_base_link.cloud.erase(jt);
                continue;
            }

            pcl::PointXY point{jt->x, jt->y};
            pcl::PointXYZ point_global{it->x, it->y, 0};
            float distance = std::sqrt(point.x * point.x + point.y * point.y);
            if (distance < this->min_range)
            {
                it = layer.cloud.erase(it);
                jt = layer_base_link.cloud.erase(jt);
                continue;
            }

            std::string key = std::to_string(point_global.x).substr(0, std::to_string(point_global.x).find(".") + 2) +
                              "," +
                              std::to_string(point_global.y).substr(0, std::to_string(point_global.y).find(".") + 2);
            if (node->getDropOffLayerMap().find(key) == node->getDropOffLayerMap().end())
            {
                auto line_layer = Layer();
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
                line_layer.sensor_timestamp = layer.sensor_timestamp;
                line_layer.timestamp = now;
                line_layer.cloud = *cloud;
                node->getDropOffLayerMap()[key] = line_layer;
                // 1 -> DROP_OFF STOP
                node->sendActionStop(1);
                layer.isDeletable = false;
                if (logIntervalPassed())
                {
                    RCLCPP_INFO(
                        node->get_logger(),
                        "Detect drop off. robot_xy(%.3f, %.3f), drop(x:%.3f, y:%.3f, dist: %.3f)",
                        position.x,
                        position.y,
                        point_global.x,
                        point_global.y,
                        distance);
                }
            }
            it = layer.cloud.erase(it);
            jt = layer_base_link.cloud.erase(jt);
        }
    }

    return layer_vector;
}

DropOffDiffFilter::DropOffDiffFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config)
    : BaseFilter(node_ptr_)
{
    this->dist_diff = getYamlValue<float>(__FUNCTION__, config, "dist_diff", 0.1);
    this->standard_dist = getYamlValue<float>(__FUNCTION__, config, "standard_dist", 0.5);
    this->line_length = getYamlValue<float>(__FUNCTION__, config, "line_length", 0.05);
    this->resolution = getYamlValue<float>(__FUNCTION__, config, "resolution", 0.05);
}

LayerVector DropOffDiffFilter::updateImpl(LayerVector layer_vector)
{
    auto node = this->getNodePtr();
    MotorStatus motor_status = node->getMotorStatus();
    if (node->isClimb())  // || !motor_status.isMoveToFoward())
    {
        for (auto& layer : layer_vector)
        {
            if (layer.isDeletable)
                layer.cloud.clear();
        }
        return layer_vector;
    }
    // coordinate transform
    auto position = node->getPosition();
    geometry_msgs::msg::Transform transform_msg;
    tf2::toMsg(position.getTransform().inverse(), transform_msg);
    Eigen::Affine3f inverse_transform = tf2::transformToEigen(transform_msg).cast<float>();

    auto now = std::chrono::steady_clock::now();

    double heading_x = cos(position.yaw);
    double heading_y = sin(position.yaw);

    double orthogonal_x = -heading_y;
    double orthogonal_y = heading_x;
    double length = this->line_length;
    double resolution = this->resolution;

    for (auto& layer : layer_vector)
    {
        if (layer.isDeletable == false)
        {
            continue;
        }
        auto layer_base_link = layer;
        pcl::transformPointCloud(layer.cloud, layer_base_link.cloud, inverse_transform);

        auto it = layer.cloud.begin();
        auto jt = layer_base_link.cloud.begin();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (; it != layer.cloud.end();)
        {
            pcl::PointXY point{jt->x, jt->y};
            pcl::PointXYZ point_global{it->x, it->y, 0};
            float distance = std::sqrt(point.x * point.x + point.y * point.y);
            float diff = distance - this->standard_dist;
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->prev_time).count();
            // RCLCPP_INFO(
            //     node->get_logger(),
            //     "Drop off check prev dist: %.3f, cur dist:%.3f dist diff:%.3f",
            //     this->prev_dist,
            //     distance,
            //     diff);
            if (diff < this->dist_diff || elapsed > 20)
            {
                it = layer.cloud.erase(it);
                jt = layer_base_link.cloud.erase(jt);
                continue;
            }

            // RCLCPP_INFO(node->get_logger(), "Drop off detect dist diff:%.3f", diff);
            std::string key = std::to_string(point_global.x).substr(0, std::to_string(point_global.x).find(".") + 2) +
                              "," +
                              std::to_string(point_global.y).substr(0, std::to_string(point_global.y).find(".") + 2);
            if (node->getDropOffLayerMap().find(key) == node->getDropOffLayerMap().end())
            {
                auto line_layer = Layer();
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
                line_layer.sensor_timestamp = layer.sensor_timestamp;
                line_layer.timestamp = node->now();
                line_layer.cloud = *cloud;
                node->getDropOffLayerMap()[key] = line_layer;
                // 1 -> DROP_OFF STOP
                if (motor_status.isMoveToFoward())
                    node->sendActionStop(1);
                if (logIntervalPassed())
                {
                    RCLCPP_INFO(
                        node->get_logger(),
                        "Detect drop off. robot_xy(%.3f, %.3f), drop(x:%.3f, y:%.3f, dist: %.3f, diff:%.3f)",
                        position.x,
                        position.y,
                        point_global.x,
                        point_global.y,
                        distance,
                        diff);
                }
            }
            // double start_x = point_global.x - orthogonal_x * length;
            // double start_y = point_global.y - orthogonal_y * length;
            // for (double t = 0; t <= length * 2; t += resolution)
            // {
            //     pcl::PointXYZ new_point;
            //     new_point.x = start_x + orthogonal_x * t;
            //     new_point.y = start_y + orthogonal_y * t;
            //     new_point.z = 0.0;
            //     cloud->points.push_back(new_point);
            // }
            // // 1 -> DROP_OFF STOP
            // node->sendActionStop(1);
            // layer.isDeletable = false;
            // if (logIntervalPassed())
            // {
            //     RCLCPP_INFO(
            //         node->get_logger(),
            //         "Detect drop off. robot_xy(%.3f, %.3f), drop(x:%.3f, y:%.3f, dist: %.3f, diff:%.3f)",
            //         position.x,
            //         position.y,
            //         point_global.x,
            //         point_global.y,
            //         distance,
            //         diff);
            // }

            it = layer.cloud.erase(it);
            jt = layer_base_link.cloud.erase(jt);
        }
        // layer.cloud.insert(layer.cloud.end(), cloud->begin(), cloud->end());
    }
    this->prev_time = now;
    return layer_vector;
}

OneDTofFilter::OneDTofFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config)
    : BaseFilter(node_ptr_)
{
    this->layer_vector_size = 0;
    this->line_length = getYamlValue<float>(__FUNCTION__, config, "line_length", 0.05);
    this->resolution = getYamlValue<float>(__FUNCTION__, config, "resolution", 0.05);
    this->stop_and_back_dist = getYamlValue<float>(__FUNCTION__, config, "stop_and_back_dist", 0.33);
    this->move_back_log_time = node_ptr->now();
    this->use_stop = getYamlValue<bool>(__FUNCTION__, config, "use_stop", false);
}

LayerVector OneDTofFilter::updateImpl(LayerVector layer_vector)
{
    // coordinate transform
    auto node = this->getNodePtr();
    // int cloud_size_sum = 0;
    auto position = node->getPosition();
    double heading_x = cos(position.yaw);
    double heading_y = sin(position.yaw);

    double orthogonal_x = -heading_y;
    double orthogonal_y = heading_x;
    double length = this->line_length;
    double resolution = this->resolution;
    MotorStatus motor_status = node->getMotorStatus();

    geometry_msgs::msg::Transform transform_msg;
    tf2::toMsg(position.getTransform().inverse(), transform_msg);
    Eigen::Affine3f inverse_transform = tf2::transformToEigen(transform_msg).cast<float>();

    for (auto it = layer_vector.begin(); it != layer_vector.end();)
    {
        if (it->cloud.size() == 0)
        {
            layer_vector.erase(it);
        }
        else
        {
            ++it;
        }
    }

    bool robot_back_flag = false;
    const int layer_size = layer_vector.size();
    if (layer_size > 0)
    {
        auto now = this->getNodePtr()->now();
        auto& last_layer = *layer_vector.rbegin();

        rclcpp::Duration diff = now - last_layer.timestamp;
        if (last_layer.cloud.size() == 1 && diff.seconds() < 0.01)
        {
            auto global_point = last_layer.cloud[0];
            Layer layer_base_link;
            pcl::transformPointCloud(last_layer.cloud, layer_base_link.cloud, inverse_transform);
            auto one_d_point = layer_base_link.cloud[0];
            auto dist = std::sqrt(one_d_point.x * one_d_point.x + one_d_point.y * one_d_point.y);

            auto it = last_layer.cloud.begin();
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointXY point_global{it->x, it->y};
            double start_x = point_global.x - orthogonal_x * length;
            double start_y = point_global.y - orthogonal_y * length;
            if (dist <= this->stop_and_back_dist)
            {
                robot_back_flag = true;
            }
            if (logIntervalPassed())
            {
                RCLCPP_INFO(
                    node->get_logger(),
                    "1D ToF detected. robot_xy(%.3f, %.3f), 1D(x:%.3f, y:%.3f, Dist:%.3f)",
                    position.x,
                    position.y,
                    global_point.x,
                    global_point.y,
                    dist);
            }
            for (double t = 0; t <= length * 2; t += resolution)
            {
                pcl::PointXYZ new_point;
                new_point.x = start_x + orthogonal_x * t;
                new_point.y = start_y + orthogonal_y * t;
                new_point.z = 0.0;
                cloud->points.push_back(new_point);
            }
            last_layer.cloud.insert(last_layer.cloud.end(), cloud->begin(), cloud->end());
        }
    }

    if (this->layer_vector_size < layer_size)
    {
        if (robot_back_flag && (motor_status.isMoveToFoward() || motor_status.isRotate()) && this->use_stop)
        {
            // 2 -> One_D_TOF STOP
            node->sendActionStop(2);
            rclcpp::Time now = node_ptr->now();
            if ((now - this->move_back_log_time).seconds() >= 1.0)
            {
                this->move_back_log_time = now;
                RCLCPP_INFO(
                    node->get_logger(),
                    "1D ToF data too close(range:%.3f). Request move to back.",
                    this->stop_and_back_dist);
            }
        }
    }
    this->layer_vector_size = layer_size;
    return layer_vector;
}

LidarDistCheckFilter::LidarDistCheckFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config)
    : BaseFilter(node_ptr_)
{
    this->min_range = getYamlValue<float>(__FUNCTION__, config, "min_range", 0.0);
    this->max_range = getYamlValue<float>(__FUNCTION__, config, "max_range", 0.4);
}

LayerVector LidarDistCheckFilter::updateImpl(LayerVector layer_vector)
{
    auto node = this->getNodePtr();
    auto position = node->getPosition();
    geometry_msgs::msg::Transform transform_msg;
    tf2::toMsg(position.getTransform().inverse(), transform_msg);
    Eigen::Affine3f inverse_transform = tf2::transformToEigen(transform_msg).cast<float>();

    // lidar check
    Layer lidar_pc = node->getSensorLayer("lidar_pc_baselink");
    for (auto& layer : layer_vector)
    {
        Layer layer_base_link;
        pcl::transformPointCloud(layer.cloud, layer_base_link.cloud, inverse_transform);

        auto it = layer.cloud.begin();
        auto jt = layer_base_link.cloud.begin();
        for (; it != layer.cloud.end();)
        {
            pcl::PointXY point{jt->x, jt->y};
            bool erased = false;  // 삭제 여부 체크
            for (const auto& lidar_point : lidar_pc.cloud)
            {
                auto dist = std::sqrt(
                    (lidar_point.x - point.x) * (lidar_point.x - point.x) +
                    (lidar_point.y - point.y) * (lidar_point.y - point.y));

                if (dist < this->max_range && dist > this->min_range)
                {
                    it = layer.cloud.erase(it);
                    jt = layer_base_link.cloud.erase(jt);
                    erased = true;
                    break;
                }
            }
            if (!erased)  // 삭제되지 않았을 때만 증가
            {
                ++it;
                ++jt;
            }
        }
    }

    return layer_vector;
}

CollisionFilter::CollisionFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config)
    : BaseFilter(node_ptr_)
{
    this->line_length = getYamlValue<float>(__FUNCTION__, config, "line_length", 0.05);
    this->resolution = getYamlValue<float>(__FUNCTION__, config, "resolution", 0.05);
}

LayerVector CollisionFilter::updateImpl(LayerVector layer_vector)
{
    // coordinate transform
    auto node = this->getNodePtr();
    node->setSensorLayerMap("collision", Layer());
    // int cloud_size_sum = 0;
    auto position = node->getPosition();
    double heading_x = cos(position.yaw);
    double heading_y = sin(position.yaw);

    double orthogonal_x = -heading_y;
    double orthogonal_y = heading_x;
    double length = this->line_length;
    double resolution = this->resolution;
    MotorStatus motor_status = node->getMotorStatus();

    const int layer_size = layer_vector.size();
    if (layer_size > 0)
    {
        auto& last_layer = *layer_vector.rbegin();
        if (last_layer.cloud.size() == 1)
        {
            auto it = last_layer.cloud.begin();
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

            pcl::PointXY point_global{it->x, it->y};
            auto line_layer = Layer();
            double start_x = point_global.x - orthogonal_x * length;
            double start_y = point_global.y - orthogonal_y * length;

            for (double t = 0; t <= length * 2; t += resolution)
            {
                pcl::PointXYZ new_point;
                new_point.x = start_x + orthogonal_x * t;
                new_point.y = start_y + orthogonal_y * t;
                new_point.z = 0.0;
                cloud->points.push_back(new_point);
            }
            last_layer.cloud.insert(last_layer.cloud.end(), cloud->begin(), cloud->end());
            // 2025.09.19 ksj, 전진일 때만 tof 데이터 저장
            if (motor_status.isMoveToFoward())
            {
                node->is_collision = true;
                if (logIntervalPassed())
                {
                    RCLCPP_INFO(
                        node->get_logger(),
                        "Collision detected. robot_xy(%.3f, %.3f), Coord(x:%.3f, y:%.3f).",
                        position.x,
                        position.y,
                        point_global.x,
                        point_global.y);
                }
            }
            else
            {
                if (logIntervalPassed())
                {
                    RCLCPP_INFO(
                        node->get_logger(),
                        "Back Collision detected. robot_xy(%.3f, %.3f), Coord(x:%.3f, y:%.3f).",
                        position.x,
                        position.y,
                        point_global.x,
                        point_global.y);
                }
            }
        }
    }
    return layer_vector;
}

CollisionTofFilter::CollisionTofFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config)
    : BaseFilter(node_ptr_)
{
    this->use_data = getYamlValue<bool>(__FUNCTION__, config, "use_data", false);
}

LayerVector CollisionTofFilter::updateImpl(LayerVector layer_vector)
{
    // coordinate transform
    auto node = this->getNodePtr();
    if (node->is_collision == false || this->use_data == false)
    {
        for (auto& layer : layer_vector)
        {
            if (layer.isDeletable)
                layer.cloud.clear();
        }
        return layer_vector;
    }
    node->is_collision = false;
    const int layer_size = layer_vector.size();
    if (layer_size > 0)
    {
        for (auto& layer : layer_vector)
        {
            if (layer.isDeletable == false)
            {
                continue;
            }
            layer.isDeletable = false;
            RCLCPP_INFO(node->get_logger(), "Collision detected. save Multi tof Data.");
        }
    }
    return layer_vector;
}

LowObstacleFilter::LowObstacleFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config)
    : BaseFilter(node_ptr_)
{
    this->input = getYamlValue<std::string>(__FUNCTION__, config, "input", "");
    this->dist_max = getYamlValue<float>(__FUNCTION__, config, "dist_max", 0.7);
    this->dist_min = getYamlValue<float>(__FUNCTION__, config, "dist_min", 0.0);
    this->dist_diff_threshold = getYamlValue<float>(__FUNCTION__, config, "dist_diff", 0.035);
}

LayerVector LowObstacleFilter::updateImpl(LayerVector layer_vector)
{
    auto node = this->getNodePtr();
    // 2025.05.09, 강성준, 승월시 낮은 장애물 검출 안함
    if (node->isClimb())
    {
        for (auto& layer : layer_vector)
        {
            if (layer.isDeletable)
                layer.cloud.clear();
        }
        return layer_vector;
    }

    auto position = node->getPosition();
    // geometry_msgs::msg::Transform transform_msg;
    // tf2::toMsg(position.getTransform().inverse(), transform_msg);
    // Eigen::Affine3f inverse_transform = tf2::transformToEigen(transform_msg).cast<float>();

    auto target_layer = node->getSensorLayer(this->input);
    for (auto& layer : layer_vector)
    {
        if (layer.isDeletable == false)
        {
            continue;
        }
        if (layer.cloud.size() > 0 && target_layer.cloud.size() > 0)
        {
            float dist_diff = pcl::euclideanDistance(layer.cloud[0], target_layer.cloud[0]);

            // RCLCPP_INFO(node->get_logger(), "dist_diff: %f", dist_diff);
            if (dist_diff <= this->dist_diff_threshold)
            {
                layer.isDeletable = false;
                if (logIntervalPassed())
                {
                    RCLCPP_INFO(
                        node->get_logger(),
                        "low level obstacle. robot_xy(%.3f, %.3f), Obs(x:%.3f, y:%.3f, Dist diff: %.3f)",
                        position.x,
                        position.y,
                        layer.cloud[0].x,
                        layer.cloud[0].y,
                        dist_diff);
                }
            }
            else
            {
                layer.cloud.clear();
            }
            // RCLCPP_INFO(node->get_logger(), "7/8 rows data exist. remove 7row data");
        }
        else
        {
            layer.cloud.clear();
        }
        // else if (layer.cloud.size() > 0 && target_layer.cloud.size() == 0 && layer.isDeletable)
        // {
        //     // 7행 데이터 존재, 8행 데이터 없음
        //     // 삭제가 가능한 것은 아직 로직처리가 안된 데이터
        //     // auto layer_base_link = layer;
        //     // pcl::transformPointCloud(layer.cloud, layer_base_link.cloud, inverse_transform);
        //     auto global_point = layer.cloud[0];
        //     // auto baselink_point = layer_base_link.cloud[0];

        //     float dist_diff = pcl::euclideanDistance(global_point, target_layer.cloud[0]);
        //     if (dist_diff <= 0.01)
        //     {
        //         RCLCPP_INFO(node->get_logger(), "Low obstacle -> dist_diff: %f", dist_diff);
        //     }
        //     // double dist = std::sqrt(baselink_point.x * baselink_point.x + baselink_point.y * baselink_point.y);
        //     // // 설정한 거리 범위 안이면 적용
        //     // if (dist < this->dist_max && dist > this->dist_min)
        //     // {
        //     //     layer.isDeletable = false;
        //     //     if (logIntervalPassed())
        //     //     {
        //     //         RCLCPP_INFO(
        //     //             node->get_logger(),
        //     //             "Low obstacle -> x: %f, y: %f, dist: %f",
        //     //             global_point.x,
        //     //             global_point.y,
        //     //             dist);
        //     //     }
        //     // }
        //     // else
        //     // {
        //     //     layer.cloud.clear();
        //     // }
        // }
    }

    return layer_vector;
}

DepthCameraLowObstacleFilter::DepthCameraLowObstacleFilter(
    std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config)
    : BaseFilter(node_ptr_)
{
    this->min_height = getYamlValue<float>(__FUNCTION__, config, "min_height", 0.03f);
    this->max_height = getYamlValue<float>(__FUNCTION__, config, "max_height", 0.18f);
    this->min_neighbors = getYamlValue<int>(__FUNCTION__, config, "min_neighbors", 4);
    this->radius = getYamlValue<float>(__FUNCTION__, config, "radius", 0.05f);
    this->x_min = getYamlValue<float>(__FUNCTION__, config, "x_min", 0.10f);
    this->x_max = getYamlValue<float>(__FUNCTION__, config, "x_max", 2.0f);
    this->half_width = getYamlValue<float>(__FUNCTION__, config, "half_width", 0.20f);
    this->confirm_frames = getYamlValue<int>(__FUNCTION__, config, "confirm_frames", 2);
    this->lidar_check_radius = getYamlValue<float>(__FUNCTION__, config, "lidar_check_radius", 0.10f);
    this->last_snapshot_stamp = node_ptr_->now();

    RCLCPP_INFO(node_ptr->get_logger(),
                "[DepthCameraLowObstacleFilter] height(map z)[%.3f,%.3f]m, "
                "corridor x[%.2f,%.2f] |y|<=%.2f, "
                "confirm: min_neighbors=%d in radius=%.3fm x %d frames, "
                "lidar_check_radius=%.2fm (pre-lock only), "
                "lock-and-persist (no timeout)",
                min_height, max_height, x_min, x_max, half_width,
                min_neighbors, radius, confirm_frames, lidar_check_radius);
}

pcl::PointCloud<pcl::PointXYZ> DepthCameraLowObstacleFilter::extractCandidates(
    const pcl::PointCloud<pcl::PointXYZ>& snapshot_cloud)
{
    auto node = this->getNodePtr();
    auto position = node->getPosition();
    geometry_msgs::msg::Transform transform_msg;
    tf2::toMsg(position.getTransform().inverse(), transform_msg);
    Eigen::Affine3f inverse_transform = tf2::transformToEigen(transform_msg).cast<float>();

    pcl::PointCloud<pcl::PointXYZ> cloud_bl;
    pcl::transformPointCloud(snapshot_cloud, cloud_bl, inverse_transform);

    // 1) 높이 band(map z) + 전방 코리도(base_link) 크롭 - 한 패스 O(N)
    pcl::PointCloud<pcl::PointXYZ> band;     // map frame
    pcl::PointCloud<pcl::PointXYZ> band_bl;  // base_link frame (lidar 비교용)
    for (std::size_t i = 0; i < snapshot_cloud.size(); ++i)
    {
        const auto& pm = snapshot_cloud[i];  // map frame
        const auto& pb = cloud_bl[i];        // base_link frame
        if (pm.z < this->min_height || pm.z > this->max_height) continue;
        if (pb.x < this->x_min || pb.x > this->x_max || std::abs(pb.y) > this->half_width) continue;
        band.push_back(pm);
        band_bl.push_back(pb);
    }
    if (band.empty())
    {
        return {};
    }

    // 2) 공간 확인: 반경 radius 내 이웃 min_neighbors 이상 (고립 노이즈 제거)
    std::vector<bool> keep(band.size(), true);
    if (this->min_neighbors > 0 && this->radius > 0.0f)
    {
        if (band.size() <= static_cast<std::size_t>(this->min_neighbors))
        {
            return {};
        }
        pcl::search::KdTree<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(band.makeShared());
        for (std::size_t i = 0; i < band.size(); ++i)
        {
            std::vector<int> indices;
            std::vector<float> sqr_dists;
            int found = kdtree.radiusSearch(band[i], this->radius, indices, sqr_dists);
            keep[i] = (found - 1 >= this->min_neighbors);  // 자기 자신 제외
        }
    }

    Layer lidar_pc = node->getSensorLayer("lidar_pc_baselink");
    const bool use_lidar_check =
        this->lidar_check_radius > 0.0f && !lidar_pc.cloud.empty();
    const float lidar_r_sqr = this->lidar_check_radius * this->lidar_check_radius;

    pcl::PointCloud<pcl::PointXYZ> candidates;
    for (std::size_t i = 0; i < band.size(); ++i)
    {
        if (!keep[i]) continue;
        if (use_lidar_check)
        {
            bool near_lidar = false;
            for (const auto& lp : lidar_pc.cloud)
            {
                const float dx = lp.x - band_bl[i].x;
                const float dy = lp.y - band_bl[i].y;
                if (dx * dx + dy * dy <= lidar_r_sqr)
                {
                    near_lidar = true;
                    break;
                }
            }
            if (near_lidar) continue;
        }
        candidates.push_back(band[i]);
    }
    return candidates;
}

LayerVector DepthCameraLowObstacleFilter::updateImpl(LayerVector layer_vector)
{
    auto node = this->getNodePtr();

    // locked(확정) 레이어와 미처리 최신 스냅샷을 찾는다.
    // timerCallback 이 매 tick 같은 스냅샷을 중복 push 하므로 timestamp 로
    // 신규 여부를 판별하고, 미확정 레이어는 이번 tick 에 전부 소모한다.
    std::ptrdiff_t locked_idx = -1;
    std::ptrdiff_t newest_idx = -1;
    for (std::size_t i = 0; i < layer_vector.size(); ++i)
    {
        auto& layer = layer_vector[i];
        if (layer.isDeletable == false)
        {
            locked_idx = static_cast<std::ptrdiff_t>(i);
        }
        else if (layer.timestamp > this->last_snapshot_stamp)
        {
            if (newest_idx < 0 || layer.timestamp > layer_vector[newest_idx].timestamp)
            {
                newest_idx = static_cast<std::ptrdiff_t>(i);
            }
        }
    }

    // 승월 중 신규 감지 중단 (기존 mtof low_obstacle 과 동일 정책).
    // pitch 상황에서는 upstream RANSAC 바닥 제거가 실패할 수 있어 오확정 위험이 크다.
    // locked 점은 유지된다.
    if (node->isClimb())
    {
        this->pending.clear();
        for (auto& layer : layer_vector)
        {
            if (layer.isDeletable)
                layer.cloud.clear();
        }
        return layer_vector;
    }

    pcl::PointCloud<pcl::PointXYZ> new_locked_points;
    if (newest_idx >= 0)
    {
        this->last_snapshot_stamp = layer_vector[newest_idx].timestamp;
        auto candidates = this->extractCandidates(layer_vector[newest_idx].cloud);

        const float match_sqr = this->radius * this->radius;
        const pcl::PointCloud<pcl::PointXYZ>* locked_cloud =
            (locked_idx >= 0) ? &layer_vector[locked_idx].cloud : nullptr;

        // 시간 확인: 연속 confirm_frames 스냅샷에서 관측된 후보만 확정.
        // 이미 확정된 점 근방(radius)의 후보는 중복이므로 버린다 (밀도 자체 제한).
        std::vector<PendingPoint> next_pending;
        for (const auto& p : candidates)
        {
            bool duplicated = false;
            if (locked_cloud != nullptr)
            {
                for (const auto& lp : *locked_cloud)
                {
                    const float dx = lp.x - p.x, dy = lp.y - p.y;
                    if (dx * dx + dy * dy <= match_sqr)
                    {
                        duplicated = true;
                        break;
                    }
                }
            }
            for (const auto& np : new_locked_points)
            {
                const float dx = np.x - p.x, dy = np.y - p.y;
                if (dx * dx + dy * dy <= match_sqr)
                {
                    duplicated = true;
                    break;
                }
            }
            if (duplicated) continue;

            int32_t hits = 1;
            for (const auto& prev : this->pending)
            {
                const float dx = prev.point.x - p.x, dy = prev.point.y - p.y;
                if (dx * dx + dy * dy <= match_sqr)
                {
                    hits = prev.hits + 1;
                    break;
                }
            }

            if (hits >= this->confirm_frames)
            {
                new_locked_points.push_back(p);
            }
            else
            {
                next_pending.push_back({p, hits});
            }
        }
        // 이번 스냅샷에서 관측되지 않은 pending 은 폐기 (연속 관측 조건)
        this->pending = std::move(next_pending);
    }

    // 미확정(스냅샷) 레이어는 모두 소모: 판정은 위에서 끝났고,
    // 빈 cloud 레이어는 BaseFilter::update() 가 vector 에서 제거한다.
    for (auto& layer : layer_vector)
    {
        if (layer.isDeletable)
            layer.cloud.clear();
    }

    // 신규 확정 점을 locked 레이어에 누적 (없으면 생성).
    if (!new_locked_points.empty())
    {
        auto now = node->now();
        if (locked_idx < 0)
        {
            Layer locked_layer;
            locked_layer.isDeletable = false;
            layer_vector.push_back(locked_layer);
            locked_idx = static_cast<std::ptrdiff_t>(layer_vector.size() - 1);
        }
        auto& locked_layer = layer_vector[locked_idx];
        locked_layer.cloud.insert(
            locked_layer.cloud.end(), new_locked_points.begin(), new_locked_points.end());
        locked_layer.timestamp = now;
        locked_layer.sensor_timestamp = now;

        if (logIntervalPassed())
        {
            auto position = node->getPosition();
            const auto& p0 = new_locked_points[0];
            RCLCPP_INFO(
                node->get_logger(),
                "Low obstacle locked (depth). robot_xy(%.3f, %.3f), Obs(x:%.3f, y:%.3f, z:%.3f), "
                "new:%zu total:%zu",
                position.x,
                position.y,
                p0.x,
                p0.y,
                p0.z,
                new_locked_points.size(),
                locked_layer.cloud.size());
        }
    }

    return layer_vector;
}

ObstacleLoggerFilter::ObstacleLoggerFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config)
    : BaseFilter(node_ptr_)
{
    this->tag = getYamlValue<std::string>(__FUNCTION__, config, "tag", "OBS");
}

LayerVector ObstacleLoggerFilter::updateImpl(LayerVector layer_vector)
{
    if (!this->logIntervalPassed())
    {
        return layer_vector;
    }

    auto node = this->getNodePtr();
    auto position = node->getPosition();
    geometry_msgs::msg::Transform transform_msg;
    tf2::toMsg(position.getTransform().inverse(), transform_msg);
    Eigen::Affine3f inverse_transform = tf2::transformToEigen(transform_msg).cast<float>();

    std::size_t total_pts = 0;
    std::size_t locked_layers = 0;
    float min_range = std::numeric_limits<float>::max();
    float fwd = 0.0f, lat = 0.0f;
    float h_min = std::numeric_limits<float>::max();
    float h_max = -std::numeric_limits<float>::max();
    for (const auto& layer : layer_vector)
    {
        if (layer.isDeletable == false) ++locked_layers;
        if (layer.cloud.empty()) continue;

        pcl::PointCloud<pcl::PointXYZ> cloud_bl;
        pcl::transformPointCloud(layer.cloud, cloud_bl, inverse_transform);
        for (std::size_t i = 0; i < cloud_bl.size(); ++i)
        {
            const float range =
                std::sqrt(cloud_bl[i].x * cloud_bl[i].x + cloud_bl[i].y * cloud_bl[i].y);
            if (range < min_range)
            {
                min_range = range;
                fwd = cloud_bl[i].x;
                lat = cloud_bl[i].y;
            }
            h_min = std::min(h_min, layer.cloud[i].z);
            h_max = std::max(h_max, layer.cloud[i].z);
            ++total_pts;
        }
    }

    if (total_pts > 0)
    {
        RCLCPP_INFO(node->get_logger(),
                    "[%s] published pts=%zu layers=%zu(locked=%zu) nearest=%.2fm "
                    "(fwd=%.2f lat=%+.2f) height=[%.2f,%.2f]m",
                    this->tag.c_str(), total_pts, layer_vector.size(), locked_layers,
                    min_range, fwd, lat, h_min, h_max);
    }

    return layer_vector;
}

OneDRoIFilter::OneDRoIFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config)
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

bool OneDRoIFilter::isInside(pcl::PointXY point)
{
    return pcl::isXYPointIn2DXYPolygon<pcl::PointXY>(point, this->polygon);
}

LayerVector OneDRoIFilter::updateImpl(LayerVector layer_vector)
{
    // coordinate transform
    auto node = this->node_ptr;
    auto position = node->getPosition();
    geometry_msgs::msg::Transform transform_msg;
    tf2::toMsg(position.getTransform().inverse(), transform_msg);
    Eigen::Affine3f inverse_transform = tf2::transformToEigen(transform_msg).cast<float>();

    // roi filter
    for (auto& layer : layer_vector)
    {
        if (layer.isDeletable == false)
        {
            continue;
        }

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
                layer.isDeletable = false;
            }
        }
    }
    return layer_vector;
}
}  // namespace A1::perception
