#include "filter/filter.hpp"

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

    this->compare_dist_diff_max = getYamlValue<float>(__FUNCTION__, config, "compare_dist_diff_max", 0.5);
    this->inputs = getYamlValue<std::vector<std::string>>(__FUNCTION__, config, "inputs", std::vector<std::string>{});
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

    Layer target_layer;
    for (auto input : this->inputs)
    {
        auto target_sensor_layer = node->getSensorLayer(input);
        target_layer.cloud.insert(
            target_layer.cloud.end(), target_sensor_layer.cloud.begin(), target_sensor_layer.cloud.end());
    }

    for (auto& layer : layer_vector)
    {
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

            float max_diff_ = 0;
            float min_diff_ = 1.0;
            bool diff_check = false;
            for (const auto& tgt_point : target_layer.cloud)
            {
                float dist_diff = pcl::euclideanDistance(point_global, tgt_point);
                max_diff_ = std::max(max_diff_, dist_diff);
                min_diff_ = std::min(min_diff_, dist_diff);
                // if (dist_diff > this->row3_dist_diff_min && dist_diff < this->row3_dist_diff_max)
                if (dist_diff < this->compare_dist_diff_max)
                {
                    // RCLCPP_INFO(node->get_logger(), "pass ! 3row dist diff %f", dist_diff);
                    diff_check = true;
                    break;
                }
            }
            if (diff_check)
            {
                it = layer.cloud.erase(it);
                jt = layer_base_link.cloud.erase(jt);
                continue;
            }

            std::string key = std::to_string(point_global.x).substr(0, std::to_string(point_global.x).find(".") + 3) +
                              "," +
                              std::to_string(point_global.y).substr(0, std::to_string(point_global.y).find(".") + 3);
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
    }
    return layer_vector;
}

LowObstacleFilter::LowObstacleFilter(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config)
    : BaseFilter(node_ptr_)
{
    this->input = getYamlValue<std::string>(__FUNCTION__, config, "input", "");
    this->dist_max = getYamlValue<float>(__FUNCTION__, config, "dist_max", 0.7);
    this->dist_min = getYamlValue<float>(__FUNCTION__, config, "dist_min", 0.0);
    this->dist_diff = getYamlValue<float>(__FUNCTION__, config, "dist_diff", 0.035);

    // RCLCPP_INFO(
    //     this->getNodePtr()->get_logger(), "Low obstacle dist_max-> %f, dist_min-> %f", this->dist_max,
    //     this->dist_min);
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
    geometry_msgs::msg::Transform transform_msg;
    tf2::toMsg(position.getTransform().inverse(), transform_msg);
    Eigen::Affine3f inverse_transform = tf2::transformToEigen(transform_msg).cast<float>();

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
            if (dist_diff <= 0.035)
            {
                layer.isDeletable = false;
                if (logIntervalPassed())
                {
                    RCLCPP_INFO(
                        node->get_logger(),
                        "low level obstacle. robot_xy(%.3f, %.3f), Obs(x:%.3f, y:%.3f, Dist diff: %.3f)",
                        position.x,
                        position.y,
                        global_point.x,
                        global_point.y,
                        dist);
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
